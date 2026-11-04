"""Transport-independent ownership and evidence rules for navigation."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AuthorityConfig:
    """Single source of truth for cooperative control endpoints and deadlines."""
    accept_timeout_s: float = 10.0
    cancel_timeout_s: float = 5.0
    feedback_timeout_s: float = 10.0
    result_grace_s: float = 0.5
    explorer_cancel_grace_s: float = 1.0
    explorer_sync_timeout_s: float = 5.0
    explorer_sync_retry_s: float = 1.0
    backend_action: str = '/navigation_backend/navigate_to_pose'
    auto_action: str = '/exploration/navigate_to_pose'
    manual_action: str = '/navigate_to_pose'
    explorer_control: str = '/exploration/control_exploration'


@dataclass
class Authority:
    mode: str = 'AUTO'
    phase: str = 'IDLE'
    owner: str = ''
    generation: int = 0
    cancel_requested: bool = False
    fault: str = ''
    mode_requested_at: float | None = None
    cancel_sent_at: float | None = None

    def reserve(self, owner):
        if self.phase != 'IDLE' or self.mode != owner:
            return None
        self.generation += 1
        self.owner, self.phase = owner, 'SENDING'
        self.cancel_requested, self.fault = False, ''
        self.mode_requested_at = None
        self.cancel_sent_at = None
        return self.generation

    def set_mode(self, mode):
        if mode not in ('AUTO', 'MANUAL', 'STOPPED'):
            raise ValueError(mode)
        changed = self.mode != mode
        if not changed:
            return False
        self.mode = mode
        if self.phase != 'IDLE' and self.owner != mode:
            self.cancel_requested = True
            if self.mode_requested_at is None:
                self.mode_requested_at = 0.0
            if self.phase == 'ACTIVE':
                self.phase = 'CANCELING'
        return True

    def mark_mode_request(self, monotonic_now):
        if self.mode_requested_at in (None, 0.0):
            self.mode_requested_at = monotonic_now

    def should_force_cancel(self, monotonic_now, config=AuthorityConfig()):
        return bool(self.cancel_requested and self.mode_requested_at is not None and
                    monotonic_now - self.mode_requested_at >= config.explorer_cancel_grace_s)

    def mark_cancel_sent(self, monotonic_now):
        self.cancel_sent_at = monotonic_now

    def accepted(self, generation):
        if generation != self.generation or self.phase == 'IDLE':
            return False
        # A late acceptance resolves SENDING uncertainty, but must honor a stop.
        self.phase = 'CANCELING' if self.cancel_requested else 'ACTIVE'
        self.fault = ''
        return True

    def request_cancel(self, generation):
        if generation != self.generation or self.phase == 'IDLE':
            return False
        self.cancel_requested = True
        if self.phase == 'ACTIVE':
            self.phase = 'CANCELING'
        return True

    def uncertain(self, generation, reason):
        if generation == self.generation and self.phase != 'IDLE':
            self.phase, self.fault = 'UNKNOWN', reason

    def terminal(self, generation):
        if generation != self.generation or self.phase == 'IDLE':
            return False
        self.phase, self.owner, self.fault = 'IDLE', '', ''
        self.cancel_requested = False
        self.mode_requested_at = None
        self.cancel_sent_at = None
        return True


@dataclass
class RecoveryEvidence:
    kind: str
    start: tuple | None
    distance_before: float | None
    action_succeeded: bool = False
    movement_verified: bool = False
    recovered: bool = False
    outcome: str = 'RUNNING'
    end: tuple | None = None

    def finish(self, succeeded, pose):
        self.action_succeeded, self.end = succeeded, pose
        if not succeeded:
            self.outcome = 'ACTION_FAILED'
            return
        if self.start is None or pose is None:
            self.outcome = 'UNVERIFIED_NO_ODOMETRY'
            return
        dx, dy = pose[0]-self.start[0], pose[1]-self.start[1]
        yaw = abs(math.atan2(math.sin(pose[2]-self.start[2]),
                             math.cos(pose[2]-self.start[2])))
        self.movement_verified = (yaw >= .25 if self.kind == 'spin'
                                  else math.hypot(dx, dy) >= .04)
        self.outcome = 'AWAITING_NAVIGATION' if self.movement_verified else 'UNVERIFIED_NO_MOTION'

    def note_navigation(self, distance, following, pose, goal_succeeded=False):
        if self.recovered or not self.action_succeeded or not self.movement_verified:
            return False
        if not goal_succeeded:
            if not following or pose is None or self.end is None:
                return False
            moved = math.hypot(pose[0]-self.end[0], pose[1]-self.end[1]) >= .05
            improved = (distance is not None and self.distance_before is not None
                        and self.distance_before-distance >= .10)
            if not moved or not improved:
                return False
        self.recovered, self.outcome = True, 'NAVIGATION_RESTORED'
        return True
