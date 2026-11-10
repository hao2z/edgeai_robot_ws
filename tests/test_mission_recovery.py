"""A failed optional observation must be recorded, never reported as success."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from edgeai_autonomy.mission import Mission


class ObservationRecoveryTests(unittest.TestCase):
    def state(self):
        grid = SimpleNamespace(data=[0], info=SimpleNamespace(height=1, width=1, resolution=1.0,
                                origin=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.))))
        return SimpleNamespace(transition=Mock(),action=Mock(return_value=False),spin_action=object(),
                               grid=grid, position=lambda: (0.5,0.5,0.), config={'observation_clearance_m':0.55})

    @patch('edgeai_autonomy.mission.clearance_at', return_value=0.375)
    def test_no_full_spin_inside_narrow_door(self, clearance):
        state=self.state()
        self.assertFalse(Mission.scan(state, required=False))
        state.action.assert_not_called()
        self.assertEqual(state.transition.call_args.args[0], 'observation_deferred')

    def test_optional_spin_failure_is_recorded(self):
        state=self.state()
        self.assertFalse(Mission.scan(state,required=False))
        self.assertEqual(state.transition.call_args.args[0],'observation_incomplete')

    def test_initial_spin_failure_stops_mission(self):
        state=self.state()
        with self.assertRaises(RuntimeError):
            Mission.scan(state)


if __name__=='__main__':unittest.main()
