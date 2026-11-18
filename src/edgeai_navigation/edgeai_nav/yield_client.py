"""Explicit preplanning adapter to the same navigation-owned yielding action."""
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def request_peer_yield(node,robot,blocked_goal,wait):
    clients=getattr(node,'peer_yield_clients',None)
    if clients is None:clients=node.peer_yield_clients={}
    if robot not in clients:
        clients[robot]=ActionClient(node,NavigateToPose,
            '/'+(robot+'/' if robot else '')+'navigation/yield_peer')
    client=clients[robot]
    if not client.server_is_ready():return False
    sent=client.send_goal_async(NavigateToPose.Goal(pose=blocked_goal))
    try:wait(sent.done,10.)
    except Exception as cause:
        # A late acceptance can already have committed the peer's maneuver.
        # Report uncertainty instead of canceling another robot's navigation.
        error=RuntimeError('Yield request acceptance unknown');error.category='fatal';raise error from cause
    handle=sent.result()
    if not handle.accepted:return False
    # This is a request to another robot, not a navigation handle owned by C1.
    # Task cleanup must not cancel a committed peer maneuver.
    result=handle.get_result_async()
    try:wait(result.done,120.)
    except BaseException:
        try:node.wait(result.done,120.)
        except BaseException as cause:
            error=RuntimeError('Independent peer yield terminal unknown');error.category='fatal';raise error from cause
        raise
    terminal=result.result()
    if terminal.result.error_code==902:
        error=RuntimeError(terminal.result.error_msg);error.category='fatal';raise error
    return terminal.status==4 and terminal.result.error_code==0
