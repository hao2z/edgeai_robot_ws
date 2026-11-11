"""ROS request routes and RViz input; execution ownership lives in the authority."""
from rclpy.action import ActionClient,ActionServer
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from goal_policy import ACTION_ROUTES


class NavigationRequests:
    def __init__(self,node):
        self.node=node;self.require_heading=False
        self.position=ActionClient(node,NavigateToPose,node.config.manual_action,callback_group=node.group)
        self.heading=ActionClient(node,NavigateToPose,'navigation/heading_to_pose',callback_group=node.group)
        self.servers=[ActionServer(node,NavigateToPose,endpoint,
            execute_callback=node.execute,callback_group=node.group,
            handle_accepted_callback=node.upstream_accepted,
            goal_callback=lambda request,owner=owner,policy=policy:node.admit(request,owner,policy),
            cancel_callback=node.cancel) for endpoint,owner,policy in ACTION_ROUTES]
        self.preference=node.create_service(SetBool,'navigation_authority/set_goal_heading',
            self.set_goal_heading,callback_group=node.group)
        self.subscription=node.create_subscription(PoseStamped,'goal_pose',self.manual_pose,1)

    def set_goal_heading(self,request,response):
        self.require_heading=request.data
        response.success=True;response.message='heading' if request.data else 'position'
        self.node.event('MANUAL_GOAL_POLICY','后续RViz手动目标朝向要求已更新',require_heading=request.data)
        return response

    def manual_pose(self,pose):
        n=self.node
        if n.policy.mode!='MANUAL' or n.policy.phase!='IDLE':
            n.event('MANUAL_GOAL_REJECTED','RViz目标需先取得手动控制权，且等待旧任务结束');return
        if not n.backend.server_is_ready():
            n.event('MANUAL_GOAL_REJECTED','Nav2导航后端未就绪，请检查nav.log中的启动错误');return
        client=self.heading if self.require_heading else self.position
        client.send_goal_async(NavigateToPose.Goal(pose=pose)).add_done_callback(self.goal_response)

    def goal_response(self,future):
        try:
            handle=future.result()
            if handle.accepted:handle.get_result_async().add_done_callback(lambda done:done.result())
        except Exception as error:
            self.node.event('MANUAL_CLIENT_ERROR','手动目标请求异常',error=str(error))
