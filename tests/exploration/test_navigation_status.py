"""Test status evidence and absence handling without RViz or simulation."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/ros'))
from navigation_status import render


def sample():
    return dict(authority=dict(mode='AUTO',phase='ACTIVE',generation=7,owner='AUTO',
                arrival_policy='position',following_path=True,distance_remaining=1.2),
                authority_age=.1, settings=dict(depth_enabled=True),
                ages={k:.1 for k in ('scan','odom','depth_cloud','local_costmap')},
                wall_ages={k:.1 for k in ('scan','odom','depth_cloud','local_costmap','cmd_vel')},
                rates=dict(depth_cloud=7.5), velocities=dict(cmd_vel=(.2,.1),odom=(.19,.09)))


class NavigationStatus(unittest.TestCase):
    def test_active_and_missing_inputs(self):
        state = sample()
        result = render(state)
        self.assertEqual(result['heading'],'正在跟踪路径')
        self.assertEqual(result['warnings'],[])
        self.assertIn('1.20 m',result['body'])
        state['wall_ages']['depth_cloud']=3.
        self.assertTrue(any('深度输入' in w for w in render(state)['warnings']))
        state['settings']['depth_enabled']=False
        self.assertFalse(any('深度输入' in w for w in render(state)['warnings']))
        self.assertIn('已关闭，单雷达',render(state)['body'])

    def test_no_false_active_from_stale_authority(self):
        state = sample()
        state['authority_age']=3.
        state['stationary_s']=20.
        result = render(state)
        self.assertNotIn('正在跟踪',result['heading'])
        self.assertNotIn('路径跟踪停滞',result['heading'])
        self.assertIn('无新鲜活动任务',result['body'])

    def test_recovery_displacement_is_not_navigation_success(self):
        state = sample()
        state['authority'].update(following_path=False,recovery=dict(kind='backup',outcome='RUNNING'))
        self.assertEqual(render(state)['heading'],'正在执行恢复动作')
        state['authority']['recovery']['outcome']='AWAITING_NAVIGATION'
        self.assertIn('尚未证明导航恢复',render(state)['body'])
        self.assertNotIn('已验证导航恢复',render(state)['body'])

    def test_idle_old_paths_and_clock_pause_do_not_imply_stall(self):
        state = sample()
        state['ages']['global_plan']=10.
        state['authority']['phase']='IDLE'
        self.assertFalse(any('旧路线' in w for w in render(state)['warnings']))
        state['clock_paused']=True
        state['stationary_s']=10.
        self.assertEqual(render(state)['heading'],'仿真时钟暂停或未推进')

    def test_stall_and_blocked_are_observations_not_collision_claims(self):
        state = sample()
        state['stationary_s']=5.1
        self.assertEqual(render(state)['heading'],'路径跟踪停滞，原因待核查')
        state.update(tracking=dict(status='BLOCKED'),tracking_age=.1)
        self.assertEqual(render(state)['heading'],'RPP控制计算受阻')
        self.assertIn('不是物理碰撞',render(state)['warnings'][-1])

    def test_html_escaping_and_missing_data(self):
        state=sample()
        state['authority']['owner']='<script>bad</script>'
        state['velocities']={}
        result=render(state)
        self.assertNotIn('<script>',result['body'])
        self.assertIn('无新数据',result['body'])
        self.assertEqual(render({})['heading'],'等待任务状态')

    def test_previous_action_not_mistaken_for_current_goal(self):
        state=sample()
        state['authority']['goal_uuid']='new'
        state['navigation_action']=dict(status=6,goal_id='old')
        state['wall_ages']['navigation_action']=.1
        result=render(state)
        self.assertIn('最近导航动作记录',result['body'])
        self.assertNotIn('当前导航动作',result['body'])
        self.assertIn('已中止',result['body'])


if __name__ == '__main__':
    unittest.main()
