"""Shared names for mission events in the terminal and RViz."""
from language import localize

TASKS = dict(find_targets='搜寻全部救援目标', s1='S1 单机救援', c1='C1 双机救援')
UNITS = dict(
    acquire='取得导航控制权', select_target='检查已定位目标', search_target='搜寻缺失目标',
    next_target='切换搜寻目标', check_targets='核对目标清单', select_approach='选择接近点',
    dispatch_navigation='派发导航', poll_navigation='等待导航结果',
    inside_rescue_region='核对救援范围', check_rescue='检查救援条件与停留时间',
    correct='微调救援位姿', handover='切换共享地图', wait_navigation_ready='等待双机导航就绪',
    localize='激光定位', prepare_target='核对 C1 位置', select_parking='选择双机停车点',
    select_parking_locked='重新选择未到位车辆的停车点', record_success='记录任务成功',
    quiesce='结束探索动作', save_map='保存地图', mapping_supplement='补充建图',
    cancel_navigation='取消导航', cancel_all_navigation='取消双机导航')
PHASES = dict(
    STARTING='启动任务调度', WAITING_EXPLORATION='等待探索器就绪', EXPLORING='探索建图',
    FRONTIERS_EXHAUSTED='暂时没有可用前沿', MAPPING_FRONTIER_AVAILABLE='重新发现可用前沿',
    OBSERVATION_SUPPRESSED_TARGETS_LOCATED='目标已齐，跳过观察转圈',
    MAPPING_AREA_READY='建图面积达标', MAPPING_AREA_INSUFFICIENT='建图面积不足或地图过期')


def _mission_description(data):
    phase = data.get('phase', '')
    task = TASKS.get(data.get('task'), data.get('task', ''))
    unit = data.get('unit', data.get('step', ''))
    unit = UNITS.get(unit, unit)
    reason = ' '.join(str(data.get('reason') or '').split())[:140]
    if phase == 'TASK_REQUESTED':
        return f'开始任务：{task}'
    if phase == 'TASK_FEEDBACK':
        return f'{task} · {unit}'
    if phase == 'TASK_STOP_UNCONFIRMED':
        return f'{task} · 未确认任务已停止'
    if phase == 'STAGE_STARTED':
        return f'开始阶段：{unit}'
    if phase in ('TASK_FINISHED', 'STAGE_FINISHED', 'RESULT'):
        subject = task or unit or '整轮任务'
        result = '成功' if data.get('success') is True else '失败'
        return f'{subject} · {result}'+(f'：{reason}' if reason and result == '失败' else '')
    label = PHASES.get(phase, '')
    if phase in ('MAPPING_AREA_READY', 'MAPPING_AREA_INSUFFICIENT'):
        area, required = data.get('known_area_m2'), data.get('required_area_m2')
        if isinstance(area, (int, float)) and isinstance(required, (int, float)):
            label += f'：{area:.2f} / {required:.2f} m²'
    return label


def mission_description(data, selected=None):
    return localize(_mission_description(data), selected)
