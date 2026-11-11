"""Compact human search messages; numeric source data stays unrounded."""
import math


def goal_text(goal):
    return (f"目标ID={goal['target_id']}，map位置=({goal['x']:.2f}, {goal['y']:.2f}) m，"
            f"朝向={math.degrees(goal['yaw']):.0f}°")


def search_text(code,data):
    if code in ('SEARCH_CANDIDATE','SEARCH_VIEW','SEARCH_VIEW_RESULT','SEARCH_SPIN','SEARCH_SPIN_RESULT'):
        candidate=data.get('candidate',data)
        strategy=candidate.get('strategy','unknown')
        label={'suspected_panel':'疑似目标候选','frustum':'视锥候选','spatial':'空间分散候选'}.get(strategy,'未知类型候选')
        if strategy=='suspected_panel':label+=f" #{candidate['suspect_id']}"
        position=f"map观察位=({data.get('x',candidate.get('x')):.2f}, {data.get('y',candidate.get('y')):.2f}) m"
        text=f'{label}｜{position}'
        if code=='SEARCH_CANDIDATE':
            text+=f"，朝向={math.degrees(candidate['yaw']):.0f}°"
            if strategy=='suspected_panel':
                p=candidate['suspect_position']
                text+=f'｜疑似位置=({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) m'
                basis={'inferred_face_from_edge':'由薄板侧边推定板面法线',
                       'alternate_face':'歧义板面的另一法线假设',
                       'measured_surface':'实测表面法线'}.get(candidate.get('normal_basis'))
                if basis:text+='｜'+basis
                if 'view_angle_offset_deg' in candidate:
                    text+=(f"，法线={math.degrees(candidate['face_normal_yaw']):.0f}°"
                           f"，偏转={candidate['view_angle_offset_deg']:+.0f}°"
                           f"，相机距离={candidate['camera_distance_m']:.2f} m")
            elif strategy=='frustum':text+=f"｜预计收益={candidate['gain_m2']:.2f} m²"
        else:
            action='转圈' if 'SPIN' in code else '导航'
            if 'RESULT' in code:
                status={4:'成功',5:'已取消',6:'失败'}.get(data['status'],f"状态{data['status']}")
                text+=f"｜{action}{status}，错误码={data['error_code']}"
                if 'cancel_reason' in data:
                    reason={'target_found':'已发现目标','search_budget':'搜寻预算耗尽',
                            'view_timeout':'单点导航超时','spin_timeout':'转圈超时'}.get(data['cancel_reason'],data['cancel_reason'])
                    text+='，原因='+reason
            else:
                text+=f"｜{action}{'已接受' if data['accepted'] else '被拒绝'}"
                if data.get('retry',0):text+=f"，重试第{data['retry']}次"
        return text
    if code=='SEARCH_POSITIONS':
        return (f"基础搜寻候选 {data['count']} 个，本轮最多访问 {data['visit_limit']} 个位置，"
                f"净空半径 {data['clearance_m']:.2f} m；疑似目标候选启用时另行优先调度")
    if code=='SEARCH_FOUND':return '已定位救援目标｜'+goal_text(data)
    if code=='PREPARING':return '准备救援｜'+goal_text(data)
    if code=='SEARCH_FRUSTUM_READY':
        return (f"视锥评分就绪：{data['candidate_positions']} 个位置 × {data['yaw_samples']} 个方向，"
                f"{data['samples']} 个样本，预计算 {data['prepare_ms']:.0f} ms")
    return None
