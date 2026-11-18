"""Read-only graph comparisons. A jump is evidence, never automatic rejection."""
import math


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def relative(a, b):
    c,s=math.cos(a[2]),math.sin(a[2]);x,y=b[0]-a[0],b[1]-a[1]
    return [c*x+s*y,-s*x+c*y,angle(b[2]-a[2])]


def compare_graphs(before, after, translation_m=.3, yaw_rad=.25):
    if not before or before.get('epoch')!=after.get('epoch'):return None
    old={n['id']:n for n in before['nodes']};new={n['id']:n for n in after['nodes']}
    changes=[]
    for key in old.keys() & new.keys():
        a,b=old[key]['map'],new[key]['map']
        d=math.hypot(b[0]-a[0],b[1]-a[1]);yaw=angle(b[2]-a[2])
        if d>=translation_m or abs(yaw)>=yaw_rad:
            changes.append(dict(id=key,robot=new[key]['robot_id'],translation_m=d,yaw_rad=yaw,
                                before=a,after=b))
    if not changes:return None
    pairs={(e['source'],e['target']) for e in before.get('constraints',[])}
    added=[]
    for edge in after.get('constraints',[]):
        if (edge['source'],edge['target']) in pairs:continue
        a,b=new.get(edge['source']),new.get(edge['target'])
        if a is None or b is None:continue
        observed=relative(a.get('sensor_map',a['map']),b.get('sensor_map',b['map']));expected=edge['delta']
        added.append(dict(edge,cross_robot=a['robot_id']!=b['robot_id'],
            translation_residual_m=math.hypot(observed[0]-expected[0],observed[1]-expected[1]),
            residual_frame='sensor' if 'sensor_map' in a and 'sensor_map' in b else 'base_proxy',
            yaw_residual_rad=angle(observed[2]-expected[2])))
    return dict(event='SLAM_GRAPH_JUMP',stamp_ns=after['stamp_ns'],epoch=after['epoch'],
        previous_revision=before.get('revision'),revision=after.get('revision'),
        changed_nodes=len(changes),max_translation_m=max(c['translation_m'] for c in changes),
        changed_robots=sorted({c['robot'] for c in changes}),
        largest_changes=sorted(changes,key=lambda c:c['translation_m'],reverse=True)[:8],
        added_constraints=added,
        interpretation='Graph correction detected; added edges are correlated evidence, not proven cause')
