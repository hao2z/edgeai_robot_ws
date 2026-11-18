#!/usr/bin/env python3
"""Shared RViz runtime configuration for ordinary and static-map fleet missions."""
from pathlib import Path
import sys,yaml,os
from copy import deepcopy
source,output=map(Path,sys.argv[1:3]);config=yaml.safe_load(source.read_text())
manager=config['Visualization Manager']
frame_rate=int(os.environ.get('EDGEAI_RVIZ_FPS','30'))
if not 1 <= frame_rate <= 120:
    raise SystemExit('EDGEAI_RVIZ_FPS must be 1..120')
manager['Global Options']['Frame Rate']=frame_rate
nav=yaml.safe_load(Path(sys.argv[5]).read_text())
depth_enabled='depth_layer' in nav['local_costmap']['local_costmap']['ros__parameters']['plugins']
if sys.argv[3]=='true':manager['Views']['Current']['Focal Point'].update(X=2.5,Y=2.5)
for display in manager['Displays']:
    if display.get('Name')=='Depth costmap input (cyan)':
        display['Enabled']=depth_enabled;display['Value']=depth_enabled
    if sys.argv[3]=='true' and display.get('Name')=='SLAM map (occupancy)':
        display['Name']='True map (world coordinates)'
    if sys.argv[4]=='rpp' and display.get('Name')=='Local trajectory (pink)':
        display['Name']='RPP collision arc (pink)';display['Topic']['Value']='/lookahead_arc'
if sys.argv[4]=='rpp':
    manager['Displays'].append({'Class':'rviz_default_plugins/PointStamped',
        'Name':'RPP lookahead point','Enabled':True,'Value':True,
        'Color':'255; 190; 0','Radius':0.05,'History Length':1,
        'Topic':{'Value':'/lookahead_point','Depth':1,'Reliability Policy':'Reliable','Durability Policy':'Volatile'}})
if os.environ.get('EDGEAI_COOPERATIVE_RESCUE')=='true':
    robots=[[],[]]
    colors={'/plan':'255; 170; 0','/local_plan':'190; 100; 255','/lookahead_arc':'190; 100; 255',
            '/scan':'80; 170; 255',
            '/navigation_authority/dispatched_goal':'80; 200; 255',
            '/lookahead_point':'190; 100; 255'}
    shared=[]
    for display in list(manager['Displays']):
        field='Description Topic' if display.get('Class')=='rviz_default_plugins/RobotModel' else 'Topic'
        topic=display.get(field,{}).get('Value','')
        if topic in ('','/map','/targets/markers','/explore/frontiers'):
            shared.append(display)
            continue
        robots[0].append(display)
        # Fleet simulation only renders RGB-D on the primary robot.
        if topic.startswith('/camera/'):
            continue
        second=deepcopy(display)
        second[field]['Value']='/robot2'+topic
        if display.get('Update Topic',{}).get('Value','').startswith('/'):
            second['Update Topic']['Value']='/robot2'+display['Update Topic']['Value']
        if topic in colors:
            second['Color']=colors[topic]
        # Both costmaps remain available; avoid two layers covering each other on startup.
        if topic.endswith('/costmap'):
            for item in (display,second):
                item['Enabled']=item['Value']=False
        if topic=='/diagnostics/navigation':
            for item in (display,second):
                item['Name']='Aligned footprints and pose correction'
        if topic=='/plan':
            second['Name']='Global path (orange)'
        if topic=='/scan':
            second['Name']='Laser scan (blue)'
        if topic=='/navigation_authority/dispatched_goal':
            second['Name']='Dispatched goal (light blue)'
        if topic in ('/local_plan','/lookahead_arc'):
            second['Name']=second['Name'].replace('pink','purple')
        if topic=='/truth/markers':
            second['Name']='Simulation truth (robot 2 aligned)'
        robots[1].append(second)
    manager['Displays']=shared+[
        {'Class':'rviz_common/Group','Name':f'Robot {i}','Enabled':True,'Displays':displays}
        for i,displays in enumerate(robots,1)]
if os.environ.get('EDGEAI_JOINT_MAPPING')=='true':
    def joint_displays(displays,robot=''):
        for display in displays:
            name=display.get('Name','')
            current=robot
            if name=='Robot 1':current='robot1'
            elif name=='Robot 2':current='robot2'
            field='Description Topic' if display.get('Class')=='rviz_default_plugins/RobotModel' else 'Topic'
            topic=display.get(field,{})
            if current=='robot1' and topic.get('Value') in ('/robot_description','/truth/markers'):
                topic['Value']='/robot1'+topic['Value']
            if topic.get('Value')=='/explore/frontiers':topic['Value']='/robot1/explore/frontiers'
            if display.get('Displays'):joint_displays(display['Displays'],current)
    joint_displays(manager['Displays'])
def scope_primary(value):
    if isinstance(value,dict):return {k:scope_primary(v) for k,v in value.items()}
    if isinstance(value,list):return [scope_primary(v) for v in value]
    if isinstance(value,str):
        if value in ('base_link','base_footprint','odom','camera_optical_frame'):return 'robot1/'+value
        if value.startswith('/') and not value.startswith(('/robot1/','/robot2/','/map','/targets/','/mission/','/joint_mapping/','/exploration/','/target_search/','/single_rescue/','/cooperative_rescue/')):return '/robot1'+value
    return value
config=scope_primary(config)
output.write_text(yaml.safe_dump(config,sort_keys=False))
