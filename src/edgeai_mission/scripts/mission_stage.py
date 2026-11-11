"""Preparation adapters: stop exploration and export the shared map."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def quiesce():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('mission_quiesce');state={}
    node.create_subscription(String,'/navigation_authority/state',lambda m:state.update(json.loads(m.data)),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client=node.create_client(Trigger,'/navigation_authority/stop')
    deadline=time.monotonic()+20.
    def wait(predicate):
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise RuntimeError('Stop/pause confirmation timeout')
    try:
        wait(lambda:client.service_is_ready() and 'command_sequence' in state)
        seq=state['command_sequence']+1
        future=client.call_async(Trigger.Request());wait(future.done)
        if not future.result().success:raise RuntimeError('Authority rejected stop')
        wait(lambda:state.get('command_sequence',-1)>=seq and state.get('mode')=='STOPPED'
             and state.get('phase')=='IDLE' and state.get('explorer_paused'))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def run(step, c):
    root=Path(c['project_root']);directory=Path(c['directory'])
    sys.path.insert(0,str(root/'src/edgeai_navigation/scripts'))
    if step=='quiesce':quiesce();return
    if step in ('supplement_mapping','resume_exploration'):
        from mapping_supplement import run as mapping_run
        mapping_run(step,c)
        return
    if step=='save_map':
        dest=Path(c['map_basename']);dest.parent.mkdir(parents=True,exist_ok=True)
        subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(dest),
                        '--ros-args','-p','use_sim_time:=true'],check=True,timeout=25.)
        from export_handover import export
        export(directory,Path(str(dest)+'.yaml'))
        return
    raise ValueError('Unregistered preparation: '+step)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--step',choices=('quiesce','save_map','supplement_mapping','resume_exploration'),required=True)
    p.add_argument('--config',type=Path,required=True)
    a=p.parse_args();c=json.loads(a.config.read_text())
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    result=dict(success=False,step=a.step)
    try:run(a.step,c);result['success']=True
    except (Exception,KeyboardInterrupt) as error:result['reason']=str(error) or 'Interrupted'
    path=Path(c['directory'])/('stage-'+a.step+'-result.json');tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(result,ensure_ascii=False));tmp.replace(path)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
