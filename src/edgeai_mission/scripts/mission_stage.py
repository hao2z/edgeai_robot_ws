"""Preparation adapters: stop exploration and export the shared map."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def quiesce(c=None):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    joint=bool((c or {}).get('joint_mapping',False)) or os.environ.get('EDGEAI_JOINT_MAPPING')=='true'
    node=rclpy.create_node('mission_quiesce');states={};received={}
    prefixes=('/robot1/', '/robot2/') if joint else ('/robot1/',)
    def receive(prefix,message):
        states[prefix]=json.loads(message.data);received[prefix]=time.monotonic()
    subscriptions=[node.create_subscription(String,prefix+'navigation_authority/state',
        lambda m,p=prefix:receive(p,m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for prefix in prefixes]
    clients={prefix:node.create_client(Trigger,prefix+'navigation_authority/stop') for prefix in prefixes}
    deadline=time.monotonic()+20.
    def wait(predicate):
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if predicate():return
        raise RuntimeError('Stop/pause confirmation timeout')
    try:
        wait(lambda:all(client.service_is_ready() and 'command_sequence' in states.get(prefix,{})
             and (not joint or time.monotonic()-received.get(prefix,0)<2.)
             for prefix,client in clients.items()))
        sequences={prefix:states[prefix]['command_sequence']+1 for prefix in prefixes}
        requested=time.monotonic()
        # Request both stops before awaiting either acknowledgement.
        futures={prefix:client.call_async(Trigger.Request()) for prefix,client in clients.items()}
        for prefix,future in futures.items():
            wait(future.done)
            if not future.result().success:raise RuntimeError(prefix+'navigation_authority rejected stop')
        def stopped():
            for prefix in prefixes:
                state=states[prefix];seq=sequences[prefix]
                if joint and state.get('command_sequence',-1)>seq:
                    raise RuntimeError(prefix+'navigation_authority changed during stop')
                if not (state.get('command_sequence',-1)>=seq and state.get('mode')=='STOPPED'
                        and state.get('phase')=='IDLE' and state.get('explorer_paused')
                        and (not joint or received[prefix]>=requested and time.monotonic()-received[prefix]<2.)):
                    return False
            return True
        wait(stopped)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def finalize_joint_map():
    """Freeze the shared map and target anchors before any rescue motion."""
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('mission_finalize_map')
    client=node.create_client(Trigger,'/targets/finalize_map')
    try:
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('Target/map finalization service unavailable')
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=12.)
        if not future.done():raise RuntimeError('Target/map finalization timed out')
        response=future.result()
        if not response.success:raise RuntimeError(response.message)
        binding=json.loads(response.message)
        if not binding.get('finalized') or binding.get('mode')!='fixed_map':
            raise RuntimeError('Target/map finalization not confirmed')
        print('MAPPING_FINALIZED: '+json.dumps(binding),flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def run(step, c):
    root=Path(c['project_root']);directory=Path(c['directory'])
    sys.path.insert(0,str(root/'src/edgeai_navigation/scripts'))
    if step=='quiesce':quiesce(c);return
    if step in ('supplement_mapping','resume_exploration'):
        from mapping_supplement import run as mapping_run
        mapping_run(step,c)
        return
    if step=='save_map':
        if c.get('joint_mapping'):finalize_joint_map()
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
