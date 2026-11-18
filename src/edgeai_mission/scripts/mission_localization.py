"""Mission-owned localization: hand over once, remain alive through rescue tasks."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from tf2_ros import TransformException
import yaml


def write_state(directory, **data):
    path=directory/'localization-state.json';tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data));tmp.replace(path)


def prepare_localization(node, mission):
    """Preserve the final-map priors across bounded startup retries."""
    from edgeai_tasks.cooperative_policy import compose,relative
    from edgeai_tasks.localization_runtime import pose_value
    from edgeai_tasks.mission_fleet import localization_priors
    d=node.directory
    prior=d/'localization-handover.json'
    # On retry both old publishers may already be stopped. The saved map is
    # loaded by the new map_server; requiring a live /map here would deadlock.
    node.wait(lambda:(prior.exists() or node.map is not None) and node.now()>0 and
              all(ns in node.odom for ns in node.localization_namespaces),20.)
    if prior.exists():
        saved=json.loads(prior.read_text())
        if saved['namespaces']!=list(node.localization_namespaces):
            raise RuntimeError('Localization retry robot set changed')
        # Localization recovery may have rotated since the saved prior.
        poses=[compose(p,relative(saved['odom'][ns],pose_value(node.odom[ns].pose.pose)))
               for ns,p in zip(node.localization_namespaces,saved['poses'])]
    else:
        node.finalize_targets()
        poses=[]
        def capture():
            try:
                fleet=mission['fleet']
                current={robot:node.tf_pose(fleet['robots'][robot]['base_frame']) for robot in fleet['mapping_robots']}
                odom={robot:pose_value(node.odom[fleet['robots'][robot]['namespace']].pose.pose)
                      for robot in fleet['localization_robots']}
                ref=d/'mission-reference.json'
                selected=localization_priors(fleet,current,odom,json.loads(ref.read_text()) if ref.exists() else None)
                poses[:]=[selected[r] for r in fleet['localization_robots']]
                return True
            except TransformException:return False
        node.wait(capture,15.)
        # Save while SLAM still publishes TF. A failed AMCL start must not need
        # a now-finalized SLAM service to recreate its priors.
        saved=dict(namespaces=list(node.localization_namespaces),poses=poses,
                   odom={ns:pose_value(node.odom[ns].pose.pose) for ns in node.localization_namespaces})
        tmp=prior.with_suffix('.tmp');tmp.write_text(json.dumps(saved));tmp.replace(prior)
    node.event('HANDOVER',initial_poses=poses,owner='mission')
    # The map file and its target registry share the finalized graph coordinates.
    if not (d/'cooperative-map.yaml').exists():
        result=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli','-f',str(d/'cooperative-map'),
            '--ros-args','-p','use_sim_time:=true'],capture_output=True,text=True,timeout=30.)
        (d/'cooperative-map-save.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError('Final localization map save failed')
    registry=json.loads((d/'targets.json').read_text())
    if not registry.get('map_binding',{}).get('finalized'):
        raise RuntimeError('Localization requires finalized map targets')
    (d/'cooperative-targets.json').write_text(json.dumps(registry,indent=2))
    node.stop_slam()
    node.start_localization(poses[0],poses[1] if len(poses)>1 else None)


def run(mission):
    root=Path(mission['project_root']);d=Path(mission['directory'])
    sys.path[:0]=[str(root/'src/edgeai_mission'),str(root/'src/edgeai_navigation')]
    from edgeai_tasks.localization_runtime import LocalizationRuntime
    from edgeai_tasks.mission_fleet import configured_fleet
    settings_path = d/'cooperative-rescue-config.yaml'
    if not settings_path.is_file():
        settings_path = root/'src/edgeai_navigation/config/cooperative_rescue.yaml'
    settings=yaml.safe_load(settings_path.read_text())
    mission=dict(mission,fleet=configured_fleet(mission,settings['initial_relative_pose']))
    fleet=mission['fleet']
    namespaces=[fleet['robots'][r]['namespace'] for r in fleet['localization_robots']]
    settings.update(joint_mapping=mission.get('joint_mapping',False),
                    localization_namespaces=namespaces,
                    reuse_robot2_navigation='robot2' in fleet['running_navigation_robots'])
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=LocalizationRuntime(d,settings,node_name='mission_localization')
    node.mission_localization=True
    node.defer_localization_initialization=True
    node.deadline=time.monotonic()+mission.get('localization',{}).get('startup_timeout_s',240.)
    def stop(*_):node.stop_requested=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        write_state(d,ready=False,phase='STARTING')
        prepare_localization(node,mission)
        node.activate_localization()
        node.wait_navigation_active()
        node.localize()
        node.sequences.clear()  # Task workers acquire their own control next.
        node.finish_initialization()
        node.deadline=float('inf')
        write_state(d,ready=True,phase='READY',robots=list(node.localization_namespaces),sim_time=node.now())
        while rclpy.ok() and not getattr(node,'stop_requested',False):
            if node.process.poll() is not None:raise RuntimeError('Shared localization launch exited')
            rclpy.spin_once(node,timeout_sec=.1)
        return 0
    except (Exception,KeyboardInterrupt) as error:
        write_state(d,ready=False,phase='FAILED',reason=str(error) or 'Interrupted')
        node.get_logger().error('MISSION_LOCALIZATION_FAILED: '+str(error))
        return 1
    finally:
        if getattr(node,'stop_requested',False):write_state(d,ready=False,phase='STOPPING')
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',type=Path,required=True)
    args=p.parse_args();raise SystemExit(run(json.loads(args.config.read_text())))
