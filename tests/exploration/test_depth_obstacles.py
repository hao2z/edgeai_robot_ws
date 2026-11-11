"""Depth switch isolation and byte-exact organized-cloud sampling (no simulator)."""
from pathlib import Path
import struct
import os
import subprocess
import sys
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/edgeai_navigation/scripts'))
from navigation_config import load_navigation
from depth_obstacle_filter import decimate_cloud, project_depth
from fleet_config import namespaced_config


class DepthObstacles(unittest.TestCase):
    def test_depth_projection_units_endian_and_invalid_points(self):
        from sensor_msgs.msg import Image, CameraInfo
        import numpy as np
        info=CameraInfo(width=3,height=2,k=[2.,0.,1.,0.,2.,0.,0.,0.,1.])
        info.header.frame_id='optical'
        for encoding,dtype,scale in [('32FC1','<f4',1.),('16UC1','>u2',1000.)]:
            image=Image(width=3,height=2,encoding=encoding,is_bigendian=dtype.startswith('>'),
                        step=3*np.dtype(dtype).itemsize,
                        data=(np.array([[1.,2.,0.],[1.,2.,3.]])*scale).astype(dtype).tobytes())
            from copy import deepcopy
            image.header=deepcopy(info.header)
            cloud=project_depth(image,info,1)
            points=np.frombuffer(cloud.data,dtype='<f4').reshape(2,3,3)
            np.testing.assert_allclose(points[0,0],[-.5,0.,1.])
            np.testing.assert_allclose(points[1,1],[0.,1.,2.])
            self.assertTrue(np.isnan(points[0,2]).all())
            sampled=project_depth(image,info,2)
            self.assertEqual((sampled.height,sampled.width),(1,2))
            with self.assertRaises(ValueError):project_depth(image,None,1)
            info.header.frame_id='wrong'
            with self.assertRaises(ValueError):project_depth(image,info,1)
            info.header.frame_id='optical'

    def test_switch_preserves_laser_global_and_controllers(self):
        for controller in ('rpp', 'dwb', 'mppi'):
            for profile in ('rectangle', 'baseline', 'safe'):
                off = load_navigation(ROOT/'src/edgeai_navigation/config', controller, profile, False)
                on = load_navigation(ROOT/'src/edgeai_navigation/config', controller, profile, True)
                local = on['local_costmap']['local_costmap']['ros__parameters']
                self.assertLess(local['plugins'].index('obstacle_layer'), local['plugins'].index('depth_layer'))
                self.assertEqual(local['plugins'].index('depth_layer')+1, local['plugins'].index('inflation_layer'))
                self.assertEqual(local['depth_layer']['combination_method'], 1)
                del local['depth_layer']
                local['plugins'].remove('depth_layer')
                del on['depth_obstacle_filter']
                self.assertEqual(on, off)

    def test_no_stale_unused_voxel_config(self):
        common = yaml.safe_load((ROOT/'src/edgeai_navigation/config/nav2_exploration_rectangle.yaml').read_text())
        self.assertNotIn('voxel_layer', common['local_costmap']['local_costmap']['ros__parameters'])

    def test_robot2_depth_and_laser_topics_are_isolated(self):
        config = namespaced_config(load_navigation(ROOT/'src/edgeai_navigation/config', depth_obstacles=True), 'robot2')
        local = config['local_costmap']['local_costmap']['ros__parameters']
        self.assertEqual(local['depth_layer']['depth']['topic'], '/robot2/camera/obstacle_points')
        self.assertEqual(local['depth_layer']['depth_clear']['topic'], '/robot2/camera/obstacle_points')
        self.assertEqual(local['obstacle_layer']['scan']['topic'], '/robot2/scan')
        self.assertEqual(config['depth_obstacle_filter']['ros__parameters']['input_topic'], 'camera/points')

    def test_rviz_switch_and_fleet_generation(self):
        runner = (ROOT/'scripts/simulation/run-open-source-exploration.sh').read_text()
        code = runner.split("<<'TEST_RVIZ'\n",1)[1].split('\nTEST_RVIZ\n',1)[0]
        for controller in ('rpp','dwb','mppi'):
            for enabled in (False,True):
                for fleet in (False,True):
                    with tempfile.TemporaryDirectory() as directory:
                        nav = Path(directory)/'nav.yaml'
                        output = Path(directory)/'view.rviz'
                        nav.write_text(yaml.safe_dump(load_navigation(
                            ROOT/'src/edgeai_navigation/config',controller,depth_obstacles=enabled)))
                        subprocess.run([sys.executable,'-',str(ROOT/'src/edgeai_exploration/rviz/exploration.rviz'),
                                        str(output),'false',controller,str(nav)],input=code,text=True,check=True,
                                       env=dict(os.environ,EDGEAI_COOPERATIVE_RESCUE=str(fleet).lower()))
                        displays = yaml.safe_load(output.read_text())['Visualization Manager']['Displays']
                        def flatten(items):
                            for item in items:
                                yield item
                                yield from flatten(item.get('Displays', []))
                        clouds = [d for d in flatten(displays) if d['Class']=='rviz_default_plugins/PointCloud2']
                        self.assertEqual(len(clouds),1)  # Only robot1 has an RGB-D sensor.
                        for cloud in clouds:
                            self.assertEqual(cloud['Enabled'],enabled)
                            self.assertEqual(cloud['Topic']['Reliability Policy'],'Best Effort')
                        self.assertEqual(clouds[0]['Topic']['Value'],'/camera/obstacle_points')

    def test_byte_layout_padding_endian_and_nan(self):
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header
        from builtin_interfaces.msg import Time
        for endian, big in [('<', False), ('>', True)]:
            # Three rows, five points, four padding bytes per row; XYZI fields.
            rows = b''.join(b''.join(struct.pack(endian+'ffff', float(row), float(col),
                            float('nan') if col == 0 else .1, 42.)
                            for col in range(5))+b'PAD!' for row in range(3))
            fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                      for name, offset in [('x',0),('y',4),('z',8),('intensity',12)]]
            cloud = PointCloud2(header=Header(stamp=Time(sec=12, nanosec=34), frame_id='camera_optical_frame'),
                 height=3, width=5, fields=fields, point_step=16, row_step=84,
                 is_bigendian=big, is_dense=False, data=rows)
            out = decimate_cloud(cloud, 2)
            self.assertEqual((out.height,out.width,out.row_step), (2,3,48))
            self.assertEqual(out.header, cloud.header)
            self.assertEqual(out.fields, cloud.fields)
            self.assertEqual(out.is_bigendian, big)
            expected = b''.join(rows[r*84+c*16:r*84+c*16+16] for r in (0,2) for c in (0,2,4))
            self.assertEqual(bytes(out.data), expected)
            cloud.data = rows[:-5]
            with self.assertRaises(ValueError):
                decimate_cloud(cloud, 2)


if __name__ == '__main__':
    unittest.main()
