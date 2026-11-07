#!/usr/bin/env python3
"""Bound point-cloud work without altering coordinates, stamps or field layout."""
import numpy as np
import json
import time


def project_depth(message, info, stride):
    """Project sampled, undistorted axial depth in its original optical frame."""
    from sensor_msgs.msg import PointCloud2, PointField
    if info is None:
        raise ValueError('Waiting for camera_info')
    if (message.width,message.height,message.header.frame_id)!=(info.width,info.height,info.header.frame_id):
        raise ValueError('Depth image and camera_info dimensions/frame differ')
    if (any(abs(v)>1e-9 for v in info.d) or info.binning_x>1 or info.binning_y>1 or
            info.roi.x_offset or info.roi.y_offset or info.roi.width not in (0,info.width) or
            info.roi.height not in (0,info.height)):
        raise ValueError('Depth projection requires undistorted, unbinned camera_info')
    if stride<1 or message.width<1 or message.height<1:
        raise ValueError('Invalid depth dimensions/stride')
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    if not all(np.isfinite(v) for v in (fx,fy,cx,cy)) or fx<=0 or fy<=0:
        raise ValueError('Invalid camera intrinsics')
    types={'32FC1':('f4',1.),'16UC1':('u2',.001)}
    if message.encoding not in types:
        raise ValueError('Unsupported depth encoding: '+message.encoding)
    kind,scale=types[message.encoding]
    dtype=np.dtype(('>' if message.is_bigendian else '<')+kind)
    if message.step<message.width*dtype.itemsize or len(message.data)<message.step*message.height:
        raise ValueError('Truncated depth image or invalid step')
    depth=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                     strides=(message.step,dtype.itemsize))[::stride,::stride].astype(np.float32)*scale
    depth[~np.isfinite(depth)|(depth<=0)]=np.nan
    u=np.arange(0,message.width,stride,dtype=np.float32)
    v=np.arange(0,message.height,stride,dtype=np.float32)
    xyz=np.empty((*depth.shape,3),dtype='<f4')
    xyz[:,:,0]=depth*(u[None,:]-cx)/fx
    xyz[:,:,1]=depth*(v[:,None]-cy)/fy
    xyz[:,:,2]=depth
    return PointCloud2(header=message.header,height=depth.shape[0],width=depth.shape[1],
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,name in enumerate(('x','y','z'))],
        is_bigendian=False,point_step=12,row_step=depth.shape[1]*12,data=xyz.tobytes(),is_dense=False)


def decimate_cloud(message, stride):
    """Keep every Nth organized pixel (or Nth point for an unorganized cloud)."""
    if stride < 1 or message.point_step < 1 or message.height < 1 or message.width < 1:
        raise ValueError('Invalid point-cloud dimensions or stride')
    if message.row_step < message.width * message.point_step:
        raise ValueError('Invalid point-cloud row_step')
    if len(message.data) < message.row_step * message.height:
        raise ValueError('Truncated point-cloud data')
    # Explicit row stride handles organized clouds with padded rows. Copy all
    # bytes of each selected point: endian, XYZ offsets and other fields survive.
    points = np.ndarray((message.height, message.width, message.point_step),
                        dtype=np.uint8, buffer=message.data,
                        strides=(message.row_step, message.point_step, 1))
    selected = points[::stride, ::stride].copy()
    from sensor_msgs.msg import PointCloud2
    return PointCloud2(header=message.header, height=selected.shape[0],
                       width=selected.shape[1], fields=message.fields,
                       is_bigendian=message.is_bigendian, point_step=message.point_step,
                       row_step=selected.shape[1]*message.point_step,
                       data=selected.tobytes(), is_dense=message.is_dense)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, Image, CameraInfo
    from std_msgs.msg import String

    class Filter(Node):
        def __init__(self):
            super().__init__('depth_obstacle_filter')
            defaults = dict(input_topic='camera/points', output_topic='camera/obstacle_points',
                            processing_hz=8.0, pixel_stride=4, max_age_s=0.4,
                            input_mode='pointcloud',depth_topic='camera/depth_image',camera_info_topic='camera/camera_info')
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            params = {name: self.get_parameter(name).value for name in defaults}
            if params['processing_hz'] <= 0 or params['pixel_stride'] < 1 or params['max_age_s'] <= 0:
                raise ValueError('Depth filter rate, stride and age must be positive')
            self.period = 1.0 / params['processing_hz']
            self.stride, self.max_age = params['pixel_stride'], params['max_age_s']
            self.last = None
            self.counts = dict(input_messages=0,published_messages=0,expired_messages=0,
                               future_messages=0,throttled_messages=0,malformed_messages=0)
            self.last_input_wall = self.last_output_wall = None
            self.last_age = None
            self.last_status = None
            self.input_mode=params['input_mode']
            if self.input_mode not in ('pointcloud','depth_image'):
                raise ValueError('input_mode must be pointcloud or depth_image')
            self.info=None
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            # A reliable publisher also supports best-effort RViz/diagnostics
            # subscribers and Nav2 variants requesting reliable sensor input.
            output_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
            self.publisher = self.create_publisher(PointCloud2, params['output_topic'], output_qos)
            self.status_publisher = self.create_publisher(String,'camera/depth_obstacle_status',1)
            if self.input_mode=='depth_image':
                self.info_subscription=self.create_subscription(CameraInfo,params['camera_info_topic'],
                    lambda message:setattr(self,'info',message),qos)
                self.subscription=self.create_subscription(Image,params['depth_topic'],self.receive,qos)
            else:
                self.subscription = self.create_subscription(PointCloud2, params['input_topic'], self.receive, qos)
            self.create_timer(1.,self.report,clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info('DEPTH_OBSTACLES enabled: %.1f Hz, pixel stride %d' %
                                   (params['processing_hz'], self.stride))

        def receive(self, message):
            self.counts['input_messages'] += 1
            self.last_input_wall = time.monotonic()
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_age = now-stamp
            if now-stamp > self.max_age:
                self.counts['expired_messages'] += 1
                return
            if stamp-now > self.max_age:
                self.counts['future_messages'] += 1
                return
            if self.last is not None and stamp >= self.last and stamp-self.last < self.period-1e-6:
                self.counts['throttled_messages'] += 1
                return
            try:
                result = (project_depth(message,self.info,self.stride) if self.input_mode=='depth_image'
                          else decimate_cloud(message, self.stride))
            except ValueError as error:
                self.counts['malformed_messages'] += 1
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)
                return
            self.last = stamp
            self.publisher.publish(result)
            self.counts['published_messages'] += 1
            self.last_output_wall = time.monotonic()

        def report(self):
            wall = time.monotonic()
            input_age = wall-self.last_input_wall if self.last_input_wall is not None else None
            output_age = wall-self.last_output_wall if self.last_output_wall is not None else None
            state = ('WAITING_INPUT' if input_age is None else
                     'INPUT_STALE' if input_age>2. else
                     'NO_OUTPUT' if output_age is None else
                     'OUTPUT_STALE' if output_age>2. else 'OUTPUT_OK')
            data = dict(self.counts,state=state,input_mode=self.input_mode,camera_info_ready=self.info is not None,
                        input_wall_age_s=input_age,output_wall_age_s=output_age,
                        last_input_age_sim_s=self.last_age,max_age_s=self.max_age)
            self.status_publisher.publish(String(data=json.dumps(data)))
            if state!=self.last_status:
                self.get_logger().info('DEPTH_PIPELINE '+json.dumps(data))
                self.last_status=state

    rclpy.init()
    node = Filter()
    executor=rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
