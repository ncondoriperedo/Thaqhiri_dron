#!/usr/bin/env python3
"""Run an odometry-driven SIL trial or an explicitly labeled diagnostic trial."""

import argparse
from collections import defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rtabmap_msgs.msg import MapData
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32
from tf2_msgs.msg import TFMessage
from webots_ros2_msgs.msg import FloatStamped
from thaiquiri_webots.sil_metrics import (
    horizontal_speed_summary, odometry_tracking, summarize_samples,
    surface_error, trajectory_error)
from thaiquiri_webots.sil_route import RouteController

ROOT = Path(__file__).resolve().parents[1]


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def pose_row(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return [stamp(msg), p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def cloud_xyz(msg, stride=1):
    points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.asarray(points).reshape(-1, 3)[::stride].copy()


class Trial:
    def __init__(self, node, output, criteria, control_mode='odometry',
                 desired_speed_limit=.2):
        self.node, self.output, self.criteria = node, output, criteria
        self.sim = 0.; self.last_clock = 0.; self.clock_backwards = 0
        self.start_wall = time.monotonic(); self.subs = []
        self.first_clock_wall = None
        self.events = []; self.times = defaultdict(list); self.frames = defaultdict(set)
        self.truth = []; self.odom = []; self.flight = []; self.imu = None
        self.raw_imu_samples = []; self.filtered_imu_samples = []
        self.altitude = None; self.motors = []; self.contacts = 0
        self.map_cloud = None; self.map_frame = None; self.map_odom = None
        self.map_transforms = []
        self.map_node_ids = set(); self.image_quality = []; self.lidar_quality = []
        self.last_command_time = -1.; self.stage = 0
        self.control_mode = control_mode
        self.route = RouteController(criteria['route_forward_m'], desired_speed_limit,
                                     enabled=control_mode != 'hover')
        self.route_status = None; self.commands = []
        self.odometry_status = []; self.odometry_feedback = []; self.icp_valid = None
        self.image = None; self.bridge = CvBridge(); self.target_altitude = .5
        self.pub = node.create_publisher(Twist, '/cmd_pos', 10)
        topics = [('/clock', Clock, self.clock), ('/sil/ground_truth', Odometry, self.ground_truth),
                  ('/sil/contact_count', Int32, self.contact), ('/drone/odom', Odometry, self.odometry),
                  ('/imu/data', Imu, self.inertial), ('/drone/altimeter_cube', FloatStamped, self.altimeter),
                  ('/imu/data_raw', Imu, self.raw_inertial),
                  ('/drone/velocity', JointState, self.motor), ('/drone/left/image_rect_color', Image, self.camera),
                  ('/drone/lidar_livox/point_cloud', PointCloud2, self.lidar),
                  ('/drone/mapData', MapData, self.map_data), ('/drone/cloud_map', PointCloud2, self.map_points),
                  ('/tf', TFMessage, self.transforms)]
        for topic, cls, callback in topics:
            def receive(msg, topic=topic, callback=callback):
                self.times[topic].append(self.sim)
                if hasattr(msg, 'header'):
                    self.frames[topic].add(msg.header.frame_id)
                callback(msg)
            self.subs.append(node.create_subscription(cls, topic, receive, qos_profile_sensor_data))

    def clock(self, msg):
        if self.first_clock_wall is None:
            self.first_clock_wall = time.monotonic()
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock_backwards += int(now < self.last_clock)
        self.last_clock = self.sim = now

    def ground_truth(self, msg):
        row = pose_row(msg); self.truth.append(row)
        if self.control_mode == 'ground_truth':
            # Explicit diagnostic only: never mix truth into normal odometry feedback.
            v = msg.twist.twist.linear
            self.route.observe(row, [v.x, v.y, v.z])
        rpy = Rotation.from_quat(row[4:]).as_euler('xyz')
        error = [math.nan, math.nan]
        if self.imu is not None:
            q = self.imu.orientation
            sensed = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
            error = (np.arctan2(np.sin(sensed[:2]-rpy[:2]), np.cos(sensed[:2]-rpy[:2])) * 180/np.pi).tolist()
        self.flight.append([row[0], self.altitude if self.altitude is not None else math.nan,
                            self.target_altitude, *np.degrees(rpy[:2]), *error, self.contacts,
                            int(any(abs(v) >= 929.9 for v in self.motors))])

    def contact(self, msg): self.contacts = msg.data
    def inertial(self, msg):
        self.imu = msg
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        self.filtered_imu_samples.append([stamp(msg), q.x, q.y, q.z, q.w,
                                          w.x, w.y, w.z, a.x, a.y, a.z])

    def raw_inertial(self, msg):
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.raw_imu_samples.append([stamp(msg), w.x, w.y, w.z, a.x, a.y, a.z])
    def altimeter(self, msg): self.altitude = msg.data
    def motor(self, msg): self.motors = list(msg.velocity)

    def odometry(self, msg):
        row = pose_row(msg)
        valid = not (not np.isfinite(row).all()
                or not np.isfinite(msg.pose.covariance).all()
                or msg.pose.covariance[0] >= 9999
                or np.linalg.norm(row[4:]) < 1e-6)
        self.odometry_status.append([row[0], int(valid), msg.pose.covariance[0]])
        if valid != self.icp_valid:
            self.icp_valid = valid
            self.events.append({'time': self.sim, 'icp_valid': valid})
        if not valid:
            return
        self.odom.append(row)
        if self.control_mode == 'odometry':
            v = msg.twist.twist.linear
            self.odometry_feedback.append([row[0], v.x, v.y, v.z])
            self.route.observe(row, [v.x, v.y, v.z])

    def camera(self, msg):
        if len(self.times['/drone/left/image_rect_color']) % 30:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.image_quality.append([self.sim, float(gray.mean()), float(gray.std())])
        if self.image is None or int(self.sim) % 20 == 0:
            self.image = image.copy()
            cv2.imwrite(str(self.output / 'camera.png'), image)

    def lidar(self, msg):
        if len(self.times['/drone/lidar_livox/point_cloud']) % 10:
            return
        xyz = cloud_xyz(msg, 10)
        ranges = np.linalg.norm(xyz, axis=1)
        valid = ranges[(ranges > .1) & (ranges < 40)]
        self.lidar_quality.append([self.sim, len(valid), float(np.percentile(valid, 5)) if len(valid) else 0])

    def map_data(self, msg):
        self.map_node_ids.update(msg.graph.poses_id)

    def map_points(self, msg):
        self.map_cloud = cloud_xyz(msg, 2); self.map_frame = msg.header.frame_id

    def transforms(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                p, q = tf.transform.translation, tf.transform.rotation
                self.map_transforms.append([stamp(tf), p.x, p.y, p.z, q.x, q.y, q.z, q.w])
                # Anchor the map once. Using the final map->odom correction
                # would move the reference frame when SLAM closes a loop.
                if self.map_odom is None:
                    self.map_odom = (Rotation.from_quat([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z]))

    def command(self):
        if self.sim - self.last_command_time < .05:
            return
        self.last_command_time = self.sim
        msg = Twist()
        if self.sim >= 5 and self.stage == 0:
            msg.linear.z = .55; self.target_altitude = 1.05; self.stage = 1
            self.events.append({'time': self.sim, 'command': 'altitude 1.05 m'})
        elif self.sim >= 15 and self.stage == 1:
            msg.linear.z = .10; self.target_altitude = 1.15; self.stage = 2
            self.events.append({'time': self.sim, 'command': 'altitude 1.15 m'})
        msg.linear.x, msg.linear.y = self.route.command(self.sim)
        if self.route.status != self.route_status:
            self.route_status = self.route.status
            self.events.append({'time': self.sim, 'control_source': self.control_mode,
                                'route_status': self.route_status})
        age = self.sim-self.route.sample[0] if self.route.sample is not None else math.nan
        self.commands.append([self.sim, msg.linear.x, msg.linear.y, age])
        self.pub.publish(msg)

    def finish(self, metadata):
        self.pub.publish(Twist())
        for name, rows, header in [('truth', self.truth, 't,x,y,z,qx,qy,qz,qw'),
                                   ('odometry', self.odom, 't,x,y,z,qx,qy,qz,qw'),
                                   ('flight', self.flight, 't,altitude,target,roll_deg,pitch_deg,roll_error_deg,pitch_error_deg,contacts,saturated'),
                                   ('commands', self.commands, 't,pitch_disturbance,roll_disturbance,feedback_age_s'),
                                    ('odometry_status', self.odometry_status, 't,valid,pose_covariance_x'),
                                    ('odometry_feedback', self.odometry_feedback, 't,body_vx,body_vy,body_vz'),
                                   ('imu_raw', self.raw_imu_samples, 't,wx,wy,wz,ax,ay,az'),
                                   ('imu_filtered', self.filtered_imu_samples, 't,qx,qy,qz,qw,wx,wy,wz,ax,ay,az'),
                                   ('images', self.image_quality, 't,mean_gray,std_gray'),
                                   ('map_transforms', self.map_transforms, 't,x,y,z,qx,qy,qz,qw'),
                                   ('lidar', self.lidar_quality, 't,valid_sampled_points,p05_range')]:
            with (self.output / (name+'.csv')).open('w') as stream:
                stream.write(header+'\n'); csv.writer(stream).writerows(rows)
        trajectory, alignment = trajectory_error(self.truth, self.odom)
        tracking = odometry_tracking([row[0] for row in self.odom], 20., self.sim) if self.sim > 20 else None
        flight = np.asarray(self.flight)
        steady = flight[flight[:, 0] >= 30] if len(flight) else np.empty((0,9))
        airborne = flight[flight[:, 0] >= 10] if len(flight) else np.empty((0,9))
        metrics = {'trajectory': trajectory,
                   'horizontal_speed_m_s': horizontal_speed_summary(self.truth),
                   'settled_horizontal_speed_m_s': horizontal_speed_summary(
                       self.truth, start=self.criteria['duration_sim_s']-10),
                   'odometry_tracking': tracking,
                   'altitude_error_m': summarize_samples(steady[:,1]-steady[:,2]),
                   'attitude_error_deg': summarize_samples(steady[:,5:7].ravel()),
                   'tilt_deg': summarize_samples(airborne[:,3:5].ravel()),
                   'airborne_contact_samples': int(np.count_nonzero(airborne[:,7])) if len(airborne) else None,
                   'motor_saturation_fraction': float(np.mean(airborne[:,8])) if len(airborne) else 1.,
                   'clock_backwards': self.clock_backwards, 'sim_duration_s': self.sim,
                   'realtime_factor': self.sim/(time.monotonic()-self.start_wall),
                   'startup_wall_s': self.first_clock_wall-self.start_wall if self.first_clock_wall else None,
                   'running_realtime_factor': self.sim/(time.monotonic()-self.first_clock_wall) if self.first_clock_wall else None,
                   'map_updates': len(self.times['/drone/mapData']), 'map_node_count': len(self.map_node_ids),
                   'mapped_points': len(self.map_cloud) if self.map_cloud is not None else 0,
                   'frames': {t: sorted(frames) for t,frames in self.frames.items()}, 'rates_hz': {}}
        for topic, times in self.times.items():
            times = [t for t in times if t >= 20]
            metrics['rates_hz'][topic] = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
        quality = np.asarray(self.image_quality)
        metrics['camera_gray_mean'] = float(np.median(quality[:,1])) if len(quality) else 0.
        metrics['camera_gray_std'] = float(np.median(quality[:,2])) if len(quality) else 0.
        truth = np.asarray(self.truth)
        if len(truth):
            start = truth[np.argmin(abs(truth[:,0]-20)),1:4]
            # Source spawn faces +Y, and the trial traverses the +Y entry corridor.
            metrics['route_progress_m'] = float(np.max(truth[:,2])-start[1])
            metrics['route_return_error_m'] = float(np.linalg.norm(truth[-1,1:3]-start[:2]))
        surface = {'available': False}
        if self.map_cloud is not None and len(self.map_cloud) and alignment is not None and self.map_frame == 'map' and self.map_odom is not None:
            r,t = alignment; mr,mt = self.map_odom
            world = r.apply(mr.inv().apply(self.map_cloud-mt))+t
            reference = np.load(ROOT/'src/thaiquiri_webots/webots_world/protos/cerro_rico/surface_reference.npz')['points']
            surface = surface_error(world, reference)
            surface['alignment'] = 'first matched odometry pose and first map->odom transform; fixed for entire run'
            np.savez_compressed(self.output/'map_world.npz',points=world)
            np.savez_compressed(self.output/'map_raw.npz',points=self.map_cloud)
        metrics['map_surface'] = surface
        c = self.criteria; checks = {}
        def upper(name, value, limit): checks[name] = {'pass': value is not None and value <= limit, 'value': value, 'max': limit}
        def lower(name, value, limit): checks[name] = {'pass': value is not None and value >= limit, 'value': value, 'min': limit}
        lower('completed_duration',self.sim,c['duration_sim_s']-.1)
        upper('clock_monotonic',self.clock_backwards if self.times['/clock'] else None,0)
        upper('altitude',metrics['altitude_error_m']['rmse'],c['altitude_rmse_max_m'])
        upper('attitude',metrics['attitude_error_deg']['rmse'],c['attitude_error_rmse_max_deg'])
        upper('tilt',metrics['tilt_deg']['max_abs'],c['tilt_max_deg'])
        upper('contacts',metrics['airborne_contact_samples'],c['airborne_contact_samples_max'])
        upper('motor_saturation',metrics['motor_saturation_fraction'],c['motor_saturation_fraction_max'])
        upper('odometry',trajectory.get('position_rmse_m'),c['odometry_rmse_max_m'])
        upper('odometry_end_drift',trajectory.get('end_drift_m'),c['odometry_end_drift_max_m'])
        lower('route_progress',metrics.get('route_progress_m'),c['route_progress_min_m'])
        upper('route_return',metrics.get('route_return_error_m'),c['route_return_error_max_m'])
        lower('map_updates',metrics['map_updates'],c['map_updates_min'])
        lower('map_points',metrics['mapped_points'],c['mapped_points_min'])
        upper('map_surface',surface.get('p95_m'),c['map_surface_p95_max_m'])
        lower('odometry_rate',tracking['valid_rate_hz'] if tracking else 0,c['odometry_rate_min_hz'])
        upper('odometry_max_gap',tracking['max_gap_s'] if tracking else None,c['odometry_max_gap_s'])
        upper('horizontal_speed',metrics['horizontal_speed_m_s']['max_abs'],c['horizontal_speed_max_m_s'])
        upper('settled_horizontal_speed',metrics['settled_horizontal_speed_m_s']['rmse'],c['settled_horizontal_speed_rmse_max_m_s'])
        for kind,topic in [('imu','/imu/data'),('camera','/drone/left/image_rect_color'),('lidar','/drone/lidar_livox/point_cloud')]:
            lower(kind+'_rate',metrics['rates_hz'].get(topic,0),c[kind+'_rate_min_hz'])
        lower('camera_exposure',metrics['camera_gray_mean'],c['camera_gray_mean_min'])
        lower('camera_contrast',metrics['camera_gray_std'],c['camera_gray_std_min'])
        result = {'metadata':metadata,'criteria':c,'metrics':metrics,'checks':checks,'events':self.events,
                  'passed':all(check['pass'] for check in checks.values()) and metadata.get('execution_error') is None}
        result['navigation_validated'] = result['passed'] and self.control_mode == 'odometry'
        (self.output/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        lines=['# Cerro Rico SIL result','',f"Scenario: {metadata['scenario']}",
               f'Control feedback: {self.control_mode}',
               f"Navigation validated: {result['navigation_validated']}",
               f"Overall checks: {'PASS' if result['passed'] else 'FAIL'}",'',
               '| Check | Result | Value | Limit |','| --- | --- | --- | --- |']
        for name,check in checks.items():
            lines.append(f"| {name} | {'PASS' if check['pass'] else 'FAIL'} | {check['value']} | {check.get('min',check.get('max'))} |")
        feedback_note = ('DIAGNOSTIC: ground truth drives the route controller. This run cannot validate autonomous navigation.'
                         if self.control_mode == 'ground_truth' else
                         'Ground truth is scoring-only. The route controller is a test fixture, not a production autonomous navigation stack.')
        lines+=['',feedback_note,
                'This run validates only the entrance corridor and stated conditions, not the full mine, real-world flight, ArduPilot SITL, or gas sensing.']
        (self.output/'report.md').write_text('\n'.join(lines)+'\n')
        return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=['nominal','low_light'],default='nominal')
    parser.add_argument('--headlight', choices=['on', 'off'], default='on',
                        help='Onboard lamp; low_light switches off only the mine practical lights')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--wall-timeout',type=float,default=900)
    parser.add_argument('--domain-id',type=int,default=73)
    parser.add_argument('--control-mode', choices=['odometry', 'ground_truth', 'hover'],
                        default='odometry', help='ground_truth and hover are diagnostic only')
    parser.add_argument('--desired-speed-limit', type=float, default=.2,
                        help='Per-axis requested speed limit (m/s); does not cap actual speed')
    parser.add_argument('--icp-max-translation', type=float, default=.2,
                        help='Maximum accepted ICP correction (m); keep fixed except in an isolated comparison')
    parser.add_argument('--imu-filter-gain', type=float, default=.01,
                        help='Madgwick gain in [0,1]; default 0.01, old tuning 0.1')
    args=parser.parse_args()
    if not math.isfinite(args.desired_speed_limit) or args.desired_speed_limit <= 0:
        parser.error('--desired-speed-limit must be finite and positive')
    if not math.isfinite(args.icp_max_translation) or args.icp_max_translation <= 0:
        parser.error('--icp-max-translation must be finite and positive')
    if not math.isfinite(args.imu_filter_gain) or not 0 <= args.imu_filter_gain <= 1:
        parser.error('--imu-filter-gain must be finite and in [0,1]')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    # Webots otherwise silently moves to another port while controllers attach
    # to the old instance, invalidating both the trial and its measurements.
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('127.0.0.1', 1234))
        except OSError as error:
            raise RuntimeError('Webots port 1234 is occupied; stop that simulation first') from error
    output=(args.output or ROOT/'experiment_data/sil'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')).resolve()
    output.mkdir(parents=True,exist_ok=False)
    criteria=json.loads((ROOT/'config/sil_criteria.json').read_text())
    manifest=json.loads((ROOT/'src/thaiquiri_webots/webots_world/protos/cerro_rico/manifest.json').read_text())
    # Source ZIPs have no Git metadata. Do not accidentally report a parent
    # repository's revision when an archive is unpacked inside another checkout.
    git_revision = None; git_dirty = None
    try:
        git_root = subprocess.check_output(['git','rev-parse','--show-toplevel'],
                                           cwd=ROOT,text=True,stderr=subprocess.DEVNULL).strip()
        if Path(git_root).resolve() == ROOT.resolve():
            git_revision = subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
            git_dirty = bool(subprocess.check_output(['git','status','--porcelain'],cwd=ROOT,text=True))
    except (OSError, subprocess.CalledProcessError):
        pass
    metadata={'scenario':args.scenario,'started_utc':datetime.now(timezone.utc).isoformat(),
              'headlight_on':args.headlight == 'on',
              'control_mode':args.control_mode,'diagnostic_only':args.control_mode != 'odometry',
              'desired_speed_limit_m_s':args.desired_speed_limit,
              'icp_max_translation_m':args.icp_max_translation,
              'imu_filter_gain':args.imu_filter_gain,
              'source_sha256':manifest['source_sha256'],'git_revision':git_revision,
              'git_dirty':git_dirty,
              'criteria_sha256':hashlib.sha256((ROOT/'config/sil_criteria.json').read_bytes()).hexdigest(),
              'world':'cerro_rico.wbt','random_seed':42,'ros_domain_id':args.domain_id,
              'webots':subprocess.check_output([str(Path(os.environ['WEBOTS_HOME'])/'webots'),'--version'],text=True).strip(),
              'ros':os.environ.get('ROS_DISTRO'),
              'implementation_sha256':{str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest()
                  for pattern in ('scripts/*.py','src/thaiquiri_webots/thaiquiri_webots/*.py',
                                  'src/thaiquiri_webots/launch/*.py','src/thaiquiri_webots/resource/*.urdf',
                                  'src/thaiquiri_webots/webots_world/protos/*.proto',
                                  'src/thaiquiri_webots/webots_world/worlds/cerro_rico.wbt')
                  for path in ROOT.glob(pattern)}}
    rclpy.init();node=rclpy.create_node('sil_trial')
    trial=Trial(node,output,criteria,args.control_mode,args.desired_speed_limit)
    processes=[];logs=[];failure=None
    try:
        launch=['ros2','launch','thaiquiri_webots','drone_launch.py','world:=cerro_rico.wbt',
                'icp_max_translation:='+str(args.icp_max_translation),
                'imu_filter_gain:='+str(args.imu_filter_gain),
                'gui:=false','rtabmap_viz:=false','database_path:='+str(output/'rtabmap.db')]
        log=(output/'launch.log').open('w');logs.append(log)
        processes.append(subprocess.Popen(launch,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        env=os.environ.copy();env['WEBOTS_HOME']='/opt/ros/jazzy';env['WEBOTS_CONTROLLER_URL']='ipc://1234/SILObserver'
        log=(output/'observer.log').open('w');logs.append(log)
        processes.append(subprocess.Popen(['ros2','run','thaiquiri_webots','sil_observer','--ros-args',
                           '-p','lights_on:='+str(args.scenario=='nominal').lower(),
                           '-p','headlight_on:='+str(args.headlight=='on').lower()],
                           env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        deadline=time.monotonic()+args.wall_timeout
        while rclpy.ok() and trial.sim<criteria['duration_sim_s'] and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01);trial.command()
            if any(p.poll() is not None for p in processes):
                failure='A simulation process exited before the trial completed';break
        if time.monotonic()>=deadline:failure='Wall-time timeout'
    except KeyboardInterrupt:
        failure='Interrupted'
    except Exception as error:
        failure=f'{type(error).__name__}: {error}'
        import traceback
        (output/'execution_error.txt').write_text(traceback.format_exc())
    finally:
        metadata['execution_error']=failure
        try:
            result=trial.finish(metadata)
        finally:
            # Stop launch first, then the complete process groups. Wrapper exit
            # alone does not guarantee that a loading webots-bin has terminated.
            for p in processes:
                if p.poll() is None:p.send_signal(signal.SIGINT)
            for p in processes:
                try:p.wait(timeout=20)
                except subprocess.TimeoutExpired:pass
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            for log in logs:log.close()
            node.destroy_node();rclpy.try_shutdown()
    print(output)
    print(json.dumps(result['checks'],indent=2))
    raise SystemExit(0 if result['passed'] and failure is None else 1)


if __name__=='__main__':main()
