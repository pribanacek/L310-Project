#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rospy

from geometry_msgs.msg import Twist

from SimpleLaser import SimpleLaser
from Camera import Camera
from GroundtruthPose import GroundtruthPose
from PointCloud import PointCloud

from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
import actionlib
import hector_uav_msgs.msg

X, Y, Z, ROLL, PITCH, YAW = range(6)

MIN_HEIGHT = 0.61
LASER_OFFSET = -0.097
LASER_MAX = 30.0
LASER_MIN = 0.20

MAX_HEIGHT = 2.5

VIEWS = 8
R = 2.
VIEW_POSITIONS = [(R * np.cos(t), R * np.sin(t)) for t in np.linspace(-np.pi, np.pi, VIEWS+1)[:-1]]

class ScanStage(object):
  Move = 1
  Rotate = 2
  Scan = 3
  Done = 4

def cap(speed, MAX = 0.4):
  if abs(speed) > MAX:
    speed = MAX * (-1 if speed < 0 else 1)
  return speed

def record_points(pose, laser, camera):
  # return set of points to add to point cloud
  N = len(laser.measurements)

  angles = laser.angles + pose[YAW]
  angles = angles[N//4:3*N//4]
  distances = laser.measurements[N//4:3*N//4]
  colours = camera.get_colour_row()[N//4:3*N//4]

  xs = pose[X] + distances * np.cos(angles)
  ys = pose[Y] + distances * np.sin(angles)
  zs = np.full((N,), pose[Z] - LASER_OFFSET, dtype='float32')[N//4:3*N//4]

  coords = np.dstack((xs, ys, zs))[0]
  mask = np.logical_and(distances < LASER_MAX, distances > LASER_MIN)
  points = np.hstack((coords, colours))[mask]

  colours = points[:,3:]
  coords = points[:,:3]

  return coords, colours


def get_relative_position(absolute_pose, absolute_position):
  t = -absolute_pose[YAW]
  transform = np.array([
    [np.cos(t), -np.sin(t)],
    [np.sin(t),  np.cos(t)],
  ])
  relative_position = np.matmul(transform, absolute_position - absolute_pose[X:Y+1])
  return relative_position

def move(pose, view_number):
  target_position = np.array(VIEW_POSITIONS[view_number])
  target_x, target_y = get_relative_position(pose, target_position)
  target_z = MIN_HEIGHT - pose[Z]

  diffs = np.array([target_x, target_y, target_z])
  eps = 0.01
  if (np.abs(diffs) < eps).all():
    vx, vy, vz, az = 0., 0., 0., 0.
    stage = ScanStage.Rotate
  else:
    k = 1.25
    vx = k * target_x
    vy = k * target_y
    vz = k * target_z
    az = 0.
    stage = ScanStage.Move
  return vx, vy, vz, az, view_number, stage

def rotate(pose, view_number):
  target_x, target_y = np.array(VIEW_POSITIONS[view_number])
  target_yaw = np.arctan2(-target_y, -target_x)

  diff = np.abs(pose[YAW] - target_yaw)
  eps = 0.01
  if diff < eps:
    vx, vy, vz, az = 0., 0., 0., 0.
    stage = ScanStage.Scan
  else:
    k = 2.5
    vx, vy, vz = 0., 0., 0.
    az = k * diff
    stage = ScanStage.Rotate
  return vx, vy, vz, az, view_number, stage

def scan(pose, view_number):
  target_position = np.array(VIEW_POSITIONS[view_number])
  target_x, target_y = get_relative_position(pose, target_position)
  target_z = MAX_HEIGHT

  if pose[Z] < MAX_HEIGHT:
    # go up at a constant speed
    scan_speed = 0.125
    vz = scan_speed

    # correct our x and y positions in case of any drift
    k = 1.0
    vx = k * target_x
    vy = k * target_y
    az = 0.
    stage = ScanStage.Scan
  else:
    vx, vy, vz, az = 0., 0., 0., 0.
    view_number += 1
    if view_number < len(VIEW_POSITIONS):
      stage = ScanStage.Move
    else:
      stage = ScanStage.Done
  return vx, vy, vz, az, view_number, stage


def run():
  rospy.init_node('mapping')
  rospy.wait_for_service("/enable_motors")
  enabler1 = rospy.ServiceProxy("/enable_motors", EnableMotors)
  resp1 = enabler1(True)


  # Update control every 100 ms.
  rate_limiter = rospy.Rate(100)
  publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

  laser = SimpleLaser()
  groundtruth = GroundtruthPose()
  camera = Camera()
  point_cloud = PointCloud()

  view_angle = 0

  with open('/tmp/gazebo_exercise.txt', 'w'):
    pass

  count = 0
  view_number = 0
  stage = ScanStage.Move
  while not rospy.is_shutdown():
    # Make sure all measurements are ready.
    if not laser.ready or not groundtruth.ready or not camera.ready:
      rate_limiter.sleep()
      continue

    if stage == ScanStage.Move:
      vx, vy, vz, az, new_view, next_stage = move(groundtruth.pose, view_number)
    elif stage == ScanStage.Rotate:
      vx, vy, vz, az, new_view, next_stage = rotate(groundtruth.pose, view_number)
    elif stage == ScanStage.Scan:
      vx, vy, vz, az, new_view, next_stage = scan(groundtruth.pose, view_number)
      if groundtruth.pose[Z] > MIN_HEIGHT and groundtruth.pose[Z] < MAX_HEIGHT:
        coords, colours = record_points(groundtruth.pose, laser, camera)
        point_cloud.add_points(coords, colours)
      if count % 50 == 0:
        point_cloud.publish()
    elif stage == ScanStage.Done:
      vx, vy, vz, az = 0., 0., 0., 0.
      next_stage = ScanStage.Done
      point_cloud.publish()

    stage = next_stage
    if new_view != view_number:
      print('Moving from view ' + str(view_number) + ' to ' + str(new_view))
    view_number = new_view

    vel_msg = Twist()
    vel_msg.linear.x = cap(vx)
    vel_msg.linear.y = cap(vy)
    vel_msg.linear.z = cap(vz)
    vel_msg.angular.x = 0.
    vel_msg.angular.y = 0.
    vel_msg.angular.z = cap(az, MAX=0.75)
  
    publisher.publish(vel_msg)
    count += 1
    rate_limiter.sleep()


if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
