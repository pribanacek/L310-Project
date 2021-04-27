#!/usr/bin/env python

from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

import numpy as np
import rospy

X, Y, Z, ROLL, PITCH, YAW = range(6)


class GroundtruthPose(object):
  def __init__(self, name='quadrotor'):
    rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
    self._pose = np.full((6,), np.nan, dtype=np.float32)
    self._name = name

  def callback(self, msg):
    idx = [i for i, n in enumerate(msg.name) if n == self._name]
    if not idx:
      raise ValueError('Specified name "{}" does not exist.'.format(self._name))
    idx = idx[0]
    self._pose[X] = msg.pose[idx].position.x
    self._pose[Y] = msg.pose[idx].position.y
    self._pose[Z] = msg.pose[idx].position.z
    roll, pitch, yaw = euler_from_quaternion([
        msg.pose[idx].orientation.x,
        msg.pose[idx].orientation.y,
        msg.pose[idx].orientation.z,
        msg.pose[idx].orientation.w])
    self._pose[ROLL] = roll
    self._pose[PITCH] = pitch
    self._pose[YAW] = yaw

  @property
  def ready(self):
    return not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose
