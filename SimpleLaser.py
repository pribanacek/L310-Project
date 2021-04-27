#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import LaserScan

class SimpleLaser(object):
  def __init__(self):
    rospy.Subscriber('/scan', LaserScan, self.callback)
    self._ray_count = 320.
    self._min_angle = np.radians(45.)
    self._max_angle = np.radians(-45.)
    self._angles = np.linspace(self._min_angle, self._max_angle, num=self._ray_count, endpoint=True)
    self._width = self._max_angle - self._min_angle
    self._measurements = np.array([float('inf')] * len(self._angles))

  def callback(self, msg):
    self._measurements = np.array(msg.ranges)

  @property
  def ready(self):
    return not np.isnan(self._measurements[0])

  @property
  def measurements(self):
    return self._measurements

  @property
  def angles(self):
    return self._angles
