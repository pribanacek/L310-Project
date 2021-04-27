#!/usr/bin/env python

import numpy as np
import rospy

from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

SEND_RGB = True

FIELDS = [
  PointField('x', 0, PointField.FLOAT32, 1),
  PointField('y', 4, PointField.FLOAT32, 1),
  PointField('z', 8, PointField.FLOAT32, 1),
]

if SEND_RGB:
  FIELDS += [
    PointField('r', 12, PointField.FLOAT32, 1),
    PointField('g', 16, PointField.FLOAT32, 1),
    PointField('b', 20, PointField.FLOAT32, 1)
  ]

class PointCloud(object):
  def __init__(self):
    self._publisher = rospy.Publisher('/object_point_cloud', PointCloud2, queue_size=1)
    self._points = np.full((0, 3), float('inf'), dtype='float32')
    self._colours = np.full((0, 3), 0, dtype='uint8')
  
  def add_points(self, points, colours):
    self._points = np.concatenate((self._points, points))
    self._colours = np.concatenate((self._colours, colours))

  def publish(self):
    stamp = None
    frame_id = None
    seq = None
    points = np.array(self._points)
    colours = np.array(self._colours)

    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = 'point_cloud_frame'

    buf = []
    N = len(points)
    msg.height = 1
    msg.width = N
    if SEND_RGB:
      data = np.array(np.hstack([points, colours]), dtype=np.float32)
      msg.point_step = 24
    else:
      data = points
      msg.point_step = 12
    msg.fields = FIELDS
    msg.is_bigendian = False
    msg.row_step = msg.point_step * N
    msg.is_dense = False
    msg.data = data.tostring()
    self._publisher.publish(msg)
