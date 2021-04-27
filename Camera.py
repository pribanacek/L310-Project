#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import Image

class Camera(object):
  def __init__(self):
    rospy.Subscriber('/front_cam/camera/image', Image, self.callback)
    self._fov = 90
    self._res_x = 320
    self._res_y = 240
    self._channels = 3
    self._image = None

  def callback(self, msg):
    self._image = np.fromstring(msg.data, dtype='uint8')

  @property
  def ready(self):
    return self._image is not None

  @property
  def image(self):
    return self._image
  
  def get_value(self, i, j, channel):
    return self._image[self._res_x * i + self._channels * j + channel]
  
  def get_pixel(self, i, j):
    # return RGB colour tuple of pixel at position i j
    return (get_value(i, j, n) for n in range(self._channels))
  
  def get_colour_row(self):
    mid = self._res_y // 2
    image = np.reshape(self._image, (self._res_y, self._res_x, self._channels))
    row = image[mid]
    return row
