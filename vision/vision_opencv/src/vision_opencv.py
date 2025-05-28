#!/usr/bin/env python3


import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from vision_ros_msgs.msg import BoundingBox, BoundingBoxes

BOX_COLORS = {
    "blue": {"color_lower": np.array([100, 43, 46]), "color_upper": np.array([124, 255, 255])},
    "red": {"color_lower": np.array([0, 127, 67]), "color_upper": np.array([10, 255, 255])},
    "yellow": {"color_lower": np.array([26, 43, 46]), "color_upper": np.array([34, 255, 255])},
    "green": {"color_lower": np.array([35, 43, 46]), "color_upper": np.array([77, 255, 255])},
    "purple": {"color_lower": np.array([125, 43, 46]), "color_upper": np.array([155, 255, 255])},
    # "orange": {"color_lower": np.array([11, 43, 46]), "color_upper": np.array([25, 255, 255])},
    "pink": {"color_lower": np.array([155, 0, 16]), "color_upper": np.array([179, 255, 255])}
}


class image_converter:

  def __init__(self):
    self.image_sub = rospy.Subscriber(
        "/camera/rgb/image_raw", Image, self.callback)
    self.position_pub = rospy.Publisher(
        "/vision_opencv",  BoundingBoxes, queue_size=1)
    self.bridge = CvBridge()

  def cv_show(self, name, img):
      cv2.imshow(name, img)
      cv2.waitKey(3)

  def callback(self, data):
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.boundingBoxes = BoundingBoxes()
    frame_ = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame_, cv2.COLOR_BGR2HSV)

    for color in BOX_COLORS.keys():

      mask = cv2.inRange(
          hsv, BOX_COLORS[color]["color_lower"], BOX_COLORS[color]["color_upper"])
      mask = cv2.erode(mask, None, iterations=2)
      mask = cv2.dilate(mask, None, iterations=2)
      mask = cv2.GaussianBlur(mask, (3, 3), 0)

      cnts = cv2.findContours(
          mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
      for cnt in cnts:
          x, y, w, h = cv2.boundingRect(cnt)
          rect = cv2.minAreaRect(cnt)
          rotation = rect[2]
          cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
          boundingBox = BoundingBox()
          boundingBox.Class = color
          boundingBox.angle = rotation
          boundingBox.x = x + int(w/2.0)
          boundingBox.y = y + int(h/2.0)

          self.boundingBoxes.bounding_boxes.append(boundingBox)
          self.position_pub.publish(self.boundingBoxes)

    self.cv_show("img", frame)


if __name__ == '__main__':
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  print('start')
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    print('exception')
