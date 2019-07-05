#!/usr/bin/env python

import rospy
import yaml
import os
import sys
import subprocess
import numpy as np
import cv2
from copy import deepcopy
from cv_bridge import CvBridge, CvBridgeError

# ROS Messages
from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest
from darknet_ros_py.msg import RecognizedObject, RecognizedObjectArray, RecognizedObjectArrayStamped
from std_msgs.msg import String

from darknet import performDetect

class DarknetRosPy(object):

  class SuperSubscribeListener(rospy.SubscribeListener):
    def __init__(self, parent):
      self._parent = parent
      super(rospy.SubscribeListener, self).__init__()

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
      if self._parent.subscriber is None:
        self._parent.setup()
        rospy.loginfo('Created subscribers because there is a new connection to output topics')

    def peer_unsubscribe(self, topic_name, num_peers):
      if self._parent.subscriber is not None:
        if self._parent.image_pub.get_num_connections() == 0 and self._parent.result_publisher.get_num_connections() == 0:
            self._parent.shutdown()
            rospy.loginfo('Destroyed subscribers because there are no connections to output topics')


  @staticmethod
  def check_if_exists(path):
    if not os.path.exists(path):
      rospy.logerr("File not found [%s]", path)
      sys.exit(-1)
    else:
      rospy.logdebug("File found [%s]", path)

  def __init__(self):
    darknet_bin_dir_param = rospy.get_param("/darknet_bin_dir")
    self.camera_rostopic = rospy.get_param("~input_topic")
    self.data_file = rospy.get_param("~data_file", "cfg/coco.data")
    self.cfg_file = rospy.get_param("~cfg_file", "cfg/yolov3.cfg")
    self.weights_file = rospy.get_param("~weights_file", "yolov3.weights")
    self.threshold = rospy.get_param("~threshold", 0.25)
    self.debug = rospy.get_param("~debug", False)
    self.show_only_best_detection = rospy.get_param("~show_only_best_detection", False)
    self.max_detection_rate = rospy.get_param("~max_detection_rate", 100)

    # Check if using compressed images, from the topic itself
    namespace_splits = self.camera_rostopic.split('/')
    if 'compressed' in namespace_splits:
      rospy.loginfo('Using compressed images')
      self.using_compressed = True
    else:
      self.using_compressed = False

    # Create CvBridge if using raw transport
    if not self.using_compressed:
        self.bridge = CvBridge()

    # Check if files exist
    darknet_bin_dir = os.path.expanduser(darknet_bin_dir_param)
    self.check_if_exists(darknet_bin_dir)
    darknet_bin_path = os.path.join(darknet_bin_dir, "darknet")
    self.check_if_exists(darknet_bin_path)
    cfg_file_full_path = os.path.join(darknet_bin_dir, self.cfg_file)
    self.check_if_exists(cfg_file_full_path)
    weights_file_full_path = os.path.join(darknet_bin_dir, self.weights_file)
    self.check_if_exists(weights_file_full_path)

    # Subscriber and Publisher rostopics
    subscriber_status_cbs = DarknetRosPy.SuperSubscribeListener(self)

    self.image_pub = rospy.Publisher("~detection_image/compressed", CompressedImage, queue_size=1,  subscriber_listener=subscriber_status_cbs)
    self.result_publisher = rospy.Publisher("~detections", RecognizedObjectArrayStamped, queue_size=10, subscriber_listener=subscriber_status_cbs)

    # Class variables
    self.subscriber = None
    self.current_image_msg = None

    # Change directory to darknet
    os.chdir(darknet_bin_dir)

    # check if camera topic is in the rostopic list
    rostopic_list = subprocess.check_output(['rostopic', 'list'])
    if self.camera_rostopic not in rostopic_list:
      rospy.logwarn("Input topic {} not found yet".format(self.camera_rostopic))

    # init darknet
    detections = performDetect(self.current_image_msg, self.threshold, self.cfg_file, self.weights_file, self.data_file, True)

    rospy.loginfo("Darknet is ready. It will subscribe to images and process them as soon as there is a subscriber to the detections or detection image topic")

  def draw_info(self, image, class_name, confidence, x_center, y_center, width, height, color):

    left = (x_center - (width / 2))
    right = (left + width)
    top = (y_center - (height / 2))
    bottom = (top + height)

    image_h, image_w, _channels = image.shape

    if left < 0:
        left = 0
    if right > (image_w - 1):
        right = image_w - 1
    if top < 0:
        top = 0
    if bottom > (image_h - 1):
        bottom = image_h - 1

    # to make sure there are no bugs
    if left > (image_w - 1):
        left = image_w - 1
    if top > (image_h - 1):
        top = image_h - 1

    # Draw bb
    cv2.rectangle(image, (left, top), (right, bottom), color, 2)
    # Write text
    font  = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    font_color  = color
    thickness = 2
    text_width, text_height = cv2.getTextSize(class_name, font, font_scale, thickness)[0]
    offset = 5
    top -= offset
    if top - text_height < 0:
        top += text_height + 2*offset
    bottom_left_corner_of_text = (top,left)
    cv2.putText(image, class_name + ' {:.2f}'.format(confidence),
      (left, top),
      font,
      font_scale,
      font_color,
      thickness)

    return image

  def setup(self):
    # start subscriber
    img_type = CompressedImage if self.using_compressed else Image
    self.subscriber = rospy.Subscriber(self.camera_rostopic, img_type,
                                       self.detection_request_callback, queue_size=1)

  def shutdown(self):
    self.subscriber.unregister()
    self.subscriber = None

  def loop(self):
    r = rospy.Rate(self.max_detection_rate) # Hz to check if there is a new detection being requested

    while not rospy.is_shutdown():

        if self.current_image_msg is not None:
            # copy to prevent being changed from the callback
            img_msg = deepcopy(self.current_image_msg)
            self.current_image_msg = None

            if not self.using_compressed:
                try:
                    image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr(e)
                    return

            else: # compressed, cant use cv_bridge in python
                # get image from topic
                np_arr = np.fromstring(img_msg.data, np.uint8)
                # get OpenCV version
                (major_version, _minor_version, _) = cv2.__version__.split(".")
                if major_version >= 3:
                    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                else:
                    image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

            publish_image = self.image_pub.get_num_connections() > 0

            initOnly = False
            detections = performDetect(image, self.threshold, self.cfg_file, self.weights_file, self.data_file, initOnly)

            recog_obj_array = RecognizedObjectArray()

            image_h, image_w, _channels = image.shape

            if self.show_only_best_detection:
                best_object = None
                best_confidence = -1
                best_obj_details = -1

            for obj_id, detection in enumerate(detections):
                data = detection.split()
                class_name, confidence = data[:2]
                x_center, y_center, width, height = [int(float(val)) for val in data[2:]]

                confidence = float(confidence)

                # draw one detection on the image to be published
                if publish_image:
                    color = (255, 0, 0) # blue
                    image = self.draw_info(image, class_name, confidence, x_center, y_center, width, height, color)

                if self.debug:
                    rospy.loginfo('''\n'''
                    '''class_name:      %s\n'''
                    '''confidence: %s\n'''
                    '''x_center:      %s\n'''
                    '''y_center:      %s\n'''
                    '''width:      %s\n'''
                    '''height:      %s''', class_name, confidence, x_center, y_center, width, height)

                recog_obj = RecognizedObject()
                recog_obj.class_name = class_name
                recog_obj.confidence = confidence

                x_offset = min(max(0, x_center-width/2), image_w)
                y_offset = min(max(0, y_center-height/2), image_h)

                recog_obj.bounding_box = RegionOfInterest(
                    x_offset = x_offset,
                    y_offset = y_offset,
                    width = min(width, image_w - x_offset),
                    height = min(height, image_h - y_offset))

                if self.debug:
                    rospy.loginfo("recog_obj.bounding_box")
                    rospy.loginfo(recog_obj.bounding_box)

                if self.show_only_best_detection:
                    if confidence > best_confidence:
                        best_object = recog_obj
                        best_confidence = confidence
                        best_obj_details = [class_name, confidence, x_center, y_center, width, height]
                else:
                    recog_obj_array.objects.append(recog_obj)

            if self.show_only_best_detection and best_object is not None:
                recog_obj_array.objects.append(best_object)
                class_name, confidence, x_center, y_center, width, height = tuple(best_obj_details)
                color = (0, 255, 0) # green
                image = self.draw_info(image, class_name, confidence, x_center, y_center, width, height, color)

            recog_obj_array_stamped = RecognizedObjectArrayStamped()
            recog_obj_array_stamped.header = img_msg.header
            recog_obj_array_stamped.objects = recog_obj_array
            self.result_publisher.publish(recog_obj_array_stamped)

            # actually publish the image
            if publish_image:
                # Create CompressedImage and publish
                msg = CompressedImage()
                msg.header = img_msg.header
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
                # Publish new image
                self.image_pub.publish(msg)

        r.sleep()

  def detection_request_callback(self, img_msg):
    # copy image to global variable
    self.current_image_msg = img_msg

if __name__ == '__main__':
  try:
    rospy.init_node('dark_ros_py_node')
    rospy.logdebug("Node initialised")

    darknetros = DarknetRosPy()
    darknetros.loop()

    # loop to wait for output from darknet's command line
  except rospy.ROSInterruptException:
    pass
