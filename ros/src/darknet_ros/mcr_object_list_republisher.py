#!/usr/bin/env python

import rospy
from mcr_perception_msgs.msg import ObjectList, Object
from mbot_perception_msgs.msg import RecognizedObject3DList 

class McrObjRepublisher(object):
    '''
    Pipeline: yolo 2D - project 2D into 3D using pointcloud then republish in
    a format that mcr object recognition mean circle is compatible with
    '''
    def __init__(self):
        self.mcr_list = ObjectList()
        self.pub_object_list = rospy.Publisher('/mcr_perception/object_detector/object_list', ObjectList,  queue_size=1)
        rospy.Subscriber('/mbot_perception/object_2D_to_3D_pose/objects_out', RecognizedObject3DList, self.yoloObjCallback)


    def yoloObjCallback(self, msg):
        rospy.loginfo('Got objects from yolo')
        mcr_list = ObjectList()
        if len(msg.objects) == 0:
            rospy.logwarn('Receive empty object yolo object list')
            return

        mcr_object = Object()

        for obj in msg.objects:
            mcr_object.name = obj.class_name
            mcr_object.category = obj.class_name
            mcr_object.pose.header = msg.header
            mcr_object.pose.pose = obj.pose
            mcr_list.objects.append(mcr_object)
        
        self.pub_object_list.publish(mcr_list)
        rospy.loginfo('Published mcr objects')


    def start_obj_list_republisher(self):
        rospy.spin()


def main():
    rospy.init_node('mcr_object_republisher', anonymous=False)
    mcr_object_republisher = McrObjRepublisher()
    mcr_object_republisher.start_obj_list_republisher()
