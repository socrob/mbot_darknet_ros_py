#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped

from tf import TransformListener

class PoseArrayToPose(object):
    def __init__(self):

        self.tf_listener = TransformListener() 

        self.pub_pose = rospy.Publisher('/object_pose', PoseStamped,  queue_size=1)
        rospy.Subscriber('/mbot_perception/object_2D_to_3D_pose/poses_out', PoseArray, self.callback)


    def callback(self, msg):
        #rospy.loginfo('Got poses')
        if len(msg.poses) == 0:
            rospy.logwarn('Received empty pose array')
            return

        ps = PoseStamped()
        ps.header = msg.header
        for pose in msg.poses:
            ps.pose = pose
            try:
                self.tf_listener.waitForTransform('base_link', ps.header.frame_id, ps.header.stamp, rospy.Duration(1))
                ps_transf = self.tf_listener.transformPose('head_link', ps)
            except Exception as e:
                rospy.logerr(e)
                return
            # rotate around Y
            ps_transf.pose.orientation.x = ps_transf.pose.orientation.x
            ps_transf.pose.orientation.y = - ps_transf.pose.orientation.y
            ps_transf.pose.orientation.z = - ps_transf.pose.orientation.z
            self.pub_pose.publish(ps_transf)

            # add a bit in Z(which is -Y for end effector) for the handle
            to_add = 0.25
            ps_transf.pose.position.y = ps_transf.pose.position.y - to_add
        
        #rospy.loginfo('Published poses individually')


    def start(self):
        rospy.spin()


def main():
    rospy.init_node('pose_array_to_pose', anonymous=False)
    pose_republisher = PoseArrayToPose()
    pose_republisher.start()
