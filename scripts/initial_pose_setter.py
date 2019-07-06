#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped,Quaternion
from sensor_msgs.msg import NavSatFix
import geodesy.utm
import tf

class InitialPoseSetter:
    def __init__(self):
        self.roll = rospy.get_param('~initial_pose/roll')
        self.pitch = rospy.get_param('~initial_pose/pitch')
        self.yaw = rospy.get_param('~initial_pose/yaw')
        self.fix_topic = rospy.get_param('~fix_topic')
        self.map_frame = rospy.get_param('~map_frame')
        self.published = False
        self.fix_sub = rospy.Subscriber(self.fix_topic,NavSatFix,self.fixCallback,queue_size=1)
        self.initial_pose_pub = rospy.Publisher("/initialpose",PoseWithCovarianceStamped,latch=True,queue_size=1)
    def publishInitialPose(self,fix_data):
        if self.published == False:
            initial_pose = PoseWithCovarianceStamped()
            geopoint = geodesy.utm.fromLatLong(fix_data.latitude,fix_data.longitude)
            initial_pose.pose.pose.position.x = geopoint.northing
            initial_pose.pose.pose.position.y = geopoint.easting * -1
            initial_pose.pose.pose.position.z = fix_data.altitude
            initial_pose.pose.pose.orientation = self.eulerToQuaternion(self.roll,self.pitch,self.yaw)
            initial_pose.header.frame_id = self.map_frame
            initial_pose.header.stamp = fix_data.header.stamp
            self.initial_pose_pub.publish(initial_pose)
            self.published = True
        else:
            pass
    def eulerToQuaternion(self,roll,pitch,yaw):
        q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    def fixCallback(self,data):
        self.publishInitialPose(data)

if __name__ == "__main__":
    rospy.init_node('initial_pose_setter', anonymous=True)
    setter = InitialPoseSetter()
    rospy.spin()