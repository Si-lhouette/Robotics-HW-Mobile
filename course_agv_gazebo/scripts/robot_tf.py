#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion


class GazeboLinkPose:
    link_name = ''
    link_pose = Pose()
    # trans = Pose(0,0,0)
    # rot = Quaternion(0,0,0,1)

    def __init__(self, robot_name, link_name):
        self.robot_name = robot_name
        self.link_name = link_name
        # self.link_name_rectified = link_name.replace("::", "_")

        if not self.link_name:
            raise ValueError("'link_name' is an empty string")

        self.states_sub = rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self.callback,tcp_nodelay=True)
        self.ekf_sub = rospy.Subscriber(
            "/ekf_odom_tf", Pose, self.ekfcallback,tcp_nodelay=True)
        self.pf_sub = rospy.Subscriber(
            "/pf_odom_tf", Pose, self.pfcallback,tcp_nodelay=True)
        self.isFirst = 1
        self.isekfmode = 0
        self.ispfmode = 0
        print("isekf",self.isekfmode)



        # Subscribers: None
        self.pose_pub = rospy.Publisher(
            "/gazebo/" + (self.robot_name + '__' + self.link_name), Pose, queue_size=1,tcp_nodelay=True)
        self.tf_pub = tf.TransformBroadcaster()


    def callback(self, data):
        if self.isekfmode or self.ispfmode:
            pass
        else:
            try:
                ind = data.name.index(self.robot_name + "::" + self.link_name)
                self.link_pose = data.pose[ind]
                if self.isFirst:
                    print("----------GET GAZEBO----------")
                    self.isFirst = 0
            except ValueError:
                pass


    def ekfcallback(self, data):
        if self.isekfmode:
            self.link_pose = data
            if self.isFirst:
                print("----------GET EKF----------")
                self.isFirst = 0
        else:
            pass

    def pfcallback(self, data):
        if self.ispfmode:
            self.link_pose = data
            if self.isFirst:
                print("----------GET PF----------")
                self.isFirst = 0
        else:
            pass       


    
    def publish_tf(self):
        p = self.link_pose.position
        o = self.link_pose.orientation

        if self.isFirst:
            self.tf_pub.sendTransform((0,0,0),(0,0,0,1),
                                        rospy.Time.now(),self.link_name,"map")
        else:
            self.tf_pub.sendTransform((p.x,p.y,p.z),(o.x,o.y,o.z,o.w),
                                        rospy.Time.now(),self.link_name,"map")

if __name__ == '__main__':
    try:
        rospy.init_node('robot_tf')

        gp = GazeboLinkPose(rospy.get_param('~robot_name', 'course_agv'),
                            rospy.get_param('~link_name', 'robot_base'))
        publish_rate = rospy.get_param('~publish_rate', 100)
  

        rate = rospy.Rate(publish_rate)
        print("init done tf")
        while not rospy.is_shutdown():
            gp.pose_pub.publish(gp.link_pose)
            gp.publish_tf()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
