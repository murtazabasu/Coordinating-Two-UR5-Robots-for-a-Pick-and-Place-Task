#!/usr/bin/env python2
import roslib
roslib.load_manifest('ur5_notebook')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('broadcaster_fixed')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "robot1_tf/world",
                         "world")
        rate.sleep()