#!/usr/bin/env python
import rospy
import tf
from mavros_msgs.msg import RCOut

def callback(msg):
    global br
    global scale
    global angle
    global s13
    global s2
    global s22
    global s023
    br.sendTransform(
        (s13, -s2, s023),
        tf.transformations.quaternion_from_euler(0,0,angle),
        msg.header.stamp,
        "rotor_0",
        "base_link")
    br.sendTransform(
        (-s13, s22, s023),
        tf.transformations.quaternion_from_euler(0,0,angle),
        msg.header.stamp,
        "rotor_1",
        "base_link")
    br.sendTransform(
        (s13, s22, s023),
        tf.transformations.quaternion_from_euler(0,0,angle),
        msg.header.stamp,
        "rotor_2",
        "base_link")
    br.sendTransform(
        (-s13, -s2, s023),
        tf.transformations.quaternion_from_euler(0,0,angle),
        msg.header.stamp,
        "rotor_3",
        "base_link")
    if msg.channels[0] > 910:
        angle = angle + msg.channels[0]/1000

if __name__ == "__main__":

    rospy.init_node("point_to_tf", anonymous=True)

    scale = rospy.get_param('~scale', 1.0)
    
    s13 =  0.13*scale
    s2 =   0.2*scale
    s22 =  0.22*scale
    s023 = 0.023*scale
    angle = 0

    br = tf.TransformBroadcaster()
    rospy.Subscriber("/mavros/rc/out", RCOut, callback, queue_size=1)
    rospy.spin()
