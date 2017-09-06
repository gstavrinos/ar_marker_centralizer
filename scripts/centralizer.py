#!/usr/bin/env python
import tf
import rospy
import traceback
from std_msgs.msg import Empty
from tf import TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist

tf_ = None
safety_counter = 0
targeted_marker_id = 'marker'
twist_publisher = None
empty_publisher = None

def init():
    global targeted_marker_id, twist_publisher, empty_publisher
    global tf_
    rospy.init_node('ar_marker_centralizer')
    targeted_marker_id = rospy.get_param("~targeted_marker_id", 13)
    twist_topic = rospy.get_param("~twist_topic", "bebop/cmd_vel")
    tf_ = TransformListener()
    rospy.Subscriber("tf", TFMessage, tf_callback)
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=1);
    empty_publisher = rospy.Publisher('bebop/land', Empty, queue_size=1);
    while not rospy.is_shutdown():
        rospy.spin()

def tf_callback(msg):
    global targeted_marker_id, safety_counter
    global twist_publisher, empty_publisher
    global tf_
    targeted_tf = 'ar_marker_' + str(targeted_marker_id)
    try:
        t = tf_.getLatestCommonTime("/base_link", targeted_tf)
        position, quaternion = tf_.lookupTransform("/base_link", targeted_tf, t)
        twist = Twist()
        twist.linear.x = (position[0] - 0.75) * 0.12
        twist.linear.y = position[1] * 0.25
        #twist.linear.z = -position[2] * 0.025
        twist_publisher.publish(twist)
        safety_counter = 0
        #print position
    except:
        safety_counter += 1
        if safety_counter > 5:
            print 'LANDING NOW TO ENSURE DRONE SAFETY!'
            empty_publisher.publish(Empty())
            safety_counter = 0
        #print traceback.format_exc()

if __name__ == '__main__':
    init() 