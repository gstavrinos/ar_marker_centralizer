#!/usr/bin/env python
import tf
import rospy
import traceback
from tf import TransformListener
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

safety_counter = 0
targeted_marker_id = 'marker'
twist_publisher = None
empty_publisher = None

def init():
    global targeted_marker_id, twist_publisher, empty_publisher
    rospy.init_node('ar_marker_centralizer')
    targeted_marker_id = rospy.get_param("~targeted_marker_id", 13)
    twist_topic = rospy.get_param("~twist_topic", "cmd_vel")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, marker_callback)
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=1);
    empty_publisher = rospy.Publisher('bebop/land', Empty, queue_size=1);
    while not rospy.is_shutdown():
        rospy.spin()

def marker_callback(msg):
    global targeted_marker_id, safety_counter
    global twist_publisher, empty_publisher
    #if tf_.frameExists("odom") :#and tf_.frameExists(targeted_marker_id):
    found_marker = False
    for marker in msg.markers:
        if marker.id == targeted_marker_id:
            found_marker = True
            position = marker.pose.pose.position
            twist = Twist()
            twist.linear.x = (1 - position.x) * 0.025
            twist.linear.y = -position.y * 0.025
            twist.linear.z = -position.z * 0.025
            twist_publisher.publish(twist)
    if not found_marker:
        safety_counter += 1
        if safety_counter > 50:
            print 'LANDING NOW TO PREVENT DRONE DAMAGE!'
            empty_publisher.publish(Empty())
            safety_counter = 0

if __name__ == '__main__':
    init() 