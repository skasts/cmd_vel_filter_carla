#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from carla_msgs.msg import CarlaEgoVehicleControl

breaking = False

def callback(msg):
    global braking

    if (msg.linear.x == 0) and (msg.linear.y == 0) and (msg.linear.z == 0) and (msg.angular.x == 0) and (msg.angular.y == 0) and (msg.angular.z == 0):
        braking = True
        control = CarlaEgoVehicleControl()
        control.throttle = 0.
        control.brake = 1.
        control.steer = 0.
        pub_control.publish(control)
    elif braking:
        braking = False
        control = CarlaEgoVehicleControl()
        control.brake = 0.
        pub_control.publish(control)
    else:
        # Invert the velocity and publish
        msg.angular.x = -msg.angular.x
        msg.angular.y = -msg.angular.y
        msg.angular.z = -msg.angular.z*2
        pub.publish(msg)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('cmd_vel_filter', anonymous=True)
    # Get sub and pub topics from parameter server
    sub_topic = rospy.get_param('~sub_topic', 'cmd_vel')
    pub_topic = rospy.get_param('~pub_topic', 'carla/ego_vehicle/control/set_target_velocity')
    rospy.loginfo("[%s]: Subscribing to /%s", rospy.get_name(), sub_topic)
    rospy.loginfo("[%s]: Publishing to /%s", rospy.get_name(), pub_topic)
    # Create subscriber and publisher
    sub = rospy.Subscriber(sub_topic, Twist, callback)
    pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
    pub_control = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=10)
    # Spin until ctrl + c
    rospy.spin()