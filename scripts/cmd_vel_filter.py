#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from carla_msgs.msg import CarlaEgoVehicleControl

def callback(msg):
    # Invert the velocity and publish
    msg.angular.x = -msg.angular.x
    msg.angular.y = -msg.angular.y
    msg.angular.z = -msg.angular.z*2
    pub.publish(msg)
    rospy.loginfo("Passing through target velocity")
    
    # Brake if target velocity is 0
    # This prevents drifting when the target velocity is 0
    if msg == Twist():
        control = CarlaEgoVehicleControl()
        control.throttle = 0.
        control.brake = 1.
        control.steer = 0.
        pub_control.publish(control)
        rospy.loginfo("Braking as target velocity is 0")

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