#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from carla_msgs.msg import CarlaEgoVehicleControl
import threading

mutex = threading.Lock()
last_update_time = rospy.Time(0)

def timer_callback(event):
    global last_update_time, mutex

    mutex.acquire(blocking=True)

    if rospy.Time.now() - last_update_time > rospy.Duration(1.0):
        # If the last update time is more than 1 second ago, set the velocity to 0
        rospy.loginfo("Resetting target velocity to 0")
        control = CarlaEgoVehicleControl()
        control.throttle = 0.
        control.steer = 0.
        control.brake = 1.
        pub_control.publish(control)
    
    mutex.release()

def cmd_vel_callback(msg):
    global last_update_time, mutex

    mutex.acquire(blocking=True)

    last_update_time = rospy.Time.now()

    mutex.release()

    # Invert the velocity and publish
    msg.angular.x = -msg.angular.x
    msg.angular.y = -msg.angular.y
    msg.angular.z = -msg.angular.z # *2?

    # If velocity is zero but angular velocity is not, set velocity to 0.1
    if msg.linear.x == 0. and msg.linear.y == 0. and msg.linear.z == 0. and msg.angular.z != 0.:
        msg.linear.x = 0.01
        msg.angular.z = msg.angular.z * 3

    pub.publish(msg)
    rospy.loginfo("Passing through target velocity")
    
    # Brake if target velocity is 0
    # This prevents drifting when the target velocity is 0
    control = CarlaEgoVehicleControl()
    control.throttle = 0.
    control.steer = 0.
    if msg == Twist():
        control.brake = 1.
        rospy.loginfo("Braking as target velocity is 0")
    else:
        control.brake = 0.
    pub_control.publish(control)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('cmd_vel_filter', anonymous=True)
    # Get sub and pub topics from parameter server
    sub_topic = rospy.get_param('~sub_topic', 'cmd_vel')
    pub_topic = rospy.get_param('~pub_topic', 'carla/ego_vehicle/control/set_target_velocity')
    rospy.loginfo("[%s]: Subscribing to /%s", rospy.get_name(), sub_topic)
    rospy.loginfo("[%s]: Publishing to /%s", rospy.get_name(), pub_topic)
    # Create subscriber and publisher
    sub = rospy.Subscriber(sub_topic, Twist, cmd_vel_callback)
    pub = rospy.Publisher(pub_topic, Twist, queue_size=10)
    pub_control = rospy.Publisher("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, queue_size=10)
    # Create a watchdog thread
    rospy.Timer(rospy.Duration(0.5), timer_callback)

    # Spin until ctrl + c
    rospy.spin()