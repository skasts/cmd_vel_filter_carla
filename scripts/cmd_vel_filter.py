#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from carla_msgs.msg import CarlaEgoVehicleControl
import threading

mutex = threading.Lock()
last_update_time = rospy.Time(0)

# This function (watchdog) is called every 0.5 seconds and checks whether the last cmd_vel message
# is more than 1 second ago. If so, it will brake the agent in CARLA. This is to prevent the agent
# from drifting when the target velocity is 0.
def timer_callback(event):
    global last_update_time, mutex

    mutex.acquire(blocking=True)

    if rospy.Time.now() - last_update_time > rospy.Duration(0.2):
        # If the last update time is more than 1 second ago, set the velocity to 0
        # rospy.loginfo("No new cmd_vel message received for 1 second, braking")
        control = CarlaEgoVehicleControl()
        control.throttle = 0.
        control.steer = 0.
        control.brake = 1.
        pub_control.publish(control)
    
    mutex.release()

def cmd_vel_callback(msg):
    global last_update_time, mutex

    # Update the last update time
    mutex.acquire(blocking=True)
    last_update_time = rospy.Time.now()
    mutex.release()

    # Invert the velocity and publish. CARLA is using a left hand coordinate system, while ROS is
    # using a right hand coordinate system. Also, it seems like rotational velocities do not map 
    # 1 to 1 to CARLA. Therefore, multiply by a constant.
    msg.angular.z = -msg.angular.z * 5

    # If velocity is zero but angular velocity is not, it is probably using the inplace_rotate
    # planner. Here we experience a bug where the angular velocity is too low, so we multiply it 
    # by a constant. Not doing this would return in a not moving robot.
    # NOT USED AS MULTIPLYING ABOVE IS ENOUGH
    # if msg.linear.x == 0. and msg.linear.y == 0. and msg.linear.z == 0. and msg.angular.z != 0.:
    #     msg.angular.z = msg.angular.z * 1

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