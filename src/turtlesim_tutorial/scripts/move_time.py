#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys

def move_turtle(lin_vel, ang_vel, move_time):
    # Initialize the ROS node
    rospy.init_node('move_time', anonymous=False)
    
    # Create a publisher for the turtle's velocity
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Define the velocity message
    vel = Twist()
    vel.linear.x = lin_vel
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel
    
    # Get the start time
    start_time = rospy.Time.now().to_sec()
    
    # Keep moving the turtle until the specified time interval is reached
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        
        if current_time - start_time >= move_time:
            rospy.loginfo("Time interval reached, stopping robot")
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            break
        else:
            pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        # Accept linear velocity, angular velocity, and movement duration from command-line arguments
        move_turtle(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    except rospy.ROSInterruptException:
        pass