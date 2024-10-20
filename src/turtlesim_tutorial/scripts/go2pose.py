#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'go2pose' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('go2pose', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param("~x")
        self.goal_pose.y = rospy.get_param("~y")
        self.goal_pose.theta = rospy.get_param("~theta")  # Desired orientation angle
        self.distance_tolerance = rospy.get_param("~tol")
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """Proportional control for linear velocity."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Calculate the required heading angle to reach the goal."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, angle_difference, constant=6):
        """Proportional control for angular velocity."""
        return constant * angle_difference

    def move2goal(self):
        """Moves the turtle to the goal position, then adjusts to the desired orientation."""
        goal_pose = Pose()

        # Get the input goal position and orientation from parameters
        goal_pose.x = self.goal_pose.x
        goal_pose.y = self.goal_pose.y
        goal_orientation = self.goal_pose.theta

        # Distance tolerance (e.g., 0.01)
        distance_tolerance = self.distance_tolerance

        vel_msg = Twist()

        # Move to the target position
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            # Proportional controller for linear velocity
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(self.steering_angle(goal_pose) - self.pose.theta)

            # Publishing the velocity message
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot after reaching the target position
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo("Reached target position. Adjusting orientation...")

        # Adjust orientation to the desired angle
        while abs(goal_orientation - self.pose.theta) > 0.01:
            # Ensure angle difference is within -pi to pi range
            angle_difference = (goal_orientation - self.pose.theta + pi) % (2 * pi) - pi

            vel_msg.angular.z = self.angular_vel(angle_difference)

            # Publishing the velocity message for orientation adjustment
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot after adjusting orientation
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Robot Reached destination and orientation")
        rospy.logwarn("Stopping robot")

if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass
