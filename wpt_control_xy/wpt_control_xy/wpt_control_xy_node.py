#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

#System imports - not always used but necessary sometimes
import sys
import os
import numpy as np
import math

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
#import tf_transformations
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
from std_srvs.srv import SetBool

from pyquaternion import Quaternion

#Define your class - use package name with camel case capitalization
class WptControl(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('wpt_control_xy_node')

        #Define the publishers here
        #self.'publisher_var_name' = self.create_publisher('MessageType', '/topic_name', 'queue length')
        self.vector_pub_ = self.create_publisher(Vector3Stamped, 'controller/setpoint', 10)
        self.wpt_pub_ = self.create_publisher(Pose, 'wpt_recv', 10)

        #Define the subscribers here
        #self.'subscriber_var_name' = self.create_subscription('MessageType', '/topic_name', self.'callback_function', 'queue length')
        #self.feedback_sub_ = self.create_subscription(Header, '/feedback', self.feedback_callback, 10)
        self.pos_sub_ = self.create_subscription(PoseStamped, 'pose', self.pose_callback, qos_profile=qos_profile_sensor_data)
        self.wpt_sub_ = self.create_subscription(Pose, 'wpt', self.wpt_callback, qos_profile=qos_profile_sensor_data)

        self.mission_serv_ = self.create_service(SetBool, 'mission_pause', self.toggle_mission_pause)

        #Define parameters here
        self.declare_parameter('capture_radius', 0.5)
        self.declare_parameter('lead_distance', 1.0)
        self.declare_parameter('max_vel', 0.5)
        self.declare_parameter('min_vel', 0.0)
        self.declare_parameter('max_ang', 0.78)
        self.declare_parameter('origin_rotation', -0.6021965548550632)

        self.capture_rad = self.get_parameter('capture_radius').value
        self.lead_distance = self.get_parameter('lead_distance').value
        self.max_vel = self.get_parameter('max_vel').value
        self.min_vel = self.get_parameter('min_vel').value
        self.park_origin_rot = self.get_parameter('origin_rotation').value
        self.max_ang = self.get_parameter('max_ang').value

        #Variable to track crap
        self.current_time = self.get_clock().now()
        self.pose_epoch = self.get_clock().now()
        self.current_pos = PoseStamped()
        self.starting_pt = []

        self.mission_pause = True
        self.init_gps = False
        self.init_wpt = False
        self.distance_remaining = 9999.0

        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1.0/10.0
        self.timer = self.create_timer(time_period, self.timer_callback)


    #This is the timer function that runs at the desired rate from above
    def timer_callback(self):
        if self.init_gps and self.init_wpt:
            self.compute_distance_remaining()
            self.msg = Vector3Stamped()
            self.msg.header.stamp = self.get_clock().now().to_msg()

            if self.distance_remaining > self.capture_rad:
                if not self.mission_pause:
                    #Do the line follow, publish setpoint
                    #self.get_logger().warn("Publishing setpoint")
                    self.get_logger().warn('Distance: \t{0}'.format(self.distance_remaining))
                    tmp = self.compute_heading_cmd()
                    self.msg.vector.z = tmp
                    #ang = max(min(tmp, self.max_ang), -self.max_ang)
                    self.msg.vector.x = self.max_vel #- abs(ang/self.max_ang)*self.max_vel
                    self.vector_pub_.publish(self.msg)
                else:
                    self.get_logger().warn("Mission paused")
                    #self.msg.vector.x = 0.0
                    #self.msg.vector.z = 0.0
                    #self.vector_pub_.publish(self.msg)
            else:
                    self.get_logger().warn("Reached Wpt")
                    #self.msg.vector.x = 0.0
                    #self.msg.vector.z = 0.0
                    #self.vector_pub_.publish(self.msg)
        else:
            if not self.init_gps:
                self.get_logger().warn("missing gps.")
            if not self.init_wpt:
                self.get_logger().warn("missing wpt.")



        #This is how to keep track of the current time in a ROS2 node
        self.current_time = self.get_clock().now()

    #Put your callback functions here - these are run whenever the node
    #loops and when there are messages available to process.

    def compute_distance_remaining(self):
        x1 = self.current_pos.x
        y1 = self.current_pos.y
        x2 = self.wpt.x
        y2 = self.wpt.y
        self.distance_remaining = self.compute_distance(x1,y1,x2,y2)

    def compute_distance(self, x1, y1, x2, y2):
        temp = math.sqrt(((x2-x1)**2) + ((y2-y1)**2))
        return temp

    def pose_callback(self, msg):
        # self.get_logger().warn("Got pose")
        self.pose_epoch = self.get_clock().now()
        self.current_pos = msg.pose.position
        self.init_gps = True

        self.get_logger().warn('Position Update:\tX : {0}, Y : {1}'.format(self.current_pos.x, self.current_pos.y))

        # xi = msg.pose.orientation.x
        # yi = msg.pose.orientation.y
        # zi = msg.pose.orientation.z
        # wi = msg.pose.orientation.w
        #
        # q_f = Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        #
        # self.heading = math.atan2(2.0 * (q_f.w * q_f.z + q_f.x * q_f.y), 1 - 2.0* (q_f.y * q_f.y + q_f.z * q_f.z));

    def wpt_callback(self, msg):
        self.init_wpt = True
        self.wpt_epoch = self.get_clock().now()

        msg_out = msg
        self.wpt_pub_.publish(msg_out)

        self.wpt = msg.position
        self.starting_pt = self.current_pos


    def toggle_mission_pause(self, request, response):
        self.mission_pause = request.data

        if self.mission_pause:
            response.success = True
            response.message = "Mission paused."
        else:
            response.success = True
            response.message = "Mission continuing."

        return response

    def compute_heading_cmd(self):

        pk0 = np.array([self.starting_pt.x, self.starting_pt.y])
        pk1 = np.array([self.wpt.x, self.wpt.y])
        pnt = np.array([self.current_pos.x,self.current_pos.y])

        alpha_k = math.atan2(pk1[1]-pk0[1],pk1[0]-pk0[0])

        rot_mat = np.array([[math.cos(alpha_k), -math.sin(alpha_k)],
                            [math.sin(alpha_k), math.cos(alpha_k)]])

        path_error = np.matmul(rot_mat.T,pnt-pk0)

        along_track_error = path_error[0]
        cross_track_error = path_error[1]

        path_tangential_angle = alpha_k
        velocity_path_rel_angle = math.atan2(-cross_track_error, self.lead_distance)

        course_angle = path_tangential_angle + velocity_path_rel_angle

        return self.pi_clip(course_angle)

        #
        # heading_cmd = math.atan2(self.wpt.y - self.current_pos.y, self.wpt.x - self.current_pos.x)
        # return self.pi_clip(heading_cmd)

    def pi_clip(self, angle):
        if angle > 0:
            if angle > math.pi:
                return angle - 2*math.pi
        else:
            if angle < -math.pi:
                return angle + 2*math.pi
        return angle

    def quaternion_from_euler(self, roll, pitch, yaw): #roll (x), pitch (Y), yaw (z)

        #Abbreviations for the various angular functions

        cr = math.cos(roll * 0.5);
        sr = math.sin(roll * 0.5);
        cp = math.cos(pitch * 0.5);
        sp = math.sin(pitch * 0.5);
        cy = math.cos(yaw * 0.5);
        sy = math.sin(yaw * 0.5);

        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;

        q = [qw, qx, qy, qz]

        return q;

    def euler_from_quaternion(self, q):

        #roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[1]);
        roll = math.atan2(sinr_cosp, cosr_cosp);

        #pitch (y-axis rotation)
        sinp = np.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]));
        cosp = np.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]));
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2;

        #yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
        yaw = math.atan2(siny_cosp, cosy_cosp);

        angles = [roll, pitch, yaw]

        return angles;


    # def feedback_callback(self, msg):
    #     self.feedback = msg.data
    #
    #     #This is an example on how to compare the current time
    #     #to the time in a message
    #     #This next line subtracts the two times, converts to nanoseconds
    #     #then puts it in seconds and checks if its greater than 5 seconds
    #     if (self.get_clock().now()-self.current_time).nanoseconds*(10**-9) > 5:
    #         #How to print an info statement
    #         self.get_logger().info('Greater than 5 seconds')
    #         #How to print a warning statement
    #         self.get_logger().warn('Greater than 5 seconds')
    #         #How to print an error message
    #         self.get_logger().error('Greater than 5 seconds')

#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    wpt_control = WptControl()
    rclpy.spin(wpt_control)
    wpt_control.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
