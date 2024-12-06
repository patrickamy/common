#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf_transformations

#System imports - not always used but necessary sometimes
import sys
import os
import math
import numpy as np

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from uf_interfaces.msg import Position
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#To Do
# 1) Use TF to transform current position to the autonomy park frame
#    Subscribe to odometry/global
#    Publish /pose and /

#Define your class - use package name with camel case capitalization
class PositionBroadcaster(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('position_broadcaster_node')

        #Define the publishers here
        #self.'publisher_var_name' = self.create_publisher('MessageType', '/topic_name', 'queue length')
        self.declare_parameter('vehicle_domain', '')
        self.declare_parameter('vehicle_type', '')
        self.declare_parameter('vehicle_name', '')

        self.declare_parameter('origin_easting', 368304.87143102096)
        self.declare_parameter('origin_northing', 3278357.789323321)
        self.declare_parameter('origin_rotation', -0.6021965548550632)

        self.origin_easting = self.get_parameter('origin_easting').value
        self.origin_northing = self.get_parameter('origin_northing').value
        self.origin_rotation = self.get_parameter('origin_rotation').value

        self.veh_domain = self.get_parameter('vehicle_domain').value
        self.veh_type = self.get_parameter('vehicle_type').value
        self.veh_name = self.get_parameter('vehicle_name').value

        topic_name_ = ''

        if self.veh_domain != '':
            topic_name_ += '/'+self.veh_domain
        if self.veh_type != '':
            topic_name_ += '/'+self.veh_type

        if topic_name_ == '':
            self.pos_pub_ = self.create_publisher(Position, '/pose', 10)
            self.pose_pub_ = self.create_publisher(PoseStamped, 'pose', 10)
        else:
            self.pos_pub_ = self.create_publisher(Position, topic_name_+'/pos', 10)
            self.pose_pub_ = self.create_publisher(PoseStamped, topic_name_+'/pose', 10)

        #Define the subscribers here
        #self.'subscriber_var_name' = self.create_subscription('MessageType', '/topic_name', self.'callback_function', 'queue length')
        self.odom_sub = self.create_subscription(Odometry, 'odometry/global', self.odom_callback, qos_profile=qos_profile_sensor_data)

        #Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter('target_frame', 'autonomy_park').get_parameter_value().string_value
        self.source_frame = self.declare_parameter('source_frame', 'map').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_init = False
        self.odom = Odometry()
        self.odom_epoch = self.get_clock().now()

        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        if self.odom_init and self.safety_check(self.odom_epoch, "odom"):

            transformed_pos = self.local_transform()

            if transformed_pos is not None:
                pos_msg = Position()
                pos_msg.header.stamp = self.get_clock().now().to_msg()
                pos_msg.name = self.veh_name
                pos_msg.x = transformed_pos[0]
                pos_msg.y = transformed_pos[1]
                pos_msg.z = transformed_pos[2]
                pos_msg.yaw = transformed_pos[3]
                self.pos_pub_.publish(pos_msg)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.target_frame
                pose_msg.pose.position.x = transformed_pos[0]
                pose_msg.pose.position.y = transformed_pos[1]
                pose_msg.pose.position.z = transformed_pos[2]
                pose_msg.pose.orientation.x = transformed_pos[4]
                pose_msg.pose.orientation.y = transformed_pos[5]
                pose_msg.pose.orientation.z = transformed_pos[6]
                pose_msg.pose.orientation.w = transformed_pos[7]
                self.pose_pub_.publish(pose_msg)

    def local_transform(self):

        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return

        rotation = self.euler_from_quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
        heading = self.euler_from_quaternion(self.odom.pose.orientation.x, self.odom.pose.orientation.y, self.odom.pose.orientation.z, self.odom.pose.orientation.w)

        x_ = self.odom.pose.position.x + t.transform.translation.x
        y_ = self.odom.pose.position.y + t.transform.translation.y
        z_ = self.odom.pose.position.z + t.transform.translation.z

        x_ = math.cos(rotation[0])*x_ - math.sin(rotation[0])*y_
        y_ = math.sin(rotation[0])*x_ + math.cos(rotation[0])*y_
        z_ = z_

        x_ = x_ + t.transform.translation.x
        y_ = y_ + t.transform.translation.y
        z_ = z_ + t.transform.translation.z

        yaw = heading[2] - rotation[2]

        transformed_quat = self.quaternion_from_euler(rotation[0], rotation[1], yaw)

        return [x_, y_, z_, yaw, transformed_quat[0], transformed_quat[1], transformed_quat[2], transformed_quat[3]]


        # x = self.current_pos[0] - self.origin_easting
        # y = self.current_pos[1] - self.origin_northing
        # alt = self.current_pos[2]
        # w = self.origin_rotation
        # rot = [math.cos(w)*x+math.sin(w)*y, -math.sin(w)*x+math.cos(w)*y]
        # x = rot[0]
        # y = rot[1]
        #
        # pos_msg = PoseStamped()
        # pos_msg.header.stamp = self.get_clock().now().to_msg()
        # pos_msg.header.frame_id = self.veh_name
        # pos_msg.pose.position.x = x
        # pos_msg.pose.position.y = y
        # pos_msg.pose.position.z = alt
        #
        # xi = self.imu_data.orientation.x
        # yi = self.imu_data.orientation.y
        # zi = self.imu_data.orientation.z
        # wi = self.imu_data.orientation.w
        #
        # (roll, pitch, yaw) = self.euler_from_quaternion(xi,yi,zi,wi)
        # #(roll, pitch, yaw) = tf_transformations.euler_from_quaternion([xi,yi,zi,wi], 'sxyz')
        # #euler = list(euler)
        # local_yaw = yaw #self.wrap_angle(yaw + self.origin_rotation)
        #
        # quat = self.quaternion_from_euler(roll, pitch, local_yaw)
        #
        # pos_msg.pose.orientation.x = quat[1]
        # pos_msg.pose.orientation.y = quat[2]
        # pos_msg.pose.orientation.z = quat[3]
        # pos_msg.pose.orientation.w = quat[0]
        #
        # # pos_msg.pose.orientation.x = self.imu_data.orientation.x
        # # pos_msg.pose.orientation.y = self.imu_data.orientation.y
        # # pos_msg.pose.orientation.z = self.imu_data.orientation.z
        # # pos_msg.pose.orientation.w = self.imu_data.orientation.w
        #
        # # self.get_logger().warn("Publishing pose...")
        # self.pose_pub_.publish(pos_msg)
        # #self.get_logger().warn("Pose published.")
        #
        # pos_msg = Position()
        # pos_msg.name = self.veh_name
        # pos_msg.x = x
        # pos_msg.y = y
        # pos_msg.z = alt
        # pos_msg.yaw = local_yaw
        #
        # # self.get_logger().warn("Publishing position...")
        # self.pos_pub_.publish(pos_msg)
        # # self.get_logger().warn("Position published.")

    def odom_callback(self, msg):
        self.odom_init = True
        self.odom_epoch = self.get_clock().now()
        self.odom = msg.pose

    def wrap_angle(self, angle):
        if angle > 0:
            if angle > math.pi:
                return angle - 2*math.pi
        else:
            if angle < -math.pi:
                return angle + 2*math.pi
        return angle

    def safety_check(self, epoch, sensor):

        if (self.get_clock().now()-epoch).nanoseconds*(10**-9) < 2.0: #1 second since last update
            return True
        else:
            self.get_logger().warn('Missing sensor data or sensor data missing update rate: {0}'.format(sensor))
            return False

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

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians

#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    position_broadcaster = PositionBroadcaster()
    rclpy.spin(position_broadcaster)
    position_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
