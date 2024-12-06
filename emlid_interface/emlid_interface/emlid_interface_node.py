#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import os
import sys
import math
import numpy as np

from nmea import input_stream

#Note, for eloquent, tf_transformations is not available
from sensor_msgs.msg import NavSatFix
from geodesy import utm

#Subscribe to IMU, read GPS from port, publish GPS
#and publish local coordinates and orientation

#Topics In
#/imu/data

#Topics Out
#/rtk/fix
#/local/pose

class EmlidInterface(Node):
    def __init__(self):
        super().__init__('emlid_interface_node')

        self.rtk_pub_ = self.create_publisher(NavSatFix, 'rtk/fix', 10)
        self.utm_pub_ = self.create_publisher(NavSatFix, 'rtk/utm', 10)

        self.declare_parameter('baud_rate', 57600)
        # self.declare_parameter('origin_easting', 368305.0700699703)
        # self.declare_parameter('origin_northing', 3278357.100811797)
        # self.declare_parameter('origin_rotation', -0.6021965548550632)

        # self.origin_easting = self.get_parameter('origin_easting').value
        # self.origin_northing = self.get_parameter('origin_northing').value
        # self.origin_rotation = self.get_parameter('origin_rotation').value
        baud = self.get_parameter('baud_rate').value

        self.gps_init = False
        # self.imu_data = Imu()

        self.port = input_stream.GenericInputStream()
        self.port.baud = baud
        self.stream = input_stream.GenericInputStream.open_stream('/dev/emlid')

        self.get_logger().info("Connected to emlid and streaming data.")

        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        recv = self.stream.get_line()
        self.parse(str(recv))

    def parse(self, msg):
        msg_good = False
        data = msg.split(',')

        header = data[0]
        #print(header)

        if header == "b'$GNGGA":
            self.gps_init = True
            msg_good = True
            utc = data[1]
            lat = data[2]
            n_s = data[3]
            lon = data[4]
            e_w = data[5]
            gps_qual = data[6]
            num_sats = data[7]
            hdop = data[8]
            alt = data[9]
            alt_units = data[10]
            geo_sep = data[11]
            geo_units = data[12]
            age_diff_data = data[13]
            crc = data[14]

        # elif header == "b'$GNRMC":
        #     utc = data[1]
        #     valid = data[2]
        #     lat = data[3]
        #     n_s = data[4]
        #     lon = data[5]
        #     e_w = data[6]
        #     speed = data[7]
        #     course = data[8]
        #     date = data[9]
        #     mag_diff = data[10]
        #     e_w = data[11]
        #     crc = data[12]

        if msg_good:
            if lat != '' and lon != '':
                deg = int(round(float(lat)/100,0))
                dec_min = (float(lat)%100)/60
                lat = deg+dec_min

                deg = int(round(float(lon)/100,0))
                dec_min = (float(lon)%100)/60.0
                lon = deg+dec_min

                if e_w == "W":
                    lon = -lon

                gps_pos = [lat,lon]
                self.utm_transform(gps_pos, header, data)
                self.pack_navsatfix(gps_pos, header, data)
                # self.local_transform(gps_pos, float(data[9]))

    def pack_navsatfix(self, pos, header, data):
        navsat_msg = NavSatFix()
        navsat_msg.header.stamp = self.get_clock().now().to_msg()
        navsat_msg.header.frame_id = "navsat_link"

        if header == "b'$GNGGA":
            gps_qual = int(data[6])
            if gps_qual == 0:
                navsat_msg.status.status = -1
            elif gps_qual == 1:
                navsat_msg.status.status = 0
                navsat_msg.status.service = 1
            elif gps_qual == 4:
                navsat_msg.status.status = 1
                navsat_msg.status.service = 1
            elif gps_qual == 5:
                navsat_msg.status.status = 2
                navsat_msg.status.service = 1

        navsat_msg.latitude = pos[0]
        navsat_msg.longitude = pos[1]
        navsat_msg.altitude = float(data[9])

        self.rtk_pub_.publish(navsat_msg)

    def utm_transform(self, pos, header, data):
        tmp_utm = utm.fromLatLong(pos[0], pos[1])

        utm_msg = NavSatFix()
        utm_msg.header.stamp = self.get_clock().now().to_msg()
        utm_msg.header.frame_id = "utm_link"

        if header == "b'$GNGGA":
            gps_qual = int(data[6])
            if gps_qual == 0:
                utm_msg.status.status = -1
            elif gps_qual == 1:
                utm_msg.status.status = 0
                utm_msg.status.service = 1
            elif gps_qual == 4:
                utm_msg.status.status = 1
                utm_msg.status.service = 1
            elif gps_qual == 5:
                utm_msg.status.status = 2
                utm_msg.status.service = 1

        utm_msg.latitude = tmp_utm.easting
        utm_msg.longitude = tmp_utm.northing
        utm_msg.altitude = float(data[9])

        self.utm_pub_.publish(utm_msg)

    # def local_transform(self, data, alt):
    #
    #     tmp_utm = utm.fromLatLong(data[0], data[1])
    #
    #     x = tmp_utm.easting - self.origin_easting
    #     y = tmp_utm.northing - self.origin_northing
    #     w = self.origin_rotation
    #     rot = [math.cos(w)*x+math.sin(w)*y, -math.sin(w)*x+math.cos(w)*y]
    #     x = rot[0]
    #     y = rot[1]
    #
    #     pos_msg = PoseStamped()
    #     pos_msg.header.stamp = self.get_clock().now().to_msg()
    #     pos_msg.header.frame_id = "navsat_link"
    #     pos_msg.pose.position.x = x
    #     pos_msg.pose.position.y = y
    #     pos_msg.pose.position.z = alt
    #
    #     xi = self.imu_data.orientation.x
    #     yi = self.imu_data.orientation.y
    #     zi = self.imu_data.orientation.z
    #     wi = self.imu_data.orientation.w
    #
    #     #euler = self.euler_from_quaternion([wi,xi,yi,zi])
    #     euler = tf_transformations.euler_from_quaternion([wi,xi,yi,zi], axes='sxyz')
    #     euler = list(euler)
    #     euler[2] = self.wrap_angle(euler[2] + self.origin_rotation)
    #     #quat = self.quaternion_from_euler(euler[0], euler[1], euler[2])
    #     quat = tf_transformations.quaternion_from_euler(euler[0], euler[1], euler[2], 'sxyz')
    #
    #     pos_msg.pose.orientation.x = quat[1]
    #     pos_msg.pose.orientation.y = quat[2]
    #     pos_msg.pose.orientation.z = quat[3]
    #     pos_msg.pose.orientation.w = quat[0]
    #
    #     self.pos_pub_.publish(pos_msg)
    #
    # def imu_callback(self, msg):
    #     self.imu_data = msg
    #
    # def wrap_angle(self, angle):
    #     if angle > 0:
    #         if angle > math.pi:
    #             return angle - 2*math.pi
    #     else:
    #         if angle < -math.pi:
    #             return angle + 2*math.pi
    #     return angle
    #
    # def safety_check(self, epoch):
    #     time_elapsed = (self.get_clock().now()-self.current_time).nanoseconds*(10**-9)
    #     if time_elapsed < 1.0: #1 second since last update
    #         return True
    #     else:
    #         self.get_logger().warn("Missing gps data or gps data missing update rate.")
    #         return False
    #
    # def quaternion_from_euler(self, roll, pitch, yaw): #roll (x), pitch (Y), yaw (z)
    #
    #     #Abbreviations for the various angular functions
    #
    #     cr = math.cos(roll * 0.5);
    #     sr = math.sin(roll * 0.5);
    #     cp = math.cos(pitch * 0.5);
    #     sp = math.sin(pitch * 0.5);
    #     cy = math.cos(yaw * 0.5);
    #     sy = math.sin(yaw * 0.5);
    #
    #     qw = cr * cp * cy + sr * sp * sy;
    #     qx = sr * cp * cy - cr * sp * sy;
    #     qy = cr * sp * cy + sr * cp * sy;
    #     qz = cr * cp * sy - sr * sp * cy;
    #
    #     q = [qw, qx, qy, qz]
    #
    #     return q;
    #
    # def euler_from_quaternion(self, q):
    #
    #     #roll (x-axis rotation)
    #     sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    #     cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[1]);
    #     roll = math.atan2(sinr_cosp, cosr_cosp);
    #
    #     #pitch (y-axis rotation)
    #     sinp = np.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]));
    #     cosp = np.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]));
    #     pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2;
    #
    #     #yaw (z-axis rotation)
    #     siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    #     cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    #     yaw = math.atan2(siny_cosp, cosy_cosp);
    #
    #     angles = [roll, pitch, yaw]
    #
    #     return angles;

def main(args=None):
    rclpy.init(args=args)

    emlid_interface = EmlidInterface()
    rclpy.spin(emlid_interface)
    emlid_interface.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
