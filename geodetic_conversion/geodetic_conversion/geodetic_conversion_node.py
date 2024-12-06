#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os
import math

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from uf_interfaces.srv import GeoConv
from geodesy import utm

#Define your class - use package name with camel case capitalization
class GeodeticConversion(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('geodetic_conversion_node')

        #Define the publishers here
        #self.'publisher_var_name' = self.create_publisher('MessageType', '/topic_name', 'queue length')
        #self.twist_pub_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        #Define the subscribers here
        #self.'subscriber_var_name' = self.create_subscription('MessageType', '/topic_name', self.'callback_function', 'queue length')
        #self.feedback_sub_ = self.create_subscription(Header, '/feedback', self.feedback_callback, 10)

        #Define and get parameters here
        self.declare_parameter('origin_type', 'xy')
        self.declare_parameter('origin_rotation', -0.6021965548550632)
        self.declare_parameter('origin_x', 368305.0700699703)
        self.declare_parameter('origin_y', 3278357.100811797)

        self.origin_type = self.get_parameter('origin_type').value
        self.origin_rotation = self.get_parameter('origin_rotation').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value

        self.valid_origin = False
        self.valid_rotation = False
        self.init_complete = False

        if self.origin_type == 'xy':
            self.valid_origin = True
            self.get_logger().warn("Origin set to xy, will use default drone park origin")
        elif self.origin_type == 'll':
            self.valid_origin = True
            self.convert_ll_origin(self.origin_x,self.origin_y)
        elif self.origin_type == 'utm':
            self.valid_origin = True
        else:
            self.valid_origin = False
            self.get_logger().error("Invalid origin type. Please stop and change origin_type parameter to either 'll' for lat/lon, 'utm' for utm, or 'xy' for local coordinate frame.")

        if self.origin_rotation > 2*math.pi:
            self.valid_rotation = False
            self.get_logger().warn("Origin rotation set larger than 2*pi, check to make sure parameter is in radians and not degrees.")
        else:
            self.valid_rotation = True

        #Defin services here
        self.ll_to_utm = self.create_service(GeoConv, 'll_to_utm', self.ll_to_utm_callback)
        self.ll_to_xy = self.create_service(GeoConv, 'll_to_xy', self.ll_to_xy_callback)
        self.utm_to_xy = self.create_service(GeoConv, 'utm_to_xy', self.utm_to_xy_callback)
        self.xy_to_utm = self.create_service(GeoConv, 'xy_to_utm', self.xy_to_utm_callback)

        #Variable to track the current time
        self.current_time = self.get_clock().now()

        #Place other variables here

        #Set the timer period and define the timer function that loops at the desired rate
        self.init_complete = True
        time_period = 1/5
        self.timer = self.create_timer(time_period, self.timer_callback)

    #This is the timer function that runs at the desired rate from above
    def timer_callback(self):
        if not self.valid_origin:
            self.get_logger().error("Invalid origin type. Please stop and change origin_type parameter to either 'll' for lat/lon, 'utm' for utm, or 'xy' for local coordinate frame.")

        if not self.valid_rotation:
            self.get_logger().info("Origin rotation set larger than 2*pi, check to make sure parameter is in radians and not degrees.")

        #This is how to keep track of the current time in a ROS2 node
        self.current_time = self.get_clock().now()

    def convert_ll_origin(self):
        try:
            tmp = utm.fromLatLong(self.origin_x,self.origin_y)
            self.origin_x = tmp.easting
            self.origin_y = tmp.northing
        except:
            self.get_logger().error("Failed origin conversion, check parameters and restart node.")

    #Put your callback functions here - these are run whenever the node
    #loops and when there are messages available to process.
    def ll_to_utm_callback(self, request, response):

        lat = request.input_x
        lon = request.input_y

        try:
            tmp = utm.fromLatLong(lat,lon)
        except:
            response.success = False
            response.message = "Failed to convert lat/lon point to utm point."
            return response

        response.output_x = tmp.easting
        response.output_y = tmp.northing
        response.success = True
        response.message = "Successfully converted lat/lon point to utm point."
        return response

    def ll_to_xy_callback(self, request, response):

        lat = request.input_x
        lon = request.input_y

        try:
            tmp = utm.fromLatLong(lat,lon)
        except:
            response.success = False
            response.message = "Failed to convert lat/lon point to xy point."
            return response

        dx = tmp.easting - self.origin_x
        dy = tmp.northing - self.origin_y
        q = self.origin_rotation

        x = math.cos(q)*dx + math.sin(q)*dy
        y = math.cos(q)*dy - math.sin(q)*dx

        response.output_x = x
        response.output_y = y
        response.success = True
        response.message = "Successfully converted lat/lon point to xy point."
        return response

    def utm_to_xy_callback(self, request, response):

        easting = request.input_x
        northing = request.input_y
        q = self.origin_rotation

        try:
            dx = easting - self.origin_x
            dy = northing - self.origin_y
            x = math.cos(q)*dx + math.sin(q)*dy
            y = math.cos(q)*dy - math.sin(q)*dx
        except:
            response.success = False
            response.message = "Failed to convert utm point to xy point."
            return response

        response.output_x = x
        response.output_y = y
        response.success = True
        response.message = "Successfully converted utm point to xy point."
        return response

        #This is an example on how to compare the current time
        #to the time in a message
        #This next line subtracts the two times, converts to nanoseconds
        #then puts it in seconds and checks if its greater than 5 seconds
        # if (self.get_clock().now()-self.current_time).nanoseconds*(10**-9) > 5:
        #     #How to print an info statement
        #     self.get_logger().info('Greater than 5 seconds')
        #     #How to print a warning statement
        #     self.get_logger().warn('Greater than 5 seconds')
        #     #How to print an error message
        #     self.get_logger().error('Greater than 5 seconds')

    def xy_to_utm_callback(self, request, response):

        dx = request.input_x
        dy = request.input_y
        q = -self.origin_rotation

        try:
            x = math.cos(q)*dx + math.sin(q)*dy
            y = math.cos(q)*dy - math.sin(q)*dx
            x = x + self.origin_x
            y = y + self.origin_y
        except:
            response.success = False
            response.message = "Failed to convert utm point to xy point."
            return response

        response.output_x = x
        response.output_y = y
        response.success = True
        response.message = "Successfully converted xy point to utm point."
        return response

        #This is an example on how to compare the current time
        #to the time in a message
        #This next line subtracts the two times, converts to nanoseconds
        #then puts it in seconds and checks if its greater than 5 seconds
        # if (self.get_clock().now()-self.current_time).nanoseconds*(10**-9) > 5:
        #     #How to print an info statement
        #     self.get_logger().info('Greater than 5 seconds')
        #     #How to print a warning statement
        #     self.get_logger().warn('Greater than 5 seconds')
        #     #How to print an error message
        #     self.get_logger().error('Greater than 5 seconds')

#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    geodetic_conv = GeodeticConversion()
    rclpy.spin(geodetic_conv)
    geodetic_conv.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
