#!/usr/bin/env python3

#ROS2 library imports
import rclpy
from rclpy.node import Node

#System imports - not always used but necessary sometimes
import sys
import os

#Import the interfaces here
#for example, using the Header msg from the std_msgs package
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped

#Define your class - use package name with camel case capitalization
class APFCollisionAvoidance(Node):
    #Initialize the class
    def __init__(self):
        #Create the node with whatever name (package name + node)
        super().__init__('apf_collision_avoidance_node')


        self.declare_parameter('boundary_x_min')
        self.declare_parameter('boundary_x_max')
        self.declare_parameter('boundary_y_min')
        self.declare_parameter('boundary_y_max')
        self.declare_parameter('vel_min')
        self.declare_parameter('vel_max')
        self.declare_parameter('ang_max')
        self.declare_parameter('influence_radius')
        self.declare_parameter('vehicle_name')

        self.boundary_x_min = self.get_parameter('boundary_x_min').value
        self.boundary_x_max = self.get_parameter('boundary_x_max').value
        self.boundary_y_min = self.get_parameter('boundary_y_min').value
        self.boundary_y_max = self.get_parameter('boundary_y_max').value

        self.veh_name = self.get_parameter('vehicle_name').value

        x_bound = [self.boundary_x_min, self.boundary_x_min, self.boundary_x_max, self.boundary_x_max]
        y_bound = [self.boundary_y_min, self.boundary_y_max, self.boundary_y_max, self.boundary_y_min]

        #Define the publishers here
        #self.'publisher_var_name' = self.create_publisher('MessageType', '/topic_name', 'queue length')
        self.twist_pub_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        #Define the subscribers here
        #self.'subscriber_var_name' = self.create_subscription('MessageType', '/topic_name', self.'callback_function', 'queue length')
        self.agent_sub_ = self.create_subscription(Position, '/agent/pose', self.agent_callback, 10)
        self.target_sub_ = self.create_subscription(Position, '/target/pose', self.target_callback, 10)
        self.pose_sub_ = self.create_subscription(PoseStamped, 'pose', self.target_callback, 10)
        self.twist_sub_ = self.create_publisher(TwistStamped, '/unfiltered_cmd_vel', 10)

        #Variable to store data from callback messages
        self.targets = []
        self.agents = []

        #Variable to store data from callback messages that can be used globally
        #it uses that Header message type since that's the message type on the topic
        self.feedback = Header()

        #Set the timer period and define the timer function that loops at the desired rate
        time_period = 1/10
        self.timer = self.create_timer(time_period, self.timer_callback)


    #This is the timer function that runs at the desired rate from above
    def timer_callback(self):

        self.update_obstacles()

        self.compute_distance_boundary()

        #This is an example on how to create a message,
        #fill in the header information and add some
        #data, then publish
        self.twist_msg = TwistStamped()
        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.twist_msg.twist.linear.x = 10.0
        self.twist_pub_.publish(self.twist_msg)

        #This is how to keep track of the current time in a ROS2 node
        self.current_time = self.get_clock().now()

    #Put your callback functions here - these are run whenever the node
    #loops and when there are messages available to process.
    def feedback_callback(self, msg):
        self.feedback = msg.data

        #This is an example on how to compare the current time
        #to the time in a message
        #This next line subtracts the two times, converts to nanoseconds
        #then puts it in seconds and checks if its greater than 5 seconds
        if (self.get_clock().now()-self.current_time).nanoseconds*(10**-9) > 5:
            #How to print an info statement
            self.get_logger().info('Greater than 5 seconds')
            #How to print a warning statement
            self.get_logger().warn('Greater than 5 seconds')
            #How to print an error message
            self.get_logger().error('Greater than 5 seconds')

    def update_obstacles(self):
        for ii in range(len(self.agents)): #loop through list to see if its already been seen on topic
            if self.agents[ii].name == self.veh_name: #if true we've see its state before
                pass
            else:
                dist = self.compute_distance(self.agents[ii].x, self.agents[ii].y, self.x, self.y)
                if dist < self.influence_radius:
                    self.repulsive_force =


        for ii in range(len(self.targets)): #loop through list to see if its already been seen on topic
            if self.targets[ii].name == self.veh_name: #if true we've see its state before
                pass
            else:



    #Put your callback functions here - these are run whenever the node
    #loops and when there are messages available to process.
    def agent_callback(self, msg):
        agent_pos = msg

        if len(self.agents) < 1: #first agent found on topic, add its state
            agent = agent_(agent_pos.name,agent_pos.x+self.origin_easting,agent_pos.y+self.origin_northing,agent_pos.z,agent_pos.yaw)
            self.agents.append(agent)
        elif len(self.agents) > 1: #more than one agent found on topic
            it = 0
            for ii in range(len(self.agents)): #loop through list to see if its already been seen on topic
                if agent_pose.name == self.agents[ii].name: #if true we've see its state before
                    it = it+1
                    agent = agent_(agent_pos.name,agent_pos.x+self.origin_easting,agent_pos.y+self.origin_northing,agent_pos.z,agent_pos.yaw) #make new struct
                    self.agents[ii] = agent #update its state
            if it == 0: #that means we didn't see new agent in the list
                agent = agent_(agent_pos.name,agent_pos.x+self.origin_easting,agent_pos.y+self.origin_northing,agent_pos.z,agent_pos.yaw) #make new struct for this new agent
                self.agents.append(agent) #add it to the list

    def target_callback(self, msg):
        agent_pos = msg

        if len(self.targets) < 1: #first agent found on topic, add its state
            agent = agent_(agent_pos.name,agent_pos.x+self.origin_easting,agent_pos.y+self.origin_northing,agent_pos.z,agent_pos.yaw)
            self.targets.append(agent)
        elif len(self.targets) > 1: #more than one agent found on topic
            it = 0
            for ii in range(len(self.targets)): #loop through list to see if its already been seen on topic
                if agent_pose.name == self.targets[ii].name: #if true we've see its state before
                    it = it+1
                    agent = agent_(agent_pos.name,agent_pos.x+self.origin_easting,agent_pos.y+self.origin_northing,agent_pos.z,agent_pos.yaw) #make new struct
                    self.targets[ii] = agent #update its state
            if it == 0: #that means we didn't see new agent in the list
                agent = agent_(agent_pos.name,agent_pos.x+self.origin_easting,agent_pos.y+self.origin_northing,agent_pos.z,agent_pos.yaw) #make new struct for this new agent
                self.targets.append(agent) #add it to the list


#This is some boiler plate code that you slap at the bottom
#to make the node work.
#Make sure to update the name of the function and package, etc
#to match your node
def main(args=None):
    rclpy.init(args=args)

    apf_collision_avoidance = APFCollisionAvoidance()
    rclpy.spin(apf_collision_avoidance)
    apf_collision_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()
