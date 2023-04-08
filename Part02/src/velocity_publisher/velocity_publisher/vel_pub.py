import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile #qos profile for publisher
from math import cos, sin, sqrt, pi
from example_interfaces.msg import String
import time

from .planner import planner


class velocity_publsiher():
    def __init__(self):
        self.BURGER_MAX_LIN_VEL = 0.22 # in m/s
        self.BURGER_MAX_ANG_VEL = 2.84 # in rad/s
        self.r = 0.033 # in meter
        self.L = 0.16  # in meter
        self.map_width = 6 # in meter
        self.map_height = 2 # in meter
        
        self.node = Node("velocity_publisher") #Node name
        qos = QoSProfile(depth=10) #qos profile for publisher is 10
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', qos) #Topic name is /cmd_vel, queue size is qos, message type is Twist

        self.start_node_pub = self.node.create_publisher(String, '/start_node', 10) #create a publisher to publish the start node to /start_node topic
        
        self.start_node = self.node.declare_parameter('start_node') #declare a parameter to get the start node from launch file
        self.goal_node = self.node.declare_parameter('goal_node')
        self.RPM1 = self.node.declare_parameter('RPM1')
        self.RPM2 = self.node.declare_parameter('RPM2') 
        self.clearance = self.node.declare_parameter('clearance')


    #Function to constrain robott velocity between low_bound and high_bound
    #Turtlebot has a max linear velocity of 0.22 m/s and max angular velocity of 2.84 rad/s
    #Any vel above 0.22 m/s or 2.84 rad/s will be set to 0.22 m/s or 2.84 rad/s
    #Input: velocity, low_bound, high_bound
    #Output: constrained velocity
    def vel_constrain(self, input_vel, low_bound, high_bound):
        if input_vel < low_bound:
            input_vel = low_bound
        elif input_vel > high_bound:
            input_vel = high_bound
        else:
            input_vel = input_vel

        return input_vel

    #Function to check if linear velocity is within the limit and call vel_constrain function
    #Input: linear velocity list
    #Output: checked linear velocity list
    def check_linear_limit_velocity(self, lin_velocity_list):
        for i in range(len(lin_velocity_list)):
            lin_velocity_list[i] = self.vel_constrain(lin_velocity_list[i], -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)
        
        return lin_velocity_list

    #Function to check if angular velocity is within the limit and call vel_constrain function
    #Input: angular velocity list
    #Output: checked angular velocity list
    def check_angular_limit_velocity(self, ang_velocity_list):
        for i in range(len(ang_velocity_list)):
            ang_velocity_list[i] = self.vel_constrain(ang_velocity_list[i], -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)

        return ang_velocity_list
        
    

    #Function to publish velocity to "/cmd_vel" topic
    #Input: linear velocity list and angular velocity list
    #Output: publish to /cmd_vel topic
    def ros_pub(self, lin_vel, ang_vel): 
        move = Twist() #create a Twist message
        for i in range(len(lin_vel)):
        # for i in range(3):
            move.linear.x = lin_vel[i]  #set x linear velocity
            move.angular.z = -1*ang_vel[i] #set z angular velocity
            self.pub.publish(move) #publish the message
            time.sleep(1.09)
            
        move.linear.x = 0.0 #set x linear velocity
        move.angular.z = 0.0 #set z angular velocity
        self.pub.publish(move) #publish the message
        time.sleep(3)


    #Function to convert wheel velocity to robot velocity
    #Based on the formula: x = r/2 * (ul+ur) * cos(theta), y = r/2 * (ul+ur) * sin(theta), theta = r/L * (ur - ul)
    #Input: ul, ur, theta
    #Output: linear velocity list and angular velocity list
    def convert_wheel_vel_to_robot_vel(self, ul, ur, theta):
        lin__vel = []
        ang_vel = []

        for i in range(len(ul)):
            x_dot = (self.r/2) * (ul[i]+ur[i]) * cos(theta[i]) #mobile robot kinematics equation x_dot = (r/2) * (ul+ur) * cos(theta)
            y_dot = (self.r/2) * (ul[i]+ur[i]) * sin(theta[i]) #mobile robot kinematics equation y_dot = (r/2) * (ul+ur) * sin(theta)
            theta_dot = (self.r/self.L) * (ur[i] - ul[i]) #mobile robot kinematics equation theta_dot = (r/L) * (ur - ul)

            lin__vel.append(sqrt(x_dot ** 2 + y_dot ** 2)) #linear velocity = sqrt(x_dot^2 + y_dot^2)
            ang_vel.append(theta_dot)

        return lin__vel, ang_vel
    
    #Function to convert meter to centimeter for a node
    #Input: node
    #Output: converted node
    def convert_m_to_cm(self, node):
        node[0] = node[0] * 100
        node[1] = node[1] * 100
        return node

    #Function to convert gazebo coordinate to opencv coordinate
    #Input: node/cooridnate
    #Output: converted node/coordinate
    def convert_xy_gazebo_to_opencv(self, node):
        node[0] = node[0]+1
        node[1] = node[1]+0.5
        return node
    
    #Function to convert centimeter to meter for linear velocity list
    #Input: linear velocity list
    #Output: converted linear velocity list
    def convert_cms_to_ms(self, lin_velocity_list):
        for i in range(len(lin_velocity_list)):
            lin_velocity_list[i] = lin_velocity_list[i] / 100

        return lin_velocity_list

    #Function to convert degree to radian for angular velocity list
    #Input: angular velocity list
    #Output: converted angular velocity list
    def convert_deg_to_rad(self, ang_velocity_list):
        for i in range(len(ang_velocity_list)):
            ang_velocity_list[i] = (ang_velocity_list[i]) * pi/180

        return ang_velocity_list
    

    #Function to process node from user input to opencv requirement
    #Using above functions
    #Input: node
    #Output: processed node
    def node_process_user_to_opencv(self, node):
        node = self.convert_xy_gazebo_to_opencv(node)
        node = self.convert_m_to_cm(node)
        return node
    
    #Function to process node from opencv output to ROS requirement
    #Using above functions
    #Input: ul, ur, theta lists
    #Output: processed ul, ur, theta lists
    def vel_process_opencv_to_ROS(self, ul, ur, theta):
        theta = self.convert_deg_to_rad(theta)
        return ul, ur, theta

    
##Requirements:    
    ####A* & opencv:
        ##Input:
        #start node (x,y,theta_s) - x,y in cm, theta_s in degree
        #goal node (x,y,theta_s) - x,y in cm, theta_s in degree
        #RPM1 & RPM2 is revoluation per minute
        #clearance is in cm

        ##Output:
        #ul,ur in cm/s
        #theta in degree


    ####Gazebo & ROS:
        ##What I need:
        #start node (x,y,theta_s) - x,y in m, theta_s in rad 
        #ul,ur in m/s 
        #theta in rad 

    ####User Input:
        #start node (x,y,theta_s) - x,y in m, theta_s in degree (theta_s is in degree for user convenience)
        #goal node (x,y,theta_s) - x,y in m, theta_s in degree
        #RPM1 & RPM2 - revoluation per minute
        #clearance - in mm (for user convenience)


def main():
    rclpy.init() #initiate ros2
    p1 = velocity_publsiher() #create a object of class velocity_publisher

    p1.start_node = p1.start_node.get_parameter_value().double_array_value
    print("User start_node: ", p1.start_node) #Print the obtained data to terminal
    p1.node.get_logger().info("User start_node: %s" % p1.start_node) #print and log the user input to ROS2

    p1.goal_node = p1.goal_node.get_parameter_value().double_array_value
    print("User goal_node: ", p1.goal_node)
    p1.node.get_logger().info("User goal_node: %s" % p1.goal_node)

    p1.clearance = p1.clearance.get_parameter_value().double_value
    print("User clearance: ", p1.clearance)
    p1.node.get_logger().info("User clearance: %s" % p1.clearance)

    p1.RPM1 = p1.RPM1.get_parameter_value().double_value
    print("User RPM1: ", p1.RPM1)
    p1.node.get_logger().info("User RPM1: %s" % p1.RPM1)

    p1.RPM2 = p1.RPM2.get_parameter_value().double_value
    print("User RPM2: ", p1.RPM2)
    p1.node.get_logger().info("User RPM2: %s" % p1.RPM2)

    # print("Enter start node (x,y,theta_s) (x,y) in m, theta_s in degree: ")
    # start_node = [float(x) for x in input().split()]

    # print("Enter goal node (x,y,theta_s) (x,y) in m, theta_s in degree: ")
    # goal_node = [float(x) for x in input().split()]

    # print("Enter RPM1 (revoluation per minute): ")
    # RPM1 = int(input())

    # print("Enter RPM2 (revoluation per minute): ")
    # RPM2 = int(input())

    # print("Enter clearance (in mm): ")
    # clearance = int(input())

    start_node = [p1.start_node[1],p1.start_node[0], p1.start_node[2]] #convert x,y to y,x for opencv
    goal_node  = [p1.goal_node[1],p1.goal_node[0], p1.goal_node[2]]

    #Process User Input:
    print("Processing User Input...")
    start_node = p1.node_process_user_to_opencv(start_node) #Convert to cm and opencv coordinate system
    goal_node = p1.node_process_user_to_opencv(goal_node)   
    clearance = int(p1.clearance) / 10 #convert to cm
    RPM1 = (p1.RPM1*2*pi)/60 #convert to rev/s and rev/s * 2pi = angular vel of wheel (any)
    RPM2 = (p1.RPM2*2*pi)/60 #convert to rev/s and rev/s * 2pi = angular vel of wheel (any)

    # #Test Case 3:
    # start_node = [30,30,30]
    # goal_node = [180,400,30]
    # clearance = 5
    # RPM1 = (50*2*pi)/60
    # RPM2 = (100*2*pi)/60

    start_node = [int(start_node[0]), int(start_node[1]), int(start_node[2])] #convert to int
    goal_node  = [int(goal_node[0]), int(goal_node[1]), int(goal_node[2])] 

    #Print User Input fed to A* Algorithm:
    print("Processed User Input for A* Algorithm:")
    print("Start Node: ", start_node)
    print("Goal Node: ", goal_node)
    print("RPM1: ", RPM1)
    print("RPM2: ", RPM2)
    print("Clearance: ", clearance)
    print("Running A* Algorithm...")


    _planner = planner()
    if _planner.plan(RPM1, RPM2, start_node, goal_node, int(clearance)): #if A* Algorithm is successf
        ul,ur,theta = _planner.get_action_set() #get path found
        _planner.view_plan(start_node, goal_node,_planner.get_trajectory()) #view the trajectory 2D
    
        # #Test Case 1:
        # ul = [0.5,0.4,0.3,0.2,0.1,0.0,-0.1,-0.2,-0.3,-0.4,-0.5] #in m/s
        # ur = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] #in m/s
        # theta = [0.5,0.4,0.3,0.2,0.1,0.0,-0.1,-0.2,-0.3,-0.4,-0.5] #in rad

        #Process A* Output:
        print("Processing A* Output...")
        ul, ur, theta = p1.vel_process_opencv_to_ROS(ul, ur, theta) #convert to ROS coordinate system and m/s

        #Convert wheel velocity to robot velocity:
        print("Converting wheel velocity to robot velocity...")
        lin_vel, ang_vel = p1.convert_wheel_vel_to_robot_vel(ul, ur, theta)
        
        # # Check velocity limit:
        # print("Checking velocity limit for Turtlebot...")
        # lin_vel = p1.check_linear_limit_velocity(lin_vel)
        # ang_vel = p1.check_angular_limit_velocity(ang_vel)
        
        # #Test Case 2:
        # lin_vel = [0.5,0.4,0.3,0.2,0.1,0.0,-0.1,-0.2,-0.3,-0.4,-0.5] #in m/s
        # ang_vel = [0.5,0.4,0.3,0.2,0.1,0.0,-0.1,-0.2,-0.3,-0.4,-0.5] #in rad/s

        #Publish velocity:
        print("Publishing velocity...")
        p1.ros_pub(lin_vel, ang_vel)  #publish velocity to Turtlebot
        print("Velocity published")

    
    rclpy.spin(p1.node) #spin the node
    p1.node.destroy_node() #destroy the node explicitly
    rclpy.shutdown() #shutdown ros2


if __name__ == '__main__':
    main()