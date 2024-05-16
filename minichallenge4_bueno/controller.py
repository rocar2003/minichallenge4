#!/usr/bin/env python3 
import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
import numpy as np 
from rclpy.qos import qos_profile_sensor_data

#This class will compute the pose of the robot from the encoder readings. 
# This node subscribes to the /VelocityEncR and /VelocityEncL topics 
# This node publishes the pose of the robot to the /pose topic.  
class Odometry(Node):  

    def __init__(self):  
        super().__init__('odometry_node') 

        ###########  INIT PUBLISHERS ################ 
        self.pub_pose = self.create_publisher(Pose2D, 'pose', 10)  
        self.pub_vel= self.create_publisher(Twist, 'cmd_vel_aux', 10)  


        ############## SUBSCRIBERS ##################  
        self.create_subscription(Float32, "VelocityEncR",  self.wr_cb, qos_profile_sensor_data)  
        self.create_subscription(Float32, "VelocityEncL",  self.wl_cb, qos_profile_sensor_data) 
        #self.create_subscription(String, "/status", self.status_cb, 10)

        ############## PARAMETERS ###################
        self.declare_parameters(
            namespace='',
            parameters=[
                ('p0.x', rclpy.Parameter.Type.DOUBLE),
                ('p0.y', rclpy.Parameter.Type.DOUBLE),
                ('p1.x', rclpy.Parameter.Type.DOUBLE),
                ('p1.y', rclpy.Parameter.Type.DOUBLE),
                ('p2.x', rclpy.Parameter.Type.DOUBLE),
                ('p2.y', rclpy.Parameter.Type.DOUBLE),
                ('p3.x', rclpy.Parameter.Type.DOUBLE),
                ('p3.y', rclpy.Parameter.Type.DOUBLE),
                ('p4.x', rclpy.Parameter.Type.DOUBLE),
                ('p4.y', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        ############ ROBOT CONSTANTS ################ 
        self.r=0.05 #wheel radius for our simulated robot[m] 
        self.L=0.19 #wheel separation for our simulated robot [m] 
        self.wl = 0.0 #Left wheel speed [rad/s] 
        self.wr = 0.0 #Right wheel speed [rad/s] 
        self.x = 0.0 #Robot position in x-axis [m] 
        self.y = 0.0 #Robot position in y-axis [m] 
        self.theta = 0.0 #Robot orientation [m] 
        self.robot_pose = Pose2D() 
        self.vel = Twist()
        self.Kp_linear = 0.05
        self.Kp_angular = 2.20
        self.cont = 0

        ########### ACTIVITY 2 ############ 
        timer_period = 0.1 
        self.create_timer(timer_period, self.main_timer_cb) 
        self.get_logger().info("Node initialized!!") 
        self.first_time = True 

        self.state = "move"

    def main_timer_cb(self):
        if self.state == "stop":
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            self.get_logger().info("Deteniendo el robot")
        
        elif self.state == "move":
            self.get_logger().info("Moviendo el robot")
            if self.first_time: 
                self.previous_time=self.get_clock().now().nanoseconds 
                self.first_time=False 

            else: 
                self.current_time = self.get_clock().now().nanoseconds 
                self.dt= float(self.current_time - self.previous_time)/(10.0**9) #dt is in [seconds] 
                self.previous_time = self.current_time 

                [self.v, self.w] = self.get_robot_vel(self.wr, self.wl) 

                #Get the robot pose  
                self.x = self.x + self.v*np.cos(self.theta)*self.dt
                self.y = self.y + self.v*np.sin(self.theta)*self.dt
                self.theta = self.theta + self.w*self.dt

                #Fill the ros Pose2D message with the data 
                self.robot_pose.x = self.x 
                self.robot_pose.y = self.y 
                self.robot_pose.theta =  self.theta 

                print("----------------------") 

                #print("x:", self.robot_pose.x) 
                #print("y:", self.robot_pose.y) 
                #print("theta:", self.robot_pose.theta) 
                ####ACTIVITY 2 
                if self.cont < 5:
                    self.xG = self.get_parameter('p' + str(self.cont) + '.x').get_parameter_value().double_value
                    self.yG = self.get_parameter('p' + str(self.cont) + '.y').get_parameter_value().double_value


                else:
                    self.state = "stop"

                if np.isclose(self.x, self.xG, atol=0.1) and np.isclose(self.y, self.yG, atol=0.1):
                    self.cont += 1

                ed = np.sqrt((self.xG-self.x)*2+(self.yG-self.y)*2) 
                thetaG = np.arctan2((self.yG-self.y),(self.xG-self.x)) 
                e_theta = thetaG-self.theta 
                e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta)) # Limit the angle to [-pi,pi] 

                print("ed: ", ed) 
                print("e_theta: ", e_theta) 
                print("contador", self.cont)

                V = self.Kp_linear * ed
                w = self.Kp_angular * e_theta

                self.vel.angular.z = w
                self.vel.linear.x = V

                self.pub_pose.publish(self.robot_pose) #publish the robot's speed  
                self.pub_vel.publish(self.vel) #publish the robot's speed

    '''def status_cb(self, state):
        self.get_logger().info("Nuevo estado recibido: %s" % state.data)
'''
    def wl_cb(self, wl):  
        ## This function receives the left wheel speed from the encoders  
        self.wl = wl.data  

    def wr_cb(self, wr):  
        ## This function receives the right wheel speed from the encoders 
        self.wr = wr.data 

    def get_robot_vel(self, wr, wl): 
        v= self.r *(wr+wl) / 2.0 
        w = (self.r/self.L)*(wr-wl) 
        return [v,w] 
    


def main(args=None): 
    rclpy.init(args=args) 
    my_node=Odometry() 
    rclpy.spin(my_node) 
    my_node.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()