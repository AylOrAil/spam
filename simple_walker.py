#!/usr/bin/env python3

import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import time 

#Problems right now:
#SOLVED-Time initialization and switching from stand to prepare 
#SOLVED- Being able to maintain prepared state 
#Create timer for keeping track of the periodical walk and duty cycle 


class SimpleWalker(Node):
    def __init__(self):
        super().__init__('simple_walker')
        
        # PARAMETERS
        self.declare_parameter('state', 0)

        self.declare_parameter('simple_walker_enable', False)
        self.declare_parameter('cmd_tau', [0.0]*6)
        self.declare_parameter('cmd_vel', [0.0]*6)
        self.declare_parameter('cmd_pos', [0.0]*6)
        self.declare_parameter('cmd_kp', [0.0]*6)
        self.declare_parameter('cmd_kd', [0.0]*6)
        
        
        
        # VARIABLES
        
        self.newdata = False
        
        self.start_time = time.time()
        self.simulation_speedup = 1.516  
        
        self.simple_walker_enable = False
        
        self.state = 0
        
        self.cmd_tau = [0.0] * 6  # Initialize as a list of zeros with length 6
        self.cmd_pos = [0.0] * 6
        self.cmd_vel = [0.0] * 6

        self.cmd_kp = [1.65, 1.65, 1.65, 1.65, 1.65, 1.65]
        self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
        
        self.currPos = np.zeros(6)
        self.currVel = np.zeros(6)
        self.currTorq = np.zeros(6)
        self.currPose = np.zeros(4)
        self.globalPos = np.zeros(3)
        
        self.timer_counter = 0
        
        # TOPICS
        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        self.Odometry_Subscriber_ = self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints_Subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subIMU_Subscriber_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        
        # TIMERS
        self.create_timer(0.001, self.run)  
        self.get_logger().info("**************SimpleWalker initialized****************")
        
        self.state = 0  # Initial state)
        

                
    def callback_position(self, msg):
        self.globalPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def joint_state_callback(self, msg):
        self.currPos = constrain_angle(np.array([*msg.position]))
        self.currVel = np.array([*msg.velocity])
        self.currTorq = np.array([*msg.effort])
        self.currPos = self.currPos[[4, 2, 0, 5, 3, 1]]     
        self.currVel = self.currVel[[4, 2, 0, 5, 3, 1]]
        self.currTorq = self.currTorq[[4, 2, 0, 5, 3, 1]]
        
        self.newdata = True  

    def imu_callback(self, msg):
        self.currPose = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.newdata = True 
        
    def print_joint_state(self):
        self.get_logger().info(f"current pose:{self.currPos}")



    def compute_controls(self):
        cmd_tau = np.array(self.cmd_tau)
        cmd_vel = np.array(self.cmd_vel)
        cmd_pos = np.array(self.cmd_pos)
        cmd_kp = np.array(self.cmd_kp)
        cmd_kd = np.array(self.cmd_kd)
        curr_Pos = np.array(self.currPos)
        curr_Vel = np.array(self.currVel)
        curr_Torq = np.array(self.currTorq)
    
        posErr = (cmd_pos) - curr_Pos
        posErr = np.mod(posErr+np.pi, 2*np.pi) - np.pi
        velErr = (cmd_vel) - curr_Vel
        commTorque = np.multiply(cmd_kp, posErr) + np.multiply(cmd_kd, velErr) + cmd_tau
        np.clip(commTorque, -20, 20, out=commTorque)
        
        return list(commTorque[[2, 5, 1, 4, 0, 3]])
    
    
    def run(self):
        
        if (self.simple_walker_enable):
            
            # SIT
            if (self.state == 1):    
                for i, pos in enumerate(self.currPos):
                    if (-3.14  <= pos < 0):
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -1.3
                        self.cmd_vel[i] = 0.0
                        
                    elif (0 <= pos<2.0):
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 3.0
                        self.cmd_vel[i] = 0.0
                        
                    elif (2.0 <= pos <= 3.14):
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 0.0
     
            # SIAND
            if (self.state == 2):    
                for i, pos in enumerate(self.currPos):
                    if -3.14 < pos < -0.4:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.5
                    elif -0.4 <= pos <= 0.4:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif 0.4 <= pos < 1.0:
                        self.cmd_tau[i] = -2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = -0.5
                    elif 1.0 <= pos < 3.14:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 0.5
                    
                        
            # PREPARE
            if (self.state == 3):   
                for i in [2, 4, 0]:
                    pos = self.currPos[i]
                    if 0 <= pos :
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 0.0
                        
                                            
                    elif pos <0:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3.14
                        self.cmd_vel[i] = 0.0
                        
                for i in [1, 3, 5]:
                    pos = self.currPos[i]
                    if -3.14 < pos < -0.4:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.5
                    elif -0.4 <= pos <= 0.4:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif 0.4 <= pos < 1.0:
                        self.cmd_tau[i] = -2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = -0.5
                    elif 1.0 <= pos < 3.14:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 0.5
            
            # CRAWL
            if (self.state == 4):   
                elapsed_time = ((time.time() - self.start_time)) 
                segment = int((elapsed_time ) %8)


                if segment < 2 :
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = -0.2
                        self.cmd_vel[i] = 4.9
                         
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = 0.2
                        self.cmd_vel[i] = 0.33
                        
                elif segment < 4:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.33

                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 4.9
                elif segment < 6:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = 0.2
                        self.cmd_vel[i] = 0.33

                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = -0.2
                        self.cmd_vel[i] = 4.9

                elif segment <9:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 4.9

                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = 3.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.33
            
                        
        self.simple_walker_enable = self.get_parameter('simple_walker_enable').get_parameter_value().bool_value
        self.state = self.get_parameter('state').get_parameter_value().integer_value

        
        if self.newdata:
            torque = Float64MultiArray()
            torque.data = self.compute_controls()
            if (self.simple_walker_enable):
                self.publisher.publish(torque)
            self.newdata = False
        
def main (args = None):
    rclpy.init(args = args)
    node = SimpleWalker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
    
if __name__ == '__main__':
    main()
    


"""
#!/usr/bin/env python3

import rclpy
from miscellaneous import constrain_angle
import numpy as np
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
import rclpy.parameter
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import time 

#Problems right now:
#SOLVED-Time initialization and switching from stand to prepare 
#SOLVED- Being able to maintain prepared state 
#Create timer for keeping track of the periodical walk and duty cycle 


class SimpleWalker(Node):
    def __init__(self):
        super().__init__('simple_walker')
        
        # PARAMETERS
        self.declare_parameter('state', 0)

        self.declare_parameter('simple_walker_enable', False)
        self.declare_parameter('cmd_tau', [0.0]*6)
        self.declare_parameter('cmd_vel', [0.0]*6)
        self.declare_parameter('cmd_pos', [0.0]*6)
        self.declare_parameter('cmd_kp', [0.0]*6)
        self.declare_parameter('cmd_kd', [0.0]*6)
        
        
        
        # VARIABLES
        
        self.newdata = False
        
        self.start_time = time.time()
        self.simulation_speedup = 1.516  
        
        self.simple_walker_enable = False
        
        self.state = 0
        
        self.cmd_tau = [0.0] * 6  # Initialize as a list of zeros with length 6
        self.cmd_pos = [0.0] * 6
        self.cmd_vel = [0.0] * 6

        self.cmd_kp = [0.65, 0.65, 0.65, 0.65, 0.65, 0.65]
        self.cmd_kd = [0.35, 0.35, 0.35, 0.35, 0.35, 0.35]
        
        self.currPos = np.zeros(6)
        self.currVel = np.zeros(6)
        self.currTorq = np.zeros(6)
        self.currPose = np.zeros(4)
        self.globalPos = np.zeros(3)
        
        self.timer_counter = 0
        
        # TOPICS
        self.publisher = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        self.Odometry_Subscriber_ = self.create_subscription(Odometry, '/odom/robot_pos', self.callback_position, 10)
        self.subJoints_Subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.subIMU_Subscriber_ = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        
        # TIMERS
        self.create_timer(0.01, self.run)  
        self.get_logger().info("**************SimpleWalker initialized****************")
        
        self.state = 0  # Initial state)
        

                
    def callback_position(self, msg):
        self.globalPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def joint_state_callback(self, msg):
        self.currPos = constrain_angle(np.array([*msg.position]))
        self.currVel = np.array([*msg.velocity])
        self.currTorq = np.array([*msg.effort])
        self.currPos = self.currPos[[4, 2, 0, 5, 3, 1]]     
        self.currVel = self.currVel[[4, 2, 0, 5, 3, 1]]
        self.currTorq = self.currTorq[[4, 2, 0, 5, 3, 1]]
        
        self.newdata = True  

    def imu_callback(self, msg):
        self.currPose = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.newdata = True 
        
    def print_joint_state(self):
        self.get_logger().info(f"current pose:{self.currPos}")



    def compute_controls(self):
        cmd_tau = np.array(self.cmd_tau)
        cmd_vel = np.array(self.cmd_vel)
        cmd_pos = np.array(self.cmd_pos)
        cmd_kp = np.array(self.cmd_kp)
        cmd_kd = np.array(self.cmd_kd)
        curr_Pos = np.array(self.currPos)
        curr_Vel = np.array(self.currVel)
        curr_Torq = np.array(self.currTorq)
    
        posErr = (-cmd_pos) - curr_Pos
        posErr = np.mod(posErr+np.pi, 2*np.pi) - np.pi
        velErr = (-cmd_vel) - curr_Vel
        commTorque = np.multiply(cmd_kp, posErr) + np.multiply(cmd_kd, velErr) - cmd_tau
        np.clip(commTorque, -20, 20, out=commTorque)
        
        return list(commTorque[[2, 5, 1, 4, 0, 3]])
    
    
    def run(self):
        
        if (self.simple_walker_enable):
            
            # SIT
            
            if (self.state == 1):    
                for i, pos in enumerate(self.currPos):
                    if (-3.14  <= pos < -1.8):
                        self.cmd_tau[i] = -2
                        self.cmd_pos[i] = -1.8
                        self.cmd_vel[i] = 0.0
                    
                    elif (-1.8 <= pos < -1.4):
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -1.57
                        self.cmd_vel[i] = 0.0
                    
                    elif (-1.4 <= pos < 0):
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = -1.4
                        self.cmd_vel[i] = 0.0
                        
                    elif (0<= pos < 3.14):
                        self.cmd_tau[i] = -2
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 0.0
                        
            
            # SIAND
            
            
            if (self.state == 2):    
                for i, pos in enumerate(self.currPos):
                    if 0.5 < pos < 1.2:
                        self.cmd_tau[i] = 0.4
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif 0 <= pos <= 0.5:
                        self.cmd_tau[i] = 0.5
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif -0.25 <= pos <0:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif pos < -0.25:
                        self.cmd_tau[i] = -1.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    else:
                        self.cmd_tau[i] = -1.5
                        self.cmd_pos[i] = 3.13
                        self.cmd_vel[i] = 0.0
                        
            # PREPARE
            
            if (self.state == 3):   
                for i in [2, 4, 0]:
                    pos = self.currPos[i]
                    if 0.5>pos >0:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3.14
                        self.cmd_vel[i] = 0.0
                        
                    elif pos >= 0.5:
                        self.cmd_tau[i] = -1.5
                        self.cmd_pos[i] = -3.14
                        self.cmd_vel[i] = 0.0
                                            
                    elif -1.8<=pos <0:
                        self.cmd_tau[i] = 1.0
                        self.cmd_pos[i] = -3.14
                        self.cmd_vel[i] = 0.0
                        
                    elif pos < -1.8:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = -3.14
                        self.cmd_vel[i] = 0.0

                for i in [1, 3, 5]:
                    pos = self.currPos[i]
                    if 0 <= pos <= 0.6:
                        self.cmd_tau[i] = 2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif -0.25 <= pos <0:
                        self.cmd_tau[i] = 0.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    elif pos < -0.25:
                        self.cmd_tau[i] = -2.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.0
                    else:
                        self.cmd_tau[i] = -1.5
                        self.cmd_pos[i] = 3.0
                        self.cmd_vel[i] = 0.0
            
            # WALK
                     
            if (self.state == 4):   
                elapsed_time = ((time.time() - self.start_time)/2) % 4
                segment = int(elapsed_time )  


                if segment == 0:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -4.0
                        self.cmd_pos[i] = -0.3
                        self.cmd_vel[i] = 1.42
                         
                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -6.0
                        self.cmd_pos[i] = 0.3
                        self.cmd_vel[i] = 0.15
                elif segment == 1:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -6.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.15

                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -4.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 1.42
                elif segment == 2:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -4.0
                        self.cmd_pos[i] = 0.3
                        self.cmd_vel[i] = 0.15

                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -4.0
                        self.cmd_pos[i] = -0.3
                        self.cmd_vel[i] = 1.42

                elif segment == 3:
                    for i in [2, 4, 0]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -4.0
                        self.cmd_pos[i] = 3.14
                        self.cmd_vel[i] = 1.42

                    for i in [1, 3, 5]:
                        pos = self.currPos[i]
                        self.cmd_tau[i] = -6.0
                        self.cmd_pos[i] = 0.0
                        self.cmd_vel[i] = 0.15
            
                        
        self.simple_walker_enable = self.get_parameter('simple_walker_enable').get_parameter_value().bool_value
        self.state = self.get_parameter('state').get_parameter_value().integer_value

        
        if self.newdata:
            torque = Float64MultiArray()
            torque.data = self.compute_controls()
            if (self.simple_walker_enable):
                self.publisher.publish(torque)
            self.newdata = False
        
def main (args = None):
    rclpy.init(args = args)
    node = SimpleWalker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        
    
if __name__ == '__main__':
    main()
    
"""