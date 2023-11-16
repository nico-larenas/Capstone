#!/usr/bin/env python3

import rospy
import time
import matplotlib.pylab  as plt
import numpy as np
from numpy               import pi, sqrt, arange
from std_msgs.msg        import Float64, Bool
from geometry_msgs.msg   import Twist, PoseArray, Pose
from nav_msgs.msg        import Odometry
from tf.transformations  import euler_from_quaternion
import serial






class Robot(object):

    def __init__(self):
        self.max_w    = 1.0  # [rad/s]
        self.max_v    = 0.2  # [m/s]
        self.rate_hz  = 45
        self.simulator = True
        rospy.init_node('reckoning_nav')
        self.rate_obj = rospy.Rate(self.rate_hz)
        self.time     = rospy.Time.now()
        self.init_pub()
        self.init_subs()
        self.init_state0()
        self.init_serial()

    def init_subs(self):
        if self.simulator:
            self.odom_sub  = rospy.Subscriber('/odom', Odometry, self.odometry)
        else:
            self.odom_sub  = rospy.Subscriber('/hoverboard_velocity_controller/odom', Odometry, self.odometry)

        # Lectura de datos de controlador
        self.linear_act  = rospy.Subscriber( '/linear_ctrl/control_effort', Float64, self.linear_act )
        self.angular_act = rospy.Subscriber( '/angular_ctrl/control_effort', Float64, self.angular_act )
    
    def init_pub(self):
        if self.simulator:
            self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size = 10)
        else:
            self.cmd_vel_mux_pub = rospy.Publisher('/hoverboard_velocity_controller/cmd_vel', Twist, queue_size = 10)
            
        # Envio de setpoints a controlador
        self.linear_ctrl_pub  = rospy.Publisher( '/linear_ctrl/setpoint', Float64, queue_size = 1 )
        self.angular_ctrl_pub = rospy.Publisher( '/angular_ctrl/setpoint', Float64, queue_size = 1 )

        # Envio de odometria a controlador
        self.linear_state  = rospy.Publisher( '/linear_ctrl/state', Float64, queue_size = 1 )
        self.angular_state = rospy.Publisher( '/angular_ctrl/state', Float64, queue_size = 1 )

        # Apagado de controlador
        self.linear_enabler  = rospy.Publisher( '/linear_ctrl/pid_enable', Bool, queue_size=1)
        self.angular_enabler = rospy.Publisher( '/angular_ctrl/pid_enable', Bool, queue_size=1)

    def init_state0(self):
        self.linear_enabled = False
        self.angular_enabled = False
        self.linear_e = 1
        self.angular_e = 1
        self.x = 0 
        self.y = 0
        self.yaw = 0
        self.x1 = 0
        self.y1 = 0
        self.yaw1 = 0

    def init_serial(self):
        self.ser = serial.Serial("/dev/ttyACM2",baudrate = 9600,timeout = 1)
        self.ser.write(str.encode(f"0"))
        
    # Callback del topico /odom
    def odometry(self, odom): # odom: Odom object
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.z = odom.pose.pose.position.z
        _, _, self.yaw = euler_from_quaternion((odom.pose.pose.orientation.x,
                                                odom.pose.pose.orientation.y,
                                                odom.pose.pose.orientation.z,
                                                odom.pose.pose.orientation.w))
        print('---------')
        print("x:",self.x)
        print("y:",self.y)
        print("z:",self.yaw)
        self.odometry_pid()

    def odometry_pid(self):
        if self.linear_enabled:
            linear_dist = sqrt((self.x-self.x1)**2 + (self.y-self.y1)**2)
            self.linear_state.publish(linear_dist)
        if self.angular_enabled:
            angular_dist = self.yaw - self.yaw1
            self.angular_state.publish(angular_dist)
    
    def desplazar_robot(self, command): # speed_command_list: [[desp_lineal, desp_angular],...]
        self.x1   = self.x
        self.y1   = self.y
        self.yaw1 = self.yaw
        linear_setpoint  = command[0]
        angular_setpoint = command[1]
        if linear_setpoint != 0:
            self.linear_enabled = True

            self.linear_enabler.publish(True)

            self.linear_ctrl_pub.publish(linear_setpoint)
            
            self.linear_e = 1
            while self.linear_e > 0.005 or self.linear_e < -0.005:
                pass
            
            self.linear_enabler.publish(False)
            self.linear_enabled = False
        
        if angular_setpoint != 0:
            #robot.activar_cepillo(1)
            self.angular_enabled = True
            self.angular_enabler.publish(True)

            self.angular_ctrl_pub.publish(angular_setpoint)

            self.angular_e = 1
            while self.angular_e > 0.0005 or self.angular_e < -0.0005:
                pass
            self.angular_enabled = False
            #robot.activar_cepillo(0)
            self.angular_enabler.publish(False)
    
    # Callback del topico /linear_ctrl/control_effort
    def linear_act(self, data):
        self.linear_e = data.data
        speed = Twist()
        speed.linear.x = data.data
        self.cmd_vel_mux_pub.publish(speed)
    
    # Callback del topico /angular_ctrl/control_effort
    def angular_act(self, data):
        self.angular_e = data.data
        speed = Twist()
        speed.angular.z = data.data
        self.cmd_vel_mux_pub.publish(speed)

    def activar_cepillo(self, estado):
        if estado == 1:
            self.ser.write(str.encode(f"150"));
        elif estado == 0:
            self.ser.write(str.encode(f"0"));
    
if __name__ == '__main__':

    robot = Robot()
    print('\n+ Robot inicializado\n')
    time.sleep(2)
     
    # Rutina  
    robot.aplicar_velocidad([1.2,0])
    #robot.aplicar_velocidad([0,np.pi/2])
    
    # robot.activar_cepillo(1)

    
    rospy.spin()