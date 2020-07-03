#!/usr/bin/env python
import rospy
import os,time
import sys,traceback
import roslib
from math import pi as PI, degrees,radians,sin,cos
from threading import Thread 
from geometry_msgs.msg import Twist,Quaternion,Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16,Int32,Float32
from sensor_msgs.msg import Imu
from tf.broadcaster import TransformBroadcaster
from std_srvs.srv import Trigger,TriggerResponse
import XMiddleWare as xmw
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


ODOM_POSE_COVARIANCE = [1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0, 
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e-3]

ODOM_TWIST_COVARIANCE = [1e-9, 0, 0, 0, 0, 0,
                        0, 1e-3, 1e-9, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3]

class XMIDDLEWARE:
    def __init__(self):

        rospy.init_node('XMiddleWare',log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
 
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel',Twist,self.handle_cmd)
        self.odom_pub    = rospy.Publisher('odom',Odometry,queue_size=5)
        self.battery_pub = rospy.Publisher('voltage',Float32,queue_size=5)
        self.imu_pub     = rospy.Publisher('imu_raw',Imu,queue_size=5)
        self.avel_pub    = rospy.Publisher('xtark/avel',Int32,queue_size=5)
        self.aset_pub    = rospy.Publisher('xtark/aset',Int32,queue_size=5)
        self.bvel_pub    = rospy.Publisher('xtark/bvel',Int32,queue_size=5)
        self.bset_pub    = rospy.Publisher('xtark/bset',Int32,queue_size=5)
        self.cvel_pub    = rospy.Publisher('xtark/cvel',Int32,queue_size=5)
        self.cset_pub    = rospy.Publisher('xtark/cset',Int32,queue_size=5)
        self.dvel_pub    = rospy.Publisher('xtark/dvel',Int32,queue_size=5)
        self.dset_pub    = rospy.Publisher('xtark/dset',Int32,queue_size=5)

        self.port_name                 = rospy.get_param('~port_name',"/dev/ttyAMA0")
        self.baud_rate                 = rospy.get_param('~baud_rate',115200)
        self.odom_frame                = rospy.get_param('~odom_frame',"odom")
        self.base_frame                = rospy.get_param('~base_frame',"base_footprint")
        self.imu_frame                 = rospy.get_param('~imu_frame',"base_imu_link")
        self.control_rate              = rospy.get_param('~control_rate',25)
        self.publish_odom_transform    = rospy.get_param('~publish_odom_transform',False)
        self.Kp                        = rospy.get_param('Kp',300)
        self.Ki                        = rospy.get_param('Ki',0)
        self.Kd                        = rospy.get_param('Kd',200)
        self.encoder_resolution        = rospy.get_param('encoder_resolution',1600)
        self.wheel_diameter            = rospy.get_param('wheel_diameter',0.15)
        self.wheel_a_mec               = rospy.get_param('wheel_a_mec',0.15)
        self.wheel_b_mec               = rospy.get_param('wheel_b_mec',0.15)
        self.ax_cm_k                   = rospy.get_param('ax_cm_k',0.08)
        self.linear_correction_factor  = rospy.get_param('linear_correction_factor',1.0)
        self.angular_correction_factor = rospy.get_param('angular_correction_factor',1.0)

        
        self.odom_data     = Odometry()
        self.vel_data      = Twist()
        self.battery_data  = Float32()
        self.wheel_a_speed = Int32()
        self.wheel_b_speed = Int32()
        self.wheel_c_speed = Int32()
        self.wheel_d_speed = Int32()
        self.rate_timer = rospy.Rate(self.control_rate)
        #self.rate_timer = rospy.Rate(25)
        self.serialport = self.port_name
        self.baudrate   = self.baud_rate
        self.x          = xmw.XMiddleWare(self.serialport,self.baudrate)
        self.connect()
    



    def connect(self):
       print(self.x.Init())
       time.sleep(2)

    def getOdom(self):
        return self.x.GetOdom()

    def getIMU(self):
        return self.x.GetIMU()

    def getWheelSpeed(self):
        return self.x.GetWheelSpeed()

    def getBattery(self):
        return self.x.GetBattery()

    def setVelocity(self,x,y,yaw):
        print("sendSpeed!:%f %f %f"%(x,y,yaw))
        self.x.SetVelocity(x,y,yaw)

    def setParams(self,robot_type = 0,encoder_resolution = 1440,wheel_diameter = 0.097,robot_linear_acc = 1.0, robot_angular_acc = 2.0, wheel_track = 0.0,wheel_a_mec = 0.095,wheel_b_mec = 0.075):
        self.x.SetParams(robot_type,encoder_resolution,wheel_diameter,robot_linear_acc,robot_angular_acc,wheel_track,wheel_a_mec,wheel_b_mec)

    def shutdown(self):
        try:
            rospy.loginfo("Robot Stoping")
            self.cmd
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Robot Stopped")

    def handle_cmd(self,req):
        self.setVelocity(req.linear.x,req.linear.y,req.angular.z)
        #print("sendSpeed!:%f %f %f"%(req.linear.x,req.linear.y,req.angular.z))
        #self.x.SetVelocity(req.linear.x,req.linear.y,req.angular.z)


    def loop(self):

        #imu_tmp  = self.getIMU()
        self.odom_data.header.frame_id = self.odom_frame
        self.odom_data.child_frame_id  = self.base_frame
        self.setParams(robot_type=0)
        print("Start Loop")

        #rospy.spin()
        while not rospy.is_shutdown():
            self.rate_timer.sleep()
            odom_list = self.getOdom()
            #print(odom_tmp[0],odom_tmp[1],odom_tmp[2])
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(odom_list[2]/2.0)
            quaternion.w = cos(odom_list[2]/2.0)
            self.odom_data.header.stamp = rospy.Time.now()
            self.odom_data.pose.pose.position.x = odom_list[0]
            self.odom_data.pose.pose.position.y = odom_list[1]
            self.odom_data.pose.pose.position.z = 0
            self.odom_data.pose.pose.orientation = quaternion

            self.odom_data.twist.twist.linear.x = odom_list[3]*self.control_rate
            self.odom_data.twist.twist.linear.y = odom_list[4]*self.control_rate
            self.odom_data.twist.twist.angular.z = odom_list[5]*self.control_rate
            self.odom_data.twist.covariance = ODOM_TWIST_COVARIANCE
            self.odom_data.pose.covariance = ODOM_POSE_COVARIANCE
            self.odom_pub.publish(self.odom_data)
            '''
            self.battery_data.data = self.getBattery()
            self.battery_pub.publish(self.battery_data)

            (self.wheel_a_speed.data, self.wheel_b_speed.data, self.wheel_c_speed.data, self.wheel_d_speed.data) = self.getWheelSpeed()
            self.avel_pub.publish(self.wheel_a_speed)
            self.bvel_pub.publish(self.wheel_b_speed)
            self.cvel_pub.publish(self.wheel_c_speed)
            self.dvel_pub.publish(self.wheel_d_speed)
            '''
            
if __name__ == '__main__':
    robot = XMIDDLEWARE()
    robot.loop()
    




    

    





                    
