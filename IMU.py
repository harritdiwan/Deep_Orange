#!/usr/bin/env python

# Author: Harrit Diwan
# Date: February 3, 2017
#

import sensor_msgs.msg
import rospy
import serial,io
import sys
import geometry_msgs.msg
import std_msgs.msg
import numpy as np

class IMUNode():

    def __init__(self):
        self.imuData = sensor_msgs.msg.Imu()
        #self.imuTopic = "vehicle/imu/data_raw"
        self.imuTopic = "auk/fcu/imu"
        self.imu_pub = rospy.Publisher(self.imuTopic, sensor_msgs.msg.Imu, queue_size=10)
        #rospy.Subscriber(self.imuTopic, sensor_msgs.msg.Imu, self.imuCallback)

    def extractIMUdata(self):
        self.imuData = sensor_msgs.msg.Imu()

        ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=9600,
        )

        sio = io.TextIOWrapper(
        io.BufferedRWPair(ser, ser, 1),
        encoding='ascii', newline='\r'
        )

        orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        linear_acceleration_covariance = [.0001, 0, 0, 0, .0001, 0, 0, 0, .0001]
        angular_velocity_covariance = [.0068, 0, 0, 0, .0068, 0, 0, 0, .0068]
        K_x = 0
        K_y = 0
        K_z = 0

        while ser.isOpen():
            ornt = geometry_msgs.msg.Quaternion()
            accl = geometry_msgs.msg.Vector3()
            ang_vel = geometry_msgs.msg.Vector3()
            line = sio.readline()
            if "x_g" in line:
                ang_vel.x = ((float(line.split('=')[1].split('m')[0])*0.01745/1000) - K_x)/0.005
                K_x = float(line.split('=')[1].split('m')[0])*0.01745/1000
                self.imuData.angular_velocity.x = ang_vel.x
            if "y_g" in line:
                ang_vel.y = ((float(line.split('=')[1].split('m')[0])*0.01745/1000) - K_y)/0.005
                K_y = float(line.split('=')[1].split('m')[0])*0.01745/1000
                self.imuData.angular_velocity.y = ang_vel.y
            if "z_g" in line:
                ang_vel.z = ((float(line.split('=')[1].split('m')[0])*0.01745/1000) - K_z)/0.005
                K_z = float(line.split('=')[1].split('m')[0])*0.01745/1000
                self.imuData.angular_velocity.z = ang_vel.z
            if "x_q" in line:
                ornt.x = float(line.split('=')[1])
                self.imuData.orientation.x = ornt.x
                #self.imuData.orientation.x = 0
            if "y_q" in line:
                ornt.y = float(line.split('=')[1])
                self.imuData.orientation.y = ornt.y
                #self.imuData.orientation.y = 0
            if "z_q" in line:
                ornt.z = float(line.split('=')[1])
                self.imuData.orientation.z = ornt.z
                #self.imuData.orientation.z = 0
            if "w_q" in line:
                ornt.w = float(line.split('=')[1])
                self.imuData.orientation.w = ornt.w
                #self.imuData.orientation.w = 0
            if "x_a" in line:
                accl.x = float(line.split('=')[1].split('m')[0])/100
                self.imuData.linear_acceleration.x = accl.x
            if "y_a" in line:
                accl.y = float(line.split('=')[1].split('m')[0])/100
                self.imuData.linear_acceleration.y = accl.y
            if "z_a" in line:
                accl.z = float(line.split('=')[1].split('m')[0])/100
                #accl.z =
                self.imuData.linear_acceleration.z = accl.z

            self.imuData.orientation_covariance = orientation_covariance
            self.imuData.angular_velocity_covariance = angular_velocity_covariance
            self.imuData.linear_acceleration_covariance = linear_acceleration_covariance

            self.imuData.header = std_msgs.msg.Header()
            self.imuData.header.frame_id = 'auk'
            self.imuData.header.stamp = rospy.Time.now()
            self.imu_pub.publish(self.imuData)

        ser.close()
        #print(self.imuData)

        return


    def IMUModule(self):
        rospy.init_node('imunode', log_level= rospy.INFO)
        rate = rospy.Rate(200) # 10hz

        while not rospy.is_shutdown():
            self.extractIMUdata()

            rate.sleep()

def main(args):
    pm = IMUNode()
    pm.IMUModule()

    print('Destroying all the windows...')

if __name__ == '__main__':
    main(sys.argv)
