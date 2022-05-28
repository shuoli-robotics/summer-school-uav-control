#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

import rospy
import math
from math import sin,cos
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3,TwistStamped,Vector3Stamped
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from mavros_msgs.msg import AttitudeTarget
from mavgnc.mavgnc_base import MavGNCBase
from time import time
import geometry_msgs


class MavGNCPositionControl(MavGNCBase):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    def __init__(self):
        super(MavGNCPositionControl,self).__init__()
        self.ready_to_takeoff = False
        self.mission_finish = False
        self.mission_ready= False
        self.att = AttitudeTarget() 
        self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.vel_setpoint_pub = rospy.Publisher('mavgnc/velocity_setpoint', TwistStamped, queue_size=1)
        self.pos_setpoint_pub = rospy.Publisher('mavgnc/position_setpoint', PoseStamped, queue_size=1)
        self.time_pub = rospy.Publisher('mavgnc/time', Float32, queue_size=1)
        self.odom_sub = rospy.Subscriber('mavros/local_position/odom',Odometry,self.odometry_cb)
        self.att_sp_euler_pub = rospy.Publisher('mavgnc/att_sp_euler',Vector3Stamped,queue_size=1)
        self.att_euler_pub = rospy.Publisher('mavgnc/att_euler',Vector3Stamped,queue_size=1)
        

        
        #---------------------------------------------------------------------------
        # Structures that need to be published. They are mainly commands/setpoints of each loop
        self.position_setpoint = PoseStamped() 
        self.velocity_setpoint = TwistStamped() 
        self.att_setpoint_euler = Vector3Stamped()
        self.attitude_euler = Vector3Stamped()
        #---------------------------------------------------------------------------
        # They are used to store the quadrotor's current measured states 
        self.current_position = np.array((3,))
        self.current_velocity= np.array((3,))
        self.current_attitude = np.array((3,))
        #---------------------------------------------------------------------------
        # They are used to store commands for low-level controller
        # We, as human, are familiar with Eulers but PX4 only accepts quaternions.
        # So, we have to first store attitude commands to these variables and then converted
        # them to quaternions
        self.phi_cmd = 0.0
        self.theta_cmd = 0.0
        self.psi_cmd = 0.0
        self.thrust_cmd = 0.0
        #---------------------------------------------------------------------------
        # PID controllers' parameters. You need to tune them
        self.kp_vx = 0.0
        self.kp_vy = -0.0
        self.kp_vz = 0.0

        self.ki_vx = 0.0
        self.ki_vy = 0.0
        self.ki_vz = 0.0

        self.kd_vx = 0.0
        self.kd_vy = -0.0
        self.kd_vz = 0.0

        self.kp_x = 0.0 
        self.kp_y = 0.0 
        self.kp_z = 0.0 
        #---------------------------------------------------------------------------
        # These two variables are used for D and I control of the velocity loop
        self.vel_err_sum = np.zeros((3,))
        self.vel_err_last_step = np.zeros((3,))
        #---------------------------------------------------------------------------
        # Create a thread ONLY to publish variables
        self.att_thread = Thread(target=self.send_att, args=())
        self.att_thread.daemon = True
        self.att_thread.start()
        self.loop_freq = 200
        self.loop_rate = rospy.Rate(self.loop_freq)

        self.time_init = time()
        self.current_time = Float32()
        self.current_time.data = .0



    def odometry_cb(self,data):
        # update the quadrotors latest states 
        self.current_position = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
        self.current_velocity = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z])
        self.current_attitude = np.array(euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]))
        
        
    def send_att(self):
        # Runs in another thread ONLY for publishing variables
        rate = rospy.Rate(200)  # Hz
        self.att.header = Header()
        self.att.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.att_setpoint_pub.publish(self.att)
            self.vel_setpoint_pub.publish(self.velocity_setpoint)
            self.pos_setpoint_pub.publish(self.position_setpoint)
            self.att_euler_pub.publish(self.attitude_euler)
            self.att_sp_euler_pub.publish(self.att_setpoint_euler)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        pass


    def run(self):
        # Main control loop
        # Important
        while True:
            self.current_time.data = time()-self.time_init
            self.time_pub.publish(self.current_time)
            if not self.ready_to_takeoff:
                self.takeoff_preparation()
            else:
                self.planner()

                self.position_control()

                # Hack the control loop here. You can add the velocity commands/setpoints to self.velocity_setpoint.twist.linear.x
                self.velocity_control()

                # Hack the control loop here. You can add the thrust commands to self.att.orientation and self.att.thrust

            self.loop_rate.sleep()



    def takeoff_preparation(self):
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)
        self.ready_to_takeoff = True
        return True

    def velocity_control(self):

        #---------------------------------------------------------------------------------------------------
        # Please write your PID controller here
        # Your PIDs should output phi, theta and thrust command to self.phi_cmd, self.theta_cmd, self.thrust_cmd, respectively.
        # Be aware that the velocity error should be converted to the local frame
        # The related controller parameters are self.kp_v*, self.kd_v*, self.ki_v*. * = x,y,z
        # TIPS: the quadrotor's current velocities are stored in self.current_velocity and its current attitudes are in self.current_attitude
        # in the sequence of [phi,theta,psi]
        vel_err = np.zeros((3,)) # You need to change this line when you write your own controller


        

        #---------------------------------------------------------------------------------------------------


        # Bound the commands in case they are too large
        self.theta_cmd = self.bound(self.theta_cmd,-0.42,0.42)
        self.phi_cmd = self.bound(self.phi_cmd,-0.62,0.62)
        self.thrust_cmd = self.bound(self.thrust_cmd,0,0.9)
        
        # convert the attitude commands to quaternions because the PX4 accepts quaternions in stead of Eulers 
        # and also prepare the data for the structure to be sent by the publisher
        self.att.orientation = Quaternion(*quaternion_from_euler(self.phi_cmd,self.theta_cmd,self.psi_cmd))
        self.att.thrust = self.thrust_cmd
        self.att.header.stamp = rospy.Time.now()
        self.att.body_rate = Vector3()
        self.att.type_mask = 7 # ignore rate

        # Prepare the attitude commands in Euler to be sent to rqt_plot for debugging
        self.att_setpoint_euler.vector.x = self.phi_cmd/3.14*180
        self.att_setpoint_euler.vector.y = self.theta_cmd/3.14*180
        self.att_setpoint_euler.vector.z = self.psi_cmd/3.14*180
        self.att_setpoint_euler.header.stamp = rospy.Time.now()

        # Prepare the current attitude in Euler to be sent to rqt_plot for debugging
        self.attitude_euler.vector.x = self.current_attitude[0]/3.14*180
        self.attitude_euler.vector.y = self.current_attitude[1]/3.14*180
        self.attitude_euler.vector.z = self.current_attitude[2]/3.14*180
        self.attitude_euler.header.stamp = rospy.Time.now()

        # Store the last step velocity error for D control and I control
        self.vel_err_last_step = vel_err



    def position_control(self):

        #---------------------------------------------------------------------------------------------------
        # Please write your PID controllers here
        # Your PIDs should output vx, vy and vz command to self.velocity_setpoint.twist.linear.x, etc 
        # TIPS: the quadrotor's current positions are stored in self.current_position
        # The related controller parameters are kp_*, * = x,y,z. You can also add I term and D term if you think 
        # it is necessary




        self.psi_cmd = 0.0
        #---------------------------------------------------------------------------------------------------

        # Prepare position setpoint data to be published for debugging
        self.position_setpoint.header.stamp = rospy.Time.now() 
        self.position_setpoint.header.frame_id = 'odom'
        self.velocity_setpoint.header.stamp = rospy.Time.now()  
        self.velocity_setpoint.header.frame_id = 'odom'


    def bound(self,data,min_value,max_value):
        if data >=max_value:
            data = max_value
        elif data <=min_value:
            data = min_value
        return data

    def planner(self):
        # You should write your planner/guidance method here. The result position setpoints should be written to self.position_setpoint.pose.position.x, etc
        pass




