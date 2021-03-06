#!/usr/bin/env python
# license removed for brevity


# This file is a basic structure to write a controller that
# communicates with ROS. It will be the students responsibility
# tune the gains and fill in the missing information

# As an example this file contains PID gains, but other
# controllers use different types of gains so the class
# will need to be modified to accomodate those changes

import rospy
import time
from whirlybird_msgs.msg import Command
from whirlybird_msgs.msg import Whirlybird
from std_msgs.msg import Float32
from time import sleep
import numpy as np
import control


class Controller():

    def __init__(self):



        # get parameters
        try:
            param_namespace = '/whirlybird'
            self.param = rospy.get_param(param_namespace)
        except KeyError:
            rospy.logfatal('Parameters not set in ~/whirlybird namespace')
            rospy.signal_shutdown('Parameters not set')


        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']
        Jm = m1*l1**2 + m2*l2**2
        self.Fe = (m1*l1 - m2*l2)*g/l1 #Note this is not the correct value for Fe, you will have to find that yourself



        self.A_lat = [[0,0,1,0],[0,0,0,1],[0,0,0,0],[l1*self.Fe/(Jm+Jz),0,0,0]]
        self.B_lat = [[0],[0],[1/Jx],[0]]
        self.C_lat = [[1,0,0,0],[0,1,0,0]]
        if 4 != np.linalg.matrix_rank(control.ctrb(self.A_lat,self.B_lat)):
            print 'Not controlable!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'



        self.A_lon = [[0,1],[(m1*l1-m2*l2)*g*np.sin(0)/(Jm + Jy),0]]
        self.B_lon = [[0],[l1/(Jm + Jy)]]
        self.C_lon = [[1,0]]
        if 2 != np.linalg.matrix_rank(control.ctrb(self.A_lon,self.B_lon)):
            print 'Not controlable!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'

        # Pitch Gains
        t_r_theta = 1.4
        zeta_theta = 0.707
        w_n_theta = 2.2/t_r_theta
        poles_lon = np.roots([1,2*zeta_theta*w_n_theta,w_n_theta**2])
        self.K_lon = control.place(self.A_lon,self.B_lon,poles_lon)
        self.kr_lon = -1/(np.matmul(self.C_lon,np.matmul(np.linalg.inv(np.subtract(self.A_lon,np.matmul(self.B_lon,self.K_lon))),self.B_lon)))

        # Yaw Gains
        t_r_phi = 0.3
        zeta_phi = 0.707
        M = 5
        t_r_psi = M*t_r_phi
        zeta_psi = 0.707
        w_n_phi = 2.2/t_r_phi
        w_n_psi = 2.2/t_r_psi
        phi_poles = np.roots([1,2*zeta_phi*w_n_phi,w_n_phi**2])
        psi_poles = np.roots([1,2*zeta_psi*w_n_psi,w_n_psi**2])
        poles_lat = []
        for pole in phi_poles:
            poles_lat.append(pole)
        for pole in psi_poles:
            poles_lat.append(pole)
        self.K_lat = control.place(self.A_lat,self.B_lat,poles_lat)
        self.kr_lat = -1/(np.matmul(self.C_lat[1],np.matmul(np.linalg.inv(np.subtract(self.A_lat,np.matmul(self.B_lat,self.K_lat))),self.B_lat)))

        # Dirty Derivative gains
        self.sigma_theta = 0.05
        self.sigma_phi = 0.05
        self.sigma_psi = 0.05

        # Initialize
        self.thetao = 0
        self.phio = 0
        self.psio = 0
        self.theta_doto = 0
        self.phi_doto = 0
        self.psi_doto = 0

        self.prev_time = rospy.Time.now()

        self.sat_low = 0.0
        self.sat_high = 0.7

        self.command_sub_ = rospy.Subscriber('whirlybird', Whirlybird, self.whirlybirdCallback, queue_size=5)
        self.psi_r_sub_ = rospy.Subscriber('psi_r', Float32, self.psiRCallback, queue_size=5)
        self.theta_r_sub_ = rospy.Subscriber('theta_r', Float32, self.thetaRCallback, queue_size=5)
        self.command_pub_ = rospy.Publisher('command', Command, queue_size=5)
        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def thetaRCallback(self, msg):
        self.theta_r = msg.data


    def psiRCallback(self, msg):
        self.psi_r = msg.data


    def whirlybirdCallback(self, msg):
        g = self.param['g']
        l1 = self.param['l1']
        l2 = self.param['l2']
        m1 = self.param['m1']
        m2 = self.param['m2']
        d = self.param['d']
        h = self.param['h']
        r = self.param['r']
        Jx = self.param['Jx']
        Jy = self.param['Jy']
        Jz = self.param['Jz']
        km = self.param['km']

        phi = msg.roll
        theta = msg.pitch
        psi = msg.yaw

        # Calculate dt (This is variable)
        now = rospy.Time.now()
        dt = (now-self.prev_time).to_sec()
        self.prev_time = now

        ##################################
        # Implement your controller here

        self.theta_doto = self.dirty_derivative(theta,self.thetao,self.theta_doto,self.sigma_theta,dt)
        self.phi_doto = self.dirty_derivative(phi,self.phio,self.phi_doto,self.sigma_phi,dt)
        self.psi_doto = self.dirty_derivative(psi,self.psio,self.psi_doto,self.sigma_psi,dt)
        self.thetao = theta
        self.phio = phi
        self.psio = psi

        x_lon = [[theta],[self.theta_doto]]
        x_lat = [[phi],[psi],[self.phi_doto],[self.psi_doto]]

        F_unsat = self.SS(self.K_lon,x_lon,self.kr_lon,self.theta_r) + (m1*l1 - m2*l2)*g/l1*np.cos(theta)
        Tau_unsat = self.SS(self.K_lat,x_lat,self.kr_lat,self.psi_r)

        sat = self.saturate(F_unsat,Tau_unsat)

        F_sat = sat[0]
        Tau_sat = sat[1]

        f_sat = self.F2f(F_sat,Tau_sat)
        f_l_sat = f_sat[0]
        f_r_sat = f_sat[1]

        l_out = self.f2pwm(f_l_sat)
        r_out = self.f2pwm(f_r_sat)

        ##################################

        # Pack up and send command
        command = Command()
        command.left_motor = l_out
        command.right_motor = r_out
        self.command_pub_.publish(command)

    def PID(self,x,xr,kp,ki,kd,dt,sigma,xdot_lim,uio=0,udo=0,eo=0,xo=0):

        # error
        e = xr - x

        # for simplicity
        beta = (2*sigma - dt)/(2*sigma + dt)

        # coeficent
        up = e
        ud = beta*udo - (1-beta)/dt*(x-xo)


        if (ud < np.abs(xdot_lim)): # and flag:
            ui = uio + dt/2*(e + eo)
        else:
            ui = uio


        # PID
        u = kp*up + ki*ui + kd*ud


        # Return result
        output = {'u':u,'ui':ui,'ud':ud,'e':e,'x':x}
        return output

    def SS(self,K,x,kr,r):
        u = kr*r-np.matmul(K,x)
        return u

    def saturate(self,F,Tau):

        individual_forces = self.F2f(F,Tau)
        l_f_unsat = individual_forces[0]
        r_f_unsat = individual_forces[1]

        l_pwm_unsat = self.f2pwm(l_f_unsat)
        r_pwm_unsat = self.f2pwm(r_f_unsat)

        if(l_pwm_unsat < self.sat_low):
            l_pwm_sat = self.sat_low
        elif(l_pwm_unsat > self.sat_high):
            l_pwm_sat = self.sat_high
        else:
            l_pwm_sat = l_pwm_unsat
        if(r_pwm_unsat < self.sat_low):
            r_pwm_sat = self.sat_low
        elif(r_pwm_unsat > self.sat_high):
            r_pwm_sat = self.sat_high
        else:
            r_pwm_sat = r_pwm_unsat

        l_f_sat = self.pwm2f(l_pwm_sat)
        r_f_sat = self.pwm2f(r_pwm_sat)

        combined_forces = self.f2F(l_f_sat,r_f_sat)

        return combined_forces


        l_f_sat = self.pwm2f(l_pwm_sat)
        r_f_sat = self.pwm2f(r_pwm_sat)


    def f2pwm(self,f):
        km = self.param['km']
        pwm = f/km
        return pwm

    def pwm2f(self,pwm):
        km = self.param['km']
        f = pwm*km
        return f

    def F2f(self,F,Tau):
        d = self.param['d']

        l_f = F/2.0 - Tau/(2.0*d)
        r_f = F/2.0 + Tau/(2.0*d)

        out = [l_f,r_f]
        return out

    def f2F(self,fl,fr):
        d = self.param['d']

        F = fl + fr
        Tau = fl*d - fr*d
        out = [F,Tau]
        return out

    def dirty_derivative(self,x,xo,xdo,sigma,dt):
        beta = (2*sigma - dt)/(2*sigma + dt)

        xd = beta*xdo + (1-beta)/dt*(x-xo)
        return xd


if __name__ == '__main__':

    rospy.init_node('SScontroller', anonymous=True)
    try:
        controller = Controller()

    except:
        rospy.ROSInterruptException
    pass
