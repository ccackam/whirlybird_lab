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


        # Roll Gains
        t_r_phi = 0.3
        zeta_phi = 0.707
        self.sigma_phi = 0.05
        self.phi_r = 0.0
        self.P_phi_ = (2.2/t_r_phi)**2*Jx
        self.I_phi_ = 0.0
        self.D_phi_ = 2*zeta_phi*(2.2/t_r_phi)*Jx
        self.prev_phi = 0.0
        self.Int_phi = 0.0
        self.Dir_phi = 0.0
        self.error_phi = 0.0
        self.phi_vlim = 0.05

        # Pitch Gains
        b_theta = l1/(m1*l1**2 + m2*l2**2 + Jy)
        t_r_theta = 1.4
        zeta_theta = 0.707
        self.sigma_theta = 0.05
        self.theta_r = 0.0
        self.P_theta_ = (2.2/t_r_theta)**2/(b_theta)
        self.I_theta_ = 0.3
        self.D_theta_ = 2*zeta_theta*(2.2/t_r_theta)/(b_theta)
        self.prev_theta = 0.0
        self.Int_theta = 0.0
        self.Dir_theta = 0.0
        self.error_theta = 0.0
        self.theta_vlim = 0.05
        # self.theta_ddot_old = [0.0];
        print self.P_theta_,self.D_theta_

        # Yaw Gains
        M = 5
        t_r_psi = M*t_r_phi
        zeta_psi = 0.707
        Fe = (m1*l1 - m2*l2)*g/l1
        b_psi = l1*Fe/(m1*l1**2 + m2*l2**2 + Jz)
        self.sigma_psi = 0.05
        self.psi_r = 0.0
        self.P_psi_ = (2.2/t_r_psi)**2/(b_psi)
        self.I_psi_ = 0.05
        self.D_psi_ = 2*zeta_psi*(2.2/t_r_psi)/(b_psi)
        self.prev_psi = 0.0
        self.Int_psi = 0.0
        self.Dir_psi = 0.0
        self.error_psi = 0.0
        self.psi_vlim = 0.05

        self.prev_time = rospy.Time.now()

        self.Fe = (m1*l1 - m2*l2)*g/l1 #Note this is not the correct value for Fe, you will have to find that yourself

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


        # Longitudinal
        x = theta
        xr = self.theta_r
        kp = self.P_theta_
        ki = self.I_theta_
        kd = self.D_theta_
        theta_vlim = self.theta_vlim
        sigma = self.sigma_theta
        uio = self.Int_theta
        udo = self.Dir_theta
        eo = self.error_theta
        xo = self.prev_theta

        output = self.PID(x,xr,kp,ki,kd,dt,theta_vlim,sigma,uio,udo,eo,xo)
        F_unsat = output['u'] + (m1*l1 - m2*l2)*g/l1*np.cos(theta)
        self.Int_theta = output['ui']
        self.Dir_theta = output['ud']
        self.error_theta = output['e']
        self.prev_theta = output['x']


        # Lateral
        x = psi
        xr = self.psi_r
        kp = self.P_psi_
        ki = self.I_psi_
        kd = self.D_psi_
        psi_vlim = self.psi_vlim
        sigma = self.sigma_psi
        uio = self.Int_psi
        udo = self.Dir_psi
        eo = self.error_psi
        xo = self.prev_psi

        output = self.PID(x,xr,kp,ki,kd,dt,psi_vlim,sigma,uio,udo,eo,xo)
        self.phi_r = output['u']
        self.Int_psi = output['ui']
        self.Dir_psi = output['ud']
        self.error_psi = output['e']
        self.prev_psi = output['x']

        x = phi
        xr = self.phi_r
        kp = self.P_phi_
        ki = self.I_phi_
        kd = self.D_phi_
        phi_vlim = self.phi_vlim
        sigma = self.sigma_phi
        uio = self.Int_phi
        udo = self.Dir_phi
        eo = self.error_phi
        xo = self.prev_phi

        output = self.PID(x,xr,kp,ki,kd,dt,phi_vlim,sigma,uio,udo,eo,xo)
        Tau_unsat = output['u']
        self.Int_phi = output['ui']
        self.Dir_phi = output['ud']
        self.error_phi = output['e']
        self.prev_phi = output['x']

        sat = self.saturate(F_unsat,Tau_unsat)

        F_sat = sat[0]
        Tau_sat = sat[1]

        if self.I_theta_ > 0:
            self.Int_theta += dt/self.I_theta_*(F_sat-F_unsat)
        if self.I_psi_ > 0:
            self.Int_psi += dt/self.I_psi_*(Tau_sat-Tau_unsat)

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

        # Check for command change (experimental)
        # Also Fix lines 263 and 76
        # if len(self.theta_ddot_old) >= 10:
        #     self.theta_ddot_old.pop(0)
        #     self.theta_ddot_old.append(beta*self.theta_ddot_old[-1] - (1-beta)/dt*(ud-udo))
        # else:
        #     self.theta_ddot_old.append(beta*self.theta_ddot_old[-1] - (1-beta)/dt*(ud-udo))
        # average = 0
        # for theta_ddot in self.theta_ddot_old:
        #     average += theta_ddot
        # average /= len(self.theta_ddot_old)
        # if e > 0:
        #     flag = average<0
        #     print flag
        # elif e < 0:
        #     flag = average>0
        #     print flag
        # else:
        #     flag = True
        #
        if (ud < np.abs(xdot_lim)): # and flag:
            ui = uio + dt/2*(e + eo)
        else:
            ui = uio


        # PID
        u = kp*up + ki*ui + kd*ud


        # Return result
        output = {'u':u,'ui':ui,'ud':ud,'e':e,'x':x}
        return output

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


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    try:
        controller = Controller()
    except:
        rospy.ROSInterruptException
    pass
