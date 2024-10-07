#! /usr/bin/env python2

import rospy

class PID_Controller:
    def __init__(self):
        self.x0_err    = 0 
        self.y0_err    = 0
        self.z0_err    = 0
        self.x_err     = 0
        self.y_err     = 0
        self.z_err     = 0
        self.x_err_err = 0
        self.y_err_err = 0
        self.z_err_err = 0
        self.x_vel     = 0
        self.y_vel     = 0
        self.z_vel     = 0
        self.kp_x      = 0.01
        self.kd_x      = 0.1
        self.kp_y      = 0.01
        self.kd_y      = 0.1
        self.kp_z      = 0.2
        self.kd_z      = 0.1


    def local_pos_control(self, pos_x, pos_y, pos_z):
        
        self.x_err = 240 - pos_y
        self.y_err = 320 - pos_x
        self.z_err = 0 - pos_z

        # if(pos_z < 200000 and pos_z >= 87251):
        #     self.z_err = -0.75
        # elif(pos_z < 87251 and pos_z >= 46116):
        #     self.z_err = -1
        # elif(pos_z < 46616 and pos_z >= 21177):
        #     self.z_err = -1.5
        # elif(pos_z < 21177 and pos_z >= 12259):
        #     self.z_err = -2
        # elif(pos_z < 12259 and pos_z >= 8131):
        #     self.z_err = -2.5
        # elif(pos_z < 8131 and pos_z >= 5789):
        #     self.z_err = -3
        # elif(pos_z < 5789 and pos_z >= 4334):
        #     self.z_err = -3.5
        # elif(pos_z < 4334 and pos_z >= 3362):
        #     self.z_err = -4
        # elif(pos_z < 3362 and pos_z >= 2687):
        #     self.z_err = -4.5
        # elif(pos_z < 2687 and pos_z >= 2202):
        #     self.z_err = -5
        # elif(pos_z < 2202 and pos_z >= 1825):
        #     self.z_err = -5.5
        # elif(pos_z < 1825 and pos_z >= 1542):
        #     self.z_err = -6
        # elif(pos_z < 1542 and pos_z >= 1323):
        #     self.z_err = -6.5
        # elif(pos_z < 1323 and pos_z >= 1143):
        #     self.z_err = -7
        # elif(pos_z < 1143 and pos_z >= 996):
        #     self.z_err = -7.5
        # elif(pos_z < 996 and pos_z >= 871):
        #     self.z_err = -8
        # elif(pos_z < 871 and pos_z >= 774):
        #     self.z_err = -8.5
        # elif(pos_z < 774 and pos_z >= 695):
        #     self.z_err = -9
        # elif(pos_z < 695 and pos_z >= 625):
        #     self.z_err = -9.5
        # elif(pos_z < 625 and pos_z >= 0):
        #     self.z_err = -10
        
        

        self.x_err_err = self.x_err - self.x0_err
        self.y_err_err = self.y_err - self.y0_err
        self.z_err_err = self.z_err - self.z0_err

        self.x0_err = self.x_err
        self.y0_err = self.y_err
        self.z0_err = self.z_err

        self.x_vel = self.kp_x * self.x_err 
        self.y_vel = self.kp_y * self.y_err 
        self.z_vel = self.kp_z * self.z_err

        #self.x_vel = self.kp_x * self.x_err + self.kd_x * self.x_err_err
        #self.y_vel = self.kp_y * self.y_err + self.kd_y * self.y_err_err
        #self.z_vel = self.kp_z * self.z_err + self.kd_z * self.z_err_err
        