# -*- coding: utf-8 -*-
"""
Created on Sat Dec 28 11:36:44 2019

@author: chenhan
"""
import rospy
from utils import earth_to_body_frame,body_to_earth_frame
from math import *
import numpy as np
#import matplotlib.pyplot as plt
#from PIL import Image
import time
#from numba import autojit
from scipy.optimize import minimize
#from skimage import measure
#@autojit 


class control_method():
    """
    用来仿真的参数，
    """
    @staticmethod
    def init(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]  # max speed
        # self.min_speed = -0.5  # [m/s]  # min speed
        self.m_speed = self.max_speed  # [m/s]  # max speed
        self.min_speed = 1.5  # [m/s]  # min speed
#        self.max_accel = self.max_speed*0.5  # [m/ss]  # max accelerate
        self.max_accel = 3.5
        self.dt = 0.02  # [s]  # simulation period
        self.max_jerk = 3
        self.predict_coe = 1  # [s]  # predict coefficient
        self.control_gain = 0.3
        self.to_goal_cost_gain = 6.0  # goal_cost_gain
        self.to_obs_cost_gain = 0.0  # obs_cost_gain
        self.speed_cost_gain = 15     # speed_cost_gain
        self.uav_r = 0.45  # [m]  # uav safe radius ,0.45 for static
        self.detect_l = 3 # [m]  # detectsion radius
        self.det_ang = pi/16
        self.map_reso = 0.2 # [m]  # map resolution
        self.startpoint=np.array([3.0,3.0,0.0])    # start point
        self.endpoint=np.array([94.0,94.0,0.0])    #goal,assume the size is 100m*100m
        self.error=0.5             # if the uav reach the goal
        self.global_l = 100 
        self.x0 = np.append(self.startpoint,np.array([0,0,0,0,0,0])).astype(float)
        self.dd = int(self.uav_r/self.map_reso)+1
        self.plc_distance=3
        self.pred_coe = 0.5
        self.upb=4
        self.downb=0.5
        self.vel_coe = 0.7
        self.iter=0
        self.p_num=70  #max number of points for collision check
        self.acc_CD=2  #parameter of acceleration expanding coefficient for avoiding moving obstacle
        self.wps=[]
        self.dmin=[]
        self.c_dyn=[]
        self.v_dyn=[]
        self.r_dyn=[]
        self.obs_v=0  #if we change the velocity for dynamic obs
        self.no_path=0
    @staticmethod
    def Angle( x,  y):
        angle = 0.0;
        x1 = x[0]
        y1 = x[1]
        x2 = y[0]
        y2 = y[1]
    
        if (x1 == 0 and y1 ==0)or(x2 == 0 and y2 ==0):
            angle =0.0
        else:
            angle=atan2(x1,y1)-atan2(x2,y2)
        if angle>pi:
            angle=angle-2*pi
        elif angle<-pi:
            angle=angle+2*pi
        # print('angle:',-angle)
        return -angle
    @staticmethod
    def get_localgoal(self,local_pos,plc,f_angle,loc_goal,loc_goal_old,path_rec,state,no_path_last):

        global no_path
        dmin=[]
        wps=[]
        angle_locgoal=np.array([self.Angle(loc_goal[0:2],[1,0]),self.Angle([np.linalg.norm(loc_goal[0:2]),loc_goal[2]],[1,0])])
        angle_vel=np.array([self.Angle(state[3:5],[1,0]),self.Angle([np.linalg.norm(state[3:5]),state[5]],[1,0])])
#        angle_vel=[self.Angle(state[3:5],[1,0]),self.Angle([np.linalg.norm(state[3:5]),state[5]],[1,0])]
#        angle_locgoal=[(angle_locgoal[0]+0.3*angle_vel[0])/1.3,(angle_locgoal[1]+0.3*angle_vel[1])/1.3]
        # if np.linalg.norm(loc_goal)<self.detect_l*self.predict_coe:
        #     self.detect_l=max(np.linalg.norm(loc_goal)/self.predict_coe,1)
        for coe in [1]:
            no_path=1
            circle_p=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*coe,
                sin(angle_locgoal[0])*self.detect_l*self.predict_coe*coe,
                sin(angle_locgoal[1])*self.detect_l*self.predict_coe*coe])

            last_if_direct = self.if_direct
            self.if_direct = self.d_obstacle(self,circle_p,plc,0,no_path_last,loc_goal_old)<self.uav_r
            if not self.if_direct:
                loc_goal = circle_p
                f_angle = angle_locgoal
#            if not self.if_direct:
            if len(plc) > 0 and self.num_dyn == 0 :#and ((not self.if_direct) and (not last_if_direct)):
#                coe_dis_goal = min(max(min(self.detect_l,np.linalg.norm(plc[0]))/self.detect_l,0.2),0.4)
                coe_dis_goal = 0.5
            elif len(plc) > 0 and self.num_dyn != 0 :
                coe_dis_goal = 1
            if (f_angle[0]!=float("Inf") and f_angle[1]!=float("Inf")):
                f_angle = np.array(f_angle)
                print(angle_locgoal,f_angle) # + angle_vel*(1-coe_dis_goal)*2/3 +f_angle*(1-coe_dis_goal)*1/3 + 
                angle_locgoal = ((angle_locgoal*coe_dis_goal + f_angle*(1-coe_dis_goal)- angle_locgoal)/(self.det_ang)).astype(int)*(self.det_ang)+angle_locgoal
#                print(angle_locgoal)
#                circle_p=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*coe,
#                    sin(angle_locgoal[0])*self.detect_l*self.predict_coe*coe,
#                    sin(angle_locgoal[1])*self.detect_l*self.predict_coe*coe])
#                loc_goal = circle_p
#                f_angle = angle_locgoal
#                self.if_direct = self.d_obstacle(self,circle_p,plc,0)<self.uav_r
            # if self.max_accel < np.linalg.norm(self.velocity):
            #     max_det_ang = asin(min(self.max_accel/np.linalg.norm(self.velocity),0.9))
            # else:
            #     max_det_ang = pi/2*1
            max_det_ang = asin(min(self.max_accel/np.linalg.norm(self.velocity),1))
            if np.linalg.norm(loc_goal)>0.1*self.detect_l and len(plc)!=0 and self.if_direct: #np.linalg.norm(loc_goal)>0.3*self.detect_l and
                if coe_dis_goal ==1:
                    start_ang = 2*self.det_ang
                else:
                    start_ang = 0
                for de_angle in np.arange(start_ang,pi/2*1.2,self.det_ang):
                    circle_p1=np.array([cos(angle_locgoal[0]+de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[0]+de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[1])*self.detect_l*self.predict_coe])
                    circle_p2=np.array([cos(angle_locgoal[0]-de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[0]-de_angle)*self.detect_l*self.predict_coe*cos(angle_locgoal[1]),
                                    sin(angle_locgoal[1])*self.detect_l*self.predict_coe])
                    circle_p3=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]+de_angle),
                                    sin(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]+de_angle),
                                    sin(angle_locgoal[1]+de_angle)*self.detect_l*self.predict_coe])
                    circle_p4=np.array([cos(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]-de_angle),
                                    sin(angle_locgoal[0])*self.detect_l*self.predict_coe*cos(angle_locgoal[1]-de_angle),
                                    sin(angle_locgoal[1]-de_angle)*self.detect_l*self.predict_coe])
                    

#                    print('check points:/n',circle_p1,circle_p2,circle_p3,circle_p4,angle_locgoal)
                    d_min1=self.d_obstacle(self,circle_p1,plc,1,no_path_last,loc_goal_old)
                    if de_angle == 0 :
                        if d_min1 > self.uav_r :
                            if abs(angle_locgoal[0]-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf"):  # and abs(angle_locgoal[0]+de_angle-angle_vel[0]) < pi/3)
                                loc_goal = circle_p1
                                f_angle[0]=angle_locgoal[0]
                            else:
                                loc_goal = loc_goal_old
                            no_path=0
                            break
                        else:
                            continue
                    elif d_min1 > self.uav_r :#and local_pos[2]<self.upb:
                        if abs(angle_locgoal[0]+de_angle-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf"):  # and abs(angle_locgoal[0]+de_angle-angle_vel[0]) < pi/3)
                            loc_goal = circle_p1
                            f_angle[0]=angle_locgoal[0]+de_angle
                        else:
                            loc_goal = loc_goal_old
                        no_path=0
                        break


                    elif self.d_obstacle(self,circle_p2,plc,2,no_path_last,loc_goal_old)> self.uav_r:
                        if abs(angle_locgoal[0]-de_angle-f_angle[0])<max_det_ang or self.pp_restart==1 or f_angle[0]==float("Inf"):
                            loc_goal = circle_p2
                            f_angle[0]=angle_locgoal[0]-de_angle
                        else:
                            loc_goal=loc_goal_old
                        no_path=0
                        break
                    elif self.d_obstacle(self,circle_p3,plc,3,no_path_last,loc_goal_old) > self.uav_r:
                        if abs(angle_locgoal[1]+de_angle-f_angle[1])<max_det_ang or f_angle[1]==float("Inf"):
                            loc_goal = circle_p3
                            f_angle[1]=angle_locgoal[1]+de_angle
                        else:
                            loc_goal=loc_goal_old
                        no_path=0
                        break
                    
                    elif local_pos[2]>self.downb and circle_p4[2]>0.3 and self.d_obstacle(self,circle_p4,plc,4,no_path_last,loc_goal_old) > self.uav_r :
                        if abs(angle_locgoal[1]-de_angle-f_angle[1])<max_det_ang or f_angle[1]==float("Inf"):
                            loc_goal = circle_p4
                            f_angle[1]=angle_locgoal[1]-de_angle
                        else:
                            loc_goal=loc_goal_old
                        no_path=0
                        break
                #loc_goal=np.clip(loc_goal[0:2]-pos[0:2],-self.detect_l*self.predict_coe,self.detect_l*self.predict_coe)
                # if d_min1<=self.uav_r and d_min2<=self.uav_r and d_min3<=self.uav_r and d_min4<=self.uav_r:
                #     no_path=1
                # else:
                #     break
                if no_path==0:
                    self.no_path=0
                    break
                elif max(self.dmin)>0.7*self.uav_r:
                    loc_goal = self.wps[np.argmax(self.dmin)]
                else:
                    no_path=2
            else:
                loc_goal=loc_goal-0
                f_angle = angle_locgoal

                break
#        if no_path==2:
#            if len(path_rec)>1:
#                loc_goal=path_rec[-3-self.no_path]-local_pos
#                self.no_path+=1
#            else:
#                loc_goal=path_rec[-1]-local_pos
#            print('!!!!go back:',loc_goal+local_pos)
        if f_angle[0] == float("Inf"):
            f_angle[0] =0
        if f_angle[1] == float("Inf"):
            f_angle[1] =0
        # elapsed = (time.clock() - starttime)
        # print('find goal time:',elapsed)
        # if np.linalg.norm(loc_goal)>self.detect_l:
        #     print('goal is too far:',loc_goal)
        # print('f_angle:',f_angle)
        return loc_goal,f_angle,no_path
       
            
    def if_edge(use_map,i,j):
        if_ed=(use_map[i-1][j]==1 or use_map[i+1][j]==1 or use_map[i][j-1]==1 or use_map[i][j+1]==1)
        return if_ed
    def cross_point(line1,line2):#计算交点函数
        x1=line1[0][0]# select four points
        y1=line1[0][1]
        x2=line1[1][0]
        y2=line1[1][1]
        
        x3=line2[0][0]
        y3=line2[0][1]
        x4=line2[1][0]
        y4=line2[1][1]
        
        k1=(y2-y1)*1.0/(x2-x1)
        b1=y1*1.0-x1*k1*1.0
        if (x4-x3)==0:
            k2=None
            b2=0
        else:
            k2=(y4-y3)*1.0/(x4-x3)
            b2=y3*1.0-x3*k2*1.0
        if k2==None:
            x=x3
        else:
            x=(b2-b1)*1.0/(k1-k2)
        y=k1*x*1.0+b1*1.0
        #print('cross:',[x,y],'line1',line1,'line2',line2)
        if x4<x3:
            xs=x3
            x3=x4
            x4=xs
        if y4<y3:
            xs=y3
            y3=y4
            y4=xs
        if x2<x1:
            xs=x1
            x1=x2
            x2=xs
        if y2<y1:
            xs=y1
            y1=y2
            y2=xs
        if 0.999*x3<=x<=1.001*x4 and 0.999*y3<=y<=1.001*y4 and 0.999*x1<=x<=1.001*x2 and 0.999*y1<=y<=1.001*y2:
            return [x,y]
        else:
            return None
    def feasible_plane(self,state,obs):  # return the 2d grid map coresponding to the feasible plane, including the obstacle's message.
        fes_vec1=np.array([0,0,1]) # assumption
        fes_vec2=-self.startpoint+self.endpoint
        fes_vec2=fes_vec2/np.linalg.norm(fes_vec2)
        if (fes_vec1 == np.array([0,0,1])).all():
            if fes_vec2[1] == 0:
                fes_vec3 = np.array([1,0,0])
            else:
                fes_vec3=np.array([1,-fes_vec2[0]/fes_vec2[1],0])
        else:
            fes_vec3=np.dot(np.array([-fes_vec1[2],-fes_vec2[2]]),
                        np.linalg.inv(np.array([fes_vec1[0:2],fes_vec2[0:2]]).T))
        fes_vec3=fes_vec3/np.linalg.norm(fes_vec3)
        trans3to2 = np.array([fes_vec3,fes_vec2,fes_vec1])
        return trans3to2
    @staticmethod
    def calculate(self,control,loc_goal,state,obstacle,b2e,path_rec,pred_dt):
    
        #x0=control  # set the initial value
        #x0=np.random.rand(4)
        #x0=np.append(control[0:3],1)
        
#        cvx,cvy,cvz = self.calculate1(control,loc_goal,state,obstacle,b2e,path_rec)
#        x0=np.array([cvx,cvy,cvz,0.1])
        d_goal=np.linalg.norm(loc_goal)
        
        ee = 0.1
        
        px = state[0]
        py = state[1]
        pz = state[2]
        vx = state[3]
        vy = state[4]
        vz = state[5]
        ax = state[6]
        ay = state[7]
        az = state[8]
        dt=self.dt
#        if np.linalg.norm(state[0:2]-self.endpoint[0:2])>0.3 and len(obstacle)!=0:
#            para_g=0.01
#        else:
#            para_g=1
#        
        
        # ve=self.max_speed*loc_goal/d_goal
#        self.max_accel = 6
#        if self.obs_v==1:
#            # print('position and goal',state[0:3],loc_goal+state[0:3],'control:',res.x)
#            print('position and goal',state[0:3],para_g*loc_goal)
#            print("current velocity:",state[3:6])
#    
#            if state[2]-self.downb<0.1:
#                ve[2] = 2
#            if state[2]-self.upb>-2 :
#                ve[2] = -0.7
#            print('set dyn-velocity:',ve)
#            return ve[0],ve[1],ve[2]
        
#        self.max_accel = 1.5 #self.max_speed*0.7

        
#        elif self.obs_v==1:
#            self.min_speed = self.max_speed-0.1
        if self.obs_v==1:
            self.max_accel*=self.acc_CD
            self.vel_coe += 0.2
        if self.no_path:
            self.max_accel*=2
            print("no path found, accel increase",self.no_path)
        
        
        # ae=ae/np.linalg.norm(ae)*min(np.linalg.norm(ae),self.max_accel)
        # Traj=d_obstacle(loc_goal,state[0],state[1],state[2],obstacle)[1]
        global no_path,back_num
        # if no_path!=0:
        #     loc_goal=path_rec[-1-back_num]
        #     back_num+=2
        # else:
        #     back_num=0
        # loc_goal = np.matmul(b2e, loc_goal)
        starttime = time.clock()
        control_gain=self.control_gain
        speed_cost_gain=self.speed_cost_gain
        if self.obs_v==1:
            para_g=np.linalg.norm(state[3:6])*pred_dt/d_goal
            ve=self.max_speed*loc_goal/d_goal
            ae=(ve-state[3:6])/pred_dt
            x0=np.array([max(min(ae[0],self.max_accel-0.2),-self.max_accel+0.2),max(min(ae[1],self.max_accel-0.2),-self.max_accel+0.2),
max(min(ae[2],self.max_accel-0.2),-self.max_accel+0.2),pred_dt])
            to_goal_cost_gain = self.to_goal_cost_gain
            bons = ((-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel), (0,pred_dt*1.2 ))
            cons = (
                    # {'type': 'ineq', 'fun': lambda x: -np.linalg.norm(x[0:3])+self.max_accel},\
                    {'type': 'eq', 'fun': lambda x: 20*np.linalg.norm(np.array([vx+x[0]*x[3],vy+x[1]*x[3],vz+x[2]*x[3]])-ve)})
                    # {'type': 'ineq', 'fun': lambda x: x[3]},\
                    # {'type': 'ineq', 'fun': lambda x: -x[3]+pred_dt*1.2}) #loc_goal is the Increment for position
            res = minimize(self.fun_dyn, x0, args=(px,py,pz,vx,vy,vz,ax,ay,az,obstacle,pred_dt,loc_goal,control_gain,speed_cost_gain,to_goal_cost_gain,para_g,ve),method='SLSQP',
                       options={'maxiter':20},constraints=cons,bounds = bons,tol = 1e-2)
            if res.x[3] > 0:
                traj_dif = np.linalg.norm(np.array([vx+res.x[0]*res.x[3],vy+res.x[1]*res.x[3],vz+res.x[2]*res.x[3]])-ve)
            else:
                traj_dif = 1
            print("dynamic traj end difference",traj_dif,res.status,res.success,res.nit)
        else:
            if self.max_speed == self.min_speed:
                self.max_speed = self.min_speed + 0.1
            para_g = 0.07
            # para_g=np.linalg.norm(state[3:6])*pred_dt/d_goal
            ve=np.linalg.norm(state[3:6])*loc_goal/d_goal
            ae=(ve-state[3:6])/(d_goal*para_g/np.linalg.norm(state[3:6])*self.vel_coe) #pred_dt 
            x0=np.array([max(min(ae[0],self.max_accel-0.2),-self.max_accel+0.2),max(min(ae[1],self.max_accel-0.2),-self.max_accel+0.2),
max(min(ae[2],self.max_accel-0.2),-self.max_accel+0.2),d_goal*para_g/np.linalg.norm(state[3:6])])
            # pp_d1 = np.linalg.norm(np.cross(traj_end,loc_goal*para_g))/np.linalg.norm(loc_goal*para_g)*para_d
            bons = ((-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel), (-self.max_accel, self.max_accel), (d_goal*para_g/self.max_speed,3 ))
            cons = (# {'type': 'ineq', 'fun': lambda x: -np.linalg.norm(x[0:3])+self.max_accel},\
                    {'type': 'ineq', 'fun': lambda x: -np.linalg.norm([vx+x[0]*x[3],vy+x[1]*x[3],vz+x[2]*x[3]])+self.max_speed},\
                    # {'type': 'ineq', 'fun': lambda x: np.linalg.norm([vx+x[0]*x[3],vy+x[1]*x[3],vz+x[2]*x[3]])-self.min_speed},\
                    {'type': 'eq', 'fun': lambda x: np.linalg.norm((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3] - para_g*loc_goal)})
                    # {'type': 'eq', 'fun': lambda x: np.linalg.norm(np.cross((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3],loc_goal*para_g))/(d_goal*para_g)})
                    # {'type': 'ineq', 'fun': lambda x: -x[3]+3}) #loc_goal is the Increment for position
            res = minimize(self.fun, x0, args=(px,py,pz,vx,vy,vz,ax,ay,az,obstacle,dt,loc_goal,control_gain,speed_cost_gain,para_g),method='SLSQP',
                       options={'maxiter':20},constraints=cons,bounds = bons, tol = 3e-2)
            # traj_end = ((np.array([vx,vy,vz])+res.x[0:3]*res.x[3]/2)*res.x[3])
            # pp_d = np.linalg.norm(np.cross(traj_end,loc_goal*para_g))/np.linalg.norm(loc_goal*para_g)
            if res.x[3] > 0:
                traj_dif = np.linalg.norm(((state[3:6]+res.x[0:3]*res.x[3]/2)*res.x[3]) - para_g*loc_goal)
                # traj_dif = np.linalg.norm(np.cross((np.array([vx,vy,vz])+res.x[0:3]*res.x[3]/2)*res.x[3],loc_goal*para_g))/(d_goal*para_g)
            else:
                traj_dif = 1
            # while traj_dif > 0.03:
            #     x0 = res.x + np.r_[np.random.rand(3)*0.3,np.array([0])]
            #     self.max_accel = min(self.max_accel*1.2,3)
            #     cons = (
            #         {'type': 'ineq', 'fun': lambda x: 2*(-abs(np.linalg.norm(x[0:3]))+self.max_accel)},
            #         {'type': 'ineq', 'fun': lambda x: -50*abs(((vx+x[0]*x[3]/2)*x[3]) - para_g*loc_goal[0])+ee}, # speed constrain
            #         {'type': 'ineq', 'fun': lambda x: -50*abs(((vy+x[1]*x[3]/2)*x[3]) - para_g*loc_goal[1])+ee}, 
            #         {'type': 'ineq', 'fun': lambda x: -50*abs(((vz+x[2]*x[3]/2)*x[3]) - para_g*loc_goal[2])+ee},
            #         {'type': 'ineq', 'fun': lambda x: -x[3]+3}) #loc_goal is the Increment for position
            #     res = minimize(self.fun, x0, args=(px,py,pz,vx,vy,vz,ax,ay,az,obstacle,dt,loc_goal,control_gain,speed_cost_gain,para_g),method='COBYLA',
            #            options={'maxiter':20},constraints=cons, tol = 0.003)
            #     traj_dif = np.linalg.norm(((state[3:6]+res.x[0:3]*res.x[3]/2)*res.x[3]) - para_g*loc_goal)
            print("static traj end difference",traj_dif,res.status,res.success,res.nit)
        opttime = (time.clock() - starttime)
        
        print("Optimize Time used:",opttime)
        
        # print('position and goal',state[0:3],loc_goal+state[0:3],'control:',res.x)
        print('position and goal',state[0:3],para_g*loc_goal,'control:',res.x)
        print("current velocity:",state[3:6])
        # if res.x[3] < 0:
        #     res.x[0:3] = -res.x[0:3]
        if vz+res.x[2]*res.x[3] > 1.5 and res.x[3] > 0:
            res.x[2] = (1.5 - vz)/res.x[3]
        elif vz+res.x[2]*res.x[3] < -0.8 and res.x[3] > 0:
            res.x[2] = (-0.8 - vz)/res.x[3]
        # elif np.linalg.norm(np.array([vx,vy,vz])+res.x[0:3]*res.x[3]) > self.max_speed:
        #     res.x[0:3] = res.x[0:3]/np.linalg.norm(res.x[0:3])*(self.max_speed - np.linalg.norm(np.array([vx,vy,vz])))/res.x[3]
        # elif np.linalg.norm(res.x[0:3]) > self.max_accel:
        #     res.x[0:3] = res.x[0:3]/np.linalg.norm(res.x[0:3])*self.max_accel
        if state[2]-self.downb<0.1 and res.x[3] >0:
            res.x[2] = 6
        if state[2]-self.upb>-1 and res.x[3] >0:
            res.x[2] = -0.7
        print('set velocity:',[vx+res.x[0]*res.x[3]*self.vel_coe,vy+res.x[1]*res.x[3]*self.vel_coe,vz+res.x[2]*res.x[3]*self.vel_coe])
        set_vel = [vx+res.x[0]*res.x[3]*self.vel_coe,vy+res.x[1]*res.x[3]*self.vel_coe,vz+res.x[2]*res.x[3]*self.vel_coe]
        return set_vel[0],set_vel[1],set_vel[2],traj_dif
    @staticmethod
    def fun(x,px,py,pz,vx,vy,vz,ax,ay,az,obstacle,dt,loc_goal,control_gain,speed_cost_gain,para_g):
        # speed =np.linalg.norm(np.array([vx,vy,vz]))
        # func = control_gain*np.linalg.norm(x[0:3])+speed_cost_gain*abs(min(x[3],np.sign(x[3])*50)) -np.linalg.norm(np.array([vx,vy,vz])+x[0:3]*x[3])+ 30*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)
        traj_end = ((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3])
        # if sum(traj_end*loc_goal)<0:
        #     para_d = 10
        # else:
        #     para_d = 1
        
        pp_d = np.linalg.norm(traj_end -para_g*loc_goal)
        func = control_gain*np.linalg.norm(x[0:3])-5*np.linalg.norm(np.array([vx,vy,vz])+x[0:3]*x[3]) + 5*pp_d +speed_cost_gain*x[3]
        return func
    
    @staticmethod
    def fun_dyn(x,px,py,pz,vx,vy,vz,ax,ay,az,obstacle,pred_dt,loc_goal,control_gain,speed_cost_gain,to_goal_cost_gain,para_g,ve):
        # func = to_goal_cost_gain*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)+speed_cost_gain*(x[3]-0.01)**2 #+speed_cost_gain*abs(min(x[3],np.sign(x[3])*50)) #+ 50*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)
        func = speed_cost_gain*x[3] + to_goal_cost_gain*np.linalg.norm(np.array([vx+x[0]*x[3],vy+x[1]*x[3],vz+x[2]*x[3]])-ve) + to_goal_cost_gain*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*x[3]/2)*x[3]) - para_g*loc_goal)
        return func
    
    @staticmethod
    def calculate1(self,control,loc_goal,state,obstacle,b2e,path_rec,min_dis):

        self.max_accel = min(0.8 , self.max_speed*0.5)
        # loc_goal = np.matmul(b2e, loc_goal)
        d_goal=np.linalg.norm(loc_goal)
        vx=self.max_speed*loc_goal[0]/d_goal#v
        vy=self.max_speed*loc_goal[1]/d_goal
        vz=self.max_speed*loc_goal[2]/d_goal
        dv=np.linalg.norm(np.array([vx,vy,vz])-state[3:6])
        aa=np.linalg.norm(np.array([vx,vy,vz]))
        bb=np.linalg.norm(state[3:6])
        cos_dv=(aa**2+bb**2-dv**2)/(2*aa*bb)
        vm=bb*cos_dv
        dv_p=bb*(1-cos_dv**2)**0.5
        
        if self.obs_v==1:
            self.max_accel*=self.acc_CD
        # if self.no_path:
        #     self.max_accel*=2
        #     print("no path found, accel increase",self.no_path)
#        elif bb<0.3 :
#            self.max_accel=self.max_speed+0.1
        min_d=np.linalg.norm(obstacle[0])
#        if min_d > 3:
#            min_d==3
        
        if dv>self.max_accel:
            if (min_d>3*self.uav_r) or cos_dv<=0 or cos_dv > 0.96 : 
#                self.max_accel=self.max_speed
#                self.max_accel+=0.3
                print("fast mode!!",[vx,vy,vz],self.max_accel)
                vx=state[3]+self.max_accel*(vx-state[3])/dv
                vy=state[4]+self.max_accel*(vy-state[4])/dv
                vz=state[5]+self.max_accel*(vz-state[5])/dv*1.5
            elif (cos_dv>0 and dv_p<self.max_accel):#and bb>0.3 :\  or abs(att[1])>pi/6
                delt_v=(self.max_accel**2-dv_p**2)**0.5
                vm=vm+delt_v
                vx=vm/aa*vx  # it is more safe
                vy=vm/aa*vy
                vz=vm/aa*vz
                print("safe mode1!!")
            elif (cos_dv>0 and dv_p>self.max_accel):
#                vx=state[3]+self.max_accel*(-state[3])/bb
#                vy=state[4]+self.max_accel*(-state[4])/bb
#                vz=state[5]+self.max_accel*(-state[5])/bb
                vx=state[3]+self.max_accel*(vx-state[3])/dv
                vy=state[4]+self.max_accel*(vy-state[4])/dv
                vz=state[5]+self.max_accel*(vz-state[5])/dv
                vx*=0.6
                vy*=0.6
                vz*=0.6
                print("safe mode2!!")
        if state[2]-self.downb<0.1:
            vz=0.5
        if np.linalg.norm(state[3:5])>self.max_speed*0.8:
            vz+=0.2
            vx*=0.9
            vy*=0.9
#        elif time.time()-dyn_time > 3:
#            vx*=0.2
#            vy*=0.2

            #vz*=0.3

            # elif vz-state[5]>0:
            #     vz=state[5]+3*self.max_accel*(vz-state[5])/dv
            # else:
            #     vz=state[5]+self.max_accel*(vz-state[5])/dv
        print('set velocity:',[vx,vy,vz],"speed limit:",self.max_speed)
        print('position and goal',state[0:3],loc_goal)
        print('state:',state)
        print('number of points:',len(obstacle))
        return vx,vy,vz
    @staticmethod
    def kinematic(self,state,control):
        dt=self.dt
        px = state[0]
        py = state[1]
        pz = state[2]
        vx = state[3]
        vy = state[4]
        vz = state[5]
        ax = state[6]
        ay = state[7]
        az = state[8]
        x = control
        target_state=np.zeros(9)
        target_state[6]=ax+x[0]*dt
        target_state[7]=ay+x[1]*dt
        target_state[8]=az+x[2]*dt
        target_state[3]=(ax+x[0]*dt/2)*dt + vx
        target_state[4]=(ay+x[1]*dt/2)*dt + vy
        target_state[5]=(az+x[2]*dt/2)*dt + vz
        target_state[0]=px+vx*dt+1/2*(ax+x[0]*dt/3)*dt**2
        target_state[1]=py+vy*dt+1/2*(ay+x[1]*dt/3)*dt**2
        target_state[2]=pz+vz*dt+1/2*(az+x[2]*dt/3)*dt**2
        print(target_state)
        return target_state
    @staticmethod
    def getDis(pointX,pointY,pointZ,lineX1,lineY1,lineZ1,lineX2,lineY2,lineZ2):
        a=lineY2-lineY1
        b=lineX1-lineX2
        c=lineX2*lineY1-lineX1*lineY2
        ab=np.linalg.norm([pointX-lineX1,pointY-lineY1,pointZ-lineZ1])**2
        ac=np.linalg.norm([pointX-lineX2,pointY-lineY2,pointZ-lineZ2])**2
        bc=np.linalg.norm([lineX2-lineX1,lineY2-lineY1,lineZ2-lineZ1])**2
        QP=np.array([pointX-lineX1,pointY-lineY1,pointZ-lineZ1])
        v=np.array([lineX2-lineX1,lineY2-lineY1,lineZ2-lineZ1])
        if ac>ab+bc or ab>ac+bc: #or ac>ab+bc:
        # ang1=abs(Angle([pointX-lineX1,pointY-lineY1],[lineX2-lineX1,lineY2-lineY1]))
        # ang2=abs(Angle([pointX-lineX2,pointY-lineY2],[lineX1-lineX2,lineY1-lineY2]))
        # if ang1>pi/2 or ang2>pi/2:
            dis=10
        else:
            #fabs(a*pointX+b*pointY+c))/(pow(a*a+b*b,0.5))
            dis = np.linalg.norm(np.cross(QP, v))/(bc**0.5)
        if dis==0:
            print("getdis input:",[[pointX,pointY,pointZ],[lineX1,lineY1,lineZ1],[lineX2,lineY2,lineZ2]])
        return dis
    @staticmethod
    def d_obstacle(self,x,obstacle,cir_num,no_path_last,loc_goal_old):
        self.iter+=1
        d_ob=float("inf")
        vmod=self.max_speed
        # Traj=[]
        # if x[3]>1:
        #     ddt=x[3]/10
        # else:
        #     ddt=0.1
        # px=0.2*x[0]+px
        # py=0.2*x[1]+py
        # for pt in np.arange(0,1,0.1/self.predict_coe):
        #     trajectory=[px+pt*x[0],
        #                 py+pt*x[1]]
        #     Traj.append(trajectory)
        # Traj=np.array([px+x[0],py+x[1]])
        if len(obstacle)==0:
            d_ob=float("inf")
        elif no_path_last == 2 and np.linalg.norm(loc_goal_old-x) < 0.3:
            d_ob = 0
        elif np.linalg.norm(obstacle[0]) < self.uav_r * 0.7:
            d_ob = 0
        else:
            #for i in range(1,len(Traj)):
            for j in range(len(obstacle)):
                #if min(px-0.1*x[1],px+self.predict_coe*x[0])<obstacle[j,0]<max(px-0.1*x[1],px+self.predict_coe*x[0]) and min(py-0.1*x[1],py+self.predict_coe*x[1])<obstacle[j,1]<max(py-0.1*x[1],py+self.predict_coe*x[1]):
                getdis=self.getDis(obstacle[j][0],obstacle[j][1],obstacle[j][2],0,0,0,x[0],x[1],x[2])
                if getdis<self.uav_r:#np.linalg.norm(Traj[i][0:2]-obstacle[j])<self.uav_r:
                        #d_ob=np.linalg.norm(Traj[i][0:2]-obstacle[j])    # only x,y for 2d map
                    d_ob=getdis
                    break
                # if d_ob!=float("inf"):
                #     break
            if len(self.c_dyn)!=0 and d_ob==float("inf"):
                avoi_ang=[]
                avoi_angyz=[]
                vlist=[]
                safe_c= 0.2
                dyn_sf_r = min(self.uav_r + safe_c +max(self.r_dyn) ,0.9)
                vc=0
                dis=[]
                msp_coe = 1.0
                # print('c_dyn:',self.c_dyn,'v_dyn:',self.v_dyn)
                uav_v=x*np.linalg.norm(self.velocity)/np.linalg.norm(x)
                self.v_dyn =np.array(self.v_dyn) * 1.0
                for i in range(len(self.c_dyn)):
                    rela_v=x*self.velocity/np.linalg.norm(x)-self.v_dyn[i]
                    bc=np.linalg.norm(rela_v)
                    QP=self.c_dyn[i]-self.local_pos
                    ll=np.linalg.norm(QP-rela_v)
                    if ll<=np.linalg.norm(QP)+bc:
                        dis.append(np.linalg.norm(np.cross(QP, rela_v))/(bc))
#                    avoi_ang.append([atan2(QP[0],QP[1])-sin(safe_c*dyn_sf_r/np.linalg.norm(QP)),atan2(QP[0],QP[1])+sin(safe_c*dyn_sf_r/np.linalg.norm(QP))])
#                    avoi_angyz.append([atan2(QP[1],QP[2])-sin(safe_c*dyn_sf_r/np.linalg.norm(QP)),atan2(QP[1],QP[2])+sin(safe_c*dyn_sf_r/np.linalg.norm(QP))])
                    avoi_ang.append([atan2(QP[0],QP[1])-sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP)),atan2(QP[0],QP[1])+sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP))])
                    avoi_angyz.append([atan2(QP[1],QP[2])-sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP)),atan2(QP[1],QP[2])+sin((dyn_sf_r+self.r_dyn[i])/np.linalg.norm(QP))]) 
                    print("relative velocity distance to dynobs",dis)
#                    print("radius of moving obstacles",self.r_dyn)
#                dis=min(dis)
#                if dis<safe_c*dyn_sf_r: #considering the shape of the dynobs and the safe radious, the dis should be much bigger than uav_r 
#                    d_ob=dis
#                print('np.array(dis)-(dyn_sf_r+self.r_dyn)',np.array(dis)-(dyn_sf_r+self.r_dyn))
                if (np.array(dis)-(dyn_sf_r)<0).any(): #considering the shape of the dynobs and the safe radious, the dis should be much bigger than uav_r 
                    d_ob=dis[np.argmin(np.array(dis)-(dyn_sf_r+self.r_dyn))]
#                    d_ob=abs(min(np.array(dis)-(dyn_sf_r+self.r_dyn)))
                    dis=d_ob
                    
                #else:
                    #self.obs_v=1
                    #self.max_speed= np.linalg.norm(self.velocity)

                if d_ob!=float("inf"):
                    avoi_ang=np.array(avoi_ang).reshape(1,-1)[0]
                    avoi_angyz=np.array(avoi_angyz).reshape(1,-1)[0]
                    # vnum=np.argmin(abs(avoi_ang-atan2(rela_v[0],rela_v[1])))
                    # max_speed=np.linalg.norm(self.velocity) + self.pred_dt * self.max_accel*self.acc_CD
                    # min_speed=np.linalg.norm(self.velocity) - self.pred_dt * self.max_accel*self.acc_CD
                    if cir_num<3 and cir_num!=0:
                        for vnum in range(len(avoi_ang)):
#                            max_speed=min(np.linalg.norm(self.v_dyn[int(vnum/2)]),self.m_speed)
                            
                            ang_v=avoi_ang[vnum]
                            avoi_v=np.array([sin(ang_v),cos(ang_v)])
                            print('avoi_v',avoi_v)
                            # print(np.r_[uav_v[0:2],-avoi_v].reshape(2,2).T)
			    
                            vmod=np.dot(np.linalg.inv(np.r_[uav_v[0:2],-avoi_v].reshape(2,2).T),np.array(self.v_dyn[int(vnum/2)][0:2]))[0]
                            
                            uav_v_nxt=x*vmod/np.linalg.norm(x)
                            print('vmod',vmod,'uav_v_nxt',uav_v_nxt)
                           # ( np.linalg.norm(uav_v_nxt[0:2]- self.velocity[0:2]) < self.acc_CD*self.max_accel*self.pred_dt and 
                            if vmod > 0 and np.linalg.norm(uav_v_nxt[0:2]- self.velocity[0:2]) < self.acc_CD*self.max_accel*self.pred_dt*2:# or (vmod<0 and vmod>-0.3*self.max_speed):
                                vlist.append(vmod)
                                d_ob=float("inf")
                                self.obs_v=1
                                vc+=1
                            if (vnum+1 & 1)==0 : #if we can't find the speed under the limit that can avoid the ith obstacle, break the circle and this current search direction is skipped 
                                if vc==0:
                                    d_ob=dis
                                    break
                                else:
                                    vc=0
                    vc=0
                    if cir_num>2 or (cir_num==0 and d_ob!=float("inf")):
                        for vnum in range(len(avoi_angyz)):
#                            max_speed=min(np.linalg.norm(self.v_dyn[int(vnum/2)]),self.m_speed)
                            # max_speed=self.m_speed
                            ang_v=avoi_angyz[vnum]
                            avoi_v=np.array([sin(ang_v),cos(ang_v)])
                            print('avoi_v',avoi_v)
                            # print(np.r_[uav_v[0:2],-avoi_v].reshape(2,2).T)
                            vmod=np.dot(np.linalg.inv(np.r_[uav_v[1:3],-avoi_v].reshape(2,2).T),np.array(self.v_dyn[int(vnum/2)][1:3]))[0]
                            
                            uav_v_nxt=x*vmod/np.linalg.norm(x)
                            print('vmod',vmod,'uav_v_nxt',uav_v_nxt)
                             # ( np.linalg.norm(uav_v_nxt[1:3]- self.velocity[1:3]) < self.acc_CD*self.max_accel*self.pred_dt and
                            if vmod > 0 and np.linalg.norm(uav_v_nxt[1:3]- self.velocity[1:3]) < self.acc_CD*self.max_accel*self.pred_dt*2:# or (vmod<0 and vmod>-0.3*self.max_speed):
                                vlist.append(vmod)
                                d_ob=float("inf")
                                self.obs_v=1
                                vc+=1
                            if (vnum+1 & 1)==0 :
                                if vc==0:
                                    d_ob=dis
                                    break
                                else:
                                    vc=0
                    if len(vlist)>0 and d_ob==float("inf"):
                        print("old speed:",self.velocity)
                        vlist=np.array(vlist)
                        sp1=max(vlist)
                        sp2=min(vlist)
                        sp=np.array([sp1,sp2])
          #              if sp1>self.max_speed and sp2<self.max_speed:            #find the final speed in the list of feasible speed (vlist)
          #                  self.max_speed=sp[np.argmin(abs(sp-self.max_speed))]
          #              elif sp1>self.max_speed and sp2>self.max_speed:
          #                  self.max_speed=max(sp)
          #              elif sp1<self.max_speed and sp2<self.max_speed:
          #                  self.max_speed=min(sp)
                        self.max_speed=sp[np.argmin(abs(sp-np.linalg.norm(self.velocity)))]
                        print("speed:",vlist,self.max_speed)
                        # dv=np.linalg.norm(x*self.max_speed/np.linalg.norm(x)-self.velocity)
                        # if dv>self.acc_CD*self.max_accel:
                        #     d_ob=dis
                    
        print('d_ob:',d_ob)

        self.dmin.append(d_ob)
        self.wps.append(x)
        if d_ob!=float("inf") and d_ob>self.uav_r:
            d_ob=self.uav_r-0.1
        return d_ob
    # @staticmethod
    # def distance_filter(self,plc):
    #     filtered_plc=[]
    #     for point in plc:
    #         if np.linalg.norm(point)<self.plc_distance:
    #             filtered_plc.append(point)
    #     return filtered_plc
    
    @staticmethod
    def distance_filter(self,plc,f_angle,loc_goal):
        min_dis=3
        filtered_plc=[]
        # for point in plc:
        #     d_point=np.linalg.norm(point)
        #     if len(filtered_plc)<=20 and d_point<self.plc_distance:
        #         filtered_plc.append(point)
        #     if d_point<min_dis:
        #         min_dis=d_point
        #         if len(filtered_plc)>20:
        #             # filtered_plc.append(point)
        #             # filtered_plc=sorted(filtered_plc)
        #             # filtered_plc=[point,filtered_plc[0],filtered_plc[1]]
        #             filtered_plc=[point]+filtered_plc[0:20-1]

        i=0
        for point in plc:
            point=plc[i]
            i+=1
            # d_point=np.linalg.norm(point)
            if i<0.6*self.p_num or (i<=self.p_num and abs(abs(self.Angle(point[0:2],[1,0]))-abs(self.Angle(loc_goal[0:2],[1,0])))<1.5):
                # print('p_in_front:',point)
                filtered_plc.append(point)
                # print('filtered_plc-point',point,[d_point])
            elif i>self.p_num:
                break
        
        filtered_plc=np.array(filtered_plc)
        # if len(filtered_plc)>0:
            
        #     filtered_plc=filtered_plc1[np.lexsort(filtered_plc1.T)]

        #     if len(filtered_plc)>30:
        #         filtered_plc=filtered_plc[0:30,0:3]
        #     else:
        #         filtered_plc=filtered_plc[:,0:3]
        # print('filtered_plc',filtered_plc)
        return filtered_plc
    
    @classmethod
    def start(self,plc,plcall,state,goal,r,p,y,loc_goal_old,f_angle,path_rec,pretime,dyn_time,if_direct,no_path,pp_restart,pcl_time):
        self.pp_restart = pp_restart
        if pretime==0:
            pred_dt=0
        else:
            pred_dt=time.time()-pretime
        self.pred_dt = pred_dt
        pretime=time.time()
        starttime1 = time.clock()
        control_method.init(self)
        e2b = earth_to_body_frame(r, p, y)
        b2e = body_to_earth_frame(r,p,y)
        # loc_goal = np.matmul(e2b, goal)
        loc_goal=goal
        print('r,p,y:',[r,p,y],'loc_goal in body:',loc_goal)
        # print(plc)
        plc1=[]
        self.velocity=state[3:6]
        local_pos=state[0:3]+self.pred_coe*pred_dt*self.velocity
        self.local_pos=state[0:3]+self.pred_coe*pred_dt*self.velocity
        # for i in range(len(plc)):
        #     point=np.array([plc[i][2],plc[i][0],-plc[i][1]])
        #     point=np.matmul(b2e,point)+local_pos
        #     if abs(point[2])>0.5:
        #         plc1.append(point-local_pos)
        
        # plc=np.array(plc)
        print('num-octomap',len(plc))
        print('num-depthcam',len(plcall))
        self.if_direct = if_direct
        # if len(plc)>0:
        #     plc=plc[np.lexsort(plc.T)]
        #     plc_z=np.around(np.sort(list(set(list(plc[:,2])))),decimals=2)
        #     ze=[]
        #     for zz in plc_z:
        #         if (round(zz+self.map_reso,1) not in plc_z) and (zz+self.map_reso*2 not in plc_z) and (zz+self.map_reso*3 not in plc_z) and int(zz*10-plc_z[0]*10)%int(10*self.map_reso)==0:
        #             # print(zz+self.map_reso,plc_z)
        #             ze.append(zz)
        #         if int(zz*10-plc_z[0]*10)%int(10*self.map_reso*2)==0 and zz-plc_z[0]>=self.map_reso*2-0.1:
        #             ze.append(zz)
            
        #     ze.append(plc_z[0])
        #     ze.append(plc_z[-1])
        #     print(ze,plc_z)
            
        #     for pt in plc:
        #         # print('got one',pt)
        #         # if (pt[2] in ze):
        #         if np.sort(abs(ze-pt[2]))[0]<0.01:
        #             plc1.append(pt)
                    
        #     # plc2=[]
        #     # for i in range(len(plc1)):
        #     #     point=plc1[i]
        #     #     plc2.append(point-local_pos)
        #     plc=np.array(plc1)
        #     print('plc-z num before df:',len(plc))
            
        #     plc1=[]
        #     plc=plc[np.lexsort(plc.T[0:2,:])]
        #     plc_y=np.around(np.sort(list(set(list(plc[:,1])))),decimals=2)
        #     ye=[]
        #     for yy in plc_y:
        #         if (round(yy+self.map_reso,1) not in plc_y) and (yy+self.map_reso*2 not in plc_y) and (yy+self.map_reso*3 not in plc_y) and int(yy*10-plc_y[0]*10)%int(10*self.map_reso)==0:
        #             # print(yy+self.map_reso,plc_y)
        #             ye.append(yy)
        #         if int(yy*10-plc_y[0]*10)%int(10*self.map_reso*2)==0 and yy-plc_y[0]>=self.map_reso*2-0.1:
        #             ye.append(yy)
            
        #     ye.append(plc_y[0])
        #     ye.append(plc_y[-1])
        #     print(ye,plc_y)
            
        #     for pt in plc:
        #         # print('got one',pt)
        #         # if (pt[2] in ye):
        #         if np.sort(abs(ye-pt[1]))[0]<0.01:
        #             plc1.append(pt)
                    
        #     plc2=[]
        #     print('plc num loc:',len(plc1))
        #     for i in range(len(plc1)):
        #         point=plc1[i]
        #         plc2.append(point-local_pos)
        #     # plc=np.array(plc2)
        #     plc=list(plc2)
        #     print('plc-y num before df:',len(plc))
            
        # for i in range(len(plc)):
        #     point=plc[i]
        #     plc1.append(point-local_pos)
        # plc=plc1

        if len(plc)>0:
            plc=plc-local_pos
            plc=plc[::2]
            
            if len(plc)>self.p_num*1:
                
                plc_1=plc[int(self.p_num*2/3)::3]
                plc=np.r_[plc[0:int(self.p_num*2/3)],plc_1]
                plc=control_method.distance_filter(self,plc,f_angle,loc_goal)
        num_dyn=0
        if len(plcall)>0:
            if plcall[-1][0]!=0:
                print('number of dyn!!!!!!:',plcall[-1][0])
                num_dyn=int(plcall[-1][0])
                now = rospy.get_rostime()
                pcl_dt = now.secs + now.nsecs*1e-9 - pcl_time
                print("pcl_dt:",pcl_dt)
                self.v_dyn=plcall[-1-2*num_dyn:-num_dyn-1]
                # print(np.array(plcall[-1-3*num_dyn:-2*num_dyn-1]) ,np.array(self.v_dyn))
                print(plcall[-1-3*num_dyn::])
                self.c_dyn=np.array(plcall[-1-3*num_dyn:-2*num_dyn-1]) + np.array(self.v_dyn) * self.pred_coe*(pred_dt+pcl_dt)
                self.r_dyn=0.5*np.array(plcall[-num_dyn-1:-1])[:,0]
                
#                print('r of mov:',0.5*np.array(plcall[-num_dyn-1:-1]))
                plcall=plcall[0:-1-3*num_dyn]
            
            if len(plcall)>0:
                # print(plcall)
                plcall=plcall-local_pos
                plcall=control_method.distance_filter(self,plcall,f_angle,loc_goal)
                
#                if len(plcall)>self.p_num*0.8:
#                    plcall_1=plcall[int(self.p_num/2)::2]
#                    plcall=np.r_[plcall[0:int(self.p_num/2)],plcall_1]
        min_dis=0
        if num_dyn != 0:
            dyn_time  =pretime
        self.num_dyn = num_dyn
        if len(plc) >0 and len(plcall)>0:
            min_dis=min(np.linalg.norm(plc[0]),np.linalg.norm(plcall[0]))
#             min_dis=np.linalg.norm(plcall[0])
        elif len(plcall)>3:
            min_dis=np.linalg.norm(plcall[0])
        elif len(plc)>3:
#            min_dis=np.linalg.norm(plc[0])*(1-0.2)+self.detect_l*0.2
            min_dis=np.linalg.norm(plc[0])
# 
        # if min_dis !=0 and num_dyn ==0:
        #     self.max_speed=max(self.max_speed*(min_dis/self.detect_l)**1,self.min_speed)
        # if np.linalg.norm(loc_goal)<self.uav_r:
        #     self.max_speed=self.min_speed
        self.uav_r=self.uav_r + np.linalg.norm(self.velocity)*0.03
        control=np.array([0, 0, 0, 1])
        if len(plcall)>0 and len(plc)>0:
            plc=np.r_[plc,plcall]
            if np.linalg.norm(plcall[0])<np.linalg.norm(plc[0]):
                plc=np.r_[[plcall[0]],plc]
        elif len(plcall)>0:
            plc=plcall
        elif len(plc)==0:    #only for dynobs rviz simulation
            plc=np.array([[100,100,1]])

        loc_goal,f_angle,no_path=control_method.get_localgoal(self,local_pos,plc,f_angle,loc_goal,loc_goal_old,path_rec,state.copy(),no_path)  # get the local goal in 2d map
        starttime2 = time.clock()
        # control,c_goal=control_method.calculate(self,control,loc_goal,state.copy(),plc,b2e,path_rec)
        # target_state = control_method.kinematic(self,state.copy(),control)
        traj_dif = 1
        if no_path == 2:
            vx,vy,vz = 0,0,0
            print("no path found, go back!!!")
        # elif num_dyn != 0 or np.linalg.norm(self.velocity) > self.max_speed*0.3 :
        else:
            vx,vy,vz,traj_dif = control_method.calculate(self,control,loc_goal,state.copy(),plc,b2e,path_rec,pred_dt)
        if traj_dif > 0.08 and num_dyn ==0:
            vx,vy,vz = control_method.calculate1(self,control,loc_goal,state.copy(),plc,b2e,path_rec,min_dis)
             #test:velocity control
        steptime = (time.clock() - starttime1)
        wptime = (starttime2 - starttime1)
        optitime = (time.clock() - starttime2)
        print("Step Time used:",steptime)
        return [vx,vy,vz],loc_goal,f_angle,steptime,wptime,optitime,self.iter,len(plc),dyn_time,pretime,plc+local_pos ,self.if_direct,no_path,traj_dif
# if __name__ == '__main__':
#     self=self()
#     timerec,fgtime,path_rec=main()
#     timerec=np.array(timerec)
#     print('\naverrage time cost:',np.mean(timerec[:,0]),
#           '\nget_localgoal time cost:',np.mean(timerec[:,1]),
#           '\ncontrol time cost:',np.mean(timerec[:,2]),
#           '\nfind goal time:',np.mean(fgtime))

