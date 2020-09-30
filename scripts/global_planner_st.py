#!/usr/bin/env python2
import rospy
from nav_msgs.msg import OccupancyGrid,Path
# import tf
import numpy as np
import threading
import time
from geometry_msgs.msg import PoseStamped,PointStamped,Point
import jps1
import math
import os
from PIL import Image
import getpass
class global_planner():
    def map_callback(self,data):
        self.map_r=data.info.height  #y
        self.map_c=data.info.width   #x
        self.map = np.array(data.data).reshape(self.map_r,self.map_c).T
        self.map[np.where(self.map==100)]=1
        self.map[np.where(self.map==-1)]=0
        # self.map=self.map.reshape(self.map_r,self.map_c).T
        self.map_o=[data.info.origin.position.x,data.info.origin.position.y]
        self.map_reso=data.info.resolution
        self.map_t=[self.map_o[0]+self.map_c*self.map_reso,self.map_o[1]+self.map_r*self.map_reso]
        self.origin=data.info.origin

    def thread_job():
        rospy.spin()
        
    def local_position_callback(self,data):
        self.pos=data
        # parse_local_position(data)
        

    def listener(self):
    
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        rospy.init_node('global_planner', anonymous=True)
        self.path_pub = rospy.Publisher('/jps_path', Path, queue_size=1)
        self.map_pub=rospy.Publisher('/global_map', OccupancyGrid, queue_size=5)
        self.goalpub = rospy.Publisher('/goal_global', Point, queue_size=1)  #static points
        # self.local_vel_sub = rospy.Subscriber('mavros/local_position/velocity_local',
        #                               TwistStamped,
        #                               self.velocity_callback,queue_size=1,buff_size=52428800)
        # self.octo_pub = rospy.Publisher('/octomap_point_cloud_centers_local', PointCloud2, queue_size=1)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                                  PoseStamped,
                                                  self.local_position_callback,queue_size=1,buff_size=52428800)
        self.globalmap_sub = rospy.Subscriber('/projected_map',
                                      OccupancyGrid,
                                      self.map_callback)
        self.globalgoal_sub = rospy.Subscriber('/clicked_point',
                              PointStamped,
                              self.goal_callback)
        # add_thread = threading.Thread(target = thread_job)
        # add_thread.start()
        # rospy.spin()
    
    def goal_callback(self,goal):
        self.global_goal = np.array([goal.point.x,goal.point.y,1.0])
        print('goal received!!')
    def parse_local_position(self,local_position, mode="q"):
        # global rx,ry,rz,rpy_eular
        rx=local_position.pose.position.x
        ry=local_position.pose.position.y
        rz=local_position.pose.position.z
        
        qx=local_position.pose.orientation.x
        qy=local_position.pose.orientation.y
        qz=local_position.pose.orientation.z
        qw=local_position.pose.orientation.w
        # print(rx,ry,rz)
        if mode == "q":
            return (rx,ry,rz)
    
        # elif mode == "e":
        #     rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
        #     return (rx,ry,rz)+ rpy_eular
        else:
            raise UnboundLocalError

    def publish_path(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.path_pub.publish(path)
        
    def publish_map(self,data,map_ori):
        data[np.where(data==1)]=100
        data[np.where(data==-1)]=0
        mapg=OccupancyGrid()
        mapg.header.frame_id = "map"
        mapg.header.stamp = rospy.Time.now()
        mapg.info.resolution=self.map_reso
        mapg.info.width=len(data)
        mapg.info.height=len(data[0])
        mapg.info.origin=self.origin
        mapg.info.origin.position.x=map_ori[0]
        mapg.info.origin.position.y=map_ori[1]
        mapg.data=list(data.T.reshape(mapg.info.width*mapg.info.height,1)[:,0]) #T?
        self.map_pub.publish(mapg)
if __name__ == '__main__':
    # global pub,rate,point2,pos

    planner=global_planner()
    planner.global_goal = None
    planner.listener()
    # convert.vel=None
    planner.pos=None
    planner.map=None
    planner.map_o=None
    path3=None
    wp=None
    map_o=None
    rosrate=80
    ifa=1
    rate = rospy.Rate(rosrate) # 
    ang_wp_tre=math.pi/4
    dis_wp_tre=2
    end_occu =0
#    global_goal=np.array([-2.5,0,1])
#    global_goal=np.array([3.2,0,1])
    # global_goal_list=np.array([[16,1,1],[-16,-1,1]])
    ct=1 #iter time
    # try:
    #     img=Image.open("map1.png")
    #     white,black=0,255
    #     img = img.point(lambda x: white if x > 200 else black)
    #     img = img.convert('1')
    #     img = np.array(img)
    #     map_pre=img[::-1].T # pre-known map before fly
    #     ori_pre=[-15,-15]
        
    #     l1_pre=len(map_pre)
    #     l2_pre=len(map_pre[0])
    #     t_pre=[ori_pre[0]+planner.map_reso*l1_pre,ori_pre[1]+planner.map_reso*l2_pre]
    # except:
    #     map_pre=None
    
    map_pre=None
    fname=None
    ii=0
    while not rospy.is_shutdown():
        mapu=planner.map
        pointw=Point()

        starttime1 = time.clock()
        # if ii<len(global_goal_list):
        #     global_goal=global_goal_list[ii]
        # print(pos)
        if planner.pos is not None and (mapu is not None and mapu is not [-1]) and (planner.map_c>2*ifa) and (planner.map_o is not None):#and (convert.vel is not None)
            map_c=planner.map_c
            map_r=planner.map_r
            map_o=planner.map_o
            map_reso=planner.map_reso
            mapc=mapu.copy()

            ori_pre=[-15,-15]  #the pre-settled origin coordinates
            if ct==1:
                xo,yo,zo=planner.parse_local_position(planner.pos)
                try:
                    img=Image.open("/home/chen/catkin_ws/src/path_plan/maps/map1.png")
                    white,black=0,1
                    
                    img = img.convert('L')
                    img = img.point(lambda x: white if x > 200 else black)
                    img = np.array(img)
                    map_pre=img[::-1].T # pre-known map before fly
                    
                    
                    l1_pre=len(map_pre)
                    l2_pre=len(map_pre[0])
                    t_pre=[ori_pre[0]+planner.map_reso*l1_pre,ori_pre[1]+planner.map_reso*l2_pre]
                except:
                    map_pre=None
            ct+=1
            px,py,pz=planner.parse_local_position(planner.pos)
            if planner.global_goal is not None:
                global_goal = planner.global_goal
            else:
                global_goal = np.array([px,py,1])
            # if map_start[0]>=map_c:
            #     map_start[0]=map_c-1
            # elif map_start[0]<0:
            #     map_start[0]=0
            # if map_start[1]>=map_r:
            #     map_start[1]=map_r-1
            # elif map_start[1]<0:
            #     map_start[1]=0
            # print('mapu',mapu)
            for i in range(ifa,map_c-ifa+1):
                for j in range(ifa,map_r-ifa+1):
                    if len(mapu)>1 and (mapu[i-ifa:i+ifa+1,j-ifa:j+ifa+1]>0).any():
                        mapc[i,j]=1
            mapu=mapc
            if map_pre is not None: #merge the prepared map and the detected map
                # print(map_o,ori_pre)
                map_o1=[min(map_o[0],ori_pre[0]),min(map_o[1],ori_pre[1])]  #oringin of the new map
                map_o=((np.array(map_o)-map_o1)/planner.map_reso).astype(int) #index of the origin of the detected map
                ori_pre=((np.array(ori_pre)-map_o1)/planner.map_reso).astype(int) #index of the origin of the prepared map
                map_c1=int((max(t_pre[0],planner.map_t[0])-map_o1[0])/planner.map_reso)
                map_r1=int((max(t_pre[1],planner.map_t[1])-map_o1[1])/planner.map_reso)
                mapu0=np.zeros([map_c1,map_r1])
                # print('t_pre,planner.map_t,map_o1,map_c1,map_r1,ori_pre,l1_pre,l2_pre',t_pre,planner.map_t,map_o1,map_c1,map_r1,ori_pre,l1_pre,l2_pre)
                mapu0[ori_pre[0]:ori_pre[0]+l1_pre,ori_pre[1]:ori_pre[1]+l2_pre]=map_pre
                mapu0[map_o[0]:map_o[0]+map_c,map_o[1]:map_o[1]+map_r]=mapu
                mapu=mapu0
                map_o=map_o1
                map_c=map_c1
                map_r=map_r1
                # print('mapu:',mapu)
            map_goal=((global_goal[0:2]-map_o)/map_reso).astype(int)
            map_start=((np.array([px,py])-map_o)/map_reso).astype(int)
            map_goal0=map_goal.copy()
            map_d=np.array([0,0])
            map_o2=np.array([0,0])
            if map_goal[0]<0 or map_start[0]<0:
                map_o2[0]=min(map_goal[0],map_start[0])
            if map_goal[1]<0 or map_start[1]<0:
                map_o2[1]=min(map_goal[1],map_start[1])
            map_d=abs(map_o2)
            map_o=list((map_o2-1)*map_reso+np.array(map_o))
            # wp=global_goal
            # print('map_o,map_start,map_goal',map_o,map_start,map_goal)
            # else:
                
                # if map_goal[0]>map_c:
                #     map_goal[0]=map_c-1
                # if map_goal[1]>map_r:
                #     map_goal[1]=map_r-1

            map_c=max(map_c,map_goal[0]+1,map_start[0]+1)+map_d[0]
            map_r=max(map_r,map_goal[1]+1,map_start[1]+1)+map_d[1]
            mapu0=np.zeros([map_c+2,map_r+2])
            mapu0[map_d[0]+1:map_d[0]+len(mapu)+1,map_d[1]+1:map_d[1]+len(mapu[0])+1]=mapu
            mapu=mapu0
            if mapu[map_goal[0],map_goal[1]]==1:  #when an obstacle locates at the goal
                try:
                    map_goal[1]=np.where(mapu[map_goal[0],:]==0)[0][np.argmin(abs(np.where(mapu[map_goal[0],:]==0)-map_goal[1]))]
                except:
                    map_goal[0]=np.where(mapu[:,map_goal[1]]==0)[0][np.argmin(abs(np.where(mapu[:,map_goal[1]]==0)-map_goal[0]))]
                end_occu = 1
                # if map_goal[0]==map_c-1:
                #     map_goal[1]=np.where(mapu[-1,:]==0)[0][np.argmin(abs(np.where(mapu[-1,:]==0)-map_goal[1]))]
                # elif map_goal[1]==map_r-1:
                #     map_goal[0]=np.where(mapu[:,-1]==0)[0][np.argmin(abs(np.where(mapu[:,-1]==0)-map_goal[0]))]
            

            if (map_start+1+map_d)[0]>map_c or (map_start+1+map_d)[1]>map_r:
                wp=global_goal
            else:
                path1 = jps1.method(mapu, tuple(map_start+map_d+1), tuple(map_goal+map_d+1), 2)
            
                if path1[0] is 0:
                    # print(mapu,map_start,map_goal,map_o,path1) #
                    print('no path')
                    wp=global_goal
                else:
                    path2 = np.array(path1[0])
                    path3=path2*map_reso+map_o
                    # path3 = np.r_[path3,[global_goal[0:2]]]
                    if end_occu == 1:
                        path3 = path3[::-1]
                    # print('path3',path3)
                    path3=np.c_[path3,np.zeros([len(path3),1])]
                    ang_wp=0
                    # if len(path2)>2:
                    #     map_wp=(path2[1]+path2[2])/2
                    # else:
                    #     map_wp=path2[-1]
                    # wp=map_wp*map_reso+map_o
                    for k in range(1,len(path2)):
                        
                        map_wp=path2[k]
                        if abs(math.atan2((path2[-1]-map_start-1)[0],(path2[-1]-map_start-1)[1])-math.atan2((map_wp-map_start-1)[0],(map_wp-map_start-1)[1]))<=ang_wp and np.linalg.norm(map_wp-(map_start+map_d+1))>2:
                            map_wp=path2[k-1]
                            wp=map_wp*map_reso+map_o
                            break
                        ang_wp=abs(math.atan2((path2[-1]-map_start-1)[0],(path2[-1]-map_start-1)[1])-math.atan2((map_wp-map_start-1)[0],(map_wp-map_start-1)[1]))
                    uav2next_wp=np.linalg.norm(wp[0:2]-np.array([px,py]))
                    if_goal=uav2next_wp+ang_wp
                    if end_occu == 1:
                        wp = np.array([px,py,pz])
                    elif not(len(path2)>2 and (uav2next_wp>dis_wp_tre or (ang_wp>ang_wp_tre and ang_wp<math.pi*0.5))):  #remove the useless points 
                        wp=global_goal
                        # wp=map_wp*map_reso+map_o
                        # if not(len(path2)>2 and np.linalg.norm(wp-np.array([px,py]))<1):
                        #     break
                    else:
                        print('wp not global goal:',wp)
            # wp=global_goal
            # if np.linalg.norm(global_goal[0:2]-np.array([px,py]))<0.5:
            #     ii+=1
            # if ii==len(global_goal_list):
            #     pointw.z=0
            # else:
            #     pointw.z=1+min(np.linalg.norm(wp[0:2]-np.array([xo,yo]))/np.linalg.norm(global_goal[0:2]-np.array([xo,yo])),1)*(global_goal[2]-1)
            pointw.z=1+min(np.linalg.norm(wp[0:2]-np.array([xo,yo]))/np.linalg.norm(global_goal[0:2]-np.array([xo,yo])),1)*(global_goal[2]-1)
            print('position:',[px,py,pz])

        else:
            if planner.pos is not None:
                px,py,pz=planner.parse_local_position(planner.pos)
                # if np.linalg.norm(global_goal[0:2]-np.array([px,py]))<0.5:
                #     ii+=1
                # if ii==len(global_goal_list):
                #     pointw.z=0
                # else:
                #     pointw.z=global_goal[2]
                wp=np.array([px,py,pz])
                pointw.z = 1
   
                
        protime=time.clock()-starttime1

        if wp is not None:
            # print(point2)
            
            pointw.x=wp[0]
            pointw.y=wp[1]
            
            planner.goalpub.publish(pointw)
            if path3 is not None:
                planner.publish_path(path3)
            if map_o is not None:
                planner.publish_map(mapu,map_o)
            rate.sleep()
        if ct%rosrate==0 and (mapu is not None):
            if fname is not None:
                os.remove(fname)
            mapsave=mapu.copy()
            mapsave[np.where(mapu!=0)]=0
            mapsave[np.where(mapu==0)]=255
            mapsave=mapsave.T[::-1]
            im = Image.fromarray(np.uint8(mapsave)).convert('RGB')
            fname="/home/"+ getpass.getuser() +"/catkin_ws/src/uav_planning_demo/path_plan/maps/"+'%.2f'%map_o[0]+'%.2f'%map_o[1]+"_out.png"
            im.save(fname)
        print('next goal:',pointw)
        print('global_planner protime:',protime)