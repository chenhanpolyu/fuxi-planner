#!/usr/bin/env python2
import rospy
from utils import q2rpy_rad

from UAVController import UAVController
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf
import numpy as np
import time
# from build_geometry_map import *
# import minimum_snap


class Program1(UAVController):
    def __init__(self):
        self.dir = 1
        self.time_pub=0
        return super(Program1, self).__init__()


    # def gate_pos_cb(self, data):

    #     self.gate_info["x"] = data.transform.translation.x
    #     self.gate_info["y"] = data.transform.translation.y
    #     self.gate_info["z"] = data.transform.translation.z

    #     qx = self.gate_info["qx"] = data.transform.rotation.x
    #     qy = self.gate_info["qy"] = data.transform.rotation.y
    #     qz = self.gate_info["qz"] = data.transform.rotation.z
    #     qw = self.gate_info["qw"] = data.transform.rotation.w
    #     r, p, y = q2rpy_rad(qx, qy, qz, qw)
    #     self.gate_info["R"] = r
    #     self.gate_info["P"] = p
    #     self.gate_info["Y"] = y

        # if "y" not in self.gate_info:
        #     self.gate_info["y"] = 0
        # self.gate_info["y"] += 0.01 * self.dir
        # if abs(self.gate_info["y"])>1.0:
        #     self.dir = -self.dir

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

    # def test_posctl(self):
    #     """Test offboard position control"""

    #     # make sure the simulation is ready to start the mission
    #     self.wait_for_topics(60)
    #     self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
    #                                10, -1)

    #     self.log_topic_vars()
    #     self.set_mode("OFFBOARD", 5)
    #     self.set_arm(True, 5)

    def publish_locwp(self,cv):

        locwp = Marker()
        locwp.header.frame_id = "map"
        locwp.header.stamp = rospy.Time.now()
        locwp.type = Marker.ARROW
#        locwp.pose.position.x = self.velo_goal[0]
#        locwp.pose.position.y = self.velo_goal[1]
#        locwp.pose.position.z = self.velo_goal[2]
        p1=Point()
        p2=Point()
        p1.x,p1.y,p1.z=self.local_pos[0],self.local_pos[1],self.local_pos[2]
        p2.x,p2.y,p2.z=self.velo_goal[0],self.velo_goal[1],self.velo_goal[2]
        locwp.points.append(p1)
        locwp.points.append(p2)
        locwp.scale.x = 0.1
        locwp.scale.y = 0.3
        locwp.scale.z = 0.5
        locwp.color.a = 1
        locwp.color.r = 1.5*cv/2
        locwp.color.g = 1.5*max(0,1.0-cv/2)
        locwp.color.b = 0.0

        # adjust gate's quaternion. align hole's central axis as x axis
        # q_90degree = [0, np.sin(np.pi / 4), 0, np.cos(np.pi / 4)]
        # q_gate2 = tf.transformations.quaternion_multiply(q_gate, q_90degree)

#        locwp.pose.orientation.x = 0
#        locwp.pose.orientation.y = 0
#        locwp.pose.orientation.z = 0
#        locwp.pose.orientation.w = 0

        # Publish the MarkerArray
        self.locwp_publisher.publish(locwp)
            
    # def publish_dyn_obs(self):
    #     dynobs = Marker()
    #     dynobs.header.frame_id = "map"
    #     dynobs.header.stamp = rospy.Time.now()
    #     dynobs.type = Marker.SPHERE
    #     dynobs.pose.position.x = self.c_dyn1[0][0]
    #     dynobs.pose.position.y = self.c_dyn1[0][1]
    #     dynobs.pose.position.z = self.c_dyn1[0][2]

    #     dynobs.scale.x = 1
    #     dynobs.scale.y = 1
    #     dynobs.scale.z = 1
    #     dynobs.color.a = 0.6
    #     dynobs.color.r = 0.1
    #     dynobs.color.g = 0.8
    #     dynobs.color.b = 0.1
        
    #     dynobs.pose.orientation.x = 0
    #     dynobs.pose.orientation.y = 0
    #     dynobs.pose.orientation.z = 0
    #     dynobs.pose.orientation.w = 0
        
    #     dynv = Marker()
    #     p1=Point()
    #     p2=Point()
    #     dynv.header.frame_id = "map"
    #     dynv.header.stamp = rospy.Time.now()
    #     dynv.type = Marker.ARROW
    #     p1.x,p1.y,p1.z=self.c_dyn1[0][0],self.c_dyn1[0][1],self.c_dyn1[0][2]
    #     p2.x,p2.y,p2.z=self.v_dyn1[0][0]+self.c_dyn1[0][0],self.c_dyn1[0][1]+self.v_dyn1[0][1],self.v_dyn1[0][2]+self.c_dyn1[0][2]
    #     # dynv.points.push_back(p1)
    #     # dynv.points.push_back(p2)
    #     dynv.points.append(p1)
    #     dynv.points.append(p2)
    #     # dynv.pose.position.x = self.velo_goal[0]
    #     # dynv.pose.position.y = self.velo_goal[1]
    #     # dynv.pose.position.z = self.velo_goal[2]

    #     dynv.scale.x = 0.2 #diameter of arrow shaft
    #     dynv.scale.y = 0.4 #diameter of arrow head
    #     dynv.scale.z = 0.6
    #     dynv.color.a = 1  #transparency
    #     dynv.color.r = 0.8
    #     dynv.color.g = 0.1
    #     dynv.color.b = 0.1
        
    #     # dynv.pose.orientation.x = 0
    #     # dynv.pose.orientation.y = 0
    #     # dynv.pose.orientation.z = 0
    #     # dynv.pose.orientation.w = 0
    #     # Publish the MarkerArray
    #     self.dynobs_publisher.publish(dynobs)
    #     self.dynv_publisher.publish(dynv)

    # def calculate_path(self, start_point):
    #     uav = UAV(0.45)
    #     ring = Ring(0.5)
    #     # start point is not necessarily the actual uav pos. it could be a fake position.
    #     uav_x, uav_y, uav_z = start_point
    #     ring_x, ring_y, ring_z = self.gate_info["x"], self.gate_info["y"], self.gate_info["z"]
    #     ring_R, ring_P, ring_Y = self.gate_info["R"], self.gate_info["P"], self.gate_info["Y"]

    #     # use ring as origin!!!!!
    #     uav_x, uav_y, uav_z = uav_x - ring_x, uav_y - ring_y, uav_z - ring_z

    #     uav.set_environmental_data(uav_x, uav_y, uav_z, 0, 0, 0)
    #     ring.set_environmental_data(0, 0, 0, ring_R, ring_P, ring_Y)
    #     csmap = self.map_builder
    #     csmap.set_ring_uav(ring, uav)
    #     try:
    #         map_2d, uav_pos_2d, target_2d = csmap.get_map_2d_far()
    #     except IndexError as e:
    #         rospy.loginfo_throttle(1, "now out of planning region.")
    #         return [target_2d] * 4
    #     # target_2d=[target_2d[0],0]
    #     path = astar(map_2d, tuple(uav_pos_2d), tuple(target_2d))  # !!!!path planning part

    #     while len(path) < 4:
    #         path.append(target_2d)

    #     e2b = earth_to_body_frame(ring_R, ring_P, ring_Y)
    #     b2e = body_to_earth_frame(ring_R, ring_P, ring_Y)
    #     rbpos_ring = np.matmul(e2b, np.array([uav_x, uav_y, uav_z]))
    #     path_result = []
    #     for p in path:
    #         pos_3d_b = csmap.recover_space_pos(p, rbpos_ring)
    #         pos_3d_e = np.matmul(b2e, np.array(pos_3d_b))
    #         pos_3d_e = pos_3d_e + np.array([ring_x, ring_y, ring_z])
    #         path_result.append(pos_3d_e)

    #     return path_result

    def user_control_init(self):
        self.gate_info = {}
        self.locwp_publisher = rospy.Publisher("local_wp", Marker, queue_size=1)
        # self.dynobs_publisher = rospy.Publisher("dyn_obs", Marker, queue_size=1)
        # self.dynv_publisher = rospy.Publisher("dyn_v", Marker, queue_size=1)
        # self.local_pos_sub = rospy.Subscriber("vicon/gate/gate",
        #                                       TransformStamped,
        #                                       self.gate_pos_cb)

        self.path_pub = rospy.Publisher('/uav_path', Path, queue_size=10)
        self.ros_rate = 50
        self.cross_times = 5

    # def plan_motion(self, path, t_interval):
    #     p3 = path[3]
    #     p2 = path[2]
    #     p1 = path[1]
    #     # unit speed..
    #     v_dir = (p3 - p2) / (norm(p3 - p2) + 1e-3)

    #     x, y, z, R, P, Y = self.parse_local_position("e")
    #     p0 = np.array([x, y, z])
    #     waypoints = [[p0[0], p1[0], p2[0]], [p0[1], p1[1], p2[1]], [p0[2], p1[2], p2[2]]]
    #     vx, vy, vz, wx, wy, wz = self.parse_velocity()
    #     # [v0,a0,ve,ae]
    #     v_a = [[vx, 0, v_dir[0], 0], [vy, 0, v_dir[1], 0], [vz, 0, v_dir[2], 0]]
    #     t = [0, t_interval, t_interval * 2]
    #     n = 4
    #     polyn = minimum_snap.plan(waypoints, v_a, t, n)
    #     pos = []
    #     for t in np.linspace(0, t_interval * 2, 6):
    #         pos.append([poly(t) for poly in polyn])
    #     return pos

    def plan(self, start_point):
        path = self.calculate_path(start_point)
        return path


    def user_control_logic(self):
        # if self.vel is not None:
        #     self.pos_setvel_pub.publish(self.vel)
        time_interval = 0.05
        
        if len(self.goal) > 0:
            px, py, pz = self.goal
            # p_gate = self.gate_info["x"], self.gate_info["y"], self.gate_info["z"]
            # q_gate = self.gate_info["qx"], self.gate_info["qy"], self.gate_info["qz"], self.gate_info["qw"]
            ux, uy, uz, uR, uP, uY = self.parse_local_position("e")
            vx,vy,vz,_,_,_=self.parse_velocity()
            if self.velo_goal !=[]:
                self.publish_locwp(np.linalg.norm([vx,vy,vz]))

            self.f2.write(' ,'.join([str(x) for x in [ux,uy,uz]])+" ,"+' ,'.join([str(x) for x in [vx,vy,vz]])+" ,"+' ,'.join([str(x) for x in [uR,uP,uY]])+" ,"+str(time.time())+"\n")
            print('ifend',self.ifend)
            pos_uav = np.array([ux, uy, uz])
            if self.ifend==0 :#and np.linalg.norm(np.array([px, py, pz])- pos_uav)> 0.3:
                
                # self.traj = self.plan(pos_uav)
                if time.time()-self.time_pub>1:
                    self.publish_path(self.path_rec[::int(len(self.path_rec)/100)+1])
                    self.time_pub=time.time()
                # dir_v = self.get_dir(self.traj, 1)  # the drone's speed
                self.my_state_mach = 1

                # if False:#np.linalg.norm(pos_uav - self.goal) < 0.01:
                #     px, py, pz = self.goal
                #     self.set_local_position(px, py, pz,uY)
                #     self.tick = 0
                    
                #     self.my_state_mach = 1

                # else:
                # if self.vel is not None:
                #     self.pos_setvel_pub.publish(self.vel)
                self.tick=0
                self.set_velocity()

            elif np.linalg.norm(np.array([px, py, pz])- pos_uav)< 0.5:
                # px, py, pz = self.goal
                if self.tick==0:
                    print('goal reached, fly to: %s , cross time remained: %s' %([px, py, pz],self.cross_times-1))
                    self.cross_times -= 1
                # if abs(uz-pz)<0.2:
                #     self.set_local_position(px, py, 0.3,0)
                # else:
                #     self.set_local_position(px, py, pz,0)#uY)
                # self.set_local_position(0,0,1.5,0)
                print("publish position command",self.tick)
                self.set_local_position(px, py, pz,uY)
                self.tick += 1
                
                
                if self.tick > 200.0 * self.ros_rate:
                    if self.cross_times > 0:
                        self.user_control_reset()
                    else:
                        return True
                    rospy.loginfo("control reset!")

                # rospy.loginfo_throttle(0.2,self.parse_local_position())

                rospy.sleep(1.0 / self.ros_rate)

    def user_control_reset(self):
        rospy.loginfo("user program reset!")
        self.my_state_mach = 0
        # self.map_builder = Map_builder()


if __name__ == "__main__":
    prog = Program1()
    prog.run()

