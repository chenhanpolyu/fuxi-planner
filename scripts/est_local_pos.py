import tf
import numpy as np
from collections import deque
import rospy
class Estimator_local_position(deque):
	def __init__(self):
		super(Estimator_local_position,self).__init__(maxlen=100)
		self.reset()

	def reset(self):
		self.ang_rpy=np.array([0,0,0])
		self.last_rpy=np.array([0,0,0])
		self.last_time=0
		self.clear()
		self.inited=False

	def estimate(self,filter_coff=0.5):
		while len(self)>1:
			local_position=self.popleft()
			rx=local_position.pose.position.x
			ry=local_position.pose.position.y
			rz=local_position.pose.position.z
			
			qx=local_position.pose.orientation.x
			qy=local_position.pose.orientation.y
			qz=local_position.pose.orientation.z
			qw=local_position.pose.orientation.w
			time=local_position.header.stamp
			time=rospy.Time(time.secs,time.nsecs).to_time()

			rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
			cur_rpy=np.array(rpy_eular)
			if not self.inited:
				self.ang_rpy=np.array([0,0,0])
				self.last_rpy=cur_rpy
				self.inited=True
			else:
				delta_time=time-self.last_time
				cur_ang_rpy=(cur_rpy-self.last_rpy)/delta_time
				self.last_rpy=cur_rpy
				self.ang_rpy=self.ang_rpy*filter_coff+cur_ang_rpy*(1-filter_coff)
			self.last_time=time

		return self.ang_rpy






