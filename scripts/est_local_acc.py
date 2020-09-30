import tf
import numpy as np
from collections import deque
import rospy
class Estimator_local_acc(deque):
	def __init__(self):
		super(Estimator_local_acc,self).__init__(maxlen=100)
		self.reset()

	def reset(self):
		self.cur_acc=np.array([0,0,0])
		self.last_v=np.array([0,0,0])
		self.last_time=0
		self.clear()
		self.inited=False

	def estimate(self,filter_coff=0.5):
		while len(self)>1:
			velocity=self.popleft()
			vx=velocity.twist.linear.x
			vy=velocity.twist.linear.y
			vz=velocity.twist.linear.z
			
			time=velocity.header.stamp
			time=rospy.Time(time.secs,time.nsecs).to_time()
			cur_v=np.array([vx,vy,vz])
			if not self.inited:
				self.cur_acc=np.array([0,0,0])
				self.last_v=cur_v
				self.inited=True
			else:
				delta_time=time-self.last_time
				calculation=(cur_v-self.last_v)/delta_time
				self.last_v=cur_v
				self.cur_acc=self.cur_acc*filter_coff+calculation*(1-filter_coff)
			self.last_time=time

		return self.cur_acc






