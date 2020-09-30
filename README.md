# FUXI-A-global-computational-efficient-path-planner-for-UAVs
It is a global computational efficient path planner for UAVs, now it can work perfectly in the Gazebo simulation environment. It can work with the default GPS positioning tools in Gazebo/PX4 and a 2D grid map which can be obtained from some mapping kit like Octomap.

cd /fuxi-planner/launch

./st_start.sh  (start the planning demo with ccmapping)

After the UAV takeoff or reach its goal, you have 3 seconds to publish the next goal, or it will land on the ground.

