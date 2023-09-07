#!/usr/bin/python3

from rrt_planner import RRT
from pgm import Environment
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("path_planner")
# read start and goal from ros topic /start_goal
subscriber_name = "/start_goal"
publisher_name = "/Path"
pgm_file = rospy.get_param('~pgm')
yaml_file = rospy.get_param('~yaml')

#pgm_file = "data/map.pgm"
#yaml_file = "data/map.yaml"
round_max = (
    10  # Maximum number of iterations, 10 is enough, 因为每次都是随机的，所以多次迭代，可以确保找到一条路径
)
iter_max = (
    5000  # Maximum number of nodes to be added to the tree in each iteration 地图越大，这个值越大
)
step = 3  # Step size 每次走的步长，越大，路径越直，但是容易碰到障碍物
env = Environment(pgm_file, yaml_file)





def rrt_callback(msg, publisher):
    # change start and goal to numpy array
    origin = np.asarray(env.origin[:2])
    x_start = np.array(msg.data[0:2]) + origin
    x_goal = np.array(msg.data[2:4]) + origin
    # change start and goal to rescale to map
    x_start = (x_start / env.resolution).astype(int)
    x_goal = (x_goal / env.resolution).astype(int)
    print(f"x_start: {x_start}\nx_goal: {x_goal}")
    print("origin = ", origin)
    rrt = RRT(env, x_start, x_goal, step=step)
    path = rrt.planning(iter_max=iter_max, round_max=round_max)
    # publish path, [num_of_points, p1_x, p1_y, p2_x, p2_y, ...]
    path = rrt.smooth_path(path)
    # change path to real world scale
    path = np.array(path).astype(float) * env.resolution - origin
    path=np.flip(path,0)
    print(path)
    path_msg = Float32MultiArray()
    path_msg.data = np.concatenate(([len(path)], path.flatten()))
    publisher.publish(path_msg)
    rospy.spin()


publisher = rospy.Publisher(publisher_name, Float32MultiArray, queue_size=1)


start_goal_sub = rospy.Subscriber(
    subscriber_name,
    Float32MultiArray,
    rrt_callback,
    queue_size=1,
    callback_args=publisher,
)

rospy.spin()
