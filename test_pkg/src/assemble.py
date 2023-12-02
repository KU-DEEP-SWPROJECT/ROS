from robot_class import robot as Robot
from Gunwoo import *
from Timeastar import *

BOT_DIR =1
BOT_LINENAR_TIME=1
BOT_ROTATE_TIME=1
BOT_STOP_TIME=1

# get camera data
cam_output  = get_points(2)

# parsing layer 1
pixel = cam_output[0]
bot_pos_array = cam_output[1]
jim_array = cam_output[2]

# set map 
MAP = [[*map(int,jim_array.split())] for _ in range(n)]

# set robot data
robots = []
for i in range(len(bot_pos_array)):
    robots.append(Robot((bot_pos_array[i],BOT_DIR,BOT_LINENAR_TIME,BOT_ROTATE_TIME,BOT_STOP_TIME,get_goal_pos(bot_pos_array,jim_array,i))))

# set Astar
astar = TimeAstar(size=n, robots=robots, MAP = MAP)
#start setting
for i in range(len(robots)):
    astar.Search(i)

# print each bot's path
for i in astar.robots:
    print(i.path)