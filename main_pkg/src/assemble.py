# -*- encoding: utf-8 -*-

from robot_class import robot as Robot

from Image_process.Gunwoo import *
from algorithm.Timeastar import *
from turtlebot import *

BOT_DIR = 0b00
BOT_LINENAR_TIME=1
BOT_ROTATE_TIME=25
BOT_STOP_TIME=1

# get camera data
cam_output  = get_points(2)

# parsing layer 1
pixel = cam_output[0]
bot_radius = 5#cam_output[1]
bot_pos_array = cam_output[2]
jim_array = cam_output[3]
obs=[]
robot_color=["R","G","B","P"]
print(cam_output)


def get_goal_pos(robot_array,jim_array,num):
    
    
    return (0,0)

# set robot data
robots = []

for i in range(len(bot_pos_array)):
    robots.append(Robot(bot_pos_array[i],BOT_DIR,BOT_LINENAR_TIME,BOT_ROTATE_TIME,BOT_STOP_TIME,robot_color[i]))

# set Astar
astar = TimeAstar(SIZE=pixel, Radius=bot_radius, robots=robots,goal=jim_array, obstacles=obs)
#start setting
#astar.Robot_sort()
for i in range(len(robots)):
    astar.Search(i)



# print each bot's path
for i in range(len(astar.robots)):
    print(astar.ToCommand(i))
'''
try:
        rospy.init_node('teleop_twist_keyboard')
        BOT_COUNT = int(input("터틀봇 운영 대수: "))
        bots = [None]
        for i in range(BOT_COUNT):
            bots.append(TurtleBot())
        
        controller = BotController(bots)
        print("터틀봇에게 전송할 명령을 입력하세요.")
        print("- 입력된 명령들을 실행하려면 `run`을 입력하세요.")
        print("- 물건 운반 작업을 진행하려면 `carry`를 입력하세요.")
        print("- 프로그램을 종료하려면 아무것도 입력하지 않고 Enter를 누르세요.")
        while True:
            temp_cmd = input("turtlebot> ")
            if temp_cmd.lower() == "run":
                controller.execute_all_commands()
                controller.run_all_turtlebots()
                print("[@] Running commands finished.")
            elif temp_cmd.lower() == "carry":
                controller.carry_object()
                print("[@] Carrying task finished.")
            elif not temp_cmd:
                break
            else:
                controller.push_command(temp_cmd)
        print("[@] Program exits.")
    except KeyboardInterrupt:
        rospy.signal_shutdown()
'''
