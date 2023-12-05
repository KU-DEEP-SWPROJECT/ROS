from Timeastar import TimeAstar
from robot_class import robot

BOT_DIR = 0b00
BOT_LINENAR_TIME = 1
BOT_ROTATE_TIME = 25
BOT_STOP_TIME = 1
BOT_COLOR = "RGBP"

def test_algorithm(
    board_size,
    bot_radius,
    bot_start_point,
    bot_target_point
):
    robots = []
    for i in range(len(bot_start_point)):
        robots.append(robot(
            bot_start_point[i],
            BOT_DIR,BOT_LINENAR_TIME,
            BOT_ROTATE_TIME,
            BOT_STOP_TIME,
            BOT_COLOR[i]
        ))
    
    astar = TimeAstar(SIZE=board_size, Radius=bot_radius, robots=robots, goal=bot_target_point, obstacles=[])
    for i in range(len(robots)):
        astar.Search(i)
        print(i, 'Complete')
    
    for i in range(len(astar.robots)):
        print(astar.ToCommand(i))