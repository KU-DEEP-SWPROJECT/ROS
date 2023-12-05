import heapq
from typing import Union, Optional

push = heapq.heappush
pop = heapq.heappop

class Attributes:
    BOARD_SIZE = 200
    ROTATE_TIME = 25
    STOP_TIME = 5
    
    @classmethod
    def init(cls):
        cls.BOT_SIZE = cls.BOARD_SIZE // 10
        cls.BOT_SIZE_SQUARE = cls.BOT_SIZE * cls.BOT_SIZE
        cls.EUCLID_DIST_TURTLEBOT = [[i*i+j*j <= cls.BOT_SIZE_SQUARE for j in range(cls.BOARD_SIZE)] for i in range(cls.BOARD_SIZE)]
        cls.EUCLID_DIST_WALL = [[i*i+j*j <= (cls.BOT_SIZE_SQUARE // 4) + 1 for j in range(cls.BOARD_SIZE)] for i in range(cls.BOARD_SIZE)]

Attributes.init()

# direction: 0 = north(0-), 1 = east(+0), 2 = south(0+), 3 = west(-0)
# left rotate = -1, right rotate = +1
DIRECTION = ((0, 1, 0, -1), (-1, 0, 1, 0))
ACTION = (
    (3, 1),
    (0, 2),
    (1, 3),
    (2, 0),
)

class PathBacktracker:
    
    def __init__(self, parent: Optional['PathBacktracker'], act: int = -1) -> None:
        self.parent = parent
        self.act = act  # -1 = None(ignore), 0 = wait, 1 = forward, 2 = rotate left, 3 = rotate right, 4 = rotate 180
        self.depth = 1 if parent is None else parent.depth + 1
    
    def get_acts(self):
        res = [-1 for _ in range(self.depth)]
        pbt = self
        while pbt is not None:
            res[pbt.depth-1] = pbt.act
            pbt = pbt.parent
        return res
    
    def get_pos_array(self, first_x, first_y, first_direction):
        x, y, direction = first_x, first_y, first_direction
        arr = [(x, y)]
        for a in self.get_acts():
            if a == 1:
                x += DIRECTION[0][direction]
                y += DIRECTION[1][direction]
            elif a == 2:
                direction = (direction - 1) % 4
            elif a == 3:
                direction = (direction + 1) % 4
            elif a == 4:
                direction = (direction + 2) % 4
            arr.append((x, y))
        return arr
    
    def get_command(self, bot_num=None):
        PIXEL_RATE = 2 / Attributes.BOARD_SIZE
        KEEPING_TIME_RATE = PIXEL_RATE * 12  # 1.2 / 0.1
        
        acts = self.get_acts()
        command = []
        state = 1  # 0 = stop, 1 = move
        prev_dist = 0
        for a in acts:
            if a == 1:
                if state == 1:
                    prev_dist += 1
                else:
                    if prev_dist: command.append('S'+str(prev_dist))
                    state = 1
                    prev_dist = 1
            elif a == 0:
                if state == 0:
                    prev_dist += 1
                else:
                    if prev_dist: command.append('F'+str(prev_dist))
                    state = 0
                    prev_dist = 1
            elif a == 2:
                if prev_dist: command.append('F%.2f,%.2f' % (prev_dist*PIXEL_RATE, prev_dist*KEEPING_TIME_RATE) if state else 'S'+str(prev_dist*PIXEL_RATE))
                state = 0
                prev_dist = 0
                command.append('R90,3')
            elif a == 3:
                if prev_dist: command.append('F%.2f,%.2f' % (prev_dist*PIXEL_RATE, prev_dist*KEEPING_TIME_RATE) if state else 'S'+str(prev_dist*PIXEL_RATE))
                state = 0
                prev_dist = 0
                command.append('R-90,3')
            elif a == 4:
                if prev_dist: command.append('F%.2f,%.2f' % (prev_dist*PIXEL_RATE, prev_dist*KEEPING_TIME_RATE) if state else 'S'+str(prev_dist*PIXEL_RATE))
                state = 0
                prev_dist = 0
                command.append('R180,3')
        if prev_dist: command.append('F%.2f,%.2f' % (prev_dist*PIXEL_RATE, prev_dist*KEEPING_TIME_RATE) if state else 'S'+str(prev_dist*PIXEL_RATE))
        return ('' if bot_num is None else str(bot_num) + ':') + '/'.join(command)
    
    def __lt__(self, other):
        if isinstance(other, PathBacktracker):
            return self.depth < other.depth
        return NotImplemented


def manhattan(p1, p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])


def expect_rotate(this_x, this_y, this_direction, target_x, target_y):
    if this_direction == 0:
        if this_x == target_x:
            return 2 * (this_y < target_y)
        else:
            return 1 + (this_y < target_y)
    elif this_direction == 1:
        if this_y == target_y:
            return 2 * (this_x > target_x)
        else:
            return 1 + (this_x > target_x)
    elif this_direction == 2:
        if this_x == target_x:
            return 2 * (this_y > target_y)
        else:
            return 1 + (this_y > target_y)
    else:
        if this_y == target_y:
            return 2 * (this_x < target_x)
        else:
            return 1 + (this_x < target_x)

# data = (time, x, y, direction, remain_distance, remain_time_to_stop_and_rotate, path_backtracker)
# remain_time_to_stop_and_rotate: only wait if positive

def time_a_star(start_point, start_direction, target_point, target_direction, obstacle, another_visited):
    q = []
    this_visited = set()
    _dist = manhattan(start_point, target_point)
    push(q, (_dist, 0, *start_point, start_direction, _dist, 0, None))
    this_visited.add((start_point, start_direction, 0))
    while q:
        _, time, x, y, direction, remain_distance, remain_time_to_stop_and_rotate, path_backtracker = pop(q)
        if (x, y) == target_point:
            flag_possible_goal = True
            for bot_path_log in another_visited:
                for t in range(time, len(bot_path_log), 1):
                    other_x, other_y = bot_path_log[t]
                    if Attributes.EUCLID_DIST_TURTLEBOT[abs(other_x-x)][abs(other_y-y)]:
                        flag_possible_goal = False
                        break
            if not flag_possible_goal:
                continue
            if target_direction == direction:
                pass
            elif target_direction == ACTION[direction][0]:
                time += Attributes.ROTATE_TIME
                path_backtracker = PathBacktracker(path_backtracker, 2)
            elif target_direction == ACTION[direction][1]:
                time += Attributes.ROTATE_TIME
                path_backtracker = PathBacktracker(path_backtracker, 3)
            else:
                time += Attributes.ROTATE_TIME * 2
                path_backtracker = PathBacktracker(path_backtracker, 4)
            direction = target_direction
            return time, x, y, direction, path_backtracker
        new_time = time + 1
        flag_now_possible = True
        for bot_path_log in another_visited:
            other_x, other_y = bot_path_log[min(time, len(bot_path_log) - 1)]
            if Attributes.EUCLID_DIST_TURTLEBOT[abs(other_x-x)][abs(other_y-y)]:
                flag_now_possible = False
                break
        if not flag_now_possible:
            continue
        if remain_time_to_stop_and_rotate > 0:
            push(q, (new_time + remain_distance + expect_rotate(x, y, direction, *target_point) * Attributes.ROTATE_TIME, 
                     new_time, x, y, direction, remain_distance, remain_time_to_stop_and_rotate-1, PathBacktracker(path_backtracker)))
            continue
        
        flag_future_possible = True
        new_x, new_y = x+DIRECTION[0][direction], y+DIRECTION[1][direction]
        new_xy = (new_x, new_y)
        if (new_xy, direction, new_time) not in this_visited and 0 <= new_x < Attributes.BOARD_SIZE and 0 <= new_y < Attributes.BOARD_SIZE:
            for wall_x, wall_y in obstacle:
                if Attributes.EUCLID_DIST_WALL[abs(wall_x-new_x)][abs(wall_y-new_y)]:
                    break
            else:
                for bot_path_log in another_visited:
                    other_x, other_y = bot_path_log[min(new_time, len(bot_path_log)-1)]
                    if Attributes.EUCLID_DIST_TURTLEBOT[abs(other_x-new_x)][abs(other_y-new_y)]:
                        flag_future_possible = False
                        break
                if flag_future_possible:
                    this_visited.add((new_xy, direction, new_time))
                    new_dist = manhattan(new_xy, target_point)
                    push(q, (new_time + new_dist + expect_rotate(new_x, new_y, direction, *target_point) * Attributes.ROTATE_TIME, 
                             new_time, new_x, new_y, direction, new_dist, 0, PathBacktracker(path_backtracker, 1)))
                else:
                    push(q, (new_time + remain_distance + expect_rotate(x, y, direction, *target_point) * Attributes.ROTATE_TIME, 
                             new_time, x, y, direction, remain_distance, Attributes.STOP_TIME-1, PathBacktracker(path_backtracker, 0)))
        push(q, (new_time + remain_distance + expect_rotate(x, y, ACTION[direction][0], *target_point) * Attributes.ROTATE_TIME, 
                 new_time, x, y, ACTION[direction][0], remain_distance, Attributes.ROTATE_TIME-1, PathBacktracker(path_backtracker, 2)))
        push(q, (new_time + remain_distance + expect_rotate(x, y, ACTION[direction][1], *target_point) * Attributes.ROTATE_TIME, 
                 new_time, x, y, ACTION[direction][1], remain_distance, Attributes.ROTATE_TIME-1, PathBacktracker(path_backtracker, 3)))


def turtlebot_astar(size, _, start_point, target_point, start_direction=2):
    import time
    Attributes.BOARD_SIZE = size
    Attributes.init()
    n = len(start_point)
    mid_x, mid_y = sum(target_point[k][0] for k in range(n)) // n, sum(target_point[k][1] for k in range(n)) // n
    rec_xy = ((min(target_point[k][0] for k in range(n)), min(target_point[k][1] for k in range(n))), 
              (max(target_point[k][0] for k in range(n)), max(target_point[k][1] for k in range(n))))
    temp_targ_point = target_point[:]
    for idx, tp in enumerate(target_point):
        temp_targ_point[idx] = (mid_x + (Attributes.BOT_SIZE // 2 + 5) * (1 if tp[0] > mid_x else -1), 
                                rec_xy[tp[1] > mid_y][1] + (Attributes.BOT_SIZE // 2 + 5) * (1 if tp[1] > mid_y else -1))
    robots = []
    
    def euclid_square(p1, p2):
        dx, dy = p1[0]-p2[0], p1[1]-p2[1]
        return dx*dx + dy*dy
    
    for i in range(n):
        sp = tuple(start_point[i])
        temp_targ_point.sort(key=lambda x: -euclid_square(sp, x))
        tp = temp_targ_point.pop()
        robots.append((sp, start_direction, tuple(tp), 2 if tp[1] < mid_y else 0))
    obstacle = set()
    rec_xy = ((min(target_point[k][0] for k in range(n)), min(target_point[k][1] for k in range(n))), 
              (max(target_point[k][0] for k in range(n)), max(target_point[k][1] for k in range(n))))
    for i in range(rec_xy[0][0], rec_xy[1][0]+1):
        obstacle.add((i, rec_xy[0][1]))
        obstacle.add((i, rec_xy[1][1]))
    for j in range(rec_xy[0][1], rec_xy[1][1]+1):
        obstacle.add((rec_xy[0][0], i))
        obstacle.add((rec_xy[1][0], i))
    start = time.time()
    cmds = []
    another_visited = []
    print("[friendship] robots:", *robots, sep='\n')
    print("[friendship] obstacle:", rec_xy)
    for idx, bot in enumerate(robots):
        print("[friendship] calculating:", idx, bot)
        time_, x, y, direction, path_backtracker = time_a_star(*bot, obstacle, another_visited)
        another_visited.append(path_backtracker.get_pos_array(*bot[0], bot[1]))
        cmd = path_backtracker.get_command(idx+1)
        print(time_, x, y, direction, cmd)
        cmds.append(cmd)
    print(time.time() - start)
    return cmds


if __name__ == '__main__':
    turtlebot_astar(
        200, 
        None, 
        [
            [154, 22],
            [30, 28],
            [70, 24],
            [116, 24],
        ],
        [
            [84, 159],
            [123, 157],
            [121, 118],
            [82, 121],
        ]
    )
