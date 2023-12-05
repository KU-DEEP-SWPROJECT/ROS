from heapq import heappush as Push
from heapq import heappop as Pop
import numpy as np
from math import *
from typing import Optional
import copy
from robot_class import robot as Robot


class Node:
    def __init__(self, parent, coordinate: tuple, cost: int, heuristic: int, dir: int):
        self.PARENT : Node = parent
        self.COORDINATE = tuple(coordinate)
        self.COST = cost
        self.HEURISTIC = heuristic
        self.DIRECTION = dir  # 0b00 : front  0b11 : back 0b01 : left 0b10: right

    def __lt__(self, other):
        return (self.COST + self.HEURISTIC) < (other.COST + other.HEURISTIC)

    def __eq__(self, other) -> bool:
        if isinstance(other, Node):
            if self.COORDINATE == other.COORDINATE and self.HEURISTIC == other.HEURISTIC and self.COST == other.COST:
                return True
        return False

    def __hash__(self) -> int:
        return self.COST + self.HEURISTIC




class TimeAstar:
    # Size, Radius , robots( center coordinate:tuple , direction:int,straight_speed:int,rotate_speed:int, stop:int, color:str)
    # , goal: ㄷ자 , Obstacle [ [ㄷ자],[ㄷ자]]
    def __init__(self, SIZE: int, Radius: int, robots: list, goal: list, obstacles: list) -> None:
        self.SIZE = SIZE
        self.robots : Robot= robots.copy()
        self.MAP = [[0] * SIZE for _ in range(SIZE)]
        self.COST_RATIO = 5
        self.RANGE = Radius * Radius
        self.AgentTable = [[] for _ in range(len(robots))]  # [ [], [], [], [], [] ]
        self.set_goal(tuple(np.mean(goal, axis=0).astype(int)))
        obstacles.append(goal)
        self.set_obstacle(obstacles)
        self.Robot_sort()
        self.robots[0].GOAL = (goal[0][0]-1,goal[0][1]+1)
        self.robots[1].GOAL = (goal[1][0]+1,goal[1][1]+1)
        self.robots[2].GOAL = (goal[2][0]+1,goal[2][1]-1)
        self.robots[3].GOAL = (goal[3][0]-1,goal[3][1]-1)

    def set_goal(self, goal: tuple):
        for robot in self.robots:
            robot.GOAL = goal

    def set_obstacle(self, obstacles) -> None:
        for obstacle in obstacles:
            self.draw_line(obstacle[0][0], obstacle[0][1], obstacle[1][0], obstacle[1][1])
            self.draw_line(obstacle[1][0], obstacle[1][1], obstacle[2][0], obstacle[2][1])
            self.draw_line(obstacle[2][0], obstacle[2][1], obstacle[3][0], obstacle[3][1])
            self.draw_line(obstacle[3][0], obstacle[3][1], obstacle[0][0], obstacle[0][1])

    @staticmethod
    def distance(A: tuple, B: tuple) -> int:  # 맨하튼 거리
        return abs(A[0] - B[0]) + abs(A[1] - B[1])


    def put_robots(self, robotlist: list):
        self.robots = robotlist.copy()

    def draw_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while x0 != x1 or y0 != y1:
            self.MAP[y0][x0] = -1
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy


    @staticmethod
    def CCW(A, B):
        # A와 B의 외적 계산
        cross_product = np.cross(A, B)

        if 0 < cross_product:
            return 0
        elif 0 > cross_product:
            return 1
        return 2

    def Robot_sort(self) -> None:
        self.robots.sort(key=lambda x: self.distance(x.coordinate, x.GOAL))

    def is_Range(self, A: tuple, B: tuple):
        dy = B[1] - A[1]
        dx = B[0] - A[0]
        return (dy * dy + dx * dx) <= self.RANGE

    def draw_path(self,idx):
        MAP = copy.deepcopy(self.MAP)
        for i in self.robots[idx].path:
            x,y = i[1]
            MAP[y][x] = '●'
        print(np.matrix(MAP))
    @staticmethod
    def Arrow(a,b)-> str:
        if a in [0,3]:
            return ("R90","R-90")[0 if b==1 else 1]
        else:
            return ("R90","R-90")[0 if b==2 else 1]



    def ToCommand(self,idx):
        command_list = []
        path = self.robots[idx].Direction_path
        realpath = self.robots[idx].path
        cnt, stopcnt = 0,0
        cur = path[0] # 첫번째 방향
        cur_path = realpath[0][1] # 첫번째 좌표
        fleg = True # 반전 있는지
        for i in range(1,len(path)):
            if cur_path== realpath[i][1]:
                if cnt > 0:
                    command_list.append('F'+str((1,-1)[0 if fleg else 1]*cnt))
                    stopcnt = 0
                    cnt = 0
                stopcnt += 1
            else:
                if stopcnt > 0:
                    command_list.append('S' + str(stopcnt))
                    cnt = 0
                    stopcnt = 0
                if cur == path[i]:
                    pass
                elif cur ^ path[i] == 3: # 반대 방향
                    command_list.append('F' + str((1, -1)[0 if fleg else 1] * cnt))
                    cnt = 1
                    fleg ^= True
                else: # cur ^ path[i] == 1 or == 2
                    command_list.append('F' + str((1, -1)[0 if fleg else 1] * cnt))
                    command_list.append(self.Arrow(cur,cur ^ path[i]))
                    cnt = 1
                cnt+=1
            cur,cur_path = path[i] , realpath[i][1]

        if cnt > 1:
            command_list.append('F' + str((-1, 1)[1 if fleg else 0] * (cnt-1)))
        return str(self.robots[idx].IDX)+":"+'/'.join(command_list)
    def path_tracking(self, idx: int, T_Node: Node) -> None:
        List = []
        Direction_List = []
        while T_Node.PARENT is not None:
            List.append((T_Node.COST//self.COST_RATIO, T_Node.COORDINATE))
            Direction_List.append(T_Node.DIRECTION)
            T_Node = T_Node.PARENT
        List.append((T_Node.COST // self.COST_RATIO, T_Node.COORDINATE))
        Direction_List.append(T_Node.DIRECTION)
        Direction_List.reverse()
        List.reverse()
        self.robots[idx].put_path(List)
        self.robots[idx].put_direction_path(Direction_List)

        j = 0
        for L in List:
            while j <= L[0]:
                self.AgentTable[idx].append(L[1])
                j+=1

    def Search(self, idx: int) -> None:  # Robot Path finding
        # Heuristic = Distance  // F = G(현재까지 온 거리) + H(맨하튼 거리)

        ROBOT : Robot = self.robots[idx]
        GOAL : tuple = ROBOT.GOAL
        SPEED : int = ROBOT.STRAIGHT * self.COST_RATIO
        ROTATE : int = ROBOT.ROTATE * self.COST_RATIO
        STOP : int = ROBOT.STOP * self.COST_RATIO
        Q = [Node(parent=None, coordinate=ROBOT.coordinate, cost=0, heuristic=self.distance(ROBOT.coordinate, GOAL), dir=ROBOT.direction)]
        visited = set()
        cnt = 0
        while Q:
            Top : Node = Pop(Q)
            # print(Top.COORDINATE,"Cost: ",Top.COST,"Heurisitc: ",Top.HEURISTIC)
            visited.add(Top.COORDINATE)

            for dir, MOV in enumerate([[0,1],[1,0],[-1,0],[0,-1],[0, 0]]):
                x,y = MOV[0] + Top.COORDINATE[0] , MOV[1] + Top.COORDINATE[1]

                if dir == 4:
                    Push(Q, Node(parent=Top, coordinate=Top.COORDINATE, cost=Top.COST + STOP, heuristic=Top.HEURISTIC, dir=Top.DIRECTION))

                else:
                    if x < 0 or y < 0 or x > self.SIZE - 1 or y > self.SIZE - 1 or self.MAP[y][x] == -1 or (x, y) in visited: continue
                    st = Top.COST + ROTATE + SPEED
                    if dir ^ Top.DIRECTION in (0, 3): # 같은 방향을 바라보거나 , 뒤로 가는 방향이라면,
                        st -= ROTATE

                    fleg= True

                    for i in range(len(self.robots)): # 로봇의 개수만큼
                        if len(self.AgentTable[i]) <= st: continue
                        if self.is_Range(self.AgentTable[i][st],(x,y)):
                            fleg = False
                    if fleg:
                        Heuristic: int = self.distance((x, y), GOAL)

                        if Heuristic == 0:  # success path find!
                            self.path_tracking(idx, Node(parent=Top,coordinate= (x, y), cost=st, heuristic=Heuristic, dir=dir))
                            Q.clear()
                            break
                        else:
                            Push(Q, Node(parent=Top, coordinate=(x, y), cost=st, heuristic=Heuristic, dir=dir))

if __name__ == "__main__":
    n = int(input())
    obstacles = [ [(2,2),(2,3),(3,3),(3,2)]]
    robots = [ Robot((5, 5), 1, 1, 2, 1, 'G'), Robot((0, 0), 1, 1, 2, 1, 'R'), Robot((0, 4),0, 1, 2, 1, 'B'), Robot((8, 8), 0, 1, 2, 1, 'P')]
    astar = TimeAstar( SIZE=n,Radius=7 ,robots=robots, goal= [(7,7), (7,8),(8,8),(8,7)], obstacles=obstacles)
    astar.Robot_sort()
    # print(np.matrix(astar.MAP))
    for i in range(4):
        print(astar.robots[i].GOAL)
        astar.Search(i)
        print(astar.ToCommand(i))


# print(astar.robots[i].path)
# for y in range(n):
#     for x in range(n):
# print(astar.TimeTable[y][x],end='    ')
# print()

# for i in range(len(robots)):
#     astar.Search(i)

# for i in astar.robots:
#     print(i.path)
'''
5 
0 0 0 0 0    
0 0 0 0 0
0 0 0 0 0
0 0 0 0 0
0 0 0 0 0

'''
