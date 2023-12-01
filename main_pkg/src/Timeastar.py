from heapq import heappush as Push
from heapq import heappop as Pop
import numpy as np
from robot_class import robot as Robot

class Node:
    def __init__(self,parent, coordinate, cost, heuristic,dir):
        self.parent = parent
        self.coordinate = coordinate
        self.cost = cost
        self.heuristic = heuristic
        self.dir = dir
        
    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    def __eq__(self, other) -> bool:
        if isinstance(other, Node):
            if self.coordinate == other.coordinate and self.heuristic == other.heuristic and self.cost == other.cost:
                return True
        return False
    def __hash__(self) -> int:
        return self.cost+self.heuristic

class TimeAstar:
    def __init__(self,size,robots,MAP) -> None:
        self.size = size
        self.robots = robots.copy()
        self.MAP = MAP.copy()
        self.TimeTable = [[[[0,0]  for _ in range(len(robots))] for _ in range(size)] for _ in range(size)]
        # for s,y in enumerate(robots):
        #     self.TimeTable[y.coordinate[0]][y.coordinate[1]][s] = 0
    def distance(self,A , B) -> int: # manathn
        return abs(A[0]-B[0]) + abs(A[1]-B[1])
    
    def put_robots(self,robots):
        self.robots = robots.copy()

    def put_MAP(self,MAP):
        self.MAP = MAP.copy()

    def Set_obstacle(obstacles) -> None:
        for obstacle in obstacles:
            pass 
    
    def CCW(self,A, B):
    # A와 B의 외적 계산
        cross_product = np.cross(A, B)
        # print(A,B,"= ",cross_product)

        # 외적의 부호에 따라 시계방향 또는 반시계방향 여부 판단
        if cross_product > 0 : return 0
        elif cross_product < 0 : return 1
        return 2
  


    def lightweight(self,vector_list):
        i = 0
        modified_list = []
        command_list = []
        dir = {(0,0) : 'S'}; dir[(0,1)] = dir[(1,0)] = 'F'

        while i < len(vector_list) - 1:
            j = i + 1
            while j < len(vector_list) and vector_list[i] == vector_list[j]:
                j += 1

            ccw = self.CCW(modified_list[-1], vector_list[j - 1]) if modified_list else None
            R = ("R90", "R-90", "")[ccw] if ccw is not None else ""
            if R:
                command_list.append(R)

            if vector_list[i] != (0, 0):
                modified_list.append(vector_list[i])
            command_list.append(dir[vector_list[i]] + str(j - i))
            i = j

        else:
            if i < len(vector_list):
                command_list.append(dir[tuple(vector_list[i])] + "1")

        return '/'.join(command_list)


    def path_tracking(self,idx,T_Node : Node) -> None:
        List = []
        Table = self.TimeTable
        while(T_Node.parent is not None):
            y,x = T_Node.coordinate
            Table[y][x][idx] = [T_Node.cost - T_Node.parent.cost,T_Node.cost]
            List.append((x-T_Node.parent.coordinate[1],y-T_Node.parent.coordinate[0]))
            T_Node = T_Node.parent
        List.reverse()
        self.robots[idx].put_path(self.lightweight(List))
        
    

    def Search(self,idx) -> None: # Robot Path finding 
        # Heuristic = Distance  // F = G(현재까지 온 거리) + H(맨하튼 거리)
        visited = set()
        robot = self.robots[idx]
        goal = robot.goal
        Q = [ Node(None, robot.coordinate , 0, self.distance(robot.coordinate , goal), robot.direction)]
        speed= robot.straight
        rotate= robot.rotate
        stop = robot.stop
        List = []
        while Q:
            Top = Pop(Q)
            if(Top.coordinate == goal): #success path find!
                List = self.path_tracking(idx,Top)
                break
            visited.add(Top.coordinate)
            
            for dir,ku in enumerate([[0,1],[0,-1],[1,0],[-1,0],[0,0]]):
                y = ku[0] + Top.coordinate[0]
                x = ku[1] + Top.coordinate[1]
                if dir == 4:
                    Push(Q, Node(Top,Top.coordinate,Top.cost+stop,Top.heuristic,Top.dir) )
                else:
                    if x < 0 or y < 0 or x > n-1 or y > n-1 or MAP[y][x]== -1 or (y,x) in visited: continue
                    st = Top.cost+rotate+speed
                    if dir ^ Top.dir in [0,3]:
                        st = Top.cost+speed
                    fleg = False
                    if all(map(lambda r: (st < r[0]  or st > r[1]),self.TimeTable[y][x])):
                        Push(Q, Node(Top,(y,x),st,self.distance((y,x),goal),dir))
                    
    

n = int(input())
MAP = [[*map(int,input().split())] for _ in range(n)]
robots = [Robot((1,0),1,1,1,1,(4,4),'G'),Robot((0,0),1,1,1,1,(4,4),'R')]
astar = TimeAstar(size=n, robots=robots, MAP = MAP)

for i in range(len(robots)):
    astar.Search(i)

for i in astar.robots:
    print(i.path)
'''
5 
0 0 0 0 0    
0 0 0 -1 0
0 -1 -1 -1 0
0 0 0 0 0
0 0 0 0 0

'''
