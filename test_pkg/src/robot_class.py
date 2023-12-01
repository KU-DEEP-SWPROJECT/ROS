class robot:
    def __init__(self,coordinate,direction,straight_speed,rotate_speed,stop,goal):
        self.coordinate = coordinate   # initial location
        self.direction = direction     # initial direction 00 : front, 11 : back , 01 : left, 11 : right
        self.idx = None
        self.path = None               # robot's path [] 
        self.straight = straight_speed
        self.rotate = rotate_speed
        self.stop = stop
        self.goal = goal
    def put_idx(self,color):
        if color == 'R':
            self.idx = 11
        elif color == 'G':
            self.idx = 12
        elif color == 'B':
            self.idx = 13
        else: 
            self.idx = 14

    def put_path(self,path):
        self.path = path
    def put_speed(self,straight_speed,rotate_speed):
        self.straight = straight_speed
        self.rotate = rotate_speed