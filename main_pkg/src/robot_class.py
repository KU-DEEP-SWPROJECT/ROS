class robot:
    def __init__(self,coordinate,direction,straight_speed,rotate_speed,stop,goal,color):
        self.coordinate = coordinate   # initial location
        self.direction = direction     # initial direction 00 : front, 11 : back , 01 : left, 11 : right
        self.path = None               # robot's path [] 
        self.straight = straight_speed
        self.rotate = rotate_speed
        self.stop = stop
        self.goal = goal
        if color == 'R':
            self.idx = 1
        elif color == 'G':
            self.idx = 2
        elif color == 'B':
            self.idx = 3
        else: 
            self.idx = 4

    def put_path(self,Path):
        
        self.path = str(self.idx)+":"+Path
    def put_speed(self,straight_speed,rotate_speed):
        self.straight = straight_speed
        self.rotate = rotate_speed