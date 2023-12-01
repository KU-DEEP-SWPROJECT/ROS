import numpy as np 
import cv2 
import math as mt
#from realsense_depth import *
import pyrealsense2 as rs
import cv2


loop_exit = 0


class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)



        # Start streaming
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def release(self):
        self.pipeline.stop()
    
def get_point_t(image, pic, contour, col, m_color, zero, color_count, count, turtle_point, m_turtle_point, re_turtle_point, re_left_side, real_sqaure_size,
                pixel_x_size, pixel_y_size ):
    
    real_size = np.empty((2) ,dtype=float) #실제 turtlebot 까지의 길이 (x, y)
    area = cv2.contourArea(contour) 
    if(area > 300): 
        
        rect = cv2.minAreaRect(contour) 
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        ang = np.empty((0,2), dtype=int)
        for i in range(4):
            ang = np.append(ang, [[box[i][0], box[i][1]]], axis=0)
            image = cv2.circle(image,(box[i][0], box[i][1]),10,m_color[col],-1)
        
        D= ang.mean(axis=0) #원점.
        D = D.ravel()
        D = np.int32(D)        
        D[0] -= zero[0]
        D[1] -= zero[1]

        if(color_count[col]<count):
            turtle_point[col][color_count[col]] = D # 노랑색은 터틀 1번, 해당 터틀봇의 좌표를 turtle_point[0]에 넣음.
            color_count[col] += 1
            

        if(color_count[col]>=count):
            x_value = np.zeros(color_count[col])
            y_value = np.zeros(color_count[col])
            for i in range(color_count[col]):
                x_value[i] = turtle_point[col][i][0]
                x_median = np.median(x_value)
                y_value[i] = turtle_point[col][i][1]
                y_median = np.median(y_value)

            get_y = int(y_median)
            get_x = int(x_median)
            m_turtle_point[col] = (get_x, get_y)

            color_count[col] = 0
            turtle_point[col] = np.zeros(shape=turtle_point[col].shape)
        
            re_turtle_point[col] = np.array([[int(m_turtle_point[col][0])], [int(m_turtle_point[col][1])]] )
            re_turtle_point[col] = np.dot(re_left_side, re_turtle_point[col]) # 원점을 중심, 원점의 x, y 기저 벡터를 통한 새로운 좌표  
            
            real_size[0] = real_sqaure_size*re_turtle_point[col][0]/pixel_x_size
            real_size[1] = real_sqaure_size*re_turtle_point[col][1]/pixel_y_size  
            
            if ( (real_size[0] is not None) and (real_size[1] is not None) ) :
                return real_size


def get_point_b(x, y, zero, re_left_side, pixel_x_size, pixel_y_size, real_sqaure_size):
    real_size = np.empty((2) ,dtype=float) #실제 turtlebot 까지의 길이 (x, y)
    
    p_burden_point = (x-zero[0], y-zero[1])

    re_burden_point = np.array([int(p_burden_point[0]), int(p_burden_point[1])])
    re_burden_point = np.dot(re_left_side, re_burden_point)
    real_size[0] = real_sqaure_size*re_burden_point[0]/pixel_x_size
    real_size[1] = real_sqaure_size*re_burden_point[1]/pixel_y_size
    if ( (real_size[0] is not None) and (real_size[1] is not None) ) :
                return real_size[0], real_size[1]


def mapping(k_real_size, turtle_num, burden_real_size, m_color):
    real_bg_size = 200
    bg_x = bg_y = 1000
    bg_cnt = 1000
    bg_size = bg_x/bg_cnt
    



    background = np.ones((bg_x, bg_y, 3), dtype=np.uint8) * 255
    color = (0, 0, 0)  
    thickness = 3

    oc = k_real_size
    
    turtle_mapping_loc = np.zeros((turtle_num, 2), dtype=int)
    burden_mapping_loc = np.zeros((4,2), dtype=int)

    m_x = np.zeros(turtle_num, dtype=int)
    m_y = np.zeros(turtle_num, dtype=int)

    burden_oc = burden_real_size
    

    burden_m_x = np.zeros(4, dtype=int)
    burden_m_y = np.zeros(4, dtype=int)

    # for i in range(bg_cnt+1):
    #     for j in range(bg_cnt+1):
    #         cv2.rectangle(background, (int(bg_size*i), int(bg_size*j)), (int(bg_size), int(bg_size)), color, thickness)
    # cv2.circle(background, (int(bg_size*bg_cnt/2), int(bg_size*bg_cnt/2)),int(bg_x/bg_cnt/2), (0,0,255), -1 )    


    block_size = real_bg_size / bg_cnt

    for i in range(turtle_num):
        if(oc[i][0]<0):
            if(oc[i][1]<0):
                m_x[i] = int(oc[i][0] / block_size) -1
                m_y[i] = int(oc[i][1] / block_size) -1
            else :
                m_x[i] = int(oc[i][0] / block_size) -1
                m_y[i] = int(oc[i][1] / block_size) +1
        else:
            if(oc[i][1]<0):
                m_x[i] = int(oc[i][0] / block_size) +1
                m_y[i] = int(oc[i][1] / block_size) -1
            else :
                m_x[i] = int(oc[i][0] / block_size) +1
                m_y[i] = int(oc[i][1] / block_size) +1
        
        if(m_x[i] < 0):
            if(m_y[i] < 0):
                m_x[i] = m_x[i] * -1  #절댓값 씌워줌, 크기만 존재
                m_y[i] = m_y[i] * -1
                p_x = int ( bg_cnt/2 - m_x[i] )
                p_y = int (bg_cnt/2 + m_y[i] )
                turtle_mapping_loc[i][0] = int (p_x*bg_size)
                turtle_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) , m_color[i], -1)
            
            else:
                m_x[i] = m_x[i] * -1
                m_y[i] = m_y[i] * 1
                p_x = int ( bg_cnt/2 - m_x[i] )
                p_y = int (bg_cnt/2 - m_y[i] )
                turtle_mapping_loc[i][0] = int (p_x*bg_size)
                turtle_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) , m_color[i], -1)
            
        else:
            if(m_y[i] < 0):
                m_x[i] = m_x[i] * 1
                m_y[i] = m_y[i] * -1
                p_x = int ( bg_cnt/2 + m_x[i] )
                p_y = int (bg_cnt/2 + m_y[i] )
                turtle_mapping_loc[i][0] = int (p_x*bg_size)
                turtle_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) , m_color[i], -1)
            else:
                m_x[i] = m_x[i] * 1
                m_y[i] = m_y[i] * 1
                p_x = int ( bg_cnt/2 + m_x[i] )
                p_y = int (bg_cnt/2 - m_y[i] )
                turtle_mapping_loc[i][0] = int (p_x*bg_size)
                turtle_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) , m_color[i], -1)
   
    #들어야 할 짐 그래프 표시
    for i in range(4):
        if(burden_oc[i][0]<0):
            if(burden_oc[i][1]<0):
                burden_m_x[i] = int(burden_oc[i][0] / block_size) -1
                burden_m_y[i] = int(burden_oc[i][1] / block_size) -1
            else :
                burden_m_x[i] = int(burden_oc[i][0] / block_size) -1
                burden_m_y[i] = int(burden_oc[i][1] / block_size) +1
        else:
            if(burden_oc[i][1]<0):
                burden_m_x[i] = int(burden_oc[i][0] / block_size) +1
                burden_m_y[i] = int(burden_oc[i][1] / block_size) -1
            else :
                burden_m_x[i] = int(burden_oc[i][0] / block_size) +1
                burden_m_y[i] = int(burden_oc[i][1] / block_size) +1
       
        if(burden_m_x[i] < 0):
            if(burden_m_y[i] < 0):
                burden_m_x[i] = burden_m_x[i] * -1  #절댓값 씌워줌, 크기만 존재
                burden_m_y[i] = burden_m_y[i] * -1
                p_x = int ( bg_cnt/2 - burden_m_x[i] )
                p_y = int (bg_cnt/2 + burden_m_y[i] )
                burden_mapping_loc[i][0] = int (p_x*bg_size)
                burden_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) , (0,0,0), -1)
            
            else:
                burden_m_x[i] = burden_m_x[i] * -1
                burden_m_y[i] = burden_m_y[i] * 1
                p_x = int ( bg_cnt/2 - burden_m_x[i] )
                p_y = int (bg_cnt/2 - burden_m_y[i] )
                burden_mapping_loc[i][0] = int (p_x*bg_size)
                burden_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) ,(0,0,0), -1)
            
        else:
            if(burden_m_y[i] < 0):
                burden_m_x[i] = burden_m_x[i] * 1
                burden_m_y[i] = burden_m_y[i] * -1
                p_x = int ( bg_cnt/2 + burden_m_x[i] )
                p_y = int (bg_cnt/2 + burden_m_y[i] )
                burden_mapping_loc[i][0] = int (p_x*bg_size)
                burden_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) ,(0,0,0), -1)
            else:
                burden_m_x[i] = burden_m_x[i] * 1
                burden_m_y[i] = burden_m_y[i] * 1
                p_x = int ( bg_cnt/2 + burden_m_x[i] )
                p_y = int (bg_cnt/2 - burden_m_y[i] )
                burden_mapping_loc[i][0] = int (p_x*bg_size)
                burden_mapping_loc[i][1] = int(p_y*bg_size)
                # cv2.rectangle(background, (int (p_x*bg_size), int(p_y*bg_size)), ( int( (p_x+1)*bg_size), int( (p_y+1)*bg_size)) ,(0,0,0) , -1)
    
    
    
    # cv2.imshow("White Square", background)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return bg_cnt, turtle_mapping_loc, burden_mapping_loc

def get_points(loop_exit):
    Aruco_Dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    goal_l = np.zeros((16,2), dtype=int)
    goal_cnt = np.zeros(4)



    real_sqaure_size = 13 #실제 네모 큐브의 길이 (cm)
    # Capturing video through webcam 
    webcam = DepthCamera()

    turtle_num = 4 # 1번은 빨강, 2번은 초록 3번은 파랑, 4번은 보라
    count = 10 # 한 번 계산에 turtle_point를 선택할 것인가
    turtle_point = np.full((turtle_num, count), None, dtype=object)
    m_turtle_point = np.empty((turtle_num, 2), dtype=int)
    color_count = np.zeros (turtle_num, dtype=int) 

    cnt_i = 20
    cnt_real_size = np.empty((turtle_num, cnt_i), dtype=object)
    cnt =[0]*turtle_num #각 터틀봇 마다 넣은 값의 수
    done = np.zeros(turtle_num) # 터틀봇 구역 정하는게 다 끝났는지


    k_real_size = np.empty((turtle_num, 2), dtype=float)
    burden_real_size = np.empty((4, 2), dtype=float)

    re_turtle_point = np.empty((turtle_num,2,1) ,dtype=int)     

    re_left_side = np.array([[0, 0], [0, 0]])
    x_vector = np.empty((0,2), dtype=int)
    y_vector = np.empty((0,2), dtype=int)
    pixel_x_size = 0
    pixel_y_size = 0
    zero = np.array([0,0]) #원점.

    color = ["빨강", "초록", "파랑", "보라"]
    m_color = [
            (81, 81 , 235), #빨강
            (45, 166, 73), # 초록
            (246, 130, 50), # 파랑
            (245, 43, 115) # 보라
        ]

    while(1): 
        _ ,_,imageFrame= webcam.get_frame()
        imageFrame = imageFrame[:,200:1000]
        aruco_image = imageFrame


        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

        low_v = 150
        red_lower = np.array([160, 70, low_v], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
        
        
        purple_lower = np.array([120, 70, low_v], np.uint8) 
        purple_upper = np.array([150, 255, 255], np.uint8) 
        purple_mask = cv2.inRange(hsvFrame, purple_lower, purple_upper)

        green_lower = np.array([70, 30, low_v], np.uint8) 
        green_upper = np.array([80, 255, 255], np.uint8) 
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 


        yellow_lower = np.array([20, 10, 254], np.uint8)
        yellow_upper = np.array([30, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)

        blue_lower = np.array([90, 180, low_v], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)



        kernel = np.ones((5, 5), "uint8") 
        
        # For red color 
        red_mask = cv2.dilate(red_mask, kernel) 
        res_red = cv2.bitwise_and(imageFrame, imageFrame,  
                                mask = red_mask) 
        
        # For green color 
        green_mask = cv2.dilate(green_mask, kernel) 
        res_green = cv2.bitwise_and(imageFrame, imageFrame, 
                                    mask = green_mask) 
        
        # For yellow color 
        yellow_mask = cv2.dilate(yellow_mask, kernel) 
        res_yellow = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = yellow_mask) 

        # For yellow color 
        blue_mask= cv2.dilate(blue_mask, kernel) 
        res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = blue_mask) 
        # For purple color 
        purple_mask = cv2.dilate(purple_mask, kernel) 
        res_purple = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = purple_mask) 

        
    ############################################
        Aruco_Params = cv2.aruco.DetectorParameters_create()
        (marker_corners, marker_id, rejected_markers) = cv2.aruco.detectMarkers(
            aruco_image, Aruco_Dict, parameters=Aruco_Params
        )
        if len(marker_corners) > 0:
            marker_id = marker_id.flatten()
            ang = np.empty((0,2), dtype=int)
            for (markerCorner, markerID) in zip(marker_corners, marker_id):
                
                if(markerID == 0):
                    marker_corners = markerCorner.reshape((4, 2))
                    (p1, p2, p3, p4) = marker_corners
                    for i in range(4):
                        ang = np.append(ang, [marker_corners[i]], axis=0)
                    sums = np.sum(ang, axis=1)
                    min = np.argmin(sums)
                    diff = np.diff(ang, axis=1)
                    top_left = ang[min]
                    top_right = ang[np.argmin(diff)]
                    bottom_left = ang[np.argmax(diff)]

                    imageFrame = cv2.circle(imageFrame, (int(top_left[0]), int(top_left[1])), 10,(0,0,255),-1)
                    imageFrame = cv2.circle(imageFrame, (int(top_right[0]), int(top_right[1])), 10,(0,255,0),-1)
                    imageFrame = cv2.circle(imageFrame, (int(bottom_left[0]), int(bottom_left[1])), 10,(255,0,0),-1)

                    pixel_x_size = np.sqrt((top_left[0]-top_right[0])**2 + (top_left[1]-top_right[1])**2 )
                    pixel_y_size = np.sqrt((top_left[0]-bottom_left[0])**2 + (top_left[1]-bottom_left[1])**2)
                    x_vector = ((top_right[0]-top_left[0])/pixel_x_size, (top_right[1]-top_left[1])/pixel_x_size) 
                    y_vector = ((top_left[0]-bottom_left[0])/pixel_y_size, (top_left[1]-bottom_left[1])/pixel_y_size)
                    
                    left_side = np.array([[x_vector[0], y_vector[0]], [x_vector[1], y_vector[1]]])
                    re_left_side = np.linalg.inv(left_side)


                    D= ang.mean(axis=0) #원점.
                    D = D.ravel()
                    D = np.int32(D)
                    zero = D
                    #print(D.shape) 
                else :
                        if(markerID == 1):
                            marker_corners = markerCorner.reshape((4, 2))

                            for i in range(4):
                                goal_l[i][0] = marker_corners[i, 0]
                                goal_l[i][1] = marker_corners[i, 1]
                                
                            goal_cnt[0] = 1 
                        
                        elif(markerID == 2):
                            marker_corners = markerCorner.reshape((4, 2))
                            for i in range(4):
                                goal_l[i+4][0] = marker_corners[i][0]
                                goal_l[i+4][1] = marker_corners[i][1]
                            goal_cnt[1] = 1 
                        
                        elif(markerID == 3):
                            marker_corners = markerCorner.reshape((4, 2))
                            for i in range(4):
                                goal_l[i+8][0] = marker_corners[i][0]
                                goal_l[i+8][1] = marker_corners[i][1]
                            goal_cnt[2] = 1 
                        
                        elif(markerID == 4):
                            marker_corners = markerCorner.reshape((4, 2))
                            for i in range(4):
                                goal_l[i+12][0] = marker_corners[i][0]
                                goal_l[i+12][1] = marker_corners[i][1]
                            goal_cnt[3] = 1   
      
        if np.all(goal_cnt == 1):
            sums = np.sum(goal_l, axis=1)
            min = np.argmin(sums)
            max = np.argmax(sums)
            diff = np.diff(goal_l, axis=1)
            top_left = goal_l[min]
            top_right = goal_l[np.argmin(diff)]
            bottom_left = goal_l[np.argmax(diff)]
            bottom_right = goal_l[max]
            
            aruco_image = cv2.circle(aruco_image, (int(top_left[0]), int(top_left[1])), 10,(0,0,255),-1)
            aruco_image = cv2.circle(aruco_image, (int(top_right[0]), int(top_right[1])), 10,(0,255,0),-1)
            aruco_image = cv2.circle(aruco_image, (int(bottom_left[0]), int(bottom_left[1])), 10,(255,0,0),-1)
            aruco_image = cv2.circle(aruco_image, (int(bottom_right[0]), int(bottom_right[1])), 10,(100,150,100),-1)

            burden_real_size[0] = get_point_b(top_left[0], top_left[1], zero, re_left_side, pixel_x_size, pixel_y_size, real_sqaure_size)
            burden_real_size[1] = get_point_b(top_right[0], top_right[1], zero, re_left_side, pixel_x_size, pixel_y_size, real_sqaure_size)
            burden_real_size[2] = get_point_b(bottom_left[0], bottom_left[1], zero, re_left_side, pixel_x_size, pixel_y_size, real_sqaure_size)
            burden_real_size[3] = get_point_b(bottom_right[0], bottom_right[1], zero, re_left_side, pixel_x_size, pixel_y_size, real_sqaure_size)

        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            if(zero is not None):
                get = get_point_t(imageFrame,pic, contour, 0, m_color, zero, color_count, count, turtle_point, m_turtle_point, re_turtle_point, re_left_side, real_sqaure_size,
                                    pixel_x_size, pixel_y_size)
            
            if get is not None:
                r_get = (get[0], get[1])
                if(cnt[0]<cnt_i):
                    cnt_real_size[0][cnt[0]] = r_get
                
                
                    if (cnt_real_size[0][cnt[0]] is None):
                        continue
                    else:
                        cnt[0] += 1
                        print(color[0], "[","0"*cnt[0]," "*(cnt_i-cnt[0]),"]")
                else:
                    done[0] = 1
                    continue
            
            
        
        contours, hierarchy = cv2.findContours(green_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            if(zero is not None):
                get = get_point_t(imageFrame,pic, contour, 1, m_color, zero, color_count, count, turtle_point, m_turtle_point, re_turtle_point, re_left_side, real_sqaure_size,
                                    pixel_x_size, pixel_y_size)
            
            if get is not None:
                r_get = (get[0], get[1])
                if(cnt[1]<cnt_i):
                    cnt_real_size[1][cnt[1]] = r_get
                
                    if (cnt_real_size[1][cnt[1]] is None):
                        continue
                
                    else:
                        cnt[1] += 1
                        print(color[1], "[","0"*cnt[1]," "*(cnt_i-cnt[1]),"]")
                else:
                    done[1] = 1
                    continue
            
        contours, hierarchy = cv2.findContours(blue_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            if(zero is not None):
                get = get_point_t(imageFrame,pic, contour, 2, m_color, zero, color_count, count, turtle_point, m_turtle_point, re_turtle_point, re_left_side, real_sqaure_size,
                                    pixel_x_size, pixel_y_size)
            if get is not None:
                r_get = (get[0], get[1])
                if(cnt[2]<cnt_i):
                    cnt_real_size[2][cnt[2]] = r_get
                
                    if (cnt_real_size[2][cnt[2]] is None):
                        continue
                
                    else:
                        cnt[2] += 1
                        print(color[2], "[","0"*cnt[2]," "*(cnt_i-cnt[2]),"]")
                else:
                    done[2] = 1
                    continue

        contours, hierarchy = cv2.findContours(purple_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            if(zero is not None):
                get = get_point_t(imageFrame,pic, contour, 3, m_color, zero, color_count, count, turtle_point, m_turtle_point, re_turtle_point, re_left_side, real_sqaure_size,
                                    pixel_x_size, pixel_y_size)
            if get is not None:
                r_get = (get[0], get[1])
                if(cnt[3]<cnt_i):
                    cnt_real_size[3][cnt[3]] = r_get
                
                    if (cnt_real_size[3][cnt[3]] is None):
                        continue
                
                    else:
                        cnt[3] += 1
                        print(color[3], "[","0"*cnt[3]," "*(cnt_i-cnt[3]),"]")
                else:
                    done[3] = 1
                    continue
        
        if np.all(done == 1):
            break
            
        
        # Program Termination
        
                
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
        if cv2.waitKey(10) & 0xFF == ord('q'): 
            webcam.release() 
            cv2.destroyAllWindows() 
            break
    
    
    for k in range(turtle_num):
        x = np.empty(cnt[k], dtype=float)
        y = np.empty(cnt[k], dtype=float)
        for i in range(cnt[k]):
            x[i] = cnt_real_size[k][i][0]
            y[i] = cnt_real_size[k][i][1]
        x_std = np.std(x)
        y_std = np.std(y)

        r_x = x[np.abs(x-np.mean(x) < x_std)]
        r_y = y[np.abs(y-np.mean(y) < y_std)]

        if ( np.isnan(np.mean(r_x)) or np.isnan(np.mean(r_y)) ):
            print( color[k], "값을 인식하지 못했습니다. 다시 진행하겠습니다.", "남은 횟수 : ",1-loop_exit)
            if(loop_exit >= 1):
                k_real_size[k][0] = np.median(x[i])
                k_real_size[k][1] = np.median(y[i])
            else:
                loop_exit += 1
                get_points(loop_exit)
        else :
            k_real_size[k] = (np.mean(r_x), np.mean(r_y))

    for i in range (turtle_num):
        print(color[i], " 좌표 = ", k_real_size[i])

    
    piece, final_turtle, final_burden = mapping(k_real_size, turtle_num, burden_real_size, m_color)
    
    # print(piece)
    # print(final_turtle)
    # print(final_turtle.shape)
    # print(final_burden)
    # print(final_burden.shape)
    return piece, final_turtle, final_burden
    
get_points(loop_exit)
