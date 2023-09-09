import cv2
import numpy as np
import math
import random
import argparse

class RRTNode:
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

def obstacle_free(x1,y1,x2,y2):
    color=[]
    x = list(np.arange(x1, x2, float(x2-x1)/100))
    x = np.array(x)
    y1 = np.array(y1)
    y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)
    color = [img[int(y[i]), int(x[i])] for i in range(len(x))]
    return True if 0 in color else False

def nearest_node(x,y):
    temp_distance=[]
    for i in range(len(node_list)):
        distance_list = distance(x,y,node_list[i].x,node_list[i].y)
        dist = distance_list[0]
        temp_distance.append(dist)
    min_index = min(enumerate(temp_distance), key=lambda x: x[1])[0]
    return min_index

def steer(x1,y1,x2,y2):
    _,theta = distance(x2,y2,x1,y1)
    x = x2 + stepSize * math.cos(theta)
    y = y2 + stepSize * math.sin(theta)
    height, width = img.shape[:2]
    direct_con = True
    node_con = True
    if not (0 <= x < width and 0 <= y < height):
        direct_con = False
        node_con = False
    else:
        direct_con = False if obstacle_free(x, y, end[0], end[1]) else True
        node_con = False if obstacle_free(x, y, x2, y2) else True

    return(x,y,direct_con,node_con)

def random_point_generator(y_coord,x_coord):
    new_x = random.randint(0, x_coord)
    new_y = random.randint(0, y_coord)
    return (new_x,new_y)

def distance(x1,y1,x2,y2):
    dist = math.hypot(x1-x2, y1-y2)
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)


def Build_RRT(img, img2, start, end, stepSize):
    x_coord,y_coord= img.shape   
    node_list[0] = RRTNode(start[0],start[1])
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])
    draw_circle(img2,start,4)
    draw_circle(img2,end,4)

    i=1
    pathExists = False
    while not pathExists:
        rand_x,rand_y = random_point_generator(x_coord,y_coord)
        nearest_index = nearest_node(rand_x,rand_y)
        nearest_x = node_list[nearest_index].x
        nearest_y = node_list[nearest_index].y
        steer_x,steer_y,direct_con,node_con = steer(rand_x,rand_y,nearest_x,nearest_y)

        if direct_con==True and node_con==True:
            new_node = RRTNode(steer_x, steer_y)
            new_node.parent_x = node_list[nearest_index].parent_x.copy() + [steer_x]
            new_node.parent_y = node_list[nearest_index].parent_y.copy() + [steer_y]
            node_list.append(new_node)
            draw_circle_withsteer(img2,steer_x,steer_y,3)
            cv2.line(img2, (int(steer_x),int(steer_y)), (int(node_list[nearest_index].x),int(node_list[nearest_index].y)), (0,255,0), thickness=1, lineType=8)
            cv2.line(img2, (int(steer_x),int(steer_y)), (end[0],end[1]), (255,0,0), thickness=3, lineType=8)
            for j in range(len(node_list[i].parent_x)-1):
                cv2.line(img2, (int(node_list[i].parent_x[j]),int(node_list[i].parent_y[j])), (int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])), (255,0,0), thickness=3, lineType=8)
            break

        elif node_con==True:
            new_node = RRTNode(steer_x, steer_y)
            new_node.parent_x = node_list[nearest_index].parent_x.copy() + [steer_x]
            new_node.parent_y = node_list[nearest_index].parent_y.copy() + [steer_y]
            node_list.append(new_node)
            i=i+1
            draw_circle_withsteer(img2,steer_x,steer_y,2)
            cv2.line(img2, (int(steer_x),int(steer_y)), (int(node_list[nearest_index].x),int(node_list[nearest_index].y)), (0,255,0), thickness=1, lineType=8)
            continue

        else:
            print("Generating new random point, no direct connection found")
            continue
    cv2.imwrite("map_"+args.imagePath,img2)
    cv2.destroyAllWindows()

def draw_circle(img,point, radius):
    cv2.circle(img, (point[0],point[1]), 3,(0,0,255),thickness=3, lineType=8)

def draw_circle_withsteer(img,point1, point2, radius):
    cv2.circle(img2, (int(point1),int(point2)), 2,(0,0,255),thickness=3, lineType=8)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'RRT path planner')
    parser.add_argument('imagePath', type=str, help='Input map file')
    parser.add_argument('start', type=int,default=[50,50],nargs=2, help='Start coordinates')
    parser.add_argument('stop', type=int, default=[400,200], nargs=2, help='Goal coordinates')
    # print("step 1")
    args = parser.parse_args()
    img = cv2.imread('maps/'+args.imagePath,0) 
    img2 = cv2.imread('maps/'+args.imagePath) 
    start = tuple(args.start) 
    end = tuple(args.stop) 
    stepSize = 20
    node_list = [0] 
    Build_RRT(img, img2, start, end, stepSize)


