import copy
from typing import List
import time
import tkinter as tk
import tkinter.messagebox
# from vehicle import Vehicle
from config import VEHICLE_MOVEMENT_DIRECTION_TO_EAST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_WEST
from config import VEHICLE_MOVEMENT_DIRECTION_TO_NORTH
from config import VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH
from config import BLOCK_DIST
from config import INTERSECTION_DIST
from config import BLOCKS
from config import INTERSECTIONS
from config import Point
from config import ROAD_MODEL
from config import X_OF_INTERSECTIONS
from config import Y_OF_INTERSECTIONS
from config import X_MIN
from config import X_MAX
from config import Y_MIN
from config import Y_MAX
from config import point_in_road_network
from config import START_STEP
from config import END_STEP
from vehicle import Vehicle
from vehicle import calc_appeared_vehicles
from vehicle import calc_vehicle_flow
from vehicle import transfer_flow_to_list_of_vehicles

from config import TRAFFIC_LIGHT_EASTWEST_PASS
from config import TRAFFIC_LIGHT_EASTWESTYELLOW_NORTHSOUTHRED
from config import TRAFFIC_LIGHT_NORTHSOUTH_PASS
from config import TRAFFIC_LIGHT_EASTWESTRED_NORTHSOUTHYELLOW

TRAIANGLE_WIDTH = int(0.5 * BLOCK_DIST)
TRAIANGLE_LENGTH = int(BLOCK_DIST)
WINDOW_WIDTH = 1200 if ROAD_MODEL in [0, 1] else 1850
WINDOW_HEIGHT = 1200
OFFSET = 6
if ROAD_MODEL == 0:
    SCALE = 50
elif ROAD_MODEL == 1:
    SCALE = 20
elif ROAD_MODEL == 2:
    SCALE = 3

# input: point, center of the short-edge of triangle
def calc_points_for_triangle(point: list, direction: str):
    assert len(point) == 2
    p1 = copy.deepcopy(point)
    p2 = copy.deepcopy(point)
    p3 = copy.deepcopy(point)
    if direction == VEHICLE_MOVEMENT_DIRECTION_TO_EAST:
        p1[1] += SCALE * TRAIANGLE_WIDTH / 2
        p2[1] -= SCALE * TRAIANGLE_WIDTH / 2
        p3[0] += SCALE * TRAIANGLE_LENGTH
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_WEST:
        p1[1] += SCALE * TRAIANGLE_WIDTH / 2
        p2[1] -= SCALE * TRAIANGLE_WIDTH / 2
        p3[0] -= SCALE * TRAIANGLE_LENGTH
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_NORTH:
        p1[0] += SCALE * TRAIANGLE_WIDTH / 2
        p2[0] -= SCALE * TRAIANGLE_WIDTH / 2
        p3[1] -= SCALE * TRAIANGLE_LENGTH
    elif direction == VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH:
        p1[0] += SCALE * TRAIANGLE_WIDTH / 2
        p2[0] -= SCALE * TRAIANGLE_WIDTH / 2
        p3[1] += SCALE * TRAIANGLE_LENGTH
    points = [p1, p2, p3]
    # scale_points = copy.deepcopy(points)
    # for i in range(len(points)):
    #     scale_points[i][0] *= SCALE
    #     scale_points[i][1] *= SCALE
    return points

# transfer and scale, output: [1, 2]
def transfer_point(old_point: Point) -> List[int]:
    new_point = Point.nil_point()
    if ROAD_MODEL == 0:
        old_intersection = INTERSECTIONS[0]
        new_intersection = Point(1.5 * BLOCK_DIST, 1.5 * BLOCK_DIST)
    elif ROAD_MODEL == 1:
        old_intersection = INTERSECTIONS[0]
        new_intersection = Point(5 * BLOCK_DIST, 5 * BLOCK_DIST)
    elif ROAD_MODEL == 2:
        old_intersection = Point(X_OF_INTERSECTIONS[4], Y_OF_INTERSECTIONS[2])
        new_intersection = Point(-X_MIN + 0.5 * BLOCK_DIST, Y_MAX + 0.5 * BLOCK_DIST)
    delta_x = new_intersection.x - old_intersection.x
    delta_y = new_intersection.y + old_intersection.y
    new_point.x = old_point.x + delta_x + OFFSET
    new_point.y = -old_point.y + delta_y + OFFSET
    scale_new_point = [SCALE * new_point.x, SCALE * new_point.y]
    return scale_new_point



def calc_points_for_block_intersection(p: Point, type: str):
    assert type in ['block', 'intersection']
    delta_dist = 0.5 * BLOCK_DIST if type == 'block' else 0.5 * INTERSECTION_DIST
    p1 = copy.deepcopy(p)
    p2 = copy.deepcopy(p)
    p3 = copy.deepcopy(p)
    p4 = copy.deepcopy(p)
    p1.x -= delta_dist
    p1.y -= delta_dist
    p2.x += delta_dist
    p2.y -= delta_dist
    p3.x += delta_dist
    p3.y += delta_dist
    p4.x -= delta_dist
    p4.y += delta_dist
    ps = [p1, p2, p3, p4]
    if type == 'intersection':
        # for u in range(len(ps)):
        #     ps[u].x *= 0.999
        #     ps[u].y *= 0.999
        pass
    transfer_ps = [transfer_point(r) for r in ps]
    # scale_qs = [[SCALE * t.x, SCALE * t.y] for t in transfer_ps]
    return transfer_ps


def plot_triangle(cv, point: list, direction: str):
    points = calc_points_for_triangle(point, direction)
    assert len(points) == 3
    # 根据点来连线
    cv.create_polygon(
        points,
        outline='black',  # 线的颜色
        fill='blue'  # 填充色
    )
    cv.pack()

def plot_block_intersection(cv, p: Point, type: str):
    points = calc_points_for_block_intersection(p, type)
    assert len(points) == 4
    # 根据点来连线
    cv.create_polygon(
        points,
        outline="Gray" if type == 'block' else 'black',  # 线的颜色
        fill='white'  # 填充色
    )
    cv.pack()

def plot_road_network(cv):
    start = 0
    end = 5500
    cnt = 0
    for p in BLOCKS:
        plot_block_intersection(cv, p, 'block')
        # cnt += 1
        # if start <= cnt <= end:
        #     plot_block_intersection(cv, p, 'block')

    plot_intersections = True
    if plot_intersections:
        start = 16
        end = 500
        cnt = 0
        for q in INTERSECTIONS:
            plot_block_intersection(cv, q, 'intersection')
            # cnt += 1
            # if start <= cnt <= end:
            #     plot_block_intersection(cv, q, 'intersection')


def plot_vehicle(cv, v: Vehicle):
    p = copy.deepcopy(v.point)
    if v.direction == VEHICLE_MOVEMENT_DIRECTION_TO_EAST:
        p.x -= 0.5 * BLOCK_DIST
    elif v.direction == VEHICLE_MOVEMENT_DIRECTION_TO_WEST:
        p.x += 0.5 * BLOCK_DIST
    elif v.direction == VEHICLE_MOVEMENT_DIRECTION_TO_NORTH:
        p.y -= 0.5 * BLOCK_DIST
    elif v.direction == VEHICLE_MOVEMENT_DIRECTION_TO_SOUTH:
        p.y += 0.5 * BLOCK_DIST
    # q = transfer_point(p)
    # scale_q = [SCALE * transfer_p.x, SCALE * transfer_p.y]
    q = transfer_point(p)
    plot_triangle(cv, point=q, direction=v.direction)

def plot_vehicles(cv, vs: List[Vehicle]):
    for i in range(len(vs)):
        plot_vehicle(cv, vs[i])

def plot_light(cv, p: Point, color: str):
    # k, j = 1, 1
    # # cv.create_oval(310 - k, 250 - k, 310 + k, 250 + k, width = 1)
    # for i in range(26):
    #     cv.create_oval(p.x - k, p.y - k, p.x + k, p.y + k, width=1,
    #                    outline=color,
    #                    fill=color)
    #     k += j
    #     j += 0.3
    if ROAD_MODEL == 0:
        offset = 12
    elif ROAD_MODEL == 1:
        offset = 9
    elif ROAD_MODEL == 2:
        offset = 3.5
    q = transfer_point(p)
    # scale_q = [SCALE * q.x, SCALE * q.y]
    cv.create_oval(q[0] - offset, q[1] - offset,
                   q[0] + offset, q[1] + offset,
                   width=1,
                   outline=color,
                   fill=color)

def plot_traffic_light_state(cv, traffic_light_state: int, intersection: Point):
    p_east = copy.deepcopy(intersection)
    p_west = copy.deepcopy(intersection)
    p_north = copy.deepcopy(intersection)
    p_south = copy.deepcopy(intersection)
    p_east.x -= int(0.5 * INTERSECTION_DIST)
    p_west.x += int(0.5 * INTERSECTION_DIST)
    p_north.y += int(0.5 * INTERSECTION_DIST)
    p_south.y -= int(0.5 * INTERSECTION_DIST)
    if traffic_light_state == TRAFFIC_LIGHT_EASTWEST_PASS:
        plot_light(cv, p_east, color='green')
        plot_light(cv, p_west, color='green')
        plot_light(cv, p_north, color='red')
        plot_light(cv, p_south, color='red')
    elif traffic_light_state == TRAFFIC_LIGHT_NORTHSOUTH_PASS:
        plot_light(cv, p_east, color='red')
        plot_light(cv, p_west, color='red')
        plot_light(cv, p_north, color='green')
        plot_light(cv, p_south, color='green')
    elif traffic_light_state == TRAFFIC_LIGHT_EASTWESTYELLOW_NORTHSOUTHRED:
        plot_light(cv, p_east, color='yellow')
        plot_light(cv, p_west, color='yellow')
        plot_light(cv, p_north, color='red')
        plot_light(cv, p_south, color='red')
    elif traffic_light_state == TRAFFIC_LIGHT_EASTWESTRED_NORTHSOUTHYELLOW:
        plot_light(cv, p_east, color='red')
        plot_light(cv, p_west, color='red')
        plot_light(cv, p_north, color='yellow')
        plot_light(cv, p_south, color='yellow')
    else:
        raise ValueError('wrong traffic_light_state')


def plot_step(step: int):
    pass

# window = tk.Tk()
# window.title('Road network')
# # window.geometry('1200x1200')
# cv = tk.Canvas(window,
#                width=WINDOW_WIDTH,
#                height=WINDOW_HEIGHT,
#                background="white")  # 创建一个Canvas，设置其背景色为红色
# cv.grid()


# cv.create_rectangle(10, 10, 2010, 2010)  # 创建一个矩形，坐标为(10,10,110,110)

# car = tk.ImageTk.PhotoImage(file="car.png")
# cv.create_image(0,0, image=car, anchor="nw")

# plot_triangle(cv=cv, point=[100, 100], direction='east')
# plot_triangle(cv=cv, point=[50, 50], direction='west')
# plot_triangle(cv=cv, point=[200, 200], direction='north')
# plot_triangle(cv=cv, point=[150, 150], direction='south')
# p = Point(0, 0)
# plot_block_intersection(cv, p, type='block')



# plot_road_network(cv=cv)

def visulization():
    # appeared_vehicles = calc_appeared_vehicles(0, 5)
    # v = appeared_vehicles[0]
    # plot_vehicle(cv, v)
    flow, queues, stationary_queues, traffic_light_states = calc_vehicle_flow(start_step=START_STEP, end_step=END_STEP)
    transferred_flow = transfer_flow_to_list_of_vehicles(flow)
    aaa = 1
    window = tk.Tk()
    window.title('Road network')
    # window.geometry('1200x1200')
    cv = tk.Canvas(window,
                   width=WINDOW_WIDTH,
                   height=WINDOW_HEIGHT,
                   background="white")  # 创建一个Canvas，设置其背景色为红色
    # plot_road_network(cv=cv)
    cv.grid()

    stop = 0

    def change_stop():  # 一个方法，每次按button就给b+1
        global stop
        stop = (stop + 1) % 2
        print(stop)

    # a = tk.Button(window, text="stop", command=change_stop)  # button，这里面的command就是调用前面的addOne方法

    for i in range(len(transferred_flow)):
        plot_road_network(cv=cv)
        plot_vehicles(cv, transferred_flow[i])
        for k in range(len(INTERSECTIONS)):
            plot_traffic_light_state(cv, traffic_light_states[i][k], INTERSECTIONS[k])

        textt = tk.StringVar()
        textt.set(str(i))
        resultText = tk.Label(window, textvariable=textt, fg='white', bg='grey', width=3)
        resultText.place(x=500, y=20)

        cv.pack()
        window.update()
        time.sleep(0.5)
    window.mainloop()

if __name__ == '__main__':
    visulization()