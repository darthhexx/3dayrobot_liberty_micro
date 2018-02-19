import copy
import serial
import time
import numpy as np
from threading import Thread

# import libraries.car as car
import libraries.settings as lsettings
import libraries.bearings as bearings
import libraries.pid as lpid
import libraries.imuv1 as imu
import libraries.gpsv1 as gps

CAR_SERIAL_PORT  = '/dev/ttyUSB1'
MAX_TIME_DIFF    = 0.2 # Maximum time diff to accept gps
DIST_THRES_METER = 3.0

# Lat, Long
waypoints = [[-27.8552616667, 153.151291667],
             [-27.8553216667, 153.151405],
             [-27.8554266667, 153.151566667],
             [-27.8555, 153.151725],
             [-27.8555866667, 153.15189],
             [-27.8557016667, 153.152083333],
             [-27.855865, 153.152198333],
             [-27.85597, 153.152228333],
             [-27.8560216667, 153.152198333],
             [-27.8560783333, 153.15216],
             [-27.8561183333, 153.15206],
             [-27.85611, 153.151958333],
             [-27.856065, 153.151771667],
             [-27.85603, 153.151628333],
             [-27.8560016667, 153.151535],
             [-27.85596, 153.151348333],
             [-27.85584, 153.151086667],
             [-27.8555833333, 153.150935],
             [-27.8554083333, 153.150948333],
             [-27.8553283333, 153.150986667],
             [-27.8552716667, 153.151021667],
             [-27.855235, 153.151121667]]

pid_steer = lpid.PID(P=2.0, I=0.001, D=0.001)


def send_to_arduino(pos):
    return True


def read_GPS_until_success(point,threads):
    gps_success = False
    while not gps_success:
        tStamp, cord_lat, cord_long = threads[2].poll
        dt = tStamp - time.now()
        if (dt <= MAX_TIME_DIFF):
            gps_success = True
        if not gps_success:
            print('GPS Error %f',dt )
    return cord_lat, cord_long


def follow_point(point, threads):

    cord_lat, cord_long = read_GPS_until_success(point, threads)

    dist_to_p = bearings.coord_dist_meters(
        point[0], point[1], cord_lat, cord_long)

    while dist_to_p > DIST_THRES_METER:
        heading_want = bearings.coord_bearing_degrees(
            cord_lat, cord_long, point[0], point[1])
        heading_actual = threads[0].poll() # Should give back degrees
        heading_error = heading_want - heading_actual


        if heading_error > 180:
            heading_error = heading_error - 360
        elif heading_error <= -180:
            heading_error = heading_error + 360

        turn_strength_pid = pid_steer.update(heading_error)

        print('turn_strength_pid: ', turn_strength_pid, ', turn direction: ', 'left' if turn_strength_pid > 0 else 'right')
        send_to_arduino()

        time.sleep(0.01)

        # Get the update for the next loop
        cord_lat, cord_long = read_GPS_until_success(point, threads)
        dist_to_p = bearings.coord_dist_meters(
            point[0], point[1], cord_lat, cord_long)


if __name__ == '__main__':

    print("Starting IMU")
    imu_ = imu.SuperRoo_IMU()
    imu_t = Thread(target=imu_.main_loop, args=())
    imu_t.daemon = True
    imu_t.start()
    print("IMU Started")

    print("Starting GPS")
    gps_ = gps.SuperRoo_GPS()
    gps_t = Thread(target=gps_.main_loop, args=())
    gps_t.daemon = True
    gps_t.start()
    print("GPS started")

    print("Starting ObjRecognition")
    objRec_ = JoystickController()
    objRec_t = Thread(target=objRec.main_loop, args=())
    objRec_t.daemon = True
    objRec_t.start()
    print("ObjRec started")
    objRec_ = 0

    threads = [imu_, gps_, objRec_]

    t = 0
    t_max = 2.0
    delta_t = 0.2

    for p in waypoints:
        follow_point(p, threads)
        if t >= t_max:
            break;
    t += delta_t

#ToDo: Find closest waypoint, wrap around

