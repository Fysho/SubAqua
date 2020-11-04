import math as m
import numpy as np
import CsvWriter
import PingInterface

ROBOT_SIZE = 0.56
MAX_W = m.pi/2
MAX_V = 1

range = 4
radius = ROBOT_SIZE/2+ROBOT_SIZE


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


def makeDecision():

    csvWriter = CsvWriter.CsvWriter("sonarData.csv")
    pingInterface = PingInterface.PingInterface(csvWriter)
    #pingInterface.readArgs()
    pingInterface.connectToPing()
    pingInterface.setSonarDistance(5);
    #data_vals = csvWriter.getEmptyCSV();
    #data_vals = pingInterface.transmitSweep(0, 360, data_vals)

    csvWriter.saveToCSV(data_vals)

    loops = 1;
    loopcount = 0;

    while(loopcount < loops):
        loopcount+= 1;
        data_vals = csvWriter.getEmptyCSV();
        data_vals, d0 = pingInterface.transmitAngle(90, data_vals)
        data_vals, d1 = pingInterface.transmitAngle(50, data_vals)
        data_vals, d2 = pingInterface.transmitAngle(30, data_vals)
        data_vals, d3 = pingInterface.transmitAngle(10, data_vals)
        data_vals, d4 = pingInterface.transmitAngle(-10, data_vals)
        data_vals, d5 = pingInterface.transmitAngle(-30, data_vals)
        data_vals, d6 = pingInterface.transmitAngle(-50, data_vals)
        data_vals, d7 = pingInterface.transmitAngle(-90, data_vals)

        # d0 = getDistance(90)
        # d1 = getDistance(50)
        # d2 = getDistance(30)
        # d3 = getDistance(10)
        # d4 = getDistance(-10)
        # d5 = getDistance(-30)
        # d6 = getDistance(-50)
        # d7 = getDistance(-90)
        csvWriter.saveToCSV(data_vals)

        p0 = Point(0, radius)
        p1 = Point(m.tan((90 - 50) * (m.pi / 180)) * (radius), radius)
        p2 = Point(m.tan((90 - 30) * (m.pi / 180)) * (radius), radius)
        p5 = Point(m.tan((90 - 30) * (m.pi / 180)) * (radius), -(radius))
        p6 = Point(m.tan((90 - 50) * (m.pi / 180)) * (radius), -(radius))
        p7 = Point(0, -(radius))

        dl_min = min(d0, d1, d2)
        dc_min = min(d2, d3, d4, d5)
        dr_min = min(d5, d6, d7)
        d_min = min(dl_min, dc_min, dr_min)

        op1 = m.sqrt(m.pow(p1.y, 2) + m.pow(p1.x, 2))
        op2 = m.sqrt(m.pow(p2.y, 2) + m.pow(p2.x, 2))

        if range <= d_min:
            weight = 1
        else:
            weight = 1

        if d_min < radius:  # walls within unsafe distance
            backwards()
        else:
            if min(d1, d6) >= op1:
                if dc_min >= op2:  # state 1
                    if min(dl_min, dr_min) > op2:
                        if(dl_min > dr_min and dc_min < dl_min - range):
                            x = getVW(-90)
                            move(x[0], x[1])
                            print("turn right")
                        elif dl_min > dr_min and dc_min < dl_min - range:
                            x = getVW(90)
                            move(x[0], x[1])
                            print("turn left")
                        else:
                            x = getVW(0)
                            move(x[0], x[1])
                            print("move forwards")
                    else:
                        x = getVW(0)
                        move(x[0], x[1])
                        print("move forward")
                else:  # state 2
                    if dc_min < dr_min - range:
                        x = getVW(-90)
                        move(x[0], x[1])
                        print("turn right")
                    else:
                        x = getVW(90)
                        move(x[0], x[1])
                        print("turn left")
            else:  # state 3
                if dl_min < op1 and dr_min > op1:
                    x = getVW(-90)
                    move(x[0], x[1])
                    print("turn right")
                elif dl_min > op1 and dr_min < op1:
                    x = getVW(90)
                    move(x[0], x[1])
                    print("turn left")
                else:
                    backwards()


def backwards():
    print("stop")

    d0 = getDistance(-90)
    d1 = getDistance(-135)
    d2 = getDistance(-180)
    d3 = getDistance(135)
    d4 = getDistance(90)

    min_r = min(d0, d1)
    min_l = min(d3, d4)
    m = min({min_r, min_l, d2})

    if m > radius:
        x = getVW(180)
        move(-x[0], x[1])
        print("Go backwards")
    else:
        if d2 > radius:
            if min_l > min_r:
                x = getVW(90)
                move(-x[0], x[1])
                print("Go back left")
            else:
                x = getVW(-90)
                move(-x[0], x[1])
                print("Go back right")
        else:
            weight = 0.5
            if min_l > min_r:
                x = getVW(90)
                move(-weight * x[0], x[1])
                print("Go back left")
            else:
                x = getVW(-90)
                move(-weight * x[0], x[1])
                print("Go back right")


def getVW(angle):
    t_0 = (180/m.pi) * m.atan(MAX_W/MAX_V)

    if t_0 > angle and -t_0 < angle:
        x = np.array([MAX_V, MAX_V*m.tan(angle)])
    elif (-90 < angle and -t_0 > angle) or (t_0 < angle and 90 > angle):
        x = np.array([MAX_W*abs(1/m.tan(angle)), MAX_W*sign(angle)])
    elif (-180 < angle and -180 + t_0 > angle) or (-180 - t_0 < angle and 180 > angle):
        x = np.array([MAX_V, MAX_V * m.tan(sign(angle)*m.pi - angle)])
    else:
        x = np.array(
            [MAX_W*abs(1/m.tan(sign(angle)*m.pi - angle)), MAX_W*sign(angle)])

    return x


def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def getDistance(angle):
    return angle


def move(v, w):
    pass

if __name__ == "__main__":
    makeDecision()
