### UNITS:
# ANGLES: Degrees, ANTICLOCKWISE IS POSIIVE
# DISTANCES: CM
# 

from time import sleep
import RPi.GPIO as GPIO
import subprocess
import pygame
import math
import multiprocessing

import serial
import time
import RPi.GPIO as GPIO
import math
from tkinter import *
import tkinter as tk

import threading
import csv
import sys


X = 650
Y = 450

sleep(0.5)


X = 650
Y = 450
# UI setup----------
import tkinter as tk
from tkinter import *

from time import sleep

X = 650
Y = 450
# UI setup----------
ui = tk.Tk()
uiframe = tk.Frame(ui)
ui.geometry("650x450")
uicanvas = tk.Canvas(ui, width = X, height = Y, background = "#AAAAAA")
ui.title("Navigation robot UI")
uicanvas.pack()

plots = tk.Toplevel(ui)
container = tk.Frame(plots)
global W
global H
X = 650
Y = 450
fontsize = 15
canvas = tk.Canvas(plots, width = X, height = Y, background="#333333")
plots.title("Plot Data for development")
canvas.pack()
vert = canvas.create_line(X*0.5, 0, X*0.5, Y, fill="white")
hor = canvas.create_line(0, Y*0.5, X, Y*0.5, fill="white")
canvas.update()



pygame.init()
clock = pygame.time.Clock() #added line
ps3 = pygame.joystick.Joystick(0)
ps3.init()

global home
home = [0.1, 0.1, 0]
global currentPosition
currentPosition = home

global prevSubGoals
prevSubGoals = []

global dynamicObstacle
dynamicObstacle = False

global manualControlMode
manualControlMode = False

#----------------------------------------------------
#----------------------------------------------------

DIRLB = 20   # Direction GPIO Pin
STEPLB = 21  # Step GPIO Pin

DIRLF = 4
STEPLF = 27

DIRRF = 23
STEPRF = 24

DIRRB = 8
STEPRB = 7

CW = 1     # Clockwise Rotation
CCW = 0    # Counterclockwise Rotation
SPR = 200   # Steps per Revolution (360 / 7.5) 

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIRLB, GPIO.OUT)
GPIO.setup(STEPLB, GPIO.OUT)

GPIO.setup(DIRLF, GPIO.OUT)
GPIO.setup(STEPLF, GPIO.OUT)

GPIO.setup(DIRRF, GPIO.OUT)
GPIO.setup(STEPRF, GPIO.OUT)

GPIO.setup(DIRRB, GPIO.OUT)
GPIO.setup(STEPRB, GPIO.OUT)

#GPIO.output(DIRLB, CW)
#GPIO.output(DIRLF, CW)



step_count = SPR * 2
delay = 0.0011
num1 = 100  #ramping
num2 = 1.02

#### --------------------------------------
####    MANUAL CONTROL SUBROUTINES
#### --------------------------------------

def cwSpin(ratio, delay, dire, step): # for steppers
    GPIO.output(dire, CW)
    counter = 0
    GPIO.output(step, GPIO.HIGH)
    sleep(delay)
    GPIO.output(step, GPIO.LOW)
    sleep(delay)
def ccwSpin(ratio, delay, dire, step): # for steppers
    GPIO.output(dire, CCW)
    counter = 0
    GPIO.output(step, GPIO.HIGH)
    sleep(delay)
    GPIO.output(step, GPIO.LOW)
    sleep(delay)
    
def forward(delay, turn):
    axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
    counter = 0
    delay1 = delay
    dCurrentPosition = MagnitudeAndAngleToxy(0.0637743308678728, currentPosition[2], [0,0])
    dx = dCurrentPosition[0]
    dy = dCurrentPosition[1]
    if turn == 0: #MANUAL CONTROL 
        while axis[3] < -0.1:
            if counter < num1:
                delay1 = delay1 / num2
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            counter += 1
        return
    elif turn > 0: #PRECISE CONTROL
        servoangle(90)
        sleep(0.5)
        for x in range(int(step_count*turn)-num1):
            if counter < num1:
                delay1 = delay1 / num2
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            if counter % 200 == 0:
                d = getRangingData()
                print(currentPosition)
                if d == None: d = 700
                if d < 30:
                    print("Dynamic obstacle detected." )
                    sleep(5)
                    d = getRangingData()
                    if d == None: d = 700
                    if d < 30: 
                        dynamicObstacle = True
                        break
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        while counter < num1 and delay1 < delay: #ramping down
            delay1 = delay1 * num2
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            counter += 1
        return
def backward(delay, turn):
    turn = 1
    axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
    counter = 0
    delay1 = delay
    dCurrentPosition = MagnitudeAndAngleToxy(-0.0637743308678728, currentPosition[2], [0,0])
    dx = dCurrentPosition[0] # I INCREASED THE DPS TO REDUCE OFFSET
    dy = dCurrentPosition[1]
    if turn == 0:
        while axis[3] > 0.1:
            if counter < num1:
                delay1 = delay1 / num2      
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            counter += 1
        return
    elif turn > 0: #PRECISE CONTROL
        for x in range(int(step_count*turn)-num1):
            if counter < num1:
                delay1 = delay1 / num2      
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[0] += dx
            currentPosition[1] += dy
            counter += 1
        return

def spinLeft(delay, turn):
    axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
    counter = 0
    delay1 = delay
    da = 0.225 #degrees t
    if turn == 0:
        while axis[2] < -0.1:
            if counter < num1:
                delay1 = delay1 / num2
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            counter += 1
        return
    elif turn > 0: #PRECISE CONTROL
        delay1 = delay1 / 3
        for x in range(int(step_count*turn)):
            if counter < 30:
                delay1 = delay1 / num2      
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            #print(currentPosition)
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        '''while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            cwSpin(turn, delay1, DIRLB, STEPLB)
            cwSpin(turn, delay1, DIRLF, STEPLF)
            cwSpin(turn, delay1, DIRRB, STEPRB)
            cwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            counter += 1
        return'''
def spinRight(delay, turn):
    axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
    counter = 0
    delay1 = delay
    da = -0.225 # note the minus
    if turn == 0:
        
        while axis[2] > 0.1:
            if counter < num1:
                delay1 = delay1 / num2   
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            counter += 1
        return
    elif turn > 0:
        delay1 = delay1 / 3
        for x in range(int(step_count*turn)):
            if counter < 30:
                delay1 = delay1 / num2   
            ccwSpin(turn, delay1, DIRLB, STEPLB)
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            
            pygame.event.pump()
            axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
            counter += 1
        counter = 0
        '''while counter < num1 and delay1 < delay:
            delay1 = delay1 * num2
            ccwSpin(turn, delay1, DIRLB, STEPLB) # ramping down 
            ccwSpin(turn, delay1, DIRLF, STEPLF)
            ccwSpin(turn, delay1, DIRRB, STEPRB)
            ccwSpin(turn, delay1, DIRRF, STEPRF)
            currentPosition[2] += da
            counter += 1
        return''' # not sure if this is necessary, but not at the moment

def spin(turn):
    if turn > 0:
        spinLeft(delay, turn) # anticlockwise
    elif turn < 0:
        spinRight(delay, -turn)
    return

def manualControl():
    global manualControlMode
    print("Manual control thread is running ")
    while manualControlMode == False:
        sleep(0.5)
        
    while manualControlMode == True:
        counter = 0
        #delay1 = delay
        turn = 0
        pygame.event.pump()
        axis = [ps3.get_axis(0),ps3.get_axis(1),ps3.get_axis(3),ps3.get_axis(4), ps3.get_axis(5)]
        print(axis)
        
        if axis[3] < -0.1:
            forward(delay, turn)
            
            
        if axis [3] > 0.1:
            backward(delay, turn)
            
        if axis[2] < -0.1:
            spinLeft(delay, turn)
            
            
        if axis[2] > 0.1:
            spinRight(delay, turn)
    manualControl()
thread = threading.Thread(target = manualControl)
thread.start()




def distanceToTurns(distance): #translates a distance value(cm) to turns of the wheel
    diameter = 8.15
    pi = math.pi
    circumference = diameter * pi
    turns = distance / circumference
    return turns

def angleToTurns(angle): # angles in degrees
    turns = angle/90 ## NOTE: if you change this, you have to change da above
    return turns





def createPlots(l, size, colour):
    XO = X*0.5
    YO = Y*0.5
    global listpoints
    listpoints = []
    damping = 0.5
    for x in l:
        for i in x:
            point = canvas.create_oval(XO-size+i[0]*damping,
                                       YO-size-i[1]*damping,
                                       XO+size+i[0]*damping,
                                       YO+size-i[1]*damping,
                                       fill=colour)
            listpoints.append(point)

    ui.update()
    plots.update()
    return listpoints

def redefinepoint(ind, p):
    xMove = listpoints[ind].coords[0] - p[0]
    yMove = listpoints[ind].coords[1] - p[1]
    animation(listpoints[ind], xMove, Ymove)
    

time.sleep(1)
def animation(obj, xMove, yMove):
    canvas.move(obj, xMove, yMove)
    canvas.update()
    canvas.after(10)
    
def changetext(l, txt):
    l.config(text=txt)
    container.pack()







#------------------------------------------------------------------
ser= serial.Serial("/dev/ttyS0", 115200)

#GPIO.setmode(GPIO.BCM) #GPIO numbering mode 
GPIO.setup(6 ,GPIO.OUT) #Pin 31 is output
servo1 = GPIO.PWM(6 ,50) # 31 is pin, 50 = 50Hz pulse
servo1.start(0)


def getRangingData():
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)  
        ser.reset_input_buffer()
        ser.reset_output_buffer()  
        # type(recv), 'str' in python2(recv[0] = 'Y'), 'bytes' in python3(recv[0] = 89)
        # type(recv[0]), 'str' in python2, 'int' in python3 
        if recv[0] == 0x59 and recv[1] == 0x59:     #python3
            distance = recv[2] + recv[3] * 256
            strength = recv[4] + recv[5] * 256
            print('(', distance, ',', strength, ')')
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            #print("DISTANCE:", distance)
            if distance == 0:
                distance = 700
            elif distance == None:
                sleep(0.3)
                distance = 0.1
                print("NONETYOE VALUE")
                distance = getRangingData()
            
            return distance

def scan(res):
    time.sleep(0.5)
    if ser.is_open == False:
        ser.open()
    l = []
    currentangle = 0
    while currentangle <= 180:
        servoangle(currentangle)
        distance = getRangingData()
        sleep(0.05)
        if distance == 0:
            distance = 700
        elif distance == None:# Sometimes if the signal is weak,
            sleep(0.3)
            continue           # the sensor can return a None value.
        l.append([])
        l[-1].append(-(currentangle-90)) # angle THEN
        l[-1].append(distance) #        distance
        currentangle += res
    return l # l is the list of ranging data and their respective angles


def MagnitudeAndAngleToxy(mag, angle, a):
    #angle = angle-90 # offset # at a, plus mag and angle, what pos would i be at?
    #CHECK ANGLE OFFSETS COULD BE PLUS OR MINUS IM NOT SURE    
    xplus = mag*(math.cos(math.radians(angle))) # x # added minis
    yplus = mag*(math.sin(math.radians(angle))) # y
    
    b = [(a[0] + xplus), (a[1] + yplus)]
    b[0], b[1] = round(b[0], 8), round(b[1], 8)
    return b  #  THIS NEEDS WORK URGENTLY IDK IF WE CAN SAY angle-90


def servoangle(angle): #angles in degrees
    #1.972
    #12.97
       
    #duty = 2.5 + (angle*0.0527777) # 0.05277777 = (12-2.5) / 180 = 9.5 / 180
    duty = 1.9722 + (angle*0.060107936)
    servo1.ChangeDutyCycle(duty)
    if angle == 0:
        time.sleep(1)
    time.sleep(0.1)

def distance(a, b):  # euclidean distance
    x_dist = b[0] - a[0]  # b = 2,  a =1
    y_dist = b[1] - a[1]
    distance = math.sqrt((x_dist)**2 + (y_dist)**2)
    return distance

def xyToMagnitudeAndAngle(a, b): # Note: a contains 'angle position' data 
    mag = distance(a, b)
    try:
        relativey = b[1] - a[1]
        relativex = b[0] - a[0]
        #grad = (b[1] - a[1]) / (b[0] - a[0])
    except:
        relativey = 5
        relativex = 0.00000001
        grad = 99999
    angle = (math.degrees(math.atan2(relativey, relativex)) - a[2]) # atan2 is more suited to x and y coordinate systems
    print("MAG, ANGLE:",mag, angle)
    return mag, angle

def createNodes(distanceAndAngleData, currentPosition):
    standardUnit = 10 # cm
    coordlist = []
    #counterDistance = standardUnit
    for x in distanceAndAngleData:
        counterDistance = standardUnit
        coordlist.append([])
        while counterDistance < x[1]:          
            coords = MagnitudeAndAngleToxy((counterDistance), (currentPosition[2]-x[0]), currentPosition)
            x.append(coords)
            coordlist[-1].append(coords)
            counterDistance += standardUnit

        coords = MagnitudeAndAngleToxy((x[1]), (currentPosition[2]-x[0]), currentPosition)
        #end data point in list will be 'wall' coords        
        coordlist[-1].append(coords)
    
    return coordlist # , distanceAndAngleData  if you want to have that in the same list

def parseNodes(nodesList, currentPosition, goalNode):
    radius = 25 # cm
    largerRadius = 40 #cm
    infNodes = []
    endnodes = []
    # This should ensure that the robot goes outside of it's range each time
    for x in nodesList:
        endnodes.append(x[-1])
    
    for i in nodesList: # i is a whole list of nodes from a single ranging
        for n in i:
            if ((n[0] - (i[-1][0]))**2 + ((n[1] - i[-1][1])**2)) <= (radius**2):
                n.append(math.inf)  #99999#math.inf is an infinite value.
                infNodes.append(n)
            else:
                g = distance(currentPosition, n)
                h = distance(n, goalNode)
                f = g + 2*h
                n.append(f)
        #i[-1].append(math.inf)
    for i in nodesList:
        for n in i:
            for x in endnodes:
                if ((n[0] - (x[0]))**2 + (n[1] - x[1])**2) <= (radius**2) and n[2] != 99999:
                    n[2] = (math.inf)
                    for a in i[(i.index(n)+1):-1]: # rest of the line is inf
                        a[2] = (math.inf)
    print("we got this far tho")
    for subGoal in prevSubGoals:
        print("WE SEARCHING SUBGOALLSSSSSSSSS")
        for i in nodesList:
            for n in i:
                if ((n[0] - subGoal[0])**2 + (n[1] - subGoal[1])**2) <= (largerRadius**2):
                    n[2] = (math.inf) 
    return nodesList


def AStarDecision(costedNodesList):
    bestNode = []
    bestCost = math.inf
    for i in costedNodesList:
        for n in i:
            if n[2] < bestCost:
                bestCost = n[2]
                bestNode = n
    return bestNode
            
def getScanCoords(currentPosition):
    angleresoloution = 3 #degrees
    danda = scan(angleresoloution)
    coordlist = createNodes(danda, currentPosition)
    return coordlist

def lineOfSight(currentPosition, goalNode):
    b = getRangingData()
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.5)
    mag, angle = xyToMagnitudeAndAngle(currentPosition, goalNode)
    spin(angleToTurns(angle))
    #currentPosition[2] += angle
    servoangle(90)
    sleep(1)
    
    ranging = 0
    for _ in range(4):
        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            sleep(0.5)
            ranging1 = getRangingData()
            ranging += ranging1
            
            sleep(0.5)
        except:
            continue
        
    ranging = ranging / 4
    print("RANGING:", ranging)
    sleep(1)
    if ranging > mag:
        return True
    else:
        spin(angleToTurns(-angle))
        #currentPosition[2] += -angle
        return False
    
def navToGoalMain(goalNode):
    #backward(delay, (distanceToTurns(7)))
    coordList1 = getScanCoords(currentPosition) # scans, and creates nodes
    #forward(delay, (distanceToTurns(7)))
    turn = angleToTurns(180)
    spin(turn)
    #currentPosition[2] += 180
    #backward(delay, (distanceToTurns(7)))
        
    coordList2 = getScanCoords(currentPosition)
    coordList = coordList1 + coordList2
    #forward(delay, (distanceToTurns(7)))
        
    costedNodesList = parseNodes(coordList, currentPosition, goalNode)
    infNodes = []
    nNodes = []
    redNodes = []
    for i in costedNodesList:
        for n in i:
            if n[2] == 99999:
                redNodes.append(n)
            elif n[2] == math.inf:
                infNodes.append(n)
            else:
                nNodes.append(n)
    print(nNodes)
    createPlots([infNodes], 1.5, "#FFFFFF", )
    createPlots([nNodes], 1.5, "#08FF08")
    createPlots([redNodes], 2, "#FF0000")
        
    bestNode = AStarDecision(costedNodesList)
    print("BEST NODE:", bestNode)
    subGoal = navToSubGoal(bestNode)
    prevSubGoals.append(subGoal)
    createPlots([[subGoal]], 3, "#1100FF")
    
    



def navToGoal(goalNode):
    global prevSubGoals
    global currentPosition
    prevSubGoals.clear()
    prevSubGoals.append(currentPosition)
    print("prevSUBGOALS: ", prevSubGoals)
    print("LETS NAV TO GOAL NOW " )
    mag, angle = xyToMagnitudeAndAngle(currentPosition, [currentPosition[0]+5, currentPosition[1]])
    spinTurns = angleToTurns(angle)
    spin(spinTurns)
    print(currentPosition)
    #------------------
    radius = 30 #cm # success radius
   
    
    createPlots([[currentPosition, goalNode]], 10, "#FFD4A1")
    print("WE DID THE plots")
    #los = lineOfSight(currentPosition, goalNode)
    los = False
    print("currentPosition: ", currentPosition)
    print("goalNode:", goalNode)
    print("THIS IS THE START THO")
    if los == True: # already at angle facing the goal therefore...
        mag, angle = xyToMagnitudeAndAngle(currentPosition, goalNode)
        forward(delay, distanceToTurns(mag))
        #currentPosition[0] += (mag*math.cos(math.radians(currentPosition[2])))
        #currentPosition[1] += (mag*math.sin(math.radians(currentPosition[2])))
    else:
        print("No direct line of sight")
        while (currentPosition[0]-goalNode[0])**2 + (currentPosition[1]-goalNode[1])**2 > radius**2:
            navToGoalMain(goalNode)
            print("Current position:", currentPosition)
            sleep(1)
            
            #los = lineOfSight(currentPosition, goalNode)
            los = False
            if los == True: # already at angle facing the goal
                mag, angle = xyToMagnitudeAndAngle(currentPosition, goalNode)
                forward(delay, distanceToTurns(mag))
                #currentPosition[0] += (mag*math.cos(math.radians(currentPosition[2])))
                #currentPosition[1] += (mag*math.sin(math.radians(currentPosition[2])))


    print("Current position:", currentPosition)
    print("I AM AT THE GOAL")
    sleep(1)
    return
    
    

    
def navToSubGoal(subGoal):
    global dynamicObstacle
    dynamicObstacle = False
    print(currentPosition, "GOING INTO xytoMandA")
    mag, angle = xyToMagnitudeAndAngle(currentPosition, subGoal)
    spinTurns = angleToTurns(angle)
    spin(spinTurns)
    ###currentPosition[2] += angle
    turns = distanceToTurns(mag)
    forward(delay, turns)
    if dynamicObstacle == True:
        
        dynamicObstacle = False
        subGoal = currentPosition
    #currentPosition[0] += (mag*math.cos(math.radians(currentPosition[2])))
    #currentPosition[1] += (mag*math.sin(math.radians(currentPosition[2])))
    print(currentPosition, "FROM NAVTOSUBGOAL  ")
    return subGoal


    
#-------------------------------------------------------------------



location1 = []  #these should be stored locally in permanent storage at some 
location2 = []  # point
location3 = []
location4 = []
location5 = []
location6 = []
location7 = []
location8 = []
locationList = [location1,
                location2,
                location3,
                location4,
                location5,
                location6,
                location7,
                location8]

l1 = StringVar()
l2 = StringVar()
l3 = StringVar()
l4 = StringVar()
l5 = StringVar()
l6 = StringVar()
l7 = StringVar()
l8 = StringVar()
locationLabelList = [l1,l2,l3,l4,l5,l6,l7,l8]

savedLocations = [] # added to when locations are saved.



        


jobQueue = []
## ---- column 1 subroutines ----
def saveLocation(name, x, y):
    locationToSaveTo = locationList[0] 
    #handles saving to next available list aswell
    for l in locationList: # top to bottom
        if len(l) == 0:
            locationToSaveTo = l
            break
    for i in [name, x, y]:
        locationList[locationList.index(locationToSaveTo)].append(i)
    locationLabelList[locationList.index(locationToSaveTo)].set(name)
    locationButtonList[locationList.index(locationToSaveTo)].config(text = name)
    with open("naviSavedLocations.csv", "a", newline = '') as file:
        writer = csv.writer(file)
        writer.writerow([name, x, y]) # writes to file.

    
def locationAddToJobQueue(l): # location
    if type(l) == list: # for when exact coordinates are used
        
        jobQueue.append(["GOTO", l])
        print("I added that list to the job queue for ya")
    elif type(l) == int: # for when a pre-saved button is pressed.
        if len(locationList[l]) == 0: #parsing data for accidental presses
            return    # of non-saved positions
        else:
            c = [locationList[l][1], locationList[l][2]] # x, y
            jobQueue.append(["GOTO", c])
    jobQueueStatusUpdate()
    
def waitJobQueueAdd(t):
    if int(t) == 0: #parsing data
        return
    else:
        if t < 0:
            t = -t
        jobQueue.append(["WAIT", t])
        jobQueueStatusUpdate()
        


        
## ---- column 2 subroutines ----
def robotHome():
    locationAddToJobQueue([0.1, 0.1, 0])

def deletePrevJob():
    if len(jobQueue) == 0:
        return
    else:
        jobQueue.remove(jobQueue[-1])
        jobQueueStatusUpdate()

def calibrateHome():
    global currentPosition
    currentPosition = [0, 0, 0]
    print("current position is now:", currentPosition)

def takeManualControl():
    global manualControlMode
    if manualControlMode == False:
        manualControlMode = True
    else:
        manualControlMode = False

## ---- column 3 subroutines ---
def createNewSetPoint():
    
    xVal = StringVar()
    yVal = StringVar()
    nameVal = StringVar()
    cnsp = tk.Toplevel(ui)
    cnspFrame = tk.Frame(cnsp)
    cnsp.title("Create new set point")
    cnsp.geometry("200x200")
    lname = tk.Label (cnsp,
                   text = "Enter Set Point Name").pack()
    nameEntry = tk.Entry(cnsp,
                      textvariable = nameVal).pack()
    def plugCurrentCoords():
        xVal.set(currentPosition[0])
        yVal.set(currentPosition[1])
    ccb = tk.Button(cnsp,   # current coordinates button
                   text = "Use Current position", 
                   width = 20,
                   height = 2,
                   command = plugCurrentCoords).pack()
    lx = tk.Label (cnsp,
                text = "Enter set point x coordinate").pack()
    xEntry = tk.Entry(cnsp,
                   textvariable = xVal).pack()
    ly = tk.Label (cnsp,
                text = "Enter set point y coordinate").pack()
    yEntry = tk.Entry(cnsp,
                   textvariable = yVal).pack()
    def terminate():
        name = nameVal.get()
        x = xVal.get()
        y = yVal.get()
        print(name, x, y)
        saveLocation(name, float(x), float(y))
        cnsp.destroy()
    B = tk.Button (cnsp,
                text = "OK",
                width = 20,
                height = 2,
                command = terminate).pack()
    return

def addWaitTime():
    tVal = StringVar()
    awt = tk.Toplevel(ui)
    awtFrame = tk.Frame(ui)
    awt.title("Add Wait Time To Job Queue")
    awt.geometry("200x200")
    l = Label (awt,
                  text = "Enter wait time (seconds)").pack()
    e = Entry (awt,
                  textvariable = tVal).pack()
    def terminate1():
        t = tVal.get() 
        t = int(t) 
        print(t)
        waitJobQueueAdd(t)
        awt.destroy()
    b = tk.Button (awt,
                   text = "Ok",
                   width = 20,
                   height = 2,
                   command = terminate1).pack()
    

def exactCoords(): 
    xVal = StringVar()
    yVal = StringVar()
    ec = tk.Toplevel(ui)
    ecFrame = tk.Frame(ec)
    ec.title("Go to exact coordinates")
    ec.geometry("200x200")
    lx = Label (ec,
                text = "Enter x coordinate").pack()
    xEntry = Entry(ec,
                   textvariable = xVal).pack()
    ly = Label (ec,
                text = "Enter y coordinate").pack()
    yEntry = Entry(ec,
                   textvariable = yVal).pack()
    def terminate():
        x = int(xVal.get())
        y = int(yVal.get())
        print(x, y)
        locationAddToJobQueue([x, y])
        ec.destroy()
    B = Button (ec,
                text = "OK",
                width = 20,
                height = 2,
                command = terminate).pack()
    
def initJobQueue():
    jobsLeft = jobQueue
    if len(jobQueue) == 0: # for accidental presses
        return
    else:
        while len(jobQueue) != 0:
            job = jobQueue[0]
            if job[0] == "GOTO":
                print(jobQueue)
                navToGoal(job[1])
                prevSubGoals = []
                sleep(1)
                jobQueue.pop(0)
                print(jobQueue)
                
            elif job[0] == "WAIT":
                print(jobQueue)
                print("Sleeping for %s seconds..." % (job[1]))
                print("BLAH")
                sleep(job[1])
                jobQueue.pop(0)
                print(jobQueue)
            jobQueueStatusUpdate()


## ------ column 1 buttons ----
location1Button = Button (ui,
                             text = l1.get(),
                             width = 20,
                             height = 2,             # These are INDEXES (down)
                             command = lambda: locationAddToJobQueue(0))
location1Button.place(x = 10, y = 10)

location2Button = Button (ui,
                             text = l2.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(1))
location2Button.place(x = 10, y = 65)

location3Button = Button (ui,
                             text = l3.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(2))
location3Button.place(x = 10, y = 120)

location4Button = Button (ui,
                             text = l4.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(3))
location4Button.place(x = 10, y = 175)

location5Button = Button (ui,
                             text = l5.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(4))
location5Button.place(x = 10, y = 230)

location6Button = Button (ui,
                             text = l6.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(5))
location6Button.place(x = 10, y = 285)

location7Button = Button (ui,
                             text = l7.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(6))
location7Button.place(x = 10, y = 340)

location8Button = Button (ui,
                             text = l8.get(),
                             width = 20,
                             height = 2,
                             command = lambda: locationAddToJobQueue(7))
location8Button.place(x = 10, y = 395)

locationButtonList = [location1Button,
                      location2Button,
                      location3Button,
                      location4Button,
                      location5Button,
                      location6Button,
                      location7Button,
                      location8Button]


try:
    results = []
    with open("naviSavedLocations.csv") as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            results.append(row) #this forms a 2D array
    print("Results:", results)
    counter = 0
    for r in results:
        name = str(r[0])
        x = r[1]
        y = r[2]
        savedLocations.append([name, x, y])
        locationLabelList[counter].set(name)
        locationButtonList[counter].config(text = name)
        locationList[counter] = [name, x, y]
        counter += 1
except:
    pass


## ------ column 2 buttons ----
robotHomeButton = tk.Button (ui,
                             text = "Robot Home",
                             width = 20,
                             height = 2,
                             command = robotHome).place(x = 217, y = 10)
calibrateHomeButton = tk.Button (ui,
                                 text = "Calibrate Home",
                                 width = 20,
                                 height = 2,
                                 command = calibrateHome).place(x = 217, y = 65)
deletePrevJobButton = tk.Button (ui,
                                 text = "Delete Previous Job in Queue",
                                 width = 20,
                                 height = 2,
                                 wraplength = 120,
                                 command = deletePrevJob).place(x = 217, y = 120)
takeManualControlButton = tk.Button (ui,
                                     text = "Take Manual Control",
                                     bg = "#FFFF00",
                                     width = 20,
                                     height = 2,
                                     wraplength = 100,
                                     command = takeManualControl).place(x = 217, y = 175)
## ---- column 3 buttons-------
createNewSetPointButton = tk.Button (ui,
                                     text = "Create New Set Point",
                                     width = 20,
                                     height = 2,
                                     command = createNewSetPoint).place(x = 435, y = 10)
addWaitTimeButton = tk.Button (ui,
                               text = "Add Wait Time",
                               width = 20,
                               height = 2,
                               command = addWaitTime).place(x = 435, y = 65)
exactCoordsButton = tk.Button (ui,
                               text = "Go to exact coordinates",
                               width = 20,
                               height = 2,
                               wraplength = 120,
                               command = exactCoords).place(x = 435, y = 120)
initButton = tk.Button (ui,
                        text = "Initialise",
                        bg = "#11FF11",
                        width = 20,
                        height = 2,
                        command = initJobQueue).place(x = 435, y = 175)


r = uicanvas.create_rectangle(217, 240, 620, 420,
                          outline="#111111", fill="#C0C0C0")
statusLabel = tk.Label(ui,
                       text = "STATUS: ",
                       font = ('Helvetica', 13, 'bold'),
                       bg = "#C0C0C0",
                       fg = "#FF2525")
statusLabel.place(x = 225, y = 247)

jobQueueLabel = tk.Label(ui,
                         text = "JOB QUEUE: ",
                         font = ('Helvetica', 13, 'bold'),
                         bg = "#C0C0C0",
                         fg = "#FF2525")
jobQueueLabel.place(x = 225, y = 283)



def jobQueueStatusUpdate(): # this is for updating the status bar information
    a1 = StringVar()
    a2 = StringVar()
    a3 = StringVar()
    a4 = StringVar()
    a5 = StringVar()
    a6 = StringVar()
    a7 = StringVar()
    a8 = StringVar()
        
    l = [a1, a2, a3, a4, a5, a6, a7, a8]
    if len(jobQueue) == 0:
        pass
    else:
        counter = 0
        for u in jobQueue[0:4]:
            try: # i have to do try, except because if there is e.g. jobqueue[1] then it will throw an error.
                l[counter].set(u[0])
                l[counter+1].set(u[1])
                counter += 2
            except:
                continue
                
    excecJobLabel = tk.Label(ui,
                             text = ("Excecuting job: " + str(a1.get()) + ", ", str(a2.get()) + (" "*30)),
                             font = ('Helvetica', 13, 'bold'),
                             bg = "#C0C0C0",
                             fg = "#11FF11")
    excecJobLabel.place(x = 225, y = 265)

    queueLabel1 = tk.Label(ui,
                           text = ("- " + str(a3.get()) + ", " + str(a4.get()) + (" "*30)),
                           font = ('Helvetica', 13, 'bold'),
                           bg = "#C0C0C0",
                           fg = "#101010")
    queueLabel1.place(x = 225, y = 301)

    queueLabel2 = tk.Label(ui,
                           text = ("- " + str(a5.get()) + ", " + str(a6.get()) + (" "*30)),
                           font = ('Helvetica', 13, 'bold'),
                           bg = "#C0C0C0",
                           fg = "#101010")
    queueLabel2.place(x = 225, y = 322)
    
    queueLabel3 = tk.Label(ui,
                           text = ("- " + str(a7.get()) + ", " + str(a8.get()) + (" "*30)),
                           font = ('Helvetica', 13, 'bold'),
                           bg = "#C0C0C0",
                           fg = "#101010")
    queueLabel3.place(x = 225, y = 343)
    ui.update()
    

jobQueueStatusUpdate()
        
                    
                
            
            
    



plots.mainloop()
uicanvas.pack()

ui.mainloop()


GPIO.cleanup()
