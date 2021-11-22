import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

import time


CELLROWS = 7
CELLCOLS = 14

# Variables
DEST_MARG = 0.3
DEST_MARGIN = 0.4           # Margem para verificar se está no destino
DISTANCE_WALL = 1.1         # Valor dos sensores para detetarem uma parede
ANGLE_MARGIN = 20           # Margem para parar a rotação
ANGLE_TOL = 5
ROTATION_SPEED = 0.1        # Velocidade para rodar
LOW_ROTATION_SPEED = 0.01   # Velocidade para alinhar
DRIVE_SPEED = 0.15          # Velocidade para andar
STOP_DIST = 4
ANGLE_TOL = 40
READ_QUANT = 2
DISTANCE_WALL_NEI = 1.4
rightCoord = (9999,9999)
leftCoord = (9999,9999)
upCoord = (9999,9999)
downCoord = (9999,9999)
center = 0
left = 1
right = 2
back = 3

maze_width, maze_heigth = 55, 27
centerx, centery = 27, 13 
Maze = [[0 for x in range(maze_width)] for y in range(maze_heigth)] 

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.xInit = 999999
        self.yInit = 999999
        self.cycle = 0
        self.toExplore = []
        self.explored = []
        self.dest = (0,0)
        self.rightMem = (9999,9999)
        self.leftMem = (9999,9999)
        self.upMem = (9999,9999)
        self.downMem = (9999,9999)
        self.path = []

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):

        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.xInit == 999999:
                self.xInit = self.measures.x
                self.yInit = self.measures.y

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True)
                #self.wander()
                self.goC1()
                self.cycle = self.cycle + 1

            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)

            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                #self.wander()
                self.goC1()
                self.cycle = self.cycle + 1
            

    def goC1(self):
        center = 0
        left = 1
        right = 2
        back = 3
        
        # If robot move away to the left of the path
        if self.measures.irSensor[right] < self.measures.irSensor[left]: 
            if self.measures.irSensor[center] > 1 or self.measures.irSensor[left] > 3:
                print('Turning right')
                self.driveMotors(0.15, -0.08)
            elif self.measures.irSensor[center] > 0.4:
                print('Adjusting right')
                self.driveMotors(0.15, 0.06)
            else:
                self.driveMotors(0.15, 0.14)
        # and if robot move away to the right of the path
        else:
            if self.measures.irSensor[center] > 1 or self.measures.irSensor[right] > 3:
                print('Turning left')
                self.driveMotors(-0.08, 0.15)
            elif self.measures.irSensor[center] > 0.5:
                print('Adjusting left')
                self.driveMotors(0.06, 0.15)
            else:
                self.driveMotors(0.14, 0.15)



    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if    self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id]   > 5.0\
           or self.measures.irSensor[right_id]  > 5.0\
           or self.measures.irSensor[back_id]   > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1,+0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1,0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0,0.1)
        else:
            print('Go')
            self.driveMotors(0.1,0.1)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()