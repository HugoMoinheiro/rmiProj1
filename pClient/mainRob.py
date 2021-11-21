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
                self.goC2()
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
                self.goC2()
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


    def goC2(self):

        def getCoord():
            return (round(self.measures.x - self.xInit, 1), round(self.measures.y - self.yInit, 1))


        def checkSensors(intX, intY):

            toAdd = []
            drawMaze(intX, intY, 'I')

            r = 0
            for i in range (0,READ_QUANT):
                r += self.measures.irSensor[right]
                self.readSensors()
            r = r/READ_QUANT
            # print('r #########################################################' + str(r))
            # Check right sensor, if is free add to 'toExplore'
            if r < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[RIGHT]')
                if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                    toAdd.insert(-1,downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                    toAdd.insert(-1,rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                    toAdd.insert(-1,leftCoord)
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                    toAdd.insert(-1,upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                else:
                    print('EXIT IN CHECK SENSORS r1')
                    sys.exit()
            # if not, it's a wall
            else:
                if intX % 2 == 0 and intY % 2 == 0:
                    if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                        drawMaze(intX, intY-1, 'H')
                    elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                        drawMaze(intX+1, intY, 'V')
                    elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                        drawMaze(intX-1, intY, 'V')
                    elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                        drawMaze(intX, intY+1, 'H')
                    else:
                        print('EXIT IN CHECK SENSORS r2')
                        sys.exit()

            l = 0
            for i in range (0,READ_QUANT):
                l += self.measures.irSensor[left]
                self.readSensors()
            l = l/READ_QUANT 
            # print('l #########################################################' + str(l))
            # Check left sensor, if is free add to 'toExplore'
            if l < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[LEFT]')
                if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                    toAdd.insert(-1,upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                    toAdd.insert(-1,leftCoord)
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                    toAdd.insert(-1,rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                    toAdd.insert(-1,downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                else:
                    print('EXIT IN CHECK SENSORS l1')
                    sys.exit()
            # if not, it's a wall
            else:
                if intX % 2 == 0 and intY % 2 == 0:
                    if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                        drawMaze(intX, intY+1, 'H')
                    elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                        drawMaze(intX-1, intY, 'V')
                    elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                        drawMaze(intX+1, intY, 'V')
                    elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                        drawMaze(intX, intY-1, 'H')
                    else:
                        print('EXIT IN CHECK SENSORS l2')
                        sys.exit()

            c = 0
            for i in range (0,READ_QUANT):
                c += self.measures.irSensor[center]
                self.readSensors()
            c = c/READ_QUANT
            # print('c #########################################################' + str(c))
            # Check front sensor, if it's free add to 'toExplore'
            if c < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[CENTER]')
                if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                    toAdd.insert(-1,rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                    toAdd.insert(-1,upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                    toAdd.insert(-1,downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                    toAdd.insert(-1,leftCoord)  
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                else:
                    print('EXIT IN CHECK SENSORS c1')
                    sys.exit()
            # if not, it's a wall
            else:
                if(intX % 2 == 0 and intY % 2 == 0):
                    if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                        drawMaze(intX+1, intY, 'V')
                    elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                        drawMaze(intX, intY+1, 'H')
                    elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                        drawMaze(intX, intY-1, 'H')
                    elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                        drawMaze(intX-1, intY, 'V')    
                    else:
                        print('EXIT IN CHECK SENSORS c2')
                        sys.exit()         

            # printMaze(Maze)
            mazeToFile(Maze)
            # print('toAdd: ' + str(toAdd))
            return toAdd


        def inDest(pos, dest):
            
            inDes = (abs((abs(dest[0]) - abs(pos[0]))) < DEST_MARG and \
                     abs((abs(dest[1]) - abs(pos[1]))) < DEST_MARG)
            # print('\t(1) ' + str(inDes))
            if not inDes:
                if self.dest == self.rightMem or self.dest == self.leftMem:
                    # print(abs(dest[0]))
                    # print(abs(pos[0]))
                    # print(abs((abs(dest[0]) - abs(pos[0]))))
                    # print(abs(dest[1]))
                    # print(abs(pos[1]))
                    # print(abs((abs(dest[1]) - abs(pos[1]))))
                    inDes = (abs((abs(dest[0]) - abs(pos[0]))) < DEST_MARG and \
                              abs((abs(dest[1]) - abs(pos[1]))) < DEST_MARGIN)
                    # print('\t(2) ' + str(inDes))
                elif self.dest == self.upMem or self.dest == self.downMem:
                    inDes = (abs((abs(dest[0]) - abs(pos[0]))) < DEST_MARGIN and \
                             abs((abs(dest[1]) - abs(pos[1]))) < DEST_MARG)
                    # print('\t(3) ' + str(inDes))
                else: 
                    print('NOT NEIGHBOOR (inDest)')
            # print('upCoord: ' + str(self.upMem))
            # print('downCoord: ' + str(self.downMem))
            # print('leftCoord: ' + str(self.leftMem))
            # print('rightCoord: ' + str(self.rightMem))
            return inDes


        def turnArround():
            if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                while self.measures.compass < 180 - ANGLE_MARGIN:
                    self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                    self.readSensors()
            elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                while self.measures.compass <= 0:
                    self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                    self.readSensors()
                while self.measures.compass > 0 + ANGLE_MARGIN:
                    self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                    self.readSensors()
            elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                while self.measures.compass > -90 + ANGLE_MARGIN:
                    self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                    self.readSensors()
            elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                while self.measures.compass < 90 - ANGLE_MARGIN:
                    self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                    self.readSensors()


        def rotate():
            # print('ROTATE')
            if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                if self.dest == downCoord:
                    # Turn right
                    while (self.measures.compass > -90 + ANGLE_MARGIN): 
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        # print('Comp(r): ' + str(self.measures.compass))
                elif self.dest == upCoord:
                    # Turn left
                    while (self.measures.compass < 90 - ANGLE_MARGIN):
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        # print('Comp(r): ' + str(self.measures.compass))
                        
            elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                # print('----ROTATE----')
                if self.measures.compass == -180: compass = 0
                elif self.measures.compass < 0: compass = self.measures.compass + 180
                elif self.measures.compass > 0: compass = -(180 - self.measures.compass)
                else: 
                    print('EXIT ROTATE')
                    sys.exit()
                # compass = self.measures.compass + 90
                if self.dest == downCoord:
                    # print('------------------------')
                    # Turn left
                    while (compass < 90 - ANGLE_MARGIN): 
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        if self.measures.compass == -180: compass = 0
                        elif self.measures.compass < 0: compass = self.measures.compass + 180
                        elif self.measures.compass > 0: compass = -(180 - self.measures.compass)
                        # print('Comp(r): ' + str(self.measures.compass))
                elif self.dest == upCoord:
                    # print('+++++++++++++++++++++++++')
                    # print(compass)
                    # Turn right
                    while (compass > -90 + ANGLE_MARGIN):
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        if self.measures.compass == -180: compass = 0
                        elif self.measures.compass < 0: compass = self.measures.compass + 180
                        elif self.measures.compass > 0: compass = -(180 - self.measures.compass)
                        # print('Comp(r): ' + str(self.measures.compass))

            elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                if self.dest == leftCoord:
                    # Turn left
                    while (self.measures.compass < 180 - ANGLE_MARGIN): 
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        # print('Comp(r): ' + str(self.measures.compass))
                elif self.dest == rightCoord:
                    # Turn right
                    while (self.measures.compass > 0 + ANGLE_MARGIN):
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        # print('Comp(r): ' + str(self.measures.compass))

            elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                if self.dest == leftCoord:
                    # Turn right
                    while (self.measures.compass > -180 + ANGLE_MARGIN): 
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        # print('Comp(r): ' + str(self.measures.compass))
                elif self.dest == rightCoord:
                    # Turn left
                    while (self.measures.compass < 0 - ANGLE_MARGIN):
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        # print('Comp(r): ' + str(self.measures.compass))

            else:
                print("ERRO NO ROTATE (NO DIRECTION)")
                sys.exit()
            self.driveMotors(0,0)


        def dist(p1,p2):
            return round(sqrt(pow(p1[0]-p2[0],2) + pow(p1[1]-p2[1],2)),2)
        

        def angleToDest(x,y):

            if dist((x,y),self.dest) == 0:
                # sys.exit()
                return 0
            else:
                if -ANGLE_TOL < self.measures.compass < ANGLE_TOL or self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                    # print('(x\tdest[1])\t(dest)  \t\t(x,y)\t\t(dest)')
                    # print(str(x) +'\t'+ str(self.dest[1]) +'\t\t'+ str(self.dest) +'\t\t'+ str((x,y)) +'\t'+ str(self.dest))
                    # print(dist((x,self.dest[1]),self.dest))
                    # print(dist((x,y),self.dest))
                    # print('Div: ' + str(dist((x,self.dest[1]),self.dest) / dist((x,y),self.dest)))
                    # print('acos: ' + str(acos( dist((x,self.dest[1]),self.dest) / dist((x,y),self.dest) )))
                    # print('degre: ' + str(degrees( acos( dist((x,self.dest[1]),self.dest) / dist((x,y),self.dest) ) )))
                    destAngleHor = degrees( acos( dist((x,self.dest[1]),self.dest) / dist((x,y),self.dest) ) )
                    if destAngleHor == None: return 0
                    if self.measures.compass > -40 and self.measures.compass < 40:
                        if (y > self.dest[1]): 
                            return round(-destAngleHor,0)
                        else: 
                            return round(destAngleHor,0)

                    elif self.measures.compass > 140 or self.measures.compass < -140:
                        if (y < self.dest[1]): 
                            return round(-destAngleHor,0)
                        else: 
                            return round(destAngleHor,0)

                if 90-ANGLE_TOL < self.measures.compass < 90+ANGLE_TOL or -90-ANGLE_TOL < self.measures.compass < -90+ANGLE_TOL:
                    # print('(x\tdest[1])\t(dest)  \t\t(x,y)\t\t(dest)')
                    # print(str(self.dest[0]) +'\t'+ str(y) +'\t\t'+ str(self.dest) +'\t\t'+ str((x,y)) +'\t'+ str(self.dest))
                    # print(dist((self.dest[0],y),self.dest))
                    # print(dist((x,y),self.dest))
                    # print('Div: ' + str(dist((self.dest[0],y),self.dest) / dist((x,y),self.dest) ))
                    # print('acos: ' + str(acos( dist((self.dest[0],y),self.dest) / dist((x,y),self.dest) ) ))
                    # print('degre: ' + str(degrees( acos( dist((self.dest[0],y),self.dest) / dist((x,y),self.dest) ) )))
                    destAngleVer = degrees( acos( dist((self.dest[0],y),self.dest) / dist((x,y),self.dest) ) )
                    if destAngleVer == None: return 0
                    if self.measures.compass > 50 and self.measures.compass < 130:
                        if (x < self.dest[0]): 
                            return round(-destAngleVer,0)
                        else : 
                            return round(destAngleVer,0)

                    elif self.measures.compass > -130 and self.measures.compass < -50:
                        if (x < self.dest[0]): 
                            return round(destAngleVer,0)
                        else : 
                            return round(-destAngleVer,0)

                else:
                    print('NOT IN RANGE (angleToDest)')
                    sys.exit()
            print('EOFunc (angleToDest)')
            sys.exit()


        def allign(x,y):
            angle = angleToDest(x,y)
            if angle == None: 
                print('ANGLE TO DEST: NONE')
                return
            self.driveMotors(0,0)
            if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                # print('1#######################################')
                # print('ANGLE: ' + str(angle))
                # print('COMPA: ' + str(self.measures.compass))
                if self.measures.compass < 0:
                    while self.measures.compass < angle:
                        self.driveMotors(-LOW_ROTATION_SPEED,LOW_ROTATION_SPEED)
                        # print('Comp(a): ' + str(self.measures.compass))
                        self.readSensors()
                elif self.measures.compass > 0:
                    while self.measures.compass > angle:
                        self.driveMotors(LOW_ROTATION_SPEED,-LOW_ROTATION_SPEED)
                        # print('Comp(a): ' + str(self.measures.compass))
                        self.readSensors()
            
            elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                # print('2#######################################')
                # print('ANGLE: ' + str(angle))
                # print('COMPA: ' + str(self.measures.compass))
                if self.measures.compass == -180: compass = 0
                elif self.measures.compass < 0: compass = self.measures.compass + 180
                elif self.measures.compass > 0: compass = -(180 - self.measures.compass)
                # compass = self.measures.compass + 180
                if compass > 0:
                    # print(' (1)')
                    while compass > angle:
                        self.driveMotors(LOW_ROTATION_SPEED,-LOW_ROTATION_SPEED)
                        # print('+' + str(self.measures.compass) + '   ' + str(compass))
                        self.readSensors()
                        # compass = self.measures.compass + 180
                        if self.measures.compass == -180: compass = 0
                        elif self.measures.compass < 0: compass = self.measures.compass + 180
                        elif self.measures.compass > 0: compass = -(180 - self.measures.compass)
                        # print('Comp(a): ' + str(self.measures.compass))
                elif compass < 0:
                    # print(' (2)')
                    while compass < angle:
                        self.driveMotors(-LOW_ROTATION_SPEED,LOW_ROTATION_SPEED)
                        # print('-' + str(self.measures.compass) + '   ' + str(compass))
                        self.readSensors()
                        # compass = self.measures.compass + 180
                        if self.measures.compass == -180: compass = 0
                        elif self.measures.compass < 0: compass = self.measures.compass + 180
                        elif self.measures.compass > 0: compass = -(180 - self.measures.compass)
                        # print('Comp(a): ' + str(self.measures.compass))
            
            elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                # print('3#######################################')
                # print('ANGLE: ' + str(angle))
                # print('COMPA: ' + str(self.measures.compass))
                if self.measures.compass < 90:
                    while self.measures.compass < 90 + angle:
                        self.driveMotors(-LOW_ROTATION_SPEED,LOW_ROTATION_SPEED)
                        # print('Comp(a): ' + str(self.measures.compass))
                        self.readSensors()
                elif self.measures.compass > 90:
                    while self.measures.compass > 90 + angle:
                        self.driveMotors(LOW_ROTATION_SPEED,-LOW_ROTATION_SPEED)
                        # print('Comp(a): ' + str(self.measures.compass))
                        self.readSensors()

            
            elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                # print('4#######################################')
                # print('ANGLE: ' + str(angle))
                # print('COMPA: ' + str(self.measures.compass))
                if self.measures.compass < -90:
                    while self.measures.compass < -90 + angle:
                        self.driveMotors(-LOW_ROTATION_SPEED,LOW_ROTATION_SPEED)
                        # print('Comp(a): ' + str(self.measures.compass))
                        self.readSensors()
                elif self.measures.compass > -90:
                    while self.measures.compass > -90 + angle:
                        self.driveMotors(LOW_ROTATION_SPEED,-LOW_ROTATION_SPEED)
                        # print('Comp(a): ' + str(self.measures.compass))
                        self.readSensors()
            # print('########################################')

	    # ( typeOfChar: I - Initial; H - Horiz. wall; V - Vert. wall; S - Space )
        def drawMaze(x, y, typeOfChar):
            if typeOfChar == 'I':
                draw = 'I'
                sentence = 'Initial point'
            elif typeOfChar == 'H':
                draw = '-'
                sentence = 'Horizontal wall'
            elif typeOfChar == 'V':        
                draw = '|'
                sentence = 'Vertical wall'
            elif typeOfChar == 'S':
                draw = "X"
                sentence = 'Space' 
            
            if(Maze[centery+y][centerx+x] == 0):
                Maze[centery+y][centerx+x] = draw
                # print(sentence+' at '+ str(x) +', '+ str(y))


        def printMaze(maze):    
            for row in maze[::-1]:
                for col in row:
                    print(col,end = " ")
                print()


        def mazeToFile(maze):
            with open("maze.txt", "w") as txt_file:
                for row in maze[::-1]:
                    for col in row:
                        txt_file.write(str(col).replace('0',' '))
                    txt_file.write("\n")            

        
        def isNeighboor2(p1,p2):
            intx1 = round(p1[0],0)
            inty1 = round(p1[1],0)
            r = (intx1 + 2, inty1)
            l = (intx1 - 2, inty1)
            u = (intx1, inty1 + 2)
            d = (intx1, inty1 - 2)
            if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL:
                ri = d
                le = u
                fr = r
                ba = l
            elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL:
                ri = u
                le = d
                fr = l
                ba = r
            elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL:
                ri = r
                le = l
                fr = u
                ba = d
            elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL:
                ri = l
                le = r
                fr = d
                ba = u
            else:
                print('NOT DIRECTION isNeighboor')
                sys.exit()
            neighboors = [ri, le, fr, ba]
            # print('-----------------------------')
            # print('if ' + str(p2) + ' in ' + str(neighboors) + ' -> ' + str(p2 in neighboors))
            # print('-----------------------------')
            
            if p2 == ri:
                # print('right: ' + str(self.measures.irSensor[right]))
                if self.measures.irSensor[right] < DISTANCE_WALL_NEI: return True
                else: return False
            elif p2 == le:
                # print('left: ' + str(self.measures.irSensor[left]))
                if self.measures.irSensor[left] < DISTANCE_WALL_NEI: return True
                else: return False
            elif p2 == fr:
                # print('center: ' + str(self.measures.irSensor[center]))
                if self.measures.irSensor[center] < DISTANCE_WALL_NEI: return True
                else: return False
            elif p2 == ba:
                # print('back: ' + str(self.measures.irSensor[back]))
                if self.measures.irSensor[back] < DISTANCE_WALL_NEI: return True
                else: return False
            else:
                # print('---FALSE---')
                return False


        # def updateDest():
            # if len(self.path) == 0:
            #     if len(self.toExplore) > 0:
            #         # print('#################################################################')
            #         # print('toExplore[0]: ' + str(self.toExplore[0]))
            #         # print('self.dest: ' + str(self.dest))
            #         # print('isNeighboor2: ' + str(isNeighboor2(self.dest, self.toExplore[0])))
            #         # print('#################################################################')
            #         if isNeighboor2(self.dest, self.toExplore[0][-1]):
            #             # print('self.dest = self.toExplore.pop(0) -> ' + str(self.toExplore[0][-1]))
            #             self.path = self.toExplore.pop(0)
            #             if len(self.path) == 0:
            #                 self.dest = self.path[0]
            #             elif len(self.path) > 0:
            #                 self.dest = self.path[0]
            #                 self.path = self.path[1:]
            #             else:
            #                 print('PATH too short')
            #                 sys.exit()
            #             # print('#################################################################')
            #         else:
                        
            #             # print('#################################################################')
            #     else:
            #         print("EXIT IN updateDest()")
            #         self.finish()
            #         sys.exit()
            # else:
            #     self.dest = self.path.pop(0)      


        def repetedInPath():
            rep = []
            tcount = 0
            for t in self.path:
                t2count = 0
                for t2 in self.path:
                    if t == t2:
                        if tcount < t2count:
                            rep.append((tcount,t2count))
                            tcount = t2count
                            t2count += 1
                    t2count += 1
                tcount += 1
            return rep
        

        def removeRep():
            if len(repetedInPath()) > 0:
                # print('=====================================================================================================================')
                # print('=====================================================================================================================')
                # print('=====================================================================================================================')
                # print(self.path)
                # print(repetedInPath())
                # print('=====================================================================================================================')
                repeted = repetedInPath()
                tmpPath = []
                for r in repeted:
                    insert = True
                    for i in range(0,len(self.path)):
                        if i != r[0] and insert:
                            tmpPath.append(self.path[i])
                        else:
                            insert = False
                        if i == r[1]:
                            insert = True
                            tmpPath.append(self.path[i])
                self.path = tmpPath
                self.toExplore[0] = self.path
                # print(tmpPath)
                # print('=====================================================================================================================')
                # print('=====================================================================================================================')
                # print('=====================================================================================================================')


        def updateDest():
            self.path = self.toExplore[0]

            for n in range(0, len(self.toExplore)):
                self.toExplore[n].insert(0, self.dest)
                # print('c1 »»»»»»»» ' + str(c1))
            
            removeRep()

            if self.path[-1] == self.dest:
                # print('++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
                # print('toExplore ' + str(self.toExplore))
                self.toExplore = self.toExplore[1:]
                self.path = self.toExplore[0]

            if self.path[0] == self.dest:
                self.path = self.path[1:]

            # print('************** ' + str(self.path) + ' ? ' + str(len(self.path)))
            if len(self.path) == 1:
                self.dest = self.path[0]
                if len(self.toExplore) > 1: 
                    self.toExplore = self.toExplore[1:]
                else:
                    self.toExplore = []

            elif len(self.path) > 1:
                self.dest = self.path[0]
                self.path = self.path[1:]
                self.toExplore[0] = self.path
                # print('toexplore ' + str(self.toExplore))
                # print('»»»»»»»»»»»»»»»»» ' + str(self.toExplore))
                # print('»»»»»»»»»»»»»»»»» ' + str(self.path))

            else:
                print('PATH TOO SHORT updateDest')
                sys.exit()



        (x,y) = getCoord()
        intX = int(round(x,0))
        intY = int(round(y,0))
        rightCoord = (intX + 2, intY)
        leftCoord = (intX - 2, intY)
        upCoord = (intX, intY + 2)
        downCoord = (intX, intY - 2)

        print()
        print("----------------------------------------")
        print('\t\t  ' + str(self.cycle))
        print("----------------------------------------")
        print('Pos: (' + str(x) + ', ' + str(y) + ')  \tDest: ' + str(self.dest))
        print('Compass: ' + str(self.measures.compass))
        print('AngleToDest:  ' + str(angleToDest(x,y)))
        # print('Explore: ')
        # for e in self.toExplore:
        #     print('      ' + str(e))
        # print('Explored: ' + str(self.explored))
        # print('Path: ' + str(self.path))
        # print('DestDistance: ' + str(dist((x,y), self.dest)))

        # if self.measures.irSensor[center] > STOP_DIST:
        #     self.driveMotors(0,0)
        #     allign(x,y)

        # if self.cycle == 0:
        #     toAdd = checkSensors(intX,intY)
        #     for coord in toAdd:
        #        self.toExplore.insert(0,coord)
        #        print(self.toExplore)
        #     updateDest()
        #     rotate()

        point = (intX,intY)
        if intX % 2 == 0 and intY % 2 == 0 and (len(self.explored) == 0 or not self.explored[0] == point):
            self.explored.insert(0,(intX,intY))

        if inDest((x,y), self.dest):

            if self.toExplore == [[]]: self.toExplore == []

            print('****************************************')
            print('***************** DEST *****************')
            print('****************************************')
            # print('x: ' + str(abs(self.dest[0]) - abs(x)))
            # print('y: ' + str(abs(self.dest[1]) - abs(y)))
            # print('upCoord: ' + str(upCoord))
            # print('downCoord: ' + str(downCoord))
            # print('leftCoord: ' + str(leftCoord))
            # print('rightCoord: ' + str(rightCoord))
            # print('Dest: ' + str(self.dest))
            # print('UpCoord: ' + str(upCoord))

            self.driveMotors(0,0)
            # time.sleep(0.1)

            toAdd = checkSensors(intX, intY)
            # print('toAdd: ' + str(toAdd))
            # print(' (1) toExplore:     ' + str(self.toExplore))

            for coord in toAdd:
                # print('coord »»»»»»»»»» ' + str(coord))
                if not coord in self.explored:
                    if len(self.toExplore) == 0: #or self.toExplore == [[]]:
                        self.toExplore.insert(0, [coord])
                        # print('1 toExplore »»»»»»»»» ' + str(self.toExplore))
                    else:
                        # print('len toExplore »»»»»»»» ' + str(len(self.toExplore[0])))
                        for c in self.toExplore:
                            # print('c »»»»»»»» ' + str(c))
                            if not coord == c[-1]:
                                self.toExplore.insert(0,[coord])
                                break       
                        # print('2 toExplore »»»»»»»»»» ' + str(self.toExplore))

            # print(' (2) toExplore:     ' + str(self.toExplore))

            r = (self.dest[0] + 2, self.dest[1])
            l = (self.dest[0] - 2, self.dest[1])
            u = (self.dest[0], self.dest[1] + 2)
            d = (self.dest[0], self.dest[1] - 2)
            if self.measures.compass > -ANGLE_TOL and self.measures.compass < ANGLE_TOL: ba = l
            elif self.measures.compass > 180-ANGLE_TOL or self.measures.compass < -180+ANGLE_TOL: ba = r
            elif self.measures.compass > 90-ANGLE_TOL and self.measures.compass < 90+ANGLE_TOL: ba = d
            elif self.measures.compass > -90-ANGLE_TOL and self.measures.compass < -90+ANGLE_TOL: ba = u

            updateDest()
            # print(' (3) toExplore:     ' + str(self.toExplore))

            if (len(toAdd) < 1) or (self.dest == ba):
                turnArround()
            else:
                rotate()

            # print('LeftSensor: ' + str(self.measures.irSensor[left]))
            # print('RightSensor: ' + str(self.measures.irSensor[right]))
            # print('CenterSensor: ' + str(self.measures.irSensor[center]))
            print('****************************************')
            # allign(x,y)  

            self.rightMem = rightCoord
            self.leftMem = leftCoord
            self.upMem = upCoord
            self.downMem = downCoord

        else:   
            if self.measures.irSensor[center] > STOP_DIST:
                self.driveMotors(0,0)
            else:
                # if dist((x,y), self.dest) > 1.5:
                self.driveMotors(DRIVE_SPEED,DRIVE_SPEED) 
            # print(dist((x,y),self.dest) > 0.5)
            if dist((x,y),self.dest) > 0.5:
                allign(x,y)
                self.driveMotors(DRIVE_SPEED,DRIVE_SPEED) 


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
    #rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
