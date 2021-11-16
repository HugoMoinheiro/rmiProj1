import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

rightCoord = (9999,9999)
leftCoord = (9999,9999)
upCoord = (9999,9999)
downCoord = (9999,9999)
compass = 0

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.xi = 999999
        self.yi = 999999
        self.cycle = 0
        self.prevDir = 0
        self.prevCoord = (0,0)
        self.toExplore = []
        self.explored = []
        self.dest = (0,0)

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

            if self.xi == 999999:
                self.xi = self.measures.x
                self.yi = self.measures.y

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
        if self.measures.irSensor[right] < self.measures.irSensor[left]: 
            print('++++++')
            if self.measures.irSensor[center] > 1 or self.measures.irSensor[left] > 3:
                print('DANGER')
                self.driveMotors(1, -0.5)
            elif self.measures.irSensor[center] > 0.4:
                print('danger')
                self.driveMotors(1, 0.4)
            else:
                self.driveMotors(1, 0.9)
        else:
            print('------')
            if self.measures.irSensor[center] > 1 or self.measures.irSensor[right] > 3:
                print('DANGER')
                self.driveMotors(-0.5, 1)
            elif self.measures.irSensor[center] > 0.5:
                print('danger')
                self.driveMotors(0.4, 1)
            else:
                self.driveMotors(0.9, 1)


    def goC2(self):

        def turnLeft():
            self.driveMotors(-0.15,0.15)
            self.readSensors()
            compass = self.measures.compass

        def turnRight():
            self.driveMotors(0.15,-0.15)
            self.readSensors()
            compass = self.measures.compass

        center = 0
        left = 1
        right = 2
        back = 3
        (x,y) = self.getCoord()
        compass = self.measures.compass
        print("----------------------------------------")
        print('\t\t  ' + str(self.cycle))
        print("----------------------------------------")
        print('Pos: (' + str(x) + ', ' + str(y) + ')  \tDest: ' + str(self.dest))
        print('Comp: ' + str(compass))
        print('Explore:  ' + str(self.toExplore))
        print('Explored: ' + str(self.explored))

        if self.inDest((x,y), self.dest):
            if self.cycle == 0:
                self.driveMotors(0.15,0.15)
            else:
                self.driveMotors(0,0)
            print('******************DEST******************')
            # Check sensors for points to explore
            intX = int(round(x,0))
            intY = int(round(y,0))
            rightCoord = (intX + 2, intY)
            leftCoord = (intX - 2, intY)
            upCoord = (intX, intY + 2)
            downCoord = (intX, intY - 2)
            print(str(intX) + ' ' + str(intY))
            print(str(self.measures.irSensor[left]) + ' ' + str(self.measures.irSensor[right]))

            # Check left sensor, if is free add to 'toExplore'
            if self.measures.irSensor[left] < 1 and intX % 2 == 0 and intY % 2 == 0:
                print('[ LEFT ]')
                if compass > -10 and compass < 10 and not upCoord in self.toExplore:
                    self.toExplore.append(upCoord)
                elif compass > 80 and compass < 100 and not downCoord in self.toExplore:
                    self.toExplore.append(downCoord)
                elif compass > -100 and compass < -80 and not leftCoord in self.toExplore:
                    self.toExplore.append(leftCoord)
                elif compass > 170 and compass < -170 and not rightCoord in self.toExplore:
                    self.toExplore.append(rightCoord)
                self.toExplore = list(set(self.toExplore))

            # Check right sensor, if is free add to 'toExplore'
            if self.measures.irSensor[right] < 1 and intX % 2 == 0 and intY % 2 == 0: 
                print('[ RIGHT ]')
                if compass > -10 and compass < 10 and not downCoord in self.toExplore:
                    self.toExplore.append(downCoord)
                elif compass > 80 and compass < 100 and not leftCoord in self.toExplore:
                    self.toExplore.append(leftCoord)
                elif compass > -100 and compass < -80 and not rightCoord in self.toExplore:
                    self.toExplore.append(rightCoord)
                elif compass > 170 and compass < -170 and not upCoord in self.toExplore:
                    self.toExplore.append(upCoord)
                self.toExplore = list(set(self.toExplore))
            
            # Check front sensor, if is free add to 'toExplore'
            if self.measures.irSensor[center] < 1 and intX % 2 == 0 and intY % 2 == 0:
                print('[ CENTER ]')
                if compass > -10 and compass < 10 and not rightCoord in self.toExplore:
                    self.toExplore.append(rightCoord)
                elif compass > 80 and compass < 100 and not downCoord in self.toExplore:
                    self.toExplore.append(downCoord)
                elif compass > -100 and compass < -80 and not upCoord in self.toExplore:
                    self.toExplore.append(upCoord)
                elif compass > 170 and compass < -170 and not leftCoord in self.toExplore:
                    self.toExplore.append(leftCoord)
                self.toExplore = list(set(self.toExplore))
        
            # add position to 'explored'
            self.explored.append(self.dest)
            print('explored.append(dest): ' + str(self.dest))
            self.dest = self.toExplore[0]
            if len(self.toExplore) > 0:
                print('toExplore.pop(0): ' + str(self.toExplore[0]))
                self.toExplore.pop(0)
            print('****************************************')

            

        else:
            self.driveMotors(0.15,0.15)
        

    def rotate(self):
        compass = self.measures.compass
        if compass > -10 and compass < 10:
                if self.dest == downCoord:
                    while (self.measures.compass < 90): 
                        self.driveMotors(0.15,-0.15)
                        self.readSensors()
                        compass = self.measures.compass
                    self.driveMotors(-0.15,0.15)
                elif self.dest == upCoord:
                    while (self.measures.compass < 90): 
                        print(self.measures.compass)
                        self.driveMotors(-0.05,0.05)
                        self.readSensors()
                        compass = self.measures.compass
                    self.driveMotors(0.15,-0.15)
        elif compass > 80 and compass < 100:
            if self.dest == rightCoord:
                while (self.measures.compass > 0): turnLeft()
                self.driveMotors(0,0)
            elif self.dest == leftCoord:
                while (self.measures.compass > 0): turnRight()
                self.driveMotors(0,0)
        elif compass > -100 and compass < -80:
            if self.dest == leftCoord:
                while (self.measures.compass < 0): turnLeft()
                self.driveMotors(0,0)
            elif self.dest == rightCoord:
                while (self.measures.compass < 0): turnRight()
                self.driveMotors(0,0)
        elif compass > 170 and compass < -170:
            if self.dest == downCoord:
                while (self.measures.compass > 90): turnLeft()
                self.driveMotors(0,0)
            elif self.dest == upCoord:
                while (self.measures.compass < -90): turnRight()
                self.driveMotors(0,0)
        return

    def angleToDest(self, pos, dest):
        return angle

    def turnRight(self):
        compass = self.measures.compass
        if compass > -10 and compass < 10:
                if self.dest == downCoord:
                    while (self.measures.compass < 90): 
                        self.driveMotors(0.15,-0.15)
                        self.readSensors()
                        compass = self.measures.compass
                    self.driveMotors(-0.15,0.15)
                elif self.dest == upCoord:
                    while (self.measures.compass < 90): 
                        print(self.measures.compass)
                        self.driveMotors(-0.05,0.05)
                        self.readSensors()
                        compass = self.measures.compass
                    self.driveMotors(0.15,-0.15)
        elif compass > 80 and compass < 100:
            if self.dest == rightCoord:
                while (self.measures.compass > 0): turnLeft()
                self.driveMotors(0,0)
            elif self.dest == leftCoord:
                while (self.measures.compass > 0): turnRight()
                self.driveMotors(0,0)
        elif compass > -100 and compass < -80:
            if self.dest == leftCoord:
                while (self.measures.compass < 0): turnLeft()
                self.driveMotors(0,0)
            elif self.dest == rightCoord:
                while (self.measures.compass < 0): turnRight()
                self.driveMotors(0,0)
        elif compass > 170 and compass < -170:
            if self.dest == downCoord:
                while (self.measures.compass > 90): turnLeft()
                self.driveMotors(0,0)
            elif self.dest == upCoord:
                while (self.measures.compass < -90): turnRight()
                self.driveMotors(0,0)
        return

    def getCoord(self):
        return (round(self.measures.x - self.xi, 2), round(self.measures.y - self.yi, 2))

    def inDest(self, pos, dest):
        inx = False
        iny = False
        x = abs(dest[0] - pos[0])
        y = abs(dest[0] - pos[0])
        if (x < 0.2): inx = True
        if (y < 0.2): iny = True
        return (inx and iny)
    
    def angleToDest(self, pos, dest, compass):
        return

    def drawHorWall(x,y):
        return
    
    def drawVerWall(x,y):
        return

    def drawSpace(x,y):
        return



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
