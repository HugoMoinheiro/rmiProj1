import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

import time


CELLROWS = 7
CELLCOLS = 14

# Variables
DEST_MARGIN = 0.1          # Margem para verificar se está no destino
DISTANCE_WALL = 1.5         # Valor dos sensores para detetarem uma parede
ANGLE_MARGIN = 10           # Margem para parar a rotação
ROTATION_SPEED = 0.1       # Velocidade para rodar
LOW_ROTATION_SPEED = 0.02   # Velocidade para alinhar
DRIVE_SPEED = 0.05          # Velocidade para andar
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
            return (round(self.measures.x - self.xInit, 2), round(self.measures.y - self.yInit, 2))

        def checkSensors(intX, intY):

            toAdd = []
            drawMaze(intX, intY, 'I')

            # Check front sensor, if it's free add to 'toExplore'
            if self.measures.irSensor[center] < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[Path at CENTER]')
                if self.measures.compass > -10 and self.measures.compass < 10 and not rightCoord in self.toExplore:
                    toAdd.insert(-1,rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > 80 and self.measures.compass < 100 and not upCoord in self.toExplore:
                    toAdd.insert(-1,upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                elif self.measures.compass > -100 and self.measures.compass < -80 and not downCoord in self.toExplore:
                    toAdd.insert(-1,downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                elif self.measures.compass > 170 and self.measures.compass < -170 and not leftCoord in self.toExplore:
                    toAdd.insert(-1,leftCoord)  
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                
            # if not, it's a wall
            else:
                if(intX % 2 == 0 and intY % 2 == 0):
                    if self.measures.compass > -10 and self.measures.compass < 10 and not rightCoord in self.toExplore:
                        drawMaze(intX+1, intY, 'V')
                    elif self.measures.compass > 80 and self.measures.compass < 100 and not upCoord in self.toExplore:
                        drawMaze(intX, intY+1, 'H')
                    elif self.measures.compass > -100 and self.measures.compass < -80 and not downCoord in self.toExplore:
                        drawMaze(intX, intY-1, 'H')
                    elif self.measures.compass > 170 and self.measures.compass < -170 and not leftCoord in self.toExplore:
                        drawMaze(intX-1, intY, 'V')              


            # Check left sensor, if is free add to 'toExplore'
            if self.measures.irSensor[left] < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[Path at LEFT]')
                if self.measures.compass > -10 and self.measures.compass < 10 and not upCoord in self.toExplore:
                    toAdd.insert(-1,upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                elif self.measures.compass > 80 and self.measures.compass < 100 and not leftCoord in self.toExplore:
                    toAdd.insert(-1,leftCoord)
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                elif self.measures.compass > -100 and self.measures.compass < -80 and not rightCoord in self.toExplore:
                    toAdd.insert(-1,rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > 170 and self.measures.compass < -170 and not downCoord in self.toExplore:
                    toAdd.insert(-1,downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')

            # if not, it's a wall
            else:
                if intX % 2 == 0 and intY % 2 == 0:
                    if self.measures.compass > -10 and self.measures.compass < 10 and not upCoord in self.toExplore:
                        drawMaze(intX, intY+1, 'H')
                    elif self.measures.compass > 80 and self.measures.compass < 100 and not leftCoord in self.toExplore:
                        drawMaze(intX-1, intY, 'V')
                    elif self.measures.compass > -100 and self.measures.compass < -80 and not rightCoord in self.toExplore:
                        drawMaze(intX+1, intY, 'V')
                    elif self.measures.compass > 170 and self.measures.compass < -170 and not downCoord in self.toExplore:
                        drawMaze(intX, intY-1, 'H')

                        

            # Check right sensor, if is free add to 'toExplore'
            if self.measures.irSensor[right] < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[Path at RIGHT]')
                if self.measures.compass > -10 and self.measures.compass < 10 and not downCoord in self.toExplore:
                    toAdd.insert(-1,downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                elif self.measures.compass > 80 and self.measures.compass < 100 and not rightCoord in self.toExplore:
                    toAdd.insert(-1,rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > -100 and self.measures.compass < -80 and not leftCoord in self.toExplore:
                    toAdd.insert(-1,leftCoord)
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                elif self.measures.compass > 170 and self.measures.compass < -170 and not upCoord in self.toExplore:
                    toAdd.insert(-1,upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')

            # if not, it's a wall
            else:
                if intX % 2 == 0 and intY % 2 == 0:
                    if self.measures.compass > -10 and self.measures.compass < 10 and not downCoord in self.toExplore:
                        drawMaze(intX, intY-1, 'H')
                    elif self.measures.compass > 80 and self.measures.compass < 100 and not rightCoord in self.toExplore:
                        drawMaze(intX+1, intY, 'V')
                    elif self.measures.compass > -100 and self.measures.compass < -80 and not leftCoord in self.toExplore:
                        drawMaze(intX-1, intY, 'V')
                    elif self.measures.compass > 170 and self.measures.compass < -170 and not upCoord in self.toExplore:
                        drawMaze(intX, intY+1, 'H')

            # printMaze(Maze)
            mazeToFile(Maze)
            return toAdd

        def inDest(pos, dest):
            # inx = False
            # iny = False
            # x = abs(dest[0]) - abs(pos[0])
            # y = abs(dest[1]) - abs(pos[1])
            # if (x < DEST_MARGIN): inx = True
            # if (y < DEST_MARGIN): iny = True
            return (abs((abs(dest[0]) - abs(pos[0]))) < DEST_MARGIN and \
                    abs((abs(dest[1]) - abs(pos[1]))) < DEST_MARGIN)

        # def turnArround():
        #     if self.measures.compass > -10 and self.measures.compass < 10:
        #         while (self.measures.compass < 90 - ANGLE_MARGIN):
        #                 self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
        #                 self.readSensors()
        #                 print('Comp: ' + str(self.measures.compass))
        #     elif self.measures.compass > 170 and self.measures.compass < -170:
        #         while (self.measures.compass < -90 - ANGLE_MARGIN): 
        #                 self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
        #                 self.readSensors()
        #                 print('Comp: ' + str(self.measures.compass))
        #     elif self.measures.compass > 80 and self.measures.compass < 100:
        #         while (self.measures.compass < 180 - ANGLE_MARGIN or self.measures.compass > 0): 
        #                 self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
        #                 self.readSensors()
        #                 print('Comp: ' + str(self.measures.compass))
        #     elif self.measures.compass > -100 and self.measures.compass < -80:
        #          while (self.measures.compass < 0 - ANGLE_MARGIN):
        #                 self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
        #                 self.readSensors()
        #                 print('Comp: ' + str(self.measures.compass))

        # def allign(x,y):
        #     # front -> UP
        #     if self.measures.compass > 70 and self.measures.compass < 110:
        #         if self.measures.compass < 90:
        #             self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
        #         elif self.measures.compass > 90:
        #             self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)
        #     # front -> DOWN
        #     elif self.measures.compass > -110 and self.measures.compass < -70:      
        #         if self.measures.compass < -90:
        #             self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
        #         elif self.measures.compass > -90:
        #             self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)
        #     # front -> RIGHT
        #     if self.measures.compass > -20 and self.measures.compass < 20:
        #         if self.measures.compass < 0:
        #             self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
        #         elif self.measures.compass > 0:
        #             self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)
        #     # front -> LEFT
        #     elif self.measures.compass > 160 and self.measures.compass < -160:          
        #         if self.measures.compass > 0:
        #             self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
        #         elif self.measures.compass < 0:
        #             self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)

        def rotate():
            if self.measures.compass > -10 and self.measures.compass < 10:
                if self.dest == downCoord:
                    # Turn right
                    while (self.measures.compass > -90 + ANGLE_MARGIN): 
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
                elif self.dest == upCoord:
                    # Turn left
                    while (self.measures.compass < 90 - ANGLE_MARGIN):
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
            elif self.measures.compass > 80 and self.measures.compass < 100:
                if self.dest == leftCoord:
                    # Turn left
                    while (self.measures.compass < 180 - ANGLE_MARGIN): 
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
                elif self.dest == rightCoord:
                    # Turn right
                    while (self.measures.compass > 0 + ANGLE_MARGIN):
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
            elif self.measures.compass > -100 and self.measures.compass < -80:
                if self.dest == leftCoord:
                    # Turn right
                    while (self.measures.compass > -180 + ANGLE_MARGIN or self.measures.compass < 0): 
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
                elif self.dest == rightCoord:
                    # Turn left
                    while (self.measures.compass < 0 - ANGLE_MARGIN):
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
            elif self.measures.compass > 170 and self.measures.compass < -170:
                if self.dest == downCoord:
                    # Turn left
                    while (self.measures.compass < -90 - ANGLE_MARGIN): 
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
                elif self.dest == upCoord:
                    # Turn right
                    while (self.measures.compass > 90 + ANGLE_MARGIN):
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        print('Comp: ' + str(self.measures.compass))
            self.driveMotors(0,0)

        def dist(p1,p2):
            return round(sqrt(pow(p1[0]-p2[0],2) + pow(p1[1]-p2[1],2)),2)
        
        def angleToDest(x,y):
            destAngle = degrees(atan(dist((x,y),(x,self.dest[1]))))
            if self.measures.compass > -10 and self.measures.compass < 10:
                if (y > self.dest[1]): return round(-destAngle,0)
                else: return round(destAngle,0)
            elif self.measures.compass > 80 and self.measures.compass < 100:
                if (x < self.dest[0]): return round(-destAngle,0)
                else : return round(destAngle,0)
            elif self.measures.compass > -100 and self.measures.compass < -80:
                if (x < self.dest[0]): return round(destAngle,0)
                else : return round(-destAngle,0)
            elif self.measures.compass > 170 and self.measures.compass < -170:
                destAngle = degrees(atan(dist((x,y),(x,self.dest[1]))))
                if (y > self.dest[1]): return round(destAngle,0)
                else: return round(-destAngle,0)

        def allign(x,y):
            angle = angleToDest(x,y)
            if angle == None: return
            if angle < 0:
                if (abs(angle) < 2):
                    self.driveMotors(1.2*LOW_ROTATION_SPEED,LOW_ROTATION_SPEED)
                    print('LOW ALLIGN RIGHT')
                else:
                    self.driveMotors(1.5*LOW_ROTATION_SPEED,LOW_ROTATION_SPEED)
                    print("ALlIGN RIGHT")
            elif angle > 0:
                if (abs(angle) < 2):
                    self.driveMotors(LOW_ROTATION_SPEED,1.2*LOW_ROTATION_SPEED)
                    print("LOW ALLIGN LEFT")
                else:
                    self.driveMotors(LOW_ROTATION_SPEED,1.5*LOW_ROTATION_SPEED)
                    print("ALLIGN LEFT")

        def updateDest():
            self.explored.insert(0,self.dest)
            # print('explored.insert(0,dest): ' + str(self.dest))
            if len(self.toExplore) > 0:
                # print('toExplore.pop(0): ' + str(self.toExplore[0]))
                # print('Explore:  ' + str(self.toExplore))
                self.dest = self.toExplore[0]
                self.toExplore.pop(0)
            else:
                print("EXIT IN updateDest()")
                self.finish()
                sys.exit()

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
                print(sentence+' at '+ str(x) +', '+ str(y))

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


        (x,y) = getCoord()
        intX = int(round(x,0))
        intY = int(round(y,0))
        rightCoord = (intX + 2, intY)
        leftCoord = (intX - 2, intY)
        upCoord = (intX, intY + 2)
        downCoord = (intX, intY - 2)

        print("----------------------------------------")
        print('\t\t  ' + str(self.cycle))
        print("----------------------------------------")
        print('Pos: (' + str(x) + ', ' + str(y) + ')  \tDest: ' + str(self.dest))
        print('Compass: ' + str(self.measures.compass))
        print('Explore:  ' + str(self.toExplore))
        print('Explored: ' + str(self.explored))
        print('AngleToDest:  ' + str(angleToDest(x,y)))
        print('DestDistance: ' + str(dist((x,y), self.dest)))

        if inDest((x,y), self.dest):

            # if self.cycle == 0:
            #     self.driveMotors(DRIVE_SPEED,DRIVE_SPEED)
            # else:
            #     self.driveMotors(0,0)

            print('****************************************')
            print('***************** DEST *****************')
            print('****************************************')
            print('x: ' + str(abs(self.dest[0]) - abs(x)))
            print('y: ' + str(abs(self.dest[1]) - abs(y)))
            # print('upCoord: ' + str(upCoord))
            # print('downCoord: ' + str(downCoord))
            # print('leftCoord: ' + str(leftCoord))
            # print('rightCoord: ' + str(rightCoord))
            # print('Dest: ' + str(self.dest))
            # print('UpCoord: ' + str(upCoord))

            self.driveMotors(0,0)
            # time.sleep(0.1)

            toAdd = checkSensors(intX, intY)
            print('toAdd: ' + str(toAdd))
            print(' (1) toExplore:     ' + str(self.toExplore))
            # self.toExplore = toAdd + self.toExplore
            for coord in toAdd:
                if not coord in self.explored:
                    if not coord in self.toExplore:
                        self.toExplore.insert(0,coord)
            print(' (2) toExplore:     ' + str(self.toExplore))
            updateDest()
            print(' (3) toExplore:     ' + str(self.toExplore))
            rotate()
            print(' (4) toExplore:     ' + str(self.toExplore))
            
            # print('RoundPos: (' + str(intX) + ', ' + str(intY) + ')')
            print('LeftSensor: ' + str(self.measures.irSensor[left]))
            print('RightSensor: ' + str(self.measures.irSensor[right]))
            # print('****************************************')
            allign(x,y)  

        else:
            self.driveMotors(DRIVE_SPEED,DRIVE_SPEED)    
            # if dist((x,y), self.dest) > 0.5:
            #     allign(x,y)


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
