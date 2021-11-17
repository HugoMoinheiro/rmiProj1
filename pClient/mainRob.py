import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET


CELLROWS = 7
CELLCOLS = 14

# Variables
DEST_MARGIN = 0.3           # Margem para verificar se está no destino
DISTANCE_WALL = 1.3         # Valor dos sensores para detetarem uma parede
ANGLE_MARGIN = 10           # Margem para parar a rotação
ROTATION_SPEED = 0.05       # Velocidade para rodar
LOW_ROTATION_SPEED = 0.02   # Velocidade para alinhar
DRIVE_SPEED = 0.15          # Velocidade para andar
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

            drawMaze(intX, intY, 'I')

            # Check front sensor, if it's free add to 'toExplore'
            if self.measures.irSensor[center] < DISTANCE_WALL:
                #if intX % 2 == 0 and intY % 2 == 0:
                print('[Path at CENTER]')
                if self.measures.compass > -10 and self.measures.compass < 10 and not rightCoord in self.toExplore:
                    self.toExplore.append(rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > 80 and self.measures.compass < 100 and not upCoord in self.toExplore:
                    self.toExplore.append(upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                elif self.measures.compass > -100 and self.measures.compass < -80 and not downCoord in self.toExplore:
                    self.toExplore.append(downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                elif self.measures.compass > 170 and self.measures.compass < -170 and not leftCoord in self.toExplore:
                    self.toExplore.append(leftCoord)  
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
                    self.toExplore.append(upCoord)
                    drawMaze(*upCoord, 'S')
                    drawMaze(intX, intY+1, 'S')
                elif self.measures.compass > 80 and self.measures.compass < 100 and not leftCoord in self.toExplore:
                    self.toExplore.append(leftCoord)
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                elif self.measures.compass > -100 and self.measures.compass < -80 and not rightCoord in self.toExplore:
                    self.toExplore.append(rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > 170 and self.measures.compass < -170 and not downCoord in self.toExplore:
                    self.toExplore.append(downCoord)
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
                    self.toExplore.append(downCoord)
                    drawMaze(*downCoord, 'S')
                    drawMaze(intX, intY-1, 'S')
                elif self.measures.compass > 80 and self.measures.compass < 100 and not rightCoord in self.toExplore:
                    self.toExplore.append(rightCoord)
                    drawMaze(*rightCoord, 'S')
                    drawMaze(intX+1, intY, 'S')
                elif self.measures.compass > -100 and self.measures.compass < -80 and not leftCoord in self.toExplore:
                    self.toExplore.append(leftCoord)
                    drawMaze(*leftCoord, 'S')
                    drawMaze(intX-1, intY, 'S')
                elif self.measures.compass > 170 and self.measures.compass < -170 and not upCoord in self.toExplore:
                    self.toExplore.append(upCoord)
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

            self.toExplore = list(set(self.toExplore))
            #printMaze(Maze)
            mazeToFile(Maze)

        def inDest(pos, dest):
            inx = False
            iny = False
            x = abs(dest[0] - pos[0])
            y = abs(dest[1] - pos[1])
            if (x < DEST_MARGIN): inx = True
            if (y < DEST_MARGIN): iny = True
            return (inx and iny)

        def rotate():
            if self.measures.compass > -10 and self.measures.compass < 10:
                if self.dest == downCoord:
                    while (self.measures.compass > -90 + ANGLE_MARGIN): 
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                elif self.dest[0] == upCoord[0] and self.dest[1] == upCoord[1]:
                    while (self.measures.compass < 90 - ANGLE_MARGIN):
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                else:
                    return False
            elif self.measures.compass > 80 and self.measures.compass < 100:
                if self.dest == leftCoord:
                    while (self.measures.compass < 180 - ANGLE_MARGIN or self.measures.compass > 0): 
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                elif self.dest == rightCoord:
                    while (self.measures.compass > 0 + ANGLE_MARGIN):
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                else:
                    return False
            elif self.measures.compass > -100 and self.measures.compass < -80:
                if self.dest == leftCoord:
                    while (self.measures.compass > -180 + ANGLE_MARGIN or self.measures.compass < 0): 
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                elif self.dest == rightCoord:
                    while (self.measures.compass < 0 - ANGLE_MARGIN):
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                else:
                    return False
            elif self.measures.compass > 170 and self.measures.compass < -170:
                if self.dest == downCoord:
                    while (self.measures.compass < -90 - ANGLE_MARGIN): 
                        self.driveMotors(-ROTATION_SPEED,ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                elif self.dest == upCoord:
                    while (self.measures.compass > 90 + ANGLE_MARGIN):
                        self.driveMotors(ROTATION_SPEED,-ROTATION_SPEED)
                        self.readSensors()
                        #print('Comp: ' + str(self.measures.compass))
                    return True
                else:
                    return False
            self.driveMotors(DRIVE_SPEED,DRIVE_SPEED)

        def allign():
            if self.measures.compass > -20 and self.measures.compass < 20:
                if self.measures.compass < 0:
                    self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
                elif self.measures.compass > 0:
                    self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)
            elif self.measures.compass > 70 and self.measures.compass < 110:
                if self.measures.compass < 90:
                    self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
                elif self.measures.compass > 90:
                    self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)
            elif self.measures.compass > -110 and self.measures.compass < -70:      
                if self.measures.compass < -90:
                    self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
                elif self.measures.compass > -90:
                    self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)          
            elif self.measures.compass > 160 and self.measures.compass < -160:          
                if self.measures.compass > 0:
                    self.driveMotors(LOW_ROTATION_SPEED, 2*LOW_ROTATION_SPEED)
                elif self.measures.compass < 0:
                    self.driveMotors(2*LOW_ROTATION_SPEED, LOW_ROTATION_SPEED)      

        def updateDest():
            self.explored.append(self.dest)
            #print('explored.append(dest): ' + str(self.dest))
            self.dest = self.toExplore[0]
            if len(self.toExplore) > 0:
                #print('toExplore.pop(0): ' + str(self.toExplore[0]))
                self.toExplore.pop(0)
            else:
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
       
        def angleToDest(self, pos, dest, compass):
            right_neighbor = (pos[0] + 2, pos[1])
            left_neighbor = (pos[0] - 2, pos[1])
            up_neighbor = (pos[0], pos[1] + 2)
            down_neighbor = (pos[0], pos[1] - 2)
            destAngle = 0

            if(dest != right_neighbor and dest != left_neighbor and dest != up_neighbor and dest != down_neighbor):
                print("Wrong destination")   
            else:
                if compass > -10 and compass < 10:
                    if (dest == left_neighbor): 
                        destAngle = 180
                    elif (dest == up_neighbor): 
                        destAngle = 90
                    elif (dest == down_neighbor): 
                        destAngle = -90        
                elif compass > 80 and compass < 100:
                    if (dest == left_neighbor): 
                        destAngle = 90
                    elif (dest == right_neighbor): 
                        destAngle = -90
                    elif (dest == down_neighbor): 
                        destAngle = 180      
                elif compass > -100 and compass < -80:
                    if (dest == left_neighbor): 
                        destAngle = -90
                    elif (dest == right_neighbor): 
                        destAngle = 90
                    elif (dest == up_neighbor): 
                        destAngle = 180 
                elif compass > 170 and compass < -170:
                    if (dest == up_neighbor): 
                        destAngle = -90
                    elif (dest == right_neighbor): 
                        destAngle = 180
                    elif (dest == down_neighbor): 
                        destAngle = 90 

            return destAngle   

        (x,y) = getCoord()
        intX = int(round(x,0))
        intY = int(round(y,0))
        rightCoord = (intX + 2, intY)
        leftCoord = (intX - 2, intY)
        upCoord = (intX, intY + 2)
        downCoord = (intX, intY - 2)

        print("----------------------------------------")
        print('\t\t Cycle ' + str(self.cycle))
        print("----------------------------------------")
        print('Pos: (' + str(x) + ', ' + str(y) + ')  \tDest: ' + str(self.dest))
        print('Compass: ' + str(self.measures.compass))
        print('To Explore:  ' + str(self.toExplore))
        print('Explored: ' + str(self.explored))


        if inDest((x,y), self.dest):

            if self.cycle == 0:
                self.driveMotors(DRIVE_SPEED,DRIVE_SPEED)
            # else:
            #     self.driveMotors(0,0)

            print('\n********** Reached Destination *********\n\n')
            # print('Dest: ' + str(self.dest))
            # print('UpCoord: ' + str(upCoord))

            checkSensors(intX, intY)
            
            updateDest()

            hadRotation = rotate()
            
            # print('RoundPos: (' + str(intX) + ', ' + str(intY) + ')')
            print('LeftSensor: ' + str(self.measures.irSensor[left]))
            print('RightSensor: ' + str(self.measures.irSensor[right]))

        else:
            self.driveMotors(DRIVE_SPEED,DRIVE_SPEED)          

        allign()  


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
