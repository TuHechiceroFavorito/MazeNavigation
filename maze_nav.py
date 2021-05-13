# --------------------------------------------------------------------------------------------------------------------
#
#   Project:      Maze navigation and mapping
#   Authors:      Evanna Niall and Daniel Álvarez Carreño
#   Created:      24/2/2021
#   Description:  The robot is able to navigate through a maze, map it and find the quickest path
#
#   Note on the code: We have noticed that at start VEXcode throughs a wait error. It asks to add a wait statement
#   in a loop. We have noticed that this error is solved if the dialog is ignored and the ⭯ symbol is pressed.
#   We think it might be because of the loading of the code
#
# --------------------------------------------------------------------------------------------------------------------
# Library imports
from vexcode import *
import copy
#List of all known tiles. It's global to make editing and reading easier.
tiles_list = []
#Solved solutions for starting and robot. List of paths that are too long
starting = []
robot = []
paths = []
#This are the coordinates of the corners of the tiles
forbidden_areas = []

#Rough width of a tile, coordinates of the first left lower corner and width of a wall
tile = 230
origin = -990
wall = 15
count_distance_starting=0
count_distance_robot=0

#This function is from assignment 2 topic 3. The formula has been adjusted to the length of 250mm of
#the squares of this playground. More detailed explanation above function mad_eye_strategy

#Initialise the tiles. Fills the list tiles_list with an 8x8 matrix. Each cell represents a tile
#of the maze. For each tile it is stored the walls following a compass system:
#[N, E, S, W]. 1 for a wall, 0 for no wall and u for unkown. It is also stored the number of tiles
#that the robot has been in the tile.
def init_tiles():
    global tiles_list
    tiles = tiles_list
    for i in range (8):     #Tens
        tiles.append([])
        for j in range(8):  #Units
            tiles[i].append([['u', 'u', 'u', 'u'], 0])
    
    #Set to 1 the known walls on the sides of the maze
    for i in range(8):
        tiles[0][i] = [['u', 'u', 1, 'u'], 0]
        tiles[i][0] = [['u', 'u', 'u', 1], 0]
        tiles[7][i] = [[1, 'u', 'u', 'u'], 0]
        tiles[i][7] = [['u', 1, 'u', 'u'], 0]
    tiles[0][0] = [['u', 'u', 1, 1], 0]
    tiles[7][0] = [[1, 'u', 'u', 1], 0]
    tiles[7][7] = [[1, 1, 'u', 'u'], 0]
    tiles[0][7] = [['u', 1, 1, 'u'], 0]

#Returns the number of a tile given its coordinates.
#The maze it's divided into 8 rows and 8 columns of tiles. The number of the tiles
#increases by 1 in the positive direction of the x axis and increases by 10 in the
#positive direction of the y axis.   
def num_predictor(x, y):
    global origin
    global tile

    a = int((y - origin) / (tile + wall))
    #Due to imprecisions in the measurements, on the first row and last column it gets the number wrong by 1
    #So a correction is needed
    if a > 7:
        a = 7
    b = int((x - origin) / (tile + wall))
    if b > 7:
        b = 7

    num = (a*10 + b)
    return num

#Sets the adjacent wall to the required value. If no orientation for the wall is passed,
#the function takes the value from the position of the robot. target specifies at which list
#is the adjacent filler applied
def adjacent_filler(tile_num, value, orientation=False, target='tiles'):
    if target == 'start':
        global starting
        tiles = starting
    elif target == 'robot':
        global robot
        tiles = robot
    else:
        global tiles_list
        tiles = tiles_list
        
    #Set the appropriate adjacent tile and the wall index
    #Also make sure not to select a tile that is out of the maze (keep track of corners)
    if orientation is False:
        if drivetrain.heading(DEGREES) == 0 and tile_num<70:        #N
            adj_tile = tile_num+10
            adj_wall = 2
        elif drivetrain.heading(DEGREES) == 90 and tile_num%10<7:     #E
            adj_tile = tile_num+1
            adj_wall = 3
        elif drivetrain.heading(DEGREES) == 180 and tile_num>10:    #S
            adj_tile = tile_num-10
            adj_wall = 0
        elif drivetrain.heading(DEGREES) == 270 and tile_num%10>0:    #W
            adj_tile = tile_num-1
            adj_wall = 1
        else:           #No adjacent tile. The robot is in a corner
            adj_tile = None

    else:
        if orientation == 0 and tile_num<70:        #N
            adj_tile = tile_num+10
            adj_wall = 2
        elif orientation == 1 and tile_num%10<7:     #E
            adj_tile = tile_num+1
            adj_wall = 3
        elif orientation == 2 and tile_num>=10:    #S
            adj_tile = tile_num-10
            adj_wall = 0
        elif orientation == 3 and tile_num%10>0:    #W
            adj_tile = tile_num-1
            adj_wall = 1
        else:           #No adjacent tile. The robot is in a corner
            adj_tile = None
    #Do only if there is an adjacent tile
    if adj_tile != None:
        tiles[int(adj_tile/10)][adj_tile % 10][0][adj_wall] = value   #Set the appropriate wall to its neighbor value

#This function adds the information of the wall right in front
def wall_mapping():
    global tiles_list
    tiles = tiles_list
    tile_num = num_predictor(location.position(X, MM), location.position(Y, MM))  #Number of the current tile
    current_tile = []
        #Check if the nearest wall is in the current tile
    if distance.get_distance(MM) > 250 and distance.get_distance(MM) < 3000:
        tiles_ahead = int(distance.get_distance(MM)/250)        #How many tiles ahead it is
        #Set the correct values for the tile number and orientation
        if drivetrain.heading(DEGREES) == 0:
            distant_tile = tile_num+10*tiles_ahead
            orientation = 0
        elif drivetrain.heading(DEGREES) == 90:
            distant_tile = tile_num+tiles_ahead
            orientation = 1
        elif drivetrain.heading(DEGREES) == 180:
            distant_tile = tile_num-10*tiles_ahead
            orientation = 2
        elif drivetrain.heading(DEGREES) == 270:
            distant_tile = tile_num-tiles_ahead
            orientation = 3
        #Loop though all the middle values in between the current tile and the distant one
        for middle in range(tiles_ahead + 1):   #+1 to include the distant_tile and the current one
            #Change the tile number according to the orientation and the current middle value
            #Working from distant tile to current one
            if orientation == 1:
                target = distant_tile - middle
            elif orientation == 2:
                target = distant_tile + 10*middle
            elif orientation == 3:
                target = distant_tile + middle
            elif orientation == 0:
                target = distant_tile - 10*middle
    
            tiles[int(target/10)][target % 10][0][orientation] = 0   #Set the appropriate wall
            #Fill adjacent tile
            adjacent_filler(target, 0)
        #Change the working tile to the distant one to assign it the value 1
        tile_num = distant_tile
    elif distance.get_distance(MM) >= 3000:     #It is facing to the void (the starting or the ending tiles)
        if drivetrain.heading(DEGREES) == 0:    #North
            tile_num = 73
        elif drivetrain.heading(DEGREES) == 180: #South
            tile_num = 4
        
    #Select the walls of the tile
    walls = tiles[int(tile_num/10)][tile_num % 10][0]
    #Detect the orientation of the wall in the tile
    if drivetrain.heading(DEGREES) == 0:    #North
        walls[0] = 1
    elif drivetrain.heading(DEGREES) == 90:  #East
        walls[1] = 1
    elif drivetrain.heading(DEGREES) == 180: #South
        walls[2] = 1
    elif drivetrain.heading(DEGREES) == 270: #West
        walls[3] = 1
    #Fill adjacent tile
    adjacent_filler(tile_num, 1)

    #Clear the output and display the map
    brain.clear()
    display_map()

#Follow the left wall until it has scanned all the walls on the maze 
def mad_eye_strategy(cup=False):
    global tiles_list
    tiles = tiles_list
    global starting
    global robot
    
    discovered = False
    while discovered == False:
        wait(5, MSEC)
        check_wall_on_left()
        if cup is not True:
            discovered = True
            #Check if there is any wall left to scan
            for row in tiles:
                for tile in row:
                    for wall in tile[0]:
                        if wall == 'u':
                            discovered = False
        else:
            tile = num_predictor(location.position(X, MM), location.position(Y, MM))    #Get the current tile
            if tile == cup:
                break
    
    #The maze is fully scan. Copy the data into both lists to solve the maze from there
    starting = copy.deepcopy(tiles)
    robot = copy.deepcopy(tiles)

#Same as mad_eye_strategy, but it doesn't scan. It only follows the left wall, but as everything
#is closed except the solution, it will drive to it.
def beat_it():
    pen.set_pen_color(RED)
    while True:
        wait(5, MSEC)
        check_wall_on_left(False, 'robot')
        tile = num_predictor(location.position(X, MM), location.position(Y, MM))    #Get the current tile
        if tile == 73:
            break


#Rotates the frame of reference of the robot. Returns the appropriate indeces for this rotation
def frame(orientation=False):
    #It takes the direction of the robot
    if orientation is False:
        orientation = drivetrain.heading(DEGREES)
    
    #If the robot is facing North 
    if orientation==0:
        N=0
        E=1
        S=2
        W=3
    #If the robot is facing East
    if orientation==90:
        N=1
        E=2
        S=3
        W=0
    
    #If the robot is facinng South
    if orientation==180:
        N=2
        E=3
        S=0
        W=1
   
    #If the robot is facing west 
    if orientation==270:
        N=3
        E=0
        S=1
        W=2
    return N, E, S, W

#This function allows the robot move around the maze. 
#It detects the angle it is to move due to the walls recorded on the lists
#It accepts 2 arguments. If scan is true, new values are accepted into the tiles list
#Target selects the list in which it's going to be applied
def check_wall_on_left(scan=True, target=0):
    
    #Determines what list to read from 
    #The list of tiles on the path from the position of the robot to the finish line. This is called to travel the shortest path 
    if target == 'robot':
        global robot
        tiles = robot
    else:
        #This list is used to explore the maze
        global tiles_list
        tiles = tiles_list
    
    step = 250 #The size of a tile 
    tile_num = num_predictor(location.position(X, MM), location.position(Y, MM))#The tile number is determined by the position of the robot 
    
    #Determining the index of the list tile relative to the robot
    N, E, S, W = frame()
    if scan:
        wall_mapping()

    #If it has been at the tile twice. This is to allow for an island in the maze to be recorded
    #An island is a part of a maze that is disconnected from the sides of the it.
    if tiles[int(tile_num/10)][tile_num % 10][1] == 2:
        #If the East tile has still not being recorded it turns right and records it. 
        if tiles[int(tile_num/10)][tile_num % 10][0][E] == 'u':
            
            terminator(RIGHT, 90, scan)

            if scan:
                wall_mapping()
            
            terminator(LEFT, 90, scan)

    #Checks if it has recoreded there is no wall on the left of the robot
    if (tiles[int(tile_num/10)][tile_num % 10][0][W]==0):
        terminator(LEFT, 90, scan)

    else:
    #Checks if it has recorded a wall to the left of the robot
        if (tiles[int(tile_num/10)][tile_num % 10][0][W]==1):
            if (tiles[int(tile_num/10)][tile_num % 10][0][N]==1):
                if (tiles[int(tile_num/10)][tile_num % 10][0][E]==1):
                    #Detected a wall to the left, ahead and to the right of the robot
                    Angle=180
                else:
                    #Detected a wall to the left and ahead relative to the robot
                    Angle=-90
            else:
                #Detected a wall to the left relative to the robot
                Angle=0
        else:
            #There is no wall detected to the left 
            if (tiles[int(tile_num/10)][tile_num % 10][0][W]==0):
                Angle = 90
            else:
                #There is no recording of a wall
                terminator(LEFT, 60, scan)
                
                #It detects a wall to the left 
                if distance.get_distance(MM)<=80:
                    #It records the wall 
                    if scan:
                        tiles_list[int(tile_num/10)][tile_num % 10][0][W] = 1
                        adjacent_filler(tile_num, 1, orientation=W)

                    Angle = -60
                #It detects there is no wall to the left 
                else:
                    Angle = 30
    
            
        #Rotate the robot given the appropriate angle 
        terminator(LEFT, Angle, scan)
    
        
        #Record wall or not
        if scan:
            wall_mapping()

        #If there is a wall infront of it, it will keep on rotating to the right until there is no wall. It scans and records every 90 degrees 
        while distance.get_distance(MM)<step:
            wait(5, MSEC)
            terminator(RIGHT, 90, scan)
            #record
            if scan:
                wall_mapping()

    if scan:
        wall_mapping()
    
    #The robot is facing the direction where it is moving to the next tile 
    #North direction relative to the direction of the robot is used to determine the next tile number
    N, E, S, W = frame()
    if N == 0:
        tile_num += 10 #moving North 
    elif N == 1:
        tile_num += 1 #moving East 
    elif N == 2:
        tile_num -= 10 #moving South 
    elif N == 3:
        tile_num -= 1 #moving West

    #It keeps track of the amount of tiles it has been on the tile
    count = 1
    tiles[int(tile_num/10)][tile_num%10][1] += 1
    
    #Keep going forward until it has to turn
    while(tiles[int(tile_num/10)][tile_num % 10][0][W]==1 and tiles[int(tile_num/10)][tile_num % 10][0][N]==0) and not (tiles[int(tile_num/10)][tile_num % 10][1] == 2 and tiles[int(tile_num/10)][tile_num % 10][0][E] == 'u'):
       
        wait(1, MSEC)
        #Add a count per tile that doesn't need a turn
        count += 1
        if N == 0:
            tile_num += 10
        elif N == 1:
            tile_num += 1
        elif N == 2:
            tile_num -= 10
        elif N == 3:
            tile_num -= 1
        tiles[int(tile_num/10)][tile_num%10][1] += 1
        
    #Drive as many steps as counts
    drivetrain.drive_for(FORWARD, step*count, MM)

#This function displays the map. 
#Each tile is represented by 2 by 2 squares I MEAN. two rows of tiles
#For each row of tiles it has to print out two colored square for each tile and two rows of tiles.
#The north and west walls of each tile is only taken into concideration. 
#The black square represents the walls in the maze. White, yellow, orange, blue squares represent tiles.
#target specifies at which list is being displayed
def display_map(target=0):
    
    #read in the list tiles
    if target== 0:
        global tiles_list
        tiles = tiles_list
    elif target == 'start':
        global starting
        tiles = starting
    elif target == 'robot':
        global robot
        tiles = robot
    
    #Starting from the left top corner and working down each row from 70 to 0 
    #initialising the row 
    ten=7
    #for the diffence rows of tiles
    for j in range (8):
       
        #initialising the column 
        row = 1
        
        # For each tile number it has to print out two square and two rows of each row of tile number 
        for i in range (2):
            
            #For the diffence rows of tiles starting with 0 to 77
            for unit in range(8):
                
                if (row==2) and (ten==0) and (unit==4):
                    
                    #The starting tile 
                    if tiles[ten][unit][0][3]==1:
                        brain.print('⬛🟩')
                    else:
                        Color_tile(ten,unit,row,1)
                        brain.print('🟩')
                else:
                    #All the sides of the walls are closed 
                    if (tiles[ten][unit][0][0]==1)and (tiles[ten][unit][0][1]==1)and (tiles[ten][unit][0][2]==1)and (tiles[ten][unit][0][3]==1):
                        brain.print('⬛⬛')
                    else:
                        #The north wall is closed  
                        if tiles[ten][unit][0][0]==1:
                            
                            if row ==1:
                                #It is at the finish line wall
                                if (ten==7)and (unit==3):
                                    brain.print('⬛🔚')
                                else:
                                    brain.print('⬛⬛')
                            else :
                                #It is at the finish line tile 
                                if (ten==7)and (unit==3):
                                    if tiles[ten][unit][0][3]==1:
                                        brain.print('⬛🟥')
                                    else:
                                        Color_tile(ten,unit,row,1)
                                        brain.print('🟥')
                                else:
                                    #There is a wall to the West and wall to the North of the tile 
                                    if tiles[ten][unit][0][3]==1:
                                        brain.print('⬛')
                                        Color_tile(ten,unit,row,2)
    
                                    else:
                                        #There is only a wall to the North of the tile 
                                        Color_tile(ten,unit,row,1)
                                        Color_tile(ten,unit,row,2)
                                        
                        #There is only a wall to the West of the tile
                        elif tiles[ten][unit][0][3]==1:
                            brain.print('⬛')
                            Color_tile(ten,unit,row,2)
                        
                        #When there is a wall to the East of the tile ahead of the tile it is displaying or there is a wall to the north of the tile that is to the left of the tile it is displaying.
                        elif (tiles[ten+1][unit][0][3]==1)or(tiles[ten][unit-1][0][0]):
                            if row==1:
                                brain.print('⬛')
                                Color_tile(ten,unit,row,2)
                            else :
                                Color_tile(ten,unit,row,1)
                                Color_tile(ten,unit,row,2)
                        
                        #There is no wall to the North or West of the tile 
                        else:

                            Color_tile(ten,unit,row,1)
                            Color_tile(ten,unit,row,1)
            
            #This prints the outside East wall that surrounds the map and moves to the next row 
            brain.print('⬛')
            brain.print('\n')
            row = row+1 
        ten= ten-1
    
    #prints the outside South wall that surrounds the map
    for p in range (17):
        #Prints out the starting tile wall 
        if (p==9):
            brain.print('🟩')
        else:
        
            brain.print('⬛')

#Each tile is broke up into a 2 by 2 squares, each square is displayed individually 
#The function is responisible for displaying the tiles
#This function is given the the tile number it is displaying, the row of the squre and the column of the square. 
#It looks through a specific list and assigns the approciate colour square to reprsent the tile.
#If it is displaying the map without displaying any paths it assigns a white square 
#It assigns a blue square to represent the path from the starting line to the finish line,
#an orange tile to represent the path of the robot initial position to the final line, and a yellow square to represent the
#joint path. It prints the square to the screen
def Color_tile(ten_p,unit_p,row,order):

    global starting
    global robot
    global count_distance_starting
    global count_distance_robot

    #It first needs to find which tile numbers are on the different list 
    starting_number=[] #keeps the list of tile numbers on the path from the starting position to the final position 
    robot_number=[] #keeps the list of tile numbers on the path from the robots initial position to the final position
    
    #It checks if it has recorded all the tiles in the maze
    total_length_start = sum(len(row) for row in starting) 
    total_length_robot = sum(len(row) for row in robot)

    #If it has recorded all the walls in the maze then it prints out the appropriate path.
    if(total_length_start == 64) and (total_length_robot==64) :
        #Determines what path it is on 
        color_s=0  
        color_r=0
        
        #Checks how many walls are closed 
        count_wall_starting=0
        count_wall_robot=0
    
        for index in range(4):
            if starting[ten_p][unit_p][0][index]==1:
                count_wall_starting=count_wall_starting+1
                
            if (robot[ten_p][unit_p][0][index])==1:
                count_wall_robot=count_wall_robot+1
                
            wait(1,MSEC)

        #For the square in the top right corner it checks the tile to the north to see if it is on the list aswell 
        if row ==1 and order==2:
            
            count_wall_starting_adj=0
            count_wall_robot_adj=0
    
            for index in range(4):
                if starting[ten_p+1][unit_p][0][index]==1:
                    count_wall_starting_adj=count_wall_starting_adj+1
                    
                if (robot[ten_p+1][unit_p][0][index])==1:
                    count_wall_robot_adj=count_wall_robot_adj+1
                
                wait(1,MSEC)
            #The tile to the north is neither on both list therefore it should display a white tile 
            if count_wall_starting_adj==4 and  count_wall_robot_adj==4:
                return brain.print("⬜")
        #For the square in the bottom left corner it checks the tile to the north to see if it is on the list as well
        if row ==2 and order ==1:
            
            count_wall_starting_adj=0
            count_wall_robot_adj=0
    
            for index in range(4):
                if starting[ten_p][unit_p-1][0][index]==1:
                    count_wall_starting_adj=count_wall_starting_adj+1
                   
                if (robot[ten_p][unit_p-1][0][index])==1:
                    count_wall_robot_adj=count_wall_robot_adj+1
               
                wait(1,MSEC)
            #The tile to the left is neither on both list therefore it should displays a white tile 
            if count_wall_starting_adj==4 and count_wall_robot_adj==4:
                return brain.print("⬜")
        
        #If all the four walls are not closed it is on the starting position to the finish position  
        if count_wall_starting is not 4 :
            color_s=1
        #If all the four walls are not closed it is on the robots initial position to the finish position
        if count_wall_robot is not 4  :
            color_r=1

        if color_s==1 and color_r==1:
            brain.print("🟨")  #joint path
            count_distance_starting = 1 + count_distance_starting
            count_distance_robot = 1 + count_distance_robot
        elif color_s==1:
            brain.print("🟦") #starting position-finish line 
            count_distance_starting = 1 + count_distance_starting
        elif color_r==1:
            brain.print("🟧") #robots position-finish line 
            count_distance_robot = 1 + count_distance_robot
        else:
            brain.print("⬜") #not on the path 
    else:
        brain.print("⬜") #Printing out normal maze with no paths 

#Checks for dead ends, which are tiles with 3 walls, which means it looks for a tile with
#3 ones. returns true if it's a dead end and the orientation of the opening
def dead_end_checker(tile):

    count = 0
    for wall in range(len(tile)):
        if tile[wall] == 1:
            count += 1
        else:
            orientation = wall
    if count == 3:
        return [True, orientation]
    else:
        return [False, 'u']

#Solves the maze. It looks for all the dead ends and close them. That converts the next tile into a dead end
#That one will be close again until it gets to a joint (only 1 wall). It is an intersection. After looping
#through all the tiles, only the ones that are part of the solution will remain open (without four 1s
#on them). Target specifies to which list it should be applied and what is the starting point, either the
#starting tile (4) or the current position of the robot
def solver(target):
  if target == 'start':
      global starting
      tiles = starting
      end = 4
  else:
      global robot
      tiles = robot
      end = num_predictor(location.position(X, MM), location.position(Y, MM))
  # global tiles
  while True:
      wait(5,MSEC)
      dead = 0
      for row in range(8):
          for tile in range(8):
              tile_dead = dead_end_checker(tiles[row][tile][0])
              if tile_dead[0] and row*10+tile != 73 and row*10+tile != end:
                  tiles[row][tile][0] = [1, 1, 1, 1]
                  adjacent_filler(row*10+tile, 1, tile_dead[1], target)
                  dead += 1
      if dead == 0:
          break

#This solves for any squares in the maze. It puts all the walls in open tiles that make a square to 1
#It can also be applied to different lists through target
def square_solver(target):
    if target == 'start':
        global starting
        tiles = starting
    else:
        global robot
        tiles = robot
    for row in range(8):
        for col in range(8):
            current_tile = row*10+col
            count = 0
            holes = 0  #Number of holes in the tile           
            for i in range(4):
                wait(1, MSEC)
                #Change the reference frame
                N, E, S, W = frame(i*90)
                total_walls = 0
                for wall in tiles[int(current_tile/10)][current_tile%10][0]:
                    total_walls += wall
                if total_walls > 2:
                    holes = 0
                    break
                #Check if it has an opening at the right positions
                if (tiles[int(current_tile/10)][current_tile%10][0][N] == 1 and tiles[int(current_tile/10)][current_tile%10][0][W] == 0) or (tiles[int(current_tile/10)][current_tile%10][0][N] == 0 and tiles[int(current_tile/10)][current_tile%10][0][W] == 1):
                    opened = current_tile
                    holes += 1
                    if holes == 2:
                        break
                #If it has 2 openings, leave the loop
                elif tiles[int(current_tile/10)][current_tile%10][0][N] == 0 and tiles[int(current_tile/10)][current_tile%10][0][W] == 0:
                    holes = 2
                    break
                
                #Turn on the square counterclockwise and starting from the left lower corner
                if i == 0 and current_tile%10<7:
                    current_tile += 1
                elif i == 1 and current_tile>10:
                    current_tile -= 10
                elif i == 2:
                    current_tile -= 1
                elif i == 3:
                    current_tile += 10
            
            #If there is 1 hole, close all the walls on it
            if holes == 1:
                tiles[int(opened/10)][opened%10][0] = [1, 1, 1, 1]
                adjacent_filler(opened, 1, 0, target)
                adjacent_filler(opened, 1, 1, target)
                adjacent_filler(opened, 1, 2, target)
                adjacent_filler(opened, 1, 3, target)

#This function counts the amount of walls in a tile. It accepts the tile number and the list to work with
def wall_count(num, target):
    if target == 'start':
        global starting
        tiles = starting

    elif target == 'robot':
        global robot
        tiles = robot

    else:
        global tiles_list
        tiles = tiles_list
    count = 0
    walls = tiles[int(num/10)][num%10][0]
    for wall in walls:
        count += wall
    return count

#Gets rid of all the paths that are too long and leaves the shortest path
def path_finder(target):
    if target == 'start':
        global starting
        tiles = starting
    elif target=='robot':
        global robot
        tiles = robot
    
    global paths
    #Intersection. These are all the tiles with 1 wall.
    #If there is more than one valid path, at some point they will join at an intersection 
    inter = []
    
    for row in range(8):
        for tile in range(8):
            count = 0
            for wall in range(len(tiles[row][tile][0])):
                wait(1, MSEC)
                count += tiles[row][tile][0][wall]
    
            if count == 1:
                inter.append(row*10+tile)


    #Keep looping while there is at least one intersection to check
    while len(inter) != 0:

        wait(5, MSEC)
        tile_num = inter[0]
    
        for wall in range(4):
            tile_num = inter[0]
            walls = tiles[int(tile_num/10)][tile_num%10][0]
            path = []
            #Take the first open wall
            if walls[wall] == 0:
                orientation = wall
    
                #Moves to the second tile
                if orientation == 0:
                    tile_num += 10
                    orientation = 2
                    
                elif orientation == 1:
                    tile_num += 1
                    orientation = 3
    
                elif orientation == 2:
                    tile_num -= 10
                    orientation = 0
    
                elif orientation == 3:
                    tile_num -= 1
                    orientation = 1

                wall_c = wall_count(tile_num, target)
    
    
                path.append(tile_num)
                current_tile = num_predictor(location.position(X, MM), location.position(Y, MM))
                #Keep walking through the path until it gets to another intersection, to the robot
                #Or to a dead end, which after being solved before, this may only happen at the end or
                #the start of the maze
                while wall_c == 2 and current_tile != tile_num:
                    wait(1, MSEC)
                    wall_c = 3
    
                    indeces = list(range(4))
                    #Take out of the list the wall that is opened
                    indeces.pop(orientation)
                    walls = tiles[int(tile_num/10)][tile_num%10][0]
    
                    #Change the index
                    for index in indeces:
                        if walls[index] == 0:
                            orientation = index
    
                    #Moves to the next tile
                    if orientation == 0:
                        tile_num += 10
                        orientation = 2
                        
                    elif orientation == 1:
                        tile_num += 1
                        orientation = 3
    
                    elif orientation == 2:
                        tile_num -= 10
                        orientation = 0
    
                    elif orientation == 3:
                        tile_num -= 1
                        orientation = 1
    
                    wall_c = wall_count(tile_num, target)
    
    
                    path.append(tile_num)
    
                paths.append(path)

        #Delete the current intersection from the list
        del inter[0]

    if target == 'start':
        start_tile = 4
    else:
        start_tile = num_predictor(location.position(X, MM), location.position(Y, MM))
    
    #If a path ends in the ending, the starting tile or the position of the robot, delete it from the list
    for path in range(len(paths)):
        if (paths[path][-1] == 73 or paths[path][-1] == start_tile):
            paths[path] = ''
    
    while '' in paths:
        wait(5, MSEC)
        paths.remove('')
    
    #Look for the shortes path. It will be the one with less tiles on it
    min_length = 64  #Set minimum to 64 as there is no path longer than 64 (number of tiles in the maze)
    min_path = []
    for path in paths:
        if min_length > len(path):
            min_length = len(path)

    
    #Remove all the paths that have the same length as min_length
    for path in range(len(paths)):
        if len(paths[path]) == min_length:
            paths[path] = ''
    
    while '' in paths:
        paths.remove('')
    
    #Close all remaining paths as these are the longer ones
    for path in paths:
        for tile in path[:-1]:
            walls = tiles[int(tile/10)][tile%10][0]
            for wall in range(len(walls)):
                if walls[wall] == 0:
                    adjacent_filler(tile, 1, wall, target)
    
            tiles[int(tile/10)][tile%10][0] = [1, 1, 1, 1]

#It returns the coordinates of a tile given it's number
def coord_predictor(num):
    global origin
    global tile
    global wall

    a = int(num/10)     #Dividing the number by ten and truncating the result gives the tens
    b = num % 10        #Modulo of the number gives the units

    x = origin+tile/2+(tile+wall)*b  #The corner of the first tile is at origin+tile/2
    y = origin+tile/2+(tile+wall)*a
    return x, y

#It return the coordinates of a point measured by the distance sensor
def get_coord():
    dist = distance.get_distance(MM)
    if dist < 3000:     #If the distance is greater or equal to 3000 no calculations will be made
        #The x coordinate of the object can be calculated by multiplying the distance times the
        #cos of the angle of the robot plus the current x position of the robot
        #For y is the same but with sin and the y position of the robot
        #90 must be substracted from the angle as the robot counts a 90º angle as a 0º angle
        dist += 72    #72 is the distance from the center of the robot to the distance sensor (measured
        if drivetrain.heading(DEGREES) < 90:
            x = dist * abs(math.cos((90 - drivetrain.heading(DEGREES)) * math.pi / 180)) + location.position(X, MM)
            y = dist * math.sin((90 - drivetrain.heading(DEGREES)) * math.pi / 180) + location.position(Y, MM)

        else:
            x = dist * math.cos((90 - drivetrain.heading(DEGREES)) * math.pi / 180) + location.position(X, MM)
            y = dist * math.sin((90 - drivetrain.heading(DEGREES)) * math.pi / 180) + location.position(Y, MM)
        return [x, y]

#Returns the orientation of the wall that is being measured in a tile
#It calculates the distance from the center of each wall to the measured point.
#The lowest distance will be the one of the wall that is being measured
def where(tile, coord):
    x, y = coord_predictor(tile)

    half_tile = 115

    xn, yn = x, y + half_tile
    xe, ye = x + half_tile, y
    xs, ys = x, y - half_tile
    xw, yw = x - half_tile, y

    distances = [math.sqrt((xn-coord[0])**2+(yn-coord[1])**2), math.sqrt((xe-coord[0])**2+(ye-coord[1])**2),
            math.sqrt((xs-coord[0])**2+(ys-coord[1])**2), math.sqrt((xw-coord[0])**2+(yw-coord[1])**2)]

    dist = min(distances)
    pos = distances.index(dist)


    return pos

#Returns the coordinates of the corners of a tile
def corners_tile(num):
    x1 = origin + (tile + wall) * (num%10)
    y1 = origin + (tile + wall) * int(num/10)

    x2 = origin + (tile + wall) * (num%10)
    y2 = origin + (tile + wall) * int(num/10)+tile

    x3 = origin + (tile + wall) * (num%10)+tile
    y3 = origin + (tile + wall) * int(num/10)

    x4 = origin + (tile + wall) * (num%10)+tile
    y4 = origin + (tile + wall) * int(num/10)+tile

    return [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]

#Returns true is a point lies within a distance of radius to the corner of a tile.
#This needs to be done as the walls in vexcode seems to be sticking out from the measuring of the sensor.
#We can take this as invalid measurements
def forbidden(x, y, tile):
    global forbidden_areas

    radius = 100
    for corner in forbidden_areas[int(tile/10)][tile%10]:
        dist = distance_to_corner(x, y, corner)
        if dist <= radius:
            return True

    return False

#Get the distance of a point to a corner
def distance_to_corner(x, y, corner):
    dist = math.sqrt((x-corner[0])**2 + (y-corner[1])**2)
    return dist

#Creates all the coordinates of all the corners of the tiles
def init_forbidden():
    global forbidden_areas

    for row in range(8):
        forbidden_areas.append([])
        for col in range(8):
            tile_num = row*10+col
            corners = corners_tile(tile_num)
            forbidden_areas[row].append(corners)

#It gives the angle in positive values depending on whether is needed to turn left or right
def current_angle(direction):
    mes_angle = drivetrain.heading(DEGREES)
    if direction == RIGHT:
        angle = mes_angle

    elif direction == LEFT:
        if mes_angle == 0:
            angle = 0
        
        #Substracting 360 swaps the direction of a positive turn
        else:
            angle = 360 - mes_angle

    return angle


#This can be use the same was as drivetrain.turn_for, but it will scan and assign values to walls as it
#turns
def terminator(direction, angle, scan):
    global tiles_list
    global forbidden_areas

    #If the angle is negative, reverse the direction of turn
    if angle < 0:
        angle = abs(angle)
        if direction == RIGHT:
            direction = LEFT
        else:
            direction = RIGHT
    
    #Initial angle
    init = current_angle(direction)
   
    #Calculate angle to turn to
    if direction == LEFT:
        objective = init + angle

    elif direction == RIGHT:
        objective = init + angle

    while objective >= 360:
        wait(1,MSEC)
        objective -= 360
    #Get the current angle according to the direction it needs to turn
    cur_angle = current_angle(direction)

    drivetrain.turn(direction)
    #Scan while the angle is in a range of +-25º of the objective.
    #This wide range is needed as vexcode does not update the values of the angles too often.
    #They are update in a specific time period we think is dependent of the internet connecton speed.
    #So if the connection is slow enough, the robot will make a 360º angle, as it would have skipped
    #the stopping range
    while cur_angle < objective-25 or cur_angle > objective+25:
        wait(1, MSEC)
        #Only scan if it is required. Needed to navigate through the solution without
        #overwritting it
        if scan:
            coord = get_coord()
            if coord != None:
                x, y = coord
                tile_num = num_predictor(x, y)
                status = forbidden(x, y, tile_num)  #True if it's too close to a corner
                #A max disamestance to measure was needed. When the robot was not perpendicular to the wall
                #It was not accurate enough to get a reliable calculation
                #The more parallel it is to the wall it's measuring, the less accurate the measurement
                #gets.
                if (status == False and distance.get_distance(MM) < 630):
                    pos = where(tile_num, [x, y])
                    tiles_list[int(tile_num/10)][tile_num%10][0][pos] = 1
                    adjacent_filler(tile_num, 1, orientation=pos)

        cur_angle = current_angle(direction)

    drivetrain.stop()
    wait(100, MSEC)
    cur_angle = current_angle(direction)
    while cur_angle != objective:
        drivetrain.turn_for(direction, objective-cur_angle, DEGREES)
        cur_angle = current_angle(direction)

#Steps to be taken to solve a maze in a list
def solve_maze(target):
    #Solve for dead ends
    solver(target)
    #Solve any squares
    square_solver(target)
    #Take out the longer paths
    path_finder(target)
    #Take remaining dead ends from previous operations
    solver(target)

def main():
    wait(2, SECONDS)
    global tiles_list
    tiles = tiles_list
    global paths

    drivetrain.set_turn_velocity(100, PERCENT)
    drivetrain.set_drive_velocity(100, PERCENT)

    global starting
    global robot
    global count_distance_starting 
    global count_distance_robot 
    monitor_variable('tiles_list')
    monitor_variable('starting')
    monitor_variable('robot')
    monitor_variable('paths')

    #Initiate tiles and corners of the tiles
    init_tiles()
    init_forbidden()

    #Navigate through the maze and scan all the walls
    mad_eye_strategy()

    brain.print('\n\n')

    #Solve the maze from the starting position and from the robot's position
    solve_maze('start')
    solve_maze('robot')

    #Display the solution are you tryin to calculate how many spaces we need?
    brain.print('             TRIWIZARD MAZE\n' )
    display_map()
    brain.print('\n')
    brain.print('\n Lengend \n 🟨 From starting line-finish line and robot position-finish line path \n 🟦 From starting line to finish line path\n 🟧 From robot position to finish line path\n ⬜ Tiles that are not on the paths\n')
    brain.print('\n')
    time = brain.timer_time(SECONDS)
    brain.print(f'🏆 WE FOUND THE TRIWIZARD CUP IN {int(time/60)}:{round(time%60)} 🏆')
    brain.print(f'\n\nShortest distance from the start to the end: {round((count_distance_starting)*125)} mm \nShortest distance from the robot initial position to the end: {round((count_distance_robot)*125)} mm \n')
    #Drive through the solution
    beat_it()

vr_thread(main())