from Vision.Vision import Vision
from SerialInterface.SerialInterface import SerialInterface
from Graph.Graph import Graph
from Graph.Vertex import Vertex
from time import sleep
from time import time
from math import pi

enableVisuals = False # disable this if you're using ssh because it slows down the program.
disableSerial = False # disable this if you're not using the serial interface

cam = Vision()
if not disableSerial:
    ser = SerialInterface('/dev/ttyUSB0', 115200)
    robotPosition = ser.getPose()

robotPosition = [0, 0 ,0]
targetPosition = robotPosition.copy()

# create the state machine
states = Graph()
searching = Vertex('searching') # searching for a new target
search_relocation = Vertex('search_rellocation') # searching for a new target
following = Vertex('following') # target is in sight and being followed
lost = Vertex('lost') # target has been permanently lost
reaquire = Vertex('reaquire') # target has been temporarily lost and is being reaquired
states.addEdge(searching, following, 1)
states.addEdge(searching, search_relocation, 1)
states.addEdge(search_relocation, searching, 1)
states.addEdge(following, reaquire, 1)
states.addEdge(reaquire, following, 1)
states.addEdge(reaquire, lost, 1)
states.addEdge(lost, searching, 1)
states.printGraph()

# we always start in the searching state
states.setCurrentVertex(searching)

print("Beginning main loop...")

while True:
    # get the robot position
    if not disableSerial:
        robotPosition = ser.getPose()
    if enableVisuals == True:
        if cam.drawTarget() == False:
            break
        
    # find a target at the current location
    if states.current_vertex is searching:
        # if this is the first time we've been in this state recently
        if not searching.isVisited:
            print("Searching for a target...")
            searching.isVisited = True
            searching.data = time()
            if not disableSerial:
                ser.setTargetPose(0, robotPosition[2])
        
        is_aquired = cam.aquire_target()
        if is_aquired:
            states.setCurrentVertex(following)
            searching.isVisited = False
        elif time() - searching.data > 2:
            states.setCurrentVertex(search_relocation)
            search_relocation.isVisited = False
    
    # rellocate to find a target
    if states.current_vertex is search_relocation:
        # if this is the first time we've been in this state recently
        if not search_relocation.isVisited:
            print("Relocating to find a target...")
            search_relocation.isVisited = True
            search_relocation.data = time()
            targetPosition = robotPosition.copy()
            targetPosition[2] += pi/2
            if not disableSerial:
                ser.setTargetPose(0, targetPosition[2])
                
        if (robotPosition[2]-targetPosition[2]) < 0.1 or time() - search_relocation.data > 10:
            states.setCurrentVertex(searching)
            searching.data = time()
            search_relocation.isVisited = False
    
    if states.current_vertex is lost:
        print("Could not relloate target...")
        states.setCurrentVertex(searching)
    
    if states.current_vertex is following:
        # if this is the first time we've been in this state recently
        if not following.isVisited:
            print("Target found, beggining to follow...")
            following.isVisited = True
            following.data = time()
        
        # get the target's position
        target = cam.getTarget()
        
        if target == False:
            # stop moving
            if not disableSerial:
                ser.setTargetPose(0, robotPosition[2])
            
            if time() - following.data > 2:
                states.setCurrentVertex(reaquire)
                following.isVisited = False
            continue
        
        
        # calculate the velocity
        velocity = 15000*(0.04 - target[2])      
        
        # calculate the angle
        angle = 3*(0.5 - target[0]/cam.width)
        
        print("Velocity:", velocity, "Angle:", angle+robotPosition[2])
               
        # set the target pose
        if not disableSerial:
            ser.setTargetPose(velocity, angle+robotPosition[2])
    
    
    # TODO: make this more than just a repeat of the search state
    if states.current_vertex is reaquire:
        # get the last known position of the target
        last_pos = cam.target
        
        # determine whether to move forward or backward based on the last known position
        vel = 7000*(0.04 - last_pos[2]) - 3000*(0.5 + last_pos[1]/cam.height)
        # determine how much to rotate based on the last known position
        angle = 3*(0.5 - last_pos[0]/cam.width)
        
        if abs(vel) < 50:
            vel = 0
            if abs(angle) < 0.1:
                angle = 0
        
        if not disableSerial:
            ser.setTargetPose(vel, angle+robotPosition[2])
        
        # if this is the first time we've been in this state recently
        if not reaquire.isVisited:
            print("Target lost, reaquiring...")
            reaquire.isVisited = True
            reaquire.data = time()
        
        # try to reaquire the target
        is_aquired = cam.aquire_target()
        
        if is_aquired == True:
            reaquire.isVisited = False
            states.setCurrentVertex(following)
            if not disableSerial:
                ser.setTargetPose(0, robotPosition[2])
            continue
            
            
        if time() - reaquire.data > 6:
            states.setCurrentVertex(lost)
            reaquire.isVisited = False
            if not disableSerial:
                ser.setTargetPose(0, robotPosition[2])

cam.cleanup()