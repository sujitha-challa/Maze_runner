import numpy as np
from numpy import random
import heapq
import time
import matplotlib.pyplot as plt

# The priorityQueue class uses heapq to perform push, pop actions.
class PriorityQueue:
    def __init__(self):
        self._data = []
        self._index = 0

    def push(self, item, priority):
        heapq.heappush(self._data, (priority, self._index, item))
        self._index += 1

    def pop(self):
        return heapq.heappop(self._data)[-1]

    def size(self):
        return len(self._data)

    def nodeAtZeroIndex(self):
        return (self._data[0])[2]

    def nodes(self):
        return self._data

# The class 'Node' stores the data of node - current position, parent of the node, 
# cost of the node(f, g and h values, where f = g + h).
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
        self.N = 0
        self.visited = False
        self.confirmed = False
        self.C = 0
        self.B = 0
        self.E = 0
        self.H = 0


    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

# The returnPath function returns the path from the search made
def returnPath(currentNode, maze):
    route = []
    presentNode = currentNode

    while presentNode is not None:
        route.append(presentNode.position)
        presentNode = presentNode.parent
    # Return the path in reverse order as we need to show the path from source to destination.
    route = route[::-1]
    return route

# AStarSearch perform A* search operation.
# With input as Grid, source node, destination node and heuristic to be used to calculate cost of node.
def AStarSearch(Grid, source, destination, heuristic):
    global solvable, totalPlanningTime, processedCells
    # Create source and destination node with initized values for g, h and f
    sourceNode = Node(None, tuple(source))
    sourceNode.g = sourceNode.h = sourceNode.f = 0
    destinationNode = Node(None, tuple(destination))
    destinationNode.g = destinationNode.h = destinationNode.f = 0

    # Initialize the fringe as priority queue and visitedList as set
    # The fringe contains the neighbours that are ready for exploration in ascending order with cost as their priority
    # And the node with lowest cost from source to destination in the fringe will be explored first.
    fringe = PriorityQueue()
    # visitedList will contain the nodes which are explored so that we don't explore it again
    visitedList = set()
    nodeCost = {}

    startTime = time.time()  # Start time
    # Add the start node
    fringe.push(sourceNode, sourceNode.f)

    # Continue in the while loop until you find the destination
    while fringe.size() > 0:

        # Get the node with least cost of f(n) to explore
        presentNode = fringe.nodeAtZeroIndex()

        # Pop present node out off fringe, and add to visitedList
        fringe.pop()

        visitedList.add(presentNode)

        # Check if the destination is reached, if reached return the path
        if presentNode == destinationNode:
            solvable = True
            endTime = time.time()  # End time
            totalPlanningTime = totalPlanningTime + (endTime - startTime)
            return returnPath(presentNode, Grid)

        # Increased processed cell count here to keep track of number of cells processed.
        processedCells += 1

        # Generate children from all adjacent nodes in four directions(North, South, Est, West)
        children = []

        for newPosition in neighbours_in_4_directions:
            # Get the position of the the node.
            nodeLocation = (presentNode.position[0] + newPosition[0], presentNode.position[1] + newPosition[1])

            # Check if the node position is within the maze dimensions
            if (nodeLocation[0] > (rowCount - 1) or
                    nodeLocation[0] < 0 or
                    nodeLocation[1] > (columnCount - 1) or
                    nodeLocation[1] < 0):
                continue

            # Check here if node is blocked or not.
            # If node is blocked, continue the loop(search for other path)
            if Grid[nodeLocation[0]][nodeLocation[1]] == 1:
                continue

            # Create a new node with pressent node as parent.
            newNode = Node(presentNode, nodeLocation)

            # Append the new node to children
            children.append(newNode)

        # Loop through children
        for child in children:

            # Check if the child is in the visitedList
            # If the child is in the visitedlist, it means that the child is already explored.
            if child in visitedList:
                continue

            # Calculate the f, g, and h values
            child.g = presentNode.g + 1
            # Heuristic cost is calculated below
            child.h = GetHeuristicValue(child, destinationNode, heuristic)
            child.f = child.g + child.h

            # If child is already in the fringe and g cost is already lower
            if child.position in nodeCost and nodeCost[child.position] <= child.g:
                continue

            # Store g value of each explored node here
            nodeCost[child.position] = child.g

            # Add the child to the fringe
            fringe.push(child, child.f)

    endTime = time.time()  # End time
    totalPlanningTime = totalPlanningTime + (endTime - endTime)
    # When fringe is empty and grid is not solvable
    solvable = False

# AStarSearch perform A* search operation.
# With input as Grid, source node, destination node and heuristic to be used to calculate cost of node.
def AStarSearchForAgent9(Grid, source, destination, heuristic):
    global solvable, totalPlanningTime, processedCells
    # Create source and destination node with initized values for g, h and f
    sourceNode = Node(None, tuple(source))
    sourceNode.g = sourceNode.h = sourceNode.f = 0
    destinationNode = Node(None, tuple(destination))
    destinationNode.g = destinationNode.h = destinationNode.f = 0

    # Initialize the fringe as priority queue and visitedList as set
    # The fringe contains the neighbours that are ready for exploration in ascending order with cost as their priority
    # And the node with lowest cost from source to destination in the fringe will be explored first.
    fringe = PriorityQueue()
    # visitedList will contain the nodes which are explored so that we don't explore it again
    visitedList = set()
    nodeCost = {}

    startTime = time.time()  # Start time
    # Add the start node
    fringe.push(sourceNode, sourceNode.f)

    # Continue in the while loop until you find the destination
    while fringe.size() > 0:

        # Get the node with least cost of f(n) to explore
        presentNode = fringe.nodeAtZeroIndex()

        # Pop present node out off fringe, and add to visitedList
        fringe.pop()

        visitedList.add(presentNode)

        # Check if the destination is reached, if reached return the path
        if presentNode == destinationNode:
            solvable = True
            endTime = time.time()  # End time
            totalPlanningTime = totalPlanningTime + (endTime - startTime)
            return returnPath(presentNode, Grid)

        # Increased processed cell count here to keep track of number of cells processed.
        processedCells += 1

        # Generate children from all adjacent nodes in four directions(North, South, Est, West)
        children = []

        for newPosition in neighbours_in_8_directions:
            # Get the position of the the node.
            nodeLocation = (presentNode.position[0] + newPosition[0], presentNode.position[1] + newPosition[1])

            # Check if the node position is within the maze dimensions
            if (nodeLocation[0] > (rowCount - 1) or
                    nodeLocation[0] < 0 or
                    nodeLocation[1] > (columnCount - 1) or
                    nodeLocation[1] < 0):
                continue

            # Check here if node is blocked or not.
            # If node is blocked, continue the loop(search for other path)
            if Grid[nodeLocation[0]][nodeLocation[1]] == 1:
                continue

            # Create a new node with pressent node as parent.
            newNode = Node(presentNode, nodeLocation)

            # Append the new node to children
            children.append(newNode)

        # Loop through children
        for child in children:

            # Check if the child is in the visitedList
            # If the child is in the visitedlist, it means that the child is already explored.
            if child in visitedList:
                continue

            # Calculate the f, g, and h values
            child.g = presentNode.g + 1
            # Heuristic cost is calculated below
            child.h = GetHeuristicValue(child, destinationNode, heuristic)
            child.f = child.g + child.h

            # If child is already in the fringe and g cost is already lower
            if child.position in nodeCost and nodeCost[child.position] <= child.g:
                continue

            # Store g value of each explored node here
            nodeCost[child.position] = child.g

            # Add the child to the fringe
            fringe.push(child, child.f)

    endTime = time.time()  # End time
    totalPlanningTime = totalPlanningTime + (endTime - endTime)
    # When fringe is empty and grid is not solvable
    solvable = False

def GetHeuristicValue(sourceNode, destinationNode, heuristic):
    if heuristic == 0:
        # This is manhattan distance
        heuristicValue = ((destinationNode.position[0] - sourceNode.position[0]) + (destinationNode.position[1] - sourceNode.position[1]))

    elif heuristic == 1:
        # This is euclidian distance
        heuristicValue = np.sqrt(((sourceNode.position[0] - destinationNode.position[0]) ** 2) + (
                (sourceNode.position[1] - destinationNode.position[1]) ** 2))

    elif heuristic == 2:
        # This is Chebysev distance
        heuristicValue = np.maximum(destinationNode.position[0] - sourceNode.position[0], destinationNode.position[1] - sourceNode.position[1])

    return heuristicValue

def Find_Unblocked_Neighbors_In_8_Directions(currentNode):
    global rowCount, columnCount
    neighbourNodes = []
    for newPosition in neighbours_in_8_directions:
    # Get the position of the node in maze
        nodeLocation = (currentNode[0] + newPosition[0], currentNode[1] + newPosition[1])

        #Check if the node position is within the maze dimensions
        if (nodeLocation[0] > (rowCount - 1) or nodeLocation[0] < 0 or nodeLocation[1] > (columnCount - 1) or nodeLocation[1] < 0):
            continue
        else:
            if maze[nodeLocation[0]][nodeLocation[0]] != 1:
                neighbourNodes.append(nodeLocation)

    return neighbourNodes

def Find_Neighbors_of_Neighbors_In_8_Directions(currentNode, unblockedNeighbors):
    global rowCount, columnCount
    neighbourNodes = []

    for x in unblockedNeighbors:
        for newPosition in neighbours_in_8_directions:
        # Get the position of the node in maze
            nodeLocation = (x[0] + newPosition[0], x[1] + newPosition[1])

            #Check if the node position is within the maze dimensions
            if (nodeLocation[0] > (rowCount - 1) or nodeLocation[0] < 0 or nodeLocation[1] > (columnCount - 1) or nodeLocation[1] < 0):
                continue
            else:
                if (nodeLocation[0] == currentNode[0] and nodeLocation[1] == currentNode[1]) == False:
                   if nodeLocation not in unblockedNeighbors:
                        if discoveredGrid[x[0]][x[1]] != 1:
                            neighbourNodes.append(nodeLocation)

    return neighbourNodes

# This function generates random mazes with probability and dimension as input.
# Considering start and goal nodes are unblocked.
# node value - 1 for blocked node, 0 for unblocked node.
def RandomGridGenerator(p, dim):
    global solvable, initialSource, actualDestination, rowCount, columnCount
    remaining_density = (1 - p) / 3
    arrays = [1, 2, 3, 4]
    probabilities = [p, remaining_density, remaining_density, remaining_density]
    solvable = False

    while solvable == False:
        matrix = np.random.choice(arrays, (dim, dim), p=probabilities)
        rowCount, columnCount = np.shape(matrix)
        
        initialSource = [0, 0]
        actualDestination = [random.randint(0, dim), random.randint(0, dim)]
        if actualDestination[0] == initialSource[0]:
            while actualDestination[1] == initialSource[1]:
                actualDestination[1] = random.randint(0, columnCount)
        
        matrix[initialSource[0]][initialSource[1]] = np.random.choice([2,3,4])
        matrix[actualDestination[0]][actualDestination[1]] = np.random.choice([2,3,4])

        # Make sure grid is solvable with source and destination nodes.
        pathInFullGrid = AStarSearch(matrix, initialSource, actualDestination, 0)
        if solvable:
            solvable = True
    
    
    return matrix

def updateProbOfFindingTargetInGrid(nodevalue, nodePosition):
    global ProbOfFindingTarget_Grid
    # This is when cell is blocked
    if nodevalue == 1:
        ProbOfFindingTarget_Grid[nodePosition[0]][nodePosition[1]] = 0
    # Unexplored cell
    elif nodevalue == -1:
        ProbOfFindingTarget_Grid[nodePosition[0]][nodePosition[1]] = 0.35 * ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] 
    else:
        # Flat terrain
        if nodevalue == 2: 
            FNR = 0.2
        # hilly terrain
        elif nodevalue == 3:
            FNR = 0.5
        #thick forests
        elif nodevalue == 4:
            FNR = 0.8
        ProbOfFindingTarget_Grid[nodePosition[0]][nodePosition[1]] = (1 - FNR) * ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]

def updateProbabilitiesForAgent6(nodevalue, nodePosition, examine = False):
    global ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid
    # This is when cell is blocked
    if nodevalue == 1:
        ProbOfContainingTarget_Grid_x_y = ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]
        ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] = 0
        for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
            ProbOfContainingTarget_Grid[rowIndex][columnIndex] = ProbOfContainingTarget_Grid[rowIndex][columnIndex] / (1 - ProbOfContainingTarget_Grid_x_y)
    else:
        if examine:
            # Flat terrain
            if nodevalue == 2: 
                FNR = 0.2
            # hilly terrain
            elif nodevalue == 3:
                FNR = 0.5
            #thick forests
            elif nodevalue == 4:
                FNR = 0.8

            ProbOfContainingTarget_Grid_x_y = ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]
            ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] = (FNR * ProbOfContainingTarget_Grid_x_y) / (1-((1 - FNR) * ProbOfContainingTarget_Grid_x_y)) 
            for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
                if not (nodePosition[0] == rowIndex and nodePosition[1] == columnIndex):
                    ProbOfContainingTarget_Grid[rowIndex][columnIndex] = ProbOfContainingTarget_Grid[rowIndex][columnIndex] / (1-((1 - FNR) * ProbOfContainingTarget_Grid_x_y))

def updateProbabilitiesForAgent7(nodevalue, nodePosition, examine = False):
    global ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid
    # This is when cell is blocked
    if nodevalue == 1:
        ProbOfContainingTarget_Grid_x_y = ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]
        ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] = 0
        for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
            ProbOfContainingTarget_Grid[rowIndex][columnIndex] = ProbOfContainingTarget_Grid[rowIndex][columnIndex] / (1 - ProbOfContainingTarget_Grid_x_y)
            updateProbOfFindingTargetInGrid(discoveredGrid[rowIndex][columnIndex], [rowIndex, columnIndex])
    else:
        if examine:
            # Flat terrain
            if nodevalue == 2: 
                FNR = 0.2
            # hilly terrain
            elif nodevalue == 3:
                FNR = 0.5
            #thick forests
            elif nodevalue == 4:
                FNR = 0.8

            ProbOfContainingTarget_Grid_x_y = ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]
            ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] = (FNR * ProbOfContainingTarget_Grid_x_y) / (1-((1 - FNR) * ProbOfContainingTarget_Grid_x_y)) 
            for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
                if not (nodePosition[0] == rowIndex and nodePosition[1] == columnIndex):
                    ProbOfContainingTarget_Grid[rowIndex][columnIndex] = ProbOfContainingTarget_Grid[rowIndex][columnIndex] / (1-((1 - FNR) * ProbOfContainingTarget_Grid_x_y))
                updateProbOfFindingTargetInGrid(discoveredGrid[rowIndex][columnIndex], [rowIndex, columnIndex])
        else:
            updateProbOfFindingTargetInGrid(nodevalue, nodePosition)

def updateProbabilitiesForAgent9(nodevalue, nodePosition, examine = False):
    global ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid
    # This is when cell is blocked
    if nodevalue == 1:
        ProbOfContainingTarget_Grid_x_y = ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]
        ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] = 0
        for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
            ProbOfContainingTarget_Grid[rowIndex][columnIndex] = ProbOfContainingTarget_Grid[rowIndex][columnIndex] / (1 - ProbOfContainingTarget_Grid_x_y)
            updateProbOfFindingTargetInGrid(discoveredGrid[rowIndex][columnIndex], [rowIndex, columnIndex])
    else:
        if examine:
            # Flat terrain
            if nodevalue == 2: 
                FNR = 0.2
            # hilly terrain
            elif nodevalue == 3:
                FNR = 0.5
            #thick forests
            elif nodevalue == 4:
                FNR = 0.8

            ProbOfContainingTarget_Grid_x_y = ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]]
            ProbOfContainingTarget_Grid[nodePosition[0]][nodePosition[1]] = (FNR * ProbOfContainingTarget_Grid_x_y) / (1-((1 - FNR) * ProbOfContainingTarget_Grid_x_y)) 
            for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
                if not (nodePosition[0] == rowIndex and nodePosition[1] == columnIndex):
                    ProbOfContainingTarget_Grid[rowIndex][columnIndex] = ProbOfContainingTarget_Grid[rowIndex][columnIndex] / (1-((1 - FNR) * ProbOfContainingTarget_Grid_x_y))
                updateProbOfFindingTargetInGrid(discoveredGrid[rowIndex][columnIndex], [rowIndex, columnIndex])
        else:
            updateProbOfFindingTargetInGrid(nodevalue, nodePosition)

def findDestinationNodeForAgent6(source):
    result = []
    target = []
    targetDistance = 10000
    nodeDistances={}

    nodesWithMaxProbabilities = np.argwhere(ProbOfContainingTarget_Grid == np.max(ProbOfContainingTarget_Grid))
    for i in range(len(nodesWithMaxProbabilities)):
        x = nodesWithMaxProbabilities[i-1][0]
        y = nodesWithMaxProbabilities[i-1][1]

        # Manhattan distance
        result = abs(x - source[0]) + abs(y - source[1])
            
        if result <= targetDistance:
            target = [x, y]
            targetDistance = result

        if result in nodeDistances:
            nodeDistances[result].append([x,y])
        else:
            nodeDistances[result]=[x,y]

    return target

def findDestinationNodeForAgent7(source):
    result = []
    target = []
    targetDistance = 10000

    nodesWithMaxProbabilities = np.argwhere(ProbOfFindingTarget_Grid == np.max(ProbOfFindingTarget_Grid))
    for i in range(len(nodesWithMaxProbabilities)):
        x = nodesWithMaxProbabilities[i-1][0]
        y = nodesWithMaxProbabilities[i-1][1]

        # Manhattan distance
        result = abs(x - source[0]) + abs(y - source[1])

        if result <= targetDistance:
            target = [x, y]
            targetDistance = result

    return target

def findDestinationNodeForAgent8(source):
    result = []
    target = []
    targetDistance = 10000
    probOfFindingTarget_Grid_WithDistance = ProbOfFindingTarget_Grid

    for x in range(rowCount):
        for y in range(columnCount):
            distance = abs((x-1) - source[0]) + abs((y-1) - source[1])
            probOfFindingTarget_Grid_WithDistance[x-1][y-1] = probOfFindingTarget_Grid_WithDistance[x-1][y-1]/(distance+1)

    nodesWithMaxProbabilities = np.argwhere(probOfFindingTarget_Grid_WithDistance == np.max(probOfFindingTarget_Grid_WithDistance))
    for i in range(len(nodesWithMaxProbabilities)):
        x = nodesWithMaxProbabilities[i-1][0]
        y = nodesWithMaxProbabilities[i-1][1]

        # Manhattan distance
        result = abs(x - source[0]) + abs(y - source[1])

        if result <= targetDistance:
            target = [x, y]
            targetDistance = result

    return target

def findDestinationNodeForAgent9(source, actualDestination):
    result = []
    target = []
    targetDistance = 10000
    global TargetDetectedInNeighborhood
    
    TargetDetectedInNeighborhood = False
    unblockedNeighbors = Find_Unblocked_Neighbors_In_8_Directions(source)
    if len(unblockedNeighbors) == 0:
        target = source

    # Sense partial information - if target is in its 8 neighbors
    for neighbor in unblockedNeighbors:
        if neighbor == actualDestination:
            TargetDetectedInNeighborhood = True

    if TargetDetectedInNeighborhood:
        randomTarget = random.randint(0, len(unblockedNeighbors))
        target = unblockedNeighbors[randomTarget]
    else:
        neighbors = Find_Neighbors_of_Neighbors_In_8_Directions(source, unblockedNeighbors)
        if len(neighbors) == 0:
            target = source
        else:
            randomTarget = random.randint(0, len(neighbors))
            target = neighbors[randomTarget]

    return target

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent6(maze, source, actualDestination, heuristic):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid, solvable, processedCells, totalPlanningTime, movements, examinations
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions, indicating the status as unknown.
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    # consider start node as (0, 0), make it unblocked
    #discoveredGrid[0][0] = 0
    #discoveredGrid[rowCount-1][columnCount-1] = 0
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    #discoveredGridWithRestBlockages[0][0] = 0
    #discoveredGridWithRestBlockages[rowCount-1][columnCount-1] = 0

    movements = 0
    examinations = 0

    initialProbOfNodeContainingTarget = 1/(rowCount * columnCount)
    initialProbOfFindingTargetInNode = 0.35 * initialProbOfNodeContainingTarget
    ProbOfContainingTarget_Grid = [[initialProbOfNodeContainingTarget for i in range(columnCount)] for j in range(rowCount)]
    ProbOfFindingTarget_Grid = [[initialProbOfFindingTargetInNode for i in range(columnCount)] for j in range(rowCount)]
    destination = [0, 0]
    destination[0] = random.randint(0, rowCount)
    destination[1] = random.randint(0, columnCount)
    if destination[0] == source[0]:
        while destination[1] == source[1]:
            destination[1] = random.randint(0, columnCount)

    processedCells = 0
    totalPlanningTime = 0
    pathFound = False
    resultPath = []

    while pathFound == False:
        #If path from source to destination is not found, then call AStar again to discover the nodes.
        path = AStarSearch(discoveredGrid, source, destination, heuristic)

        if solvable == False:
            # Update probability of destination cell to 0. As it is unreachable.
            updateProbabilitiesForAgent6(1, destination)
            # Same source, update destination and call A star again for path
            destination = findDestinationNodeForAgent6(source)
            continue

        for index, node in enumerate(path):
            currentNode = Node(None, tuple(node))

            resultPath.append(node)

            if currentNode.position == tuple(destination):
                # Examine here, if target not found continue loop
                examinations += 1

                valueOfNodeInGivenPath = maze[path[index][0]][path[index][1]]
                discoveredGrid[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                discoveredGridWithRestBlockages[path[index][0]][path[index][1]] = valueOfNodeInGivenPath

                if destination == actualDestination:
                    
                    if discoveredGrid[path[index][0]][path[index][1]] == 2:
                        FNR = 0.2
                    # hilly terrain
                    elif discoveredGrid[path[index][0]][path[index][1]] == 3:
                        FNR = 0.5
                    #thick forests
                    elif discoveredGrid[path[index][0]][path[index][1]] == 4:
                        FNR = 0.8

                    randomnumber = random.randint(1, 11)
                    if randomnumber/10 > FNR:
                        # Target found
                        for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
                            ProbOfContainingTarget_Grid[rowIndex][columnIndex] = 0
                            ProbOfFindingTarget_Grid[rowIndex][columnIndex] = 0
                        ProbOfContainingTarget_Grid[path[index][0]][path[index][1]] = 1
                        ProbOfFindingTarget_Grid[path[index][0]][path[index][1]] = 1
                        # Target found - return from here
                        pathFound = True
                        break
                    else:
                        # Examination failed
                        updateProbabilitiesForAgent6(discoveredGrid[path[index][0]][path[index][1]], path[index], True)
                        source = path[index]
                        destination = findDestinationNodeForAgent6(source)
                        break
                else:
                    # Examination failed
                    updateProbabilitiesForAgent6(discoveredGrid[path[index][0]][path[index][1]], path[index], True)
                    source = path[index]
                    destination = findDestinationNodeForAgent6(source)
                    break

            # Select new target node
            if discoveredGrid[path[index][0]][path[index][1]] == -1:
                # If agent is bind folded, it has FOV in only direction of motion
                valueOfNodeInGivenPath = maze[path[index][0]][path[index][1]]
                discoveredGrid[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                discoveredGridWithRestBlockages[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                updateProbabilitiesForAgent6(discoveredGrid[path[index][0]][path[index][1]], path[index])

            # Plan to move here
            movements += 1

            # if next cell in path is blocked , call A star with updated start and destination node.
            if maze[path[index + 1][0]][path[index + 1][1]] == 1:
                # update probabilities whwn next cell is bloceked
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = 1
                discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = 1
                updateProbabilitiesForAgent6(1, path[index + 1])
                source = path[index]
                # change target here for agent 5, 6 with highest prob. and probability of finding the target
                # Select new target node
                destination = findDestinationNodeForAgent6(source)
                break

    return resultPath

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent7(maze, source, actualDestination, heuristic):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid, solvable, processedCells, totalPlanningTime, movements, examinations
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions, indicating the status as unknown.
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    # consider start node as (0, 0), make it unblocked
    #discoveredGrid[0][0] = 0
    #discoveredGrid[rowCount-1][columnCount-1] = 0
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    #discoveredGridWithRestBlockages[0][0] = 0
    #discoveredGridWithRestBlockages[rowCount-1][columnCount-1] = 0

    movements = 0
    examinations = 0

    initialProbOfNodeContainingTarget = 1/(rowCount * columnCount)
    initialProbOfFindingTargetInNode = 0.35 * initialProbOfNodeContainingTarget
    ProbOfContainingTarget_Grid = [[initialProbOfNodeContainingTarget for i in range(columnCount)] for j in range(rowCount)]
    ProbOfFindingTarget_Grid = [[initialProbOfFindingTargetInNode for i in range(columnCount)] for j in range(rowCount)]
    destination = [0, 0]
    destination[0] = random.randint(0, rowCount)
    destination[1] = random.randint(0, columnCount)
    if destination[0] == source[0]:
        while destination[1] == source[1]:
            destination[1] = random.randint(0, columnCount)

    processedCells = 0
    totalPlanningTime = 0
    pathFound = False
    resultPath = []

    while pathFound == False:
        #If path from source to destination is not found, then call AStar again to discover the nodes.
        path = AStarSearch(discoveredGrid, source, destination, heuristic)

        if solvable == False:
            # Update probability of destination cell to 0. As it is unreachable.
            updateProbabilitiesForAgent7(1, destination)
            # Same source, update destination and call A star again for path
            destination = findDestinationNodeForAgent7(source)
            continue

        for index, node in enumerate(path):

            #currentMaxProb = np.amax(arr)

            currentNode = Node(None, tuple(node))

            resultPath.append(node)

            if currentNode.position == tuple(destination):
                # Examine here, if target not found continue loop
                examinations += 1

                valueOfNodeInGivenPath = maze[path[index][0]][path[index][1]]
                discoveredGrid[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                discoveredGridWithRestBlockages[path[index][0]][path[index][1]] = valueOfNodeInGivenPath

                if destination == actualDestination:
                    
                    if discoveredGrid[path[index][0]][path[index][1]] == 2:
                        FNR = 0.2
                    # hilly terrain
                    elif discoveredGrid[path[index][0]][path[index][1]] == 3:
                        FNR = 0.5
                    #thick forests
                    elif discoveredGrid[path[index][0]][path[index][1]] == 4:
                        FNR = 0.8

                    randomnumber = random.randint(1, 11)
                    if randomnumber/10 > FNR:
                        # Target found
                        for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
                            ProbOfContainingTarget_Grid[rowIndex][columnIndex] = 0
                            ProbOfFindingTarget_Grid[rowIndex][columnIndex] = 0
                        ProbOfContainingTarget_Grid[path[index][0]][path[index][1]] = 1
                        ProbOfFindingTarget_Grid[path[index][0]][path[index][1]] = 1
                        # Target found - return from here
                        pathFound = True
                        break
                    else:
                        # Examination failed
                        updateProbabilitiesForAgent7(discoveredGrid[path[index][0]][path[index][1]], path[index], True)
                        source = path[index]
                        destination = findDestinationNodeForAgent7(source)
                        break
                else:
                    # Examination failed
                    updateProbabilitiesForAgent7(discoveredGrid[path[index][0]][path[index][1]], path[index], True)
                    source = path[index]
                    destination = findDestinationNodeForAgent7(source)
                    break

            # Select new target node
            if discoveredGrid[path[index][0]][path[index][1]] == -1:
                # If agent is bind folded, it has FOV in only direction of motion
                valueOfNodeInGivenPath = maze[path[index][0]][path[index][1]]
                discoveredGrid[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                discoveredGridWithRestBlockages[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                updateProbabilitiesForAgent7(discoveredGrid[path[index][0]][path[index][1]], path[index])

            # Plan to move here
            movements += 1

             # if next cell in path is blocked , call A star with updated start and destination node.
            if maze[path[index + 1][0]][path[index + 1][1]] == 1:
                # update probabilities whwn next cell is bloceked
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = 1
                discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = 1
                updateProbabilitiesForAgent7(1, path[index + 1])
                source = path[index]
                # change target here for agent 5, 6 with highest prob. and probability of finding the target
                # Select new target node
                destination = findDestinationNodeForAgent7(source)
                break

    return resultPath

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent8(maze, source, actualDestination, heuristic):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid, solvable, processedCells, totalPlanningTime, movements, examinations
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions, indicating the status as unknown.
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    # consider start node as (0, 0), make it unblocked
    #discoveredGrid[0][0] = 0
    #discoveredGrid[rowCount-1][columnCount-1] = 0
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    #discoveredGridWithRestBlockages[0][0] = 0
    #discoveredGridWithRestBlockages[rowCount-1][columnCount-1] = 0

    movements = 0
    examinations = 0

    initialProbOfNodeContainingTarget = 1/(rowCount * columnCount)
    initialProbOfFindingTargetInNode = 0.35 * initialProbOfNodeContainingTarget
    ProbOfContainingTarget_Grid = [[initialProbOfNodeContainingTarget for i in range(columnCount)] for j in range(rowCount)]
    ProbOfFindingTarget_Grid = [[initialProbOfFindingTargetInNode for i in range(columnCount)] for j in range(rowCount)]
    destination = [0, 0]
    destination[0] = random.randint(0, rowCount)
    destination[1] = random.randint(0, columnCount)
    if destination[0] == source[0]:
        while destination[1] == source[1]:
            destination[1] = random.randint(0, columnCount)

    processedCells = 0
    totalPlanningTime = 0
    pathFound = False
    resultPath = []

    while pathFound == False:
        #If path from source to destination is not found, then call AStar again to discover the nodes.
        path = AStarSearch(discoveredGrid, source, destination, heuristic)

        if solvable == False:
            # Update probability of destination cell to 0. As it is unreachable.
            updateProbabilitiesForAgent7(1, destination)
            # Same source, update destination and call A star again for path
            destination = findDestinationNodeForAgent8(source)
            continue

        for index, node in enumerate(path):
            #currentMaxProb = np.amax(arr)
            currentNode = Node(None, tuple(node))

            resultPath.append(node)

            if currentNode.position == tuple(destination):
                # Examine here, if target not found continue loop
                examinations += 1

                valueOfNodeInGivenPath = maze[path[index][0]][path[index][1]]
                discoveredGrid[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                discoveredGridWithRestBlockages[path[index][0]][path[index][1]] = valueOfNodeInGivenPath

                if destination == actualDestination:
                    
                    if discoveredGrid[path[index][0]][path[index][1]] == 2:
                        FNR = 0.2
                    # hilly terrain
                    elif discoveredGrid[path[index][0]][path[index][1]] == 3:
                        FNR = 0.5
                    #thick forests
                    elif discoveredGrid[path[index][0]][path[index][1]] == 4:
                        FNR = 0.8

                    randomnumber = random.randint(1, 11)
                    if randomnumber/10 > FNR:
                        # Target found
                        for rowIndex,columnIndex in np.ndindex(np.shape(ProbOfContainingTarget_Grid)):
                            ProbOfContainingTarget_Grid[rowIndex][columnIndex] = 0
                            ProbOfFindingTarget_Grid[rowIndex][columnIndex] = 0
                        ProbOfContainingTarget_Grid[path[index][0]][path[index][1]] = 1
                        ProbOfFindingTarget_Grid[path[index][0]][path[index][1]] = 1
                        # Target found - return from here
                        pathFound = True
                        break
                    else:
                        # Examination failed
                        updateProbabilitiesForAgent7(discoveredGrid[path[index][0]][path[index][1]], path[index], True)
                        source = path[index]
                        destination = findDestinationNodeForAgent8(source)
                        break
                else:
                    # Examination failed
                    updateProbabilitiesForAgent7(discoveredGrid[path[index][0]][path[index][1]], path[index], True)
                    source = path[index]
                    destination = findDestinationNodeForAgent8(source)
                    break

            # Select new target node
            if discoveredGrid[path[index][0]][path[index][1]] == -1:
                # If agent is bind folded, it has FOV in only direction of motion
                valueOfNodeInGivenPath = maze[path[index][0]][path[index][1]]
                discoveredGrid[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                discoveredGridWithRestBlockages[path[index][0]][path[index][1]] = valueOfNodeInGivenPath
                updateProbabilitiesForAgent7(discoveredGrid[path[index][0]][path[index][1]], path[index])

            # Plan to move here
            movements += 1

             # if next cell in path is blocked , call A star with updated start and destination node.
            if maze[path[index + 1][0]][path[index + 1][1]] == 1:
                # update probabilities whwn next cell is bloceked
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = 1
                discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = 1
                updateProbabilitiesForAgent7(1, path[index + 1])
                source = path[index]
                # change target here for agent 5, 6 with highest prob. and probability of finding the target
                # Select new target node
                destination = findDestinationNodeForAgent8(source)
                break

    return resultPath

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent9(maze, source, actualDestination, heuristic):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, ProbOfContainingTarget_Grid, ProbOfFindingTarget_Grid, solvable, processedCells, totalPlanningTime, movements, examinations
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions, indicating the status as unknown.
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    movements = 0
    examinations = 0
    initialProbOfNodeContainingTarget = 1/(rowCount * columnCount)
    ProbOfContainingTarget_Grid = [[initialProbOfNodeContainingTarget for i in range(columnCount)] for j in range(rowCount)]
    destination =  findDestinationNodeForAgent9(source, actualDestination)
    processedCells = 0
    totalPlanningTime = 0
    pathFound = False
    resultPath = []

    while pathFound == False:
        #If path from source to destination is not found, then call AStar again to discover the nodes.
        path = AStarSearchForAgent9(discoveredGrid, source, destination, heuristic)

        if solvable == False:
            discoveredGrid
            # Update probability of destination cell to 0. As it is unreachable.
            discoveredGrid[destination[0]][destination[1]] = 1
            # Same source, update destination and call A star again for path
            destination = findDestinationNodeForAgent9(currentNode.position, actualDestination)
            continue

        for index, node in enumerate(path):
            #currentMaxProb = np.amax(arr)
            currentNode = Node(None, tuple(node))

            resultPath.append(node)

            if TargetDetectedInNeighborhood:
                examinations += 1
                if currentNode.position == tuple(actualDestination):
                    # Target found - return from here
                    pathFound = True
                    break


            # Select new target node
            if discoveredGrid[path[index][0]][path[index][1]] == -1:
                # If agent is bind folded, it has FOV in only direction of motion
                discoveredGrid[path[index][0]][path[index][1]] = maze[path[index][0]][path[index][1]]

            # Plan to move here
            movements += 1

            # Update target movement here 
            unblockedNeighborNodes = Find_Unblocked_Neighbors_In_8_Directions(actualDestination)
            randomNeightborNode = random.randint(0, len(unblockedNeighborNodes))
            actualDestination = unblockedNeighborNodes[randomNeightborNode]

            # Find destination node
            destination = findDestinationNodeForAgent9(currentNode.position, actualDestination)

            # if next cell in path is blocked , call A star with updated start and destination node.
            if index + 1 < len(path) and maze[path[index + 1][0]][path[index + 1][1]] == 1:
                # update probabilities whwn next cell is bloceked
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = 1
                source = path[index]
                # change target here for agent 5, 6 with highest prob. and probability of finding the target
                # Select new target node
                destination = findDestinationNodeForAgent9(currentNode.position, actualDestination)
                break

    return resultPath

def PlotPerformanceComparisionForAgents_6_7():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, maze
    dim = 10
    iterations = 3
    p = 0.3
    # Manhattan heauristic - Best heauristic from question 5
    heuristic = 0

    averageMovementsForAgent6 = 0
    averageMovementsForAgent7 = 0

    averageExaminationsForAgent6 = 0
    averageExaminationsForAgent7 = 0

    averageActionsForAgent6 = 0
    averageActionsForAgent7 = 0

    averageTimeTakenForAgent6 = 0
    averageTimeTakenForAgent7 = 0

    movementsForAgent6 = []
    movementsForAgent7 = []

    examinationsForAgent6 = []
    examinationsForAgent7 = []

    actionsForAgent6 = []
    actionsForAgent7 = []

    timeTakenForAgent6 = []
    timeTakenForAgent7 = []

    for y in range(iterations):
        # returns solvable grid
        maze = RandomGridGenerator(p, dim)
            
        path = RepeatedAStarSearchWithAgent6(maze, initialSource, actualDestination, heuristic)
        movementsForAgent6.append(movements)
        examinationsForAgent6.append(examinations)
        actionsForAgent6.append(movements + examinations)
        timeTakenForAgent6.append(totalPlanningTime)
            
        path = RepeatedAStarSearchWithAgent7(maze, initialSource, actualDestination, heuristic)
        movementsForAgent7.append(movements)
        examinationsForAgent7.append(examinations)
        actionsForAgent7.append(movements + examinations)
        timeTakenForAgent7.append(totalPlanningTime)

    averageExaminationsForAgent6 = sum(examinationsForAgent6)/len(examinationsForAgent6)
    averageExaminationsForAgent7 = sum(examinationsForAgent7)/len(examinationsForAgent7)

    averageActionsForAgent6 = sum(actionsForAgent6)/len(actionsForAgent6)
    averageActionsForAgent7 = sum(actionsForAgent7)/len(actionsForAgent7)

    averageTimeTakenForAgent6 = sum(timeTakenForAgent6)/len(timeTakenForAgent6)
    averageTimeTakenForAgent7 = sum(timeTakenForAgent7)/len(timeTakenForAgent7)

    averageMovementsForAgent6 = sum(movementsForAgent6)/len(movementsForAgent6)
    averageMovementsForAgent7 = sum(movementsForAgent7)/len(movementsForAgent7)
    
    fig,ax=plt.subplots()
    ax.set_title('Movements comparision for Agent 6 and 7')
    ax.set_ylabel("Movements")
    ax.bar(0, averageMovementsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageMovementsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_ylabel("Examinations")
    ax.set_title('Examinations comparision for Agent 6 and 7')
    ax.bar(0.5, averageExaminationsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.75, averageExaminationsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_ylabel("Actions")
    ax.set_title('Total Actions comparision for Agent 6 and 7')
    ax.bar(0, height = averageActionsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, height = averageActionsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_title('Time comparision for Agent 6 and 7')
    ax.set_ylabel("Total planning time")
    ax.bar(0, averageTimeTakenForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageTimeTakenForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

def PlotPerformanceComparisionForAgents_6_7_8():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages
    dim = 20
    iterations = 10
    p = 0.3
    # Manhattan heauristic - Best heauristic from question 5
    heuristic = 0

    averageMovementsForAgent6 = 0
    averageMovementsForAgent7 = 0
    averageMovementsForAgent8 = 0

    averageExaminationsForAgent6 = 0
    averageExaminationsForAgent7 = 0
    averageExaminationsForAgent8 = 0

    averageActionsForAgent6 = 0
    averageActionsForAgent7 = 0
    averageActionsForAgent8 = 0

    averageTimeTakenForAgent6 = 0
    averageTimeTakenForAgent7 = 0
    averageTimeTakenForAgent8 = 0

    movementsForAgent6 = []
    movementsForAgent7 = []
    movementsForAgent8 = []

    examinationsForAgent6 = []
    examinationsForAgent7 = []
    examinationsForAgent8 = []

    actionsForAgent6 = []
    actionsForAgent7 = []
    actionsForAgent8 = []

    timeTakenForAgent6 = []
    timeTakenForAgent7 = []
    timeTakenForAgent8 = []

    for y in range(iterations):
        # returns solvable grid
        maze = RandomGridGenerator(p, dim)
            
        path = RepeatedAStarSearchWithAgent6(maze, initialSource, actualDestination, heuristic)
        movementsForAgent6.append(movements)
        examinationsForAgent6.append(examinations)
        actionsForAgent6.append(movements + examinations)
        timeTakenForAgent6.append(totalPlanningTime)
            
        path = RepeatedAStarSearchWithAgent7(maze, initialSource, actualDestination, heuristic)
        movementsForAgent7.append(movements)
        examinationsForAgent7.append(examinations)
        actionsForAgent7.append(movements + examinations)
        timeTakenForAgent7.append(totalPlanningTime)

        path = RepeatedAStarSearchWithAgent8(maze, initialSource, actualDestination, heuristic)
        movementsForAgent8.append(movements)
        examinationsForAgent8.append(examinations)
        actionsForAgent8.append(movements + examinations)
        timeTakenForAgent8.append(totalPlanningTime)

    averageExaminationsForAgent6 = sum(examinationsForAgent6)/len(examinationsForAgent6)
    averageExaminationsForAgent7 = sum(examinationsForAgent7)/len(examinationsForAgent7)
    averageExaminationsForAgent8 = sum(examinationsForAgent8)/len(examinationsForAgent8)

    averageActionsForAgent6 = sum(actionsForAgent6)/len(actionsForAgent6)
    averageActionsForAgent7 = sum(actionsForAgent7)/len(actionsForAgent7)
    averageActionsForAgent8 = sum(actionsForAgent8)/len(actionsForAgent8)

    averageTimeTakenForAgent6 = sum(timeTakenForAgent6)/len(timeTakenForAgent6)
    averageTimeTakenForAgent7 = sum(timeTakenForAgent7)/len(timeTakenForAgent7)
    averageTimeTakenForAgent8 = sum(timeTakenForAgent8)/len(timeTakenForAgent8)

    averageMovementsForAgent6 = sum(movementsForAgent6)/len(movementsForAgent6)
    averageMovementsForAgent7 = sum(movementsForAgent7)/len(movementsForAgent7)
    averageMovementsForAgent8 = sum(movementsForAgent8)/len(movementsForAgent8)
    
    fig,ax=plt.subplots()
    ax.set_title('Movements comparision for Agents 6, 7, 8')
    ax.set_ylabel("Movements")
    ax.bar(0, averageMovementsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageMovementsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageMovementsForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_ylabel("Examinations")
    ax.set_title('Examinations comparision for Agents 6, 7, 8')
    ax.bar(0, averageExaminationsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageExaminationsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageExaminationsForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()
    averageActionsForAgent6 =  averageMovementsForAgent6 + averageExaminationsForAgent6
    averageActionsForAgent7 =  averageMovementsForAgent7 + averageExaminationsForAgent7
    averageActionsForAgent8 =  averageMovementsForAgent8 + averageExaminationsForAgent8
    fig,ax=plt.subplots()
    ax.set_ylabel("Actions")
    ax.set_title('Total Actions comparision for Agents 6, 7, 8')
    ax.bar(0, height = averageActionsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, height = averageActionsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageActionsForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_title('Time comparision for Agents 6, 7, 8')
    ax.set_ylabel("Total planning time")
    ax.bar(0, averageTimeTakenForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageTimeTakenForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageTimeTakenForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

def PlotPerformanceComparisionForAgents_6_7_8_9():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, maze
    dim = 5
    iterations = 1
    p = 0.3
    # Manhattan heauristic - Best heauristic from question 5
    heuristic = 0

    averageMovementsForAgent6 = 0
    averageMovementsForAgent7 = 0
    averageMovementsForAgent8 = 0
    averageMovementsForAgent9 = 0

    averageExaminationsForAgent6 = 0
    averageExaminationsForAgent7 = 0
    averageExaminationsForAgent8 = 0
    averageExaminationsForAgent9 = 0

    averageActionsForAgent6 = 0
    averageActionsForAgent7 = 0
    averageActionsForAgent8 = 0
    averageActionsForAgent9 = 0

    averageTimeTakenForAgent6 = 0
    averageTimeTakenForAgent7 = 0
    averageTimeTakenForAgent8 = 0
    averageTimeTakenForAgent9 = 0

    movementsForAgent6 = []
    movementsForAgent7 = []
    movementsForAgent8 = []
    movementsForAgent9 = []

    examinationsForAgent6 = []
    examinationsForAgent7 = []
    examinationsForAgent8 = []
    examinationsForAgent9 = []

    actionsForAgent6 = []
    actionsForAgent7 = []
    actionsForAgent8 = []
    actionsForAgent9 = []

    timeTakenForAgent6 = []
    timeTakenForAgent7 = []
    timeTakenForAgent8 = []
    timeTakenForAgent9 = []

    for y in range(iterations):
        # returns solvable grid
        maze = RandomGridGenerator(p, dim)
            
        path = RepeatedAStarSearchWithAgent6(maze, initialSource, actualDestination, heuristic)
        movementsForAgent6.append(movements)
        examinationsForAgent6.append(examinations)
        actionsForAgent6.append(movements + examinations)
        timeTakenForAgent6.append(totalPlanningTime)
            
        path = RepeatedAStarSearchWithAgent7(maze, initialSource, actualDestination, heuristic)
        movementsForAgent7.append(movements)
        examinationsForAgent7.append(examinations)
        actionsForAgent7.append(movements + examinations)
        timeTakenForAgent7.append(totalPlanningTime)

        path = RepeatedAStarSearchWithAgent8(maze, initialSource, actualDestination, heuristic)
        movementsForAgent8.append(movements)
        examinationsForAgent8.append(examinations)
        actionsForAgent8.append(movements + examinations)
        timeTakenForAgent8.append(totalPlanningTime)

        path = RepeatedAStarSearchWithAgent9(maze, initialSource, actualDestination, heuristic)
        movementsForAgent9.append(movements)
        examinationsForAgent9.append(examinations)
        actionsForAgent9.append(movements + examinations)
        timeTakenForAgent9.append(totalPlanningTime)

    averageExaminationsForAgent6 = sum(examinationsForAgent6)/len(examinationsForAgent6)
    averageExaminationsForAgent7 = sum(examinationsForAgent7)/len(examinationsForAgent7)
    averageExaminationsForAgent8 = sum(examinationsForAgent8)/len(examinationsForAgent8)
    averageExaminationsForAgent9 = sum(examinationsForAgent9)/len(examinationsForAgent9)

    averageActionsForAgent6 = sum(actionsForAgent6)/len(actionsForAgent6)
    averageActionsForAgent7 = sum(actionsForAgent7)/len(actionsForAgent7)
    averageActionsForAgent8 = sum(actionsForAgent8)/len(actionsForAgent8)
    averageActionsForAgent9 = sum(actionsForAgent9)/len(actionsForAgent9)

    averageTimeTakenForAgent6 = sum(timeTakenForAgent6)/len(timeTakenForAgent6)
    averageTimeTakenForAgent7 = sum(timeTakenForAgent7)/len(timeTakenForAgent7)
    averageTimeTakenForAgent8 = sum(timeTakenForAgent8)/len(timeTakenForAgent8)
    averageTimeTakenForAgent9 = sum(timeTakenForAgent9)/len(timeTakenForAgent9)

    averageMovementsForAgent6 = sum(movementsForAgent6)/len(movementsForAgent6)
    averageMovementsForAgent7 = sum(movementsForAgent7)/len(movementsForAgent7)
    averageMovementsForAgent8 = sum(movementsForAgent8)/len(movementsForAgent8)
    averageMovementsForAgent9 = sum(movementsForAgent8)/len(movementsForAgent9)
    
    fig,ax=plt.subplots()
    ax.set_title('Movements comparision for Agents 6, 7, 8, 9')
    ax.set_ylabel("Movements")
    ax.bar(0, averageMovementsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageMovementsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageMovementsForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.bar(0.75, averageMovementsForAgent9, color="red", width = 0.25, label= "agent 9")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_ylabel("Examinations")
    ax.set_title('Examinations comparision for Agents 6, 7, 8, 9')
    ax.bar(0, averageExaminationsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageExaminationsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageExaminationsForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.bar(0.75, averageExaminationsForAgent9, color="red", width = 0.25, label= "agent 9")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_ylabel("Actions")
    ax.set_title('Total Actions comparision for Agents 6, 7, 8, 9')
    ax.bar(0, averageActionsForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageActionsForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageActionsForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.bar(0.75, averageActionsForAgent9, color="red", width = 0.25, label= "agent 9")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_title('Time comparision for Agent 6 and 7')
    ax.set_ylabel("Total planning time")
    ax.bar(0, averageTimeTakenForAgent6, color="blue", width = 0.25, label= "agent 6")
    ax.bar(0.25, averageTimeTakenForAgent7, color="orange", width = 0.25, label= "agent 7")
    ax.bar(0.5, averageTimeTakenForAgent8, color="green", width = 0.25, label= "agent 8")
    ax.bar(0.75, averageTimeTakenForAgent9, color="red", width = 0.25, label= "agent 9")
    ax.tick_params(labelbottom=False)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    processedCells = 0
    totalPlanningTime = 0
    maze = []
    # use heuristics as below
    # 0 - manhattan
    # 1 - Euclidian
    # 2 - Chebyshev
    heuristic = 0

    solvable = True

    # We have 4 directions to explore neighbours for each node( Right, Down, Up, Left - N,S,E,W)
    neighbours_in_4_directions = [[0, 1],  # towards right
                                 [1, 0],  # towards down
                                 [-1, 0],  # towards up
                                 [0, -1]]  # towards left

    #In project 2, We have 8 directions to explore neighbours for each node( N,S,E,W,NW,NE,SW,SE)
    neighbours_in_8_directions = [[0, 1], # towards East
                                 [1, 0],  # towards south
                                 [-1, 0], # towards north
                                 [0, -1], # towards west
                                 [1, 1],  # towards SE
                                 [-1, 1], # towards NE
                                 [-1, -1],# towards NW
                                 [1, -1]] # towards SW

    rowCount = 0
    columnCount = 0
    initialSource = []
    actualDestination = []
    discoveredGrid = []
    discoveredGridWithRestBlockages = []
    movements = 0
    examinations = 0
    TargetDetectedInNeighborhood = False

    #PlotPerformanceComparisionForAgents_6_7()
    PlotPerformanceComparisionForAgents_6_7_8()
    #PlotPerformanceComparisionForAgents_6_7_8_9()