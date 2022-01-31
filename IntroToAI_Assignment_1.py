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

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearch(maze, source, destination, heuristic, blindFolded = False):
    global rowCount, columnCount, discoveredGrid
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at each position
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[0 for i in range(columnCount)] for j in range(rowCount)]
    pathFound = False
    destinationNode = Node(None, tuple(destination))
    resultPath = []
    global solvable

    while pathFound == False:
        #If path from source to destination is not found, then call AStar again to discover the nodes.
        path = AStarSearch(discoveredGrid, source, destination, heuristic)

        if solvable == False:
            return None

        for index, node in enumerate(path):
            currentNode = Node(None, tuple(node))

            if currentNode.position == destinationNode.position:
                pathFound = True
                resultPath.append(node)
                break

            # If agent is bind folded, it has FOV in only direction of motion
            if blindFolded:
                valueOfNextNodeInGivenPath = maze[path[index + 1][0]][path[index + 1][1]]
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
                discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            else:
                # If agent is not bind folded, it has FOV in 4 directions of motion
                for newPosition in neighbours:
                    # Get the position of the node in maze
                    nodeLocation = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])

                    #Check if the node position is within the maze dimensions
                    if (nodeLocation[0] > (rowCount - 1) or
                            nodeLocation[0] < 0 or
                            nodeLocation[1] > (columnCount - 1) or
                            nodeLocation[1] < 0):
                        continue

                    # Check here if node is blocked or not.
                    # Update values of nodes in the directions of field of view(North, South, East, West).
                    discoveredGrid[nodeLocation[0]][nodeLocation[1]] = maze[nodeLocation[0]][nodeLocation[1]]
                    # Update values of nodes in the directions of field of view(North, south,east west) for discoveredGridWithRestBlockages
                    discoveredGridWithRestBlockages[nodeLocation[0]][nodeLocation[1]] = maze[nodeLocation[0]][
                        nodeLocation[1]]

            # if it is blocked node, call A star with updated start node.
            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] != 0:
                source = path[index]
                break

            resultPath.append(node)

    return resultPath

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent3(maze, source, destination, heuristic, blindFolded = False):
    global rowCount, columnCount, discoveredGrid
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at each position
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[0 for i in range(columnCount)] for j in range(rowCount)]
    pathFound = False
    destinationNode = Node(None, tuple(destination))
    resultPath = []
    global solvable

    while pathFound == False:
        #If path from source to destination is not found, then call AStar again to discover the nodes.
        path = AStarSearch(discoveredGrid, source, destination, heuristic)

        if solvable == False:
            return None

        for index, node in enumerate(path):
            currentNode = Node(None, tuple(node))

            if currentNode.position == destinationNode.position:
                pathFound = True
                resultPath.append(node)
                break

            # If agent is bind folded, it has FOV in only direction of motion
            if blindFolded:
                valueOfNextNodeInGivenPath = maze[path[index + 1][0]][path[index + 1][1]]
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
                discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            else:
                # If agent is not bind folded, it has FOV in 4 directions of motion
                for newPosition in neighbours:
                    # Get the position of the node in maze
                    nodeLocation = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])

                    #Check if the node position is within the maze dimensions
                    if (nodeLocation[0] > (rowCount - 1) or
                            nodeLocation[0] < 0 or
                            nodeLocation[1] > (columnCount - 1) or
                            nodeLocation[1] < 0):
                        continue

                    # Check here if node is blocked or not.
                    # Update values of nodes in the directions of field of view(North, South, East, West).
                    discoveredGrid[nodeLocation[0]][nodeLocation[1]] = maze[nodeLocation[0]][nodeLocation[1]]
                    # Update values of nodes in the directions of field of view(North, south,east west) for discoveredGridWithRestBlockages
                    discoveredGridWithRestBlockages[nodeLocation[0]][nodeLocation[1]] = maze[nodeLocation[0]][
                        nodeLocation[1]]

            # if it is blocked node, call A star with updated start node.
            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] != 0:
                source = path[index]
                break

            resultPath.append(node)

    return resultPath

# AStarSearch perform A* search operation.
# With input as Grid, source node, destination node and heuristic to be used to calculate cost of node.
def AStarSearch(Grid, source, destination, heuristic):

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

    global solvable
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
            return returnPath(presentNode, Grid)

        global processedCells
        # Increased processed cell count here to keep track of number of cells processed.
        processedCells += 1

        # Generate children from all adjacent nodes in four directions(North, South, Est, West)
        children = []

        for newPosition in neighbours:
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
            if Grid[nodeLocation[0]][nodeLocation[1]] != 0:
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

            # Store code of each explored node here
            nodeCost[child.position] = child.g

            # Add the child to the fringe
            fringe.push(child, child.f)

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

# This function generates random mazes with probability and dimension as input.
# Considering start and goal nodes are unblocked.
# node value - 1 for blocked node, 0 for unblocked node.
def RandomGridGenerator(p, dim):
    grid = []
    randomNumbers = np.arange(1, ((dim * dim) - 2) + 1)
    random.shuffle(randomNumbers)
    gridNumbers = np.append((dim * dim) - 1, randomNumbers)
    gridNumbers = np.append(gridNumbers, (dim * dim))

    test = (np.where(gridNumbers <= (p * dim * dim), 1, 0))
    for i in range(dim):
        row = []
        for j in range(dim):
            row.append(test[(i * dim) + j])
        grid.append(row)
    return grid

    #This is code for question 4, to find po value.
    #Generating random maze for dimension 101
    # with probabilities ranging from 0 to 1 in interval of 0.025
def PlotDensityVsSolvabilityForDim():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells
    dim = 101
    iterations = 100
    probabilityStep = 0.01
    problist = []
    sovablelist = []
    
    for p in np.arange(0, 1, probabilityStep):
        IsSolvable = []
        for y in range(iterations):
            maze = RandomGridGenerator(p, dim)
            rowCount, columnCount = np.shape(maze)
            start = [0, 0]  # starting position
            end = [rowCount - 1, columnCount - 1]  # ending position
            discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
            discoveredGrid = []
            discoveredGridWithRestBlockages[start[0]][start[0]] = 0
            discoveredGridWithRestBlockages[end[0]][end[1]] = 0
            path = AStarSearch(maze, start, end, heuristic)
            IsSolvable.append(int(solvable))
        problist.append(p)
        sovablelist.append(sum(IsSolvable)/len(IsSolvable))
    plt.plot(problist, sovablelist)
    plt.ylabel('sovability')
    plt.xlabel('probability')
    plt.show()

    #This is code for question 5, to compare different hueristics.
    # Generating 100 random mazes for each dimension 1 to 101
    # with probabilitie as 0.24
def PlotDimensionVsTimeTakenFor3Distances():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells
    p = 0.24
    iterations = 100
    dimensions = 101 # From 1 to this number
    averageTimeTakenForManhattanHeuristic = []
    averageTimeTakenForEuclidianHeuristic = []
    averageTimeTakenForChebyshevHeuristic = []
    dimList = []
    for dim in range(1, dimensions + 1):
        timeTakenForManhattanHeuristic = []
        timeTakenForEuclidianHeuristic = []
        timeTakenForChebyshevHeuristic = []
        for y in range(iterations):
            solvableGrid = False
            while solvableGrid == False: 
                maze = RandomGridGenerator(p, dim)
                rowCount, columnCount = np.shape(maze)
                start = [0, 0]  # starting position
                end = [rowCount - 1, columnCount - 1]  # ending position
                discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
                discoveredGrid = []
                discoveredGridWithRestBlockages[start[0]][start[0]] = 0
                discoveredGridWithRestBlockages[end[0]][end[1]] = 0
                startTime = time.time()  # Start time
                pathInFullGrid = AStarSearch(maze, start, end, 0)
                endTime = time.time()  # End time
                timeForAStarWithFullGrid = endTime - startTime
                solvableGrid = solvable
                if solvableGrid:
                    timeTakenForManhattanHeuristic.append(timeForAStarWithFullGrid)
                    # calculating time taken for euclidian heuristic
                    startTime = time.time()  # Start time
                    pathInFullGrid = AStarSearch(maze, start, end, 1)
                    endTime = time.time()  # End time
                    timeForAStarWithFullGrid = endTime - startTime
                    timeTakenForEuclidianHeuristic.append(timeForAStarWithFullGrid)

                    # calculating time taken for chebyshev heuristic
                    startTime = time.time()  # Start time
                    pathInFullGrid = AStarSearch(maze, start, end, 2)
                    endTime = time.time()  # End time
                    timeForAStarWithFullGrid = endTime - startTime
                    timeTakenForChebyshevHeuristic.append(timeForAStarWithFullGrid)

        dimList.append(dim)
        averageTimeTakenForManhattanHeuristic.append(sum(timeTakenForManhattanHeuristic)/len(timeTakenForManhattanHeuristic))
        averageTimeTakenForEuclidianHeuristic.append(sum(timeTakenForEuclidianHeuristic)/len(timeTakenForEuclidianHeuristic))
        averageTimeTakenForChebyshevHeuristic.append(sum(timeTakenForChebyshevHeuristic)/len(timeTakenForChebyshevHeuristic))

    fig,ax=plt.subplots()
    ax.plot(dimList, averageTimeTakenForManhattanHeuristic, color="red", label= "Manhattan", marker="o")
    ax.set_xlabel("Dimension")
    ax.set_ylabel("Average time taken")
    ax.plot(dimList, averageTimeTakenForEuclidianHeuristic, color="blue", label= "Euclidian", marker="o")
    ax.plot(dimList, averageTimeTakenForChebyshevHeuristic, color="green", label= "chebyshev", marker="o")
    plt.legend()
    plt.show()

    # This is code for question 6 and 7 - to plot different graphs.
    # Generating random maze for dimension 101
    # with probabilities ranging from 0 to 1 in interval of 0.025
    # Additional input for question 7.
def PlotPerformanceForDensity(blindFolded = False):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells
    dim = 10
    iterations = 10
    # po value from question 4
    minimumP = 0.24
    # Manhattan heauristic - Best heauristic from question 5
    heuristic = 0 

    problist = []
    averageTrajectoryLength = []
    averageTrajectoryLengthByShortPathInFinalGrid = []
    averageShortPathInFinalGridByShortPathInFullGrid = []
    averageProcessedCells = []

    for p in np.arange(0, minimumP, 0.024):
        trajectoryLength = []
        trajectoryLengthByShortpathInFinalGrid = []
        shortPathInFinalGridByShortPathInFullGrid = []
        processedCellcount = []
        for y in range(iterations):
            solvableGrid = False
            while solvableGrid == False: 
                maze = RandomGridGenerator(p, dim)
                rowCount, columnCount = np.shape(maze)
                start = [0, 0]  # starting position
                end = [rowCount - 1, columnCount - 1]  # ending position
                discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
                discoveredGrid = []
                discoveredGridWithRestBlockages[start[0]][start[0]] = 0
                discoveredGridWithRestBlockages[end[0]][end[1]] = 0
                pathInFullGrid = AStarSearch(maze, start, end, heuristic)
                solvableGrid = solvable
                if solvableGrid:
                    processedCells = 0
                    path = RepeatedAStarSearch(maze, start, end, heuristic, blindFolded) #question 6.a
                    trajectoryLength.append(len(path)-1) #question 6.a
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic) #question 6.b
                    trajectoryLengthByShortpathInFinalGrid.append((len(path) - 1)/(len(pathInFinalDiscoveredGrid)-1)) #question 6.b
                    shortPathInFinalGridByShortPathInFullGrid.append((len(pathInFinalDiscoveredGrid)-1)/(len(pathInFullGrid)-1))
                    processedCellcount.append(processedCells)
        problist.append(p)
        #question 6.a
        averageTrajectoryLength.append(sum(trajectoryLength)/len(trajectoryLength))
        #question 6.b
        averageTrajectoryLengthByShortPathInFinalGrid.append(sum(trajectoryLengthByShortpathInFinalGrid)/len(trajectoryLengthByShortpathInFinalGrid))
        #question 6.c
        averageShortPathInFinalGridByShortPathInFullGrid.append(sum(shortPathInFinalGridByShortPathInFullGrid)/len(shortPathInFinalGridByShortPathInFullGrid))
        #question 6.d
        averageProcessedCells.append(sum(processedCellcount)/len(processedCellcount))
    #question 6.a
    plt.plot(problist, averageTrajectoryLength)
    plt.xlabel('probability')
    plt.ylabel('Average trajectory length')
    plt.show()

    #question 6.b
    plt.plot(problist, averageTrajectoryLengthByShortPathInFinalGrid) 
    plt.xlabel('probability')
    plt.ylabel('Average (Length of Trajectory / Length of Shortest Path in Final Discovered Gridworld)')
    plt.show()

    #question 6.c
    plt.plot(problist, averageShortPathInFinalGridByShortPathInFullGrid) 
    plt.xlabel('probability')
    plt.ylabel('Average (Length of Shortest Path in Final Discovered Gridworld / Length of ShortestPath in Full Gridworld)')
    plt.show()
    
    #question 6.d
    plt.plot(problist, averageProcessedCells)
    plt.xlabel('probability')
    plt.ylabel('Average processed cells')
    plt.show()

if __name__ == '__main__':
    processedCells = 0

    # use heuristics as below
    # 0 - manhattan
    # 1 - Euclidian
    # 2 - Chebyshev
    heuristic = 0

    solvable = True
    # We have 4 directions to explore neighbours for each node( Right, Down, Up, Left)
    neighbours = [[0, 1],  # towards right
                 [1, 0],  # towards down
                 [-1, 0],  # towards up
                 [0, -1]]  # towards left
 

    rowCount = 0
    columnCount = 0
    discoveredGrid = []
    discoveredGridWithRestBlockages = []

    # The below 4 funaction calls are for questions 4,5,6,7 - uncomment and run

    # PlotDensityVsSolvabilityForDim()
    # PlotDimensionVsTimeTakenFor3Distances()
    # PlotPerformanceForDensity()
    # PlotPerformanceForDensity(True)

    # The below code is for testing single case
    maze = [[0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0], 
            [0, 1, 0, 0, 1], 
            [0, 1, 0, 1, 0], 
            [0, 0, 0, 0, 0]]
    
    #maze = RandomGridGenerator(0.1, 12)


    rowCount, columnCount = np.shape(maze)
    source = [0, 0]  # starting position
    destination = [rowCount - 1, columnCount - 1]  # ending position

    # This is to store discovered nodes, and keeping rest as unblocked nodes for sending to A*
    discoveredGrid = []

    # This is to store only discovered nodes, and keeping rest as blocked nodes.
    # Used for calculating Shortest Path in Final Discovered Gridworld
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGridWithRestBlockages[source[0]][source[0]] = 0
    discoveredGridWithRestBlockages[destination[0]][destination[1]] = 0

    startTime = time.time()  # Start time
    path = RepeatedAStarSearch(maze, source, destination, heuristic)
    endTime = time.time()  # End time
    timeForRepeatedForwardAStar = endTime - startTime
    print("Elapsed time for Repeated forward A* is  {}".format(timeForRepeatedForwardAStar))

    if solvable:
        TrajectoryLength = len(path) - 1
    processedCellCountOfRepeatedAStar = processedCells

    # Length of Shortest Path in Final Discovered Gridworld
    startTime = time.time()  # Start time
    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, source, destination, heuristic)
    endTime = time.time()  # End time
    timeForAStarWithFinalDiscoveredGrid = endTime - startTime
    print("Elapsed time for A* with Final discoverd grid is  {}".format(timeForAStarWithFinalDiscoveredGrid))
    if solvable:
        lengthInFinalDiscoveredGrid = len(pathInFinalDiscoveredGrid) - 1

    startTime = time.time()  # Start time
    pathInFullGrid = AStarSearch(maze, source, destination, heuristic)
    endTime = time.time()  # End time
    timeForAStarWithFullGrid = endTime - startTime
    print("Elapsed time for A* with Full discoverd grid is  {}".format(timeForAStarWithFullGrid))
    if solvable:
        lengthInFullGrid = len(pathInFullGrid) - 1

    print(path)
