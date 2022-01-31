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
    global solvable, totalPlanningTime
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

        global processedCells
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

def FindNeighborsIn_8_Directions(currentNode):
    global rowCount, columnCount
    neighbourNodes = []
    for newPosition in neighbours_in_8_directions:
    # Get the position of the node in maze
        nodeLocation = (currentNode[0] + newPosition[0], currentNode[1] + newPosition[1])

        #Check if the node position is within the maze dimensions
        if (nodeLocation[0] > (rowCount - 1) or nodeLocation[0] < 0 or nodeLocation[1] > (columnCount - 1) or nodeLocation[1] < 0):
            continue
        else:
            neighbourNodes.append(nodeLocation)

    return neighbourNodes
     
# Updates values of node properties
def UpdateNodeStatus(currentNode, nodeLocation):
    if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == 0:
        currentNode.E = currentNode.E + 1  # Updating no.of unblocked neighbors from discovered grid
    if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == 1:
        currentNode.B = currentNode.B + 1  # Updating no.of blocked neighbors from discovered grid
    if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == -1:
        currentNode.H = currentNode.H + 1  # Updating no.of undiscovered neighbors from discovered grid
    return currentNode

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearch(maze, source, destination, heuristic, blindFolded = False):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells, totalPlanningTime
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions, indicating the status as unknown.
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGrid[0][0] = 0
    discoveredGrid[rowCount-1][columnCount-1] = 0
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGridWithRestBlockages[0][0] = 0
    discoveredGridWithRestBlockages[rowCount-1][columnCount-1] = 0
    processedCells = 0
    totalPlanningTime = 0
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

            resultPath.append(node)

            # If agent is bind folded, it has FOV in only direction of motion
            if blindFolded:
                valueOfNextNodeInGivenPath = maze[path[index + 1][0]][path[index + 1][1]]
                discoveredGrid[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
                discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            else:
                # If agent is not bind folded, it has FOV in 4 directions of motion
                for newPosition in neighbours_in_4_directions:
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
            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] == 1:
                source = path[index]
                break

    return resultPath

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent3(maze, source, destination, heuristic):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells, totalPlanningTime
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions,  indicating the status as unknown.
    # 1 indicated blocked, 0 indicated unblocked, -1 indicates value is unknown
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGrid[0][0] = 0
    discoveredGrid[rowCount-1][columnCount-1] = 0
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGridWithRestBlockages[0][0] = 0
    discoveredGridWithRestBlockages[rowCount-1][columnCount-1] = 0
    processedCells = 0
    totalPlanningTime = 0
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

            resultPath.append(node)
            neighbourNodes = []
            # The agent 3 can sense partial information in 8 directions of motion
            for newPosition in neighbours_in_8_directions:
                # Get the position of the node in maze
                nodeLocation = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])

                #Check if the node position is within the maze dimensions
                if (nodeLocation[0] > (rowCount - 1) or nodeLocation[0] < 0 or nodeLocation[1] > (columnCount - 1) or nodeLocation[1] < 0):
                    continue
                else:
                    neighbourNodes.append(nodeLocation)
                    currentNode.N = currentNode.N + 1
                    if maze[nodeLocation[0]][nodeLocation[1]] == 1:
                        currentNode.C = currentNode.C + 1  # Updating no.of neighbors that are sensed to be blocked
                    currentNode = UpdateNodeStatus(currentNode, nodeLocation)
                    
            # For loop ends here

            currentNode.visited = True
            currentNode.confirmed = True

            InferenceOnSingleNodeForAgent3(currentNode, neighbourNodes)

            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] == 1:
                source = path[index]
                break

            # After inference, when we want to proceed to next node in path, 
            # Update values of nodes in the directions of movement.
            valueOfNextNodeInGivenPath = maze[path[index + 1][0]][path[index + 1][1]]
            discoveredGrid[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            
            # if it is blocked node, call A star with updated start node.
            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] == 1:
                source = path[index]
                break

    return resultPath

def InferenceOnSingleNodeForAgent3(node, neighbourNodes):
    global discoveredGrid, discoveredGridWithRestBlockages
    updated = False

    # Exit inference if H = 0
    if node.H != 0:
        # Inference
        if node.C == node.B:
            # make rest as empty
            for nodeLocation in neighbourNodes:
                if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == -1:
                    updated = True
                    discoveredGrid[nodeLocation[0]][nodeLocation[1]] = 0
                    discoveredGridWithRestBlockages[nodeLocation[0]][nodeLocation[1]] = 0

        elif node.N - node.C == node.E:
            # make rest as blocked
            for nodeLocation in neighbourNodes:
                if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == -1:
                    updated = True
                    discoveredGrid[nodeLocation[0]][nodeLocation[1]] = 1
                    discoveredGridWithRestBlockages[nodeLocation[0]][nodeLocation[1]] = 1
        if updated:
            node.E = 0
            node.B = 0
            node.H = 0
            for neighbor in neighbourNodes:
                UpdateNodeStatus(node, neighbor)

    return node

# RepeatedAStarSearch calls AStarSearch function repeatedly until path to goal node is discovered.
def RepeatedAStarSearchWithAgent4(maze, source, destination, heuristic):
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells, totalPlanningTime
    # A maze named discoveredGrid is created and intialized with 0(unblocked) at start and end positions.
    # And with '-1' at rest of the positions,  indicating the status as unknown.
    # 1 indicated blocked, 0 indicated unblocked, -1 indicates value is unknown
    # Whenever a node is discovered that node position will be updated.
    discoveredGrid = [[-1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGrid[0][0] = 0
    discoveredGrid[rowCount-1][columnCount-1] = 0
    discoveredGridWithRestBlockages = [[1 for i in range(columnCount)] for j in range(rowCount)]
    discoveredGridWithRestBlockages[0][0] = 0
    discoveredGridWithRestBlockages[rowCount-1][columnCount-1] = 0
    processedCells = 0
    totalPlanningTime = 0
    pathFound = False
    destinationNode = Node(None, tuple(destination))
    resultPath = []
    global solvable
    nodeStaus = {}
    nodesWithUndeterminedNeighbours = {}

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

            resultPath.append(node)
            neighbourNodes = []
            # The agent 3 can sense partial information in 8 directions of motion
            for newPosition in neighbours_in_8_directions:
                # Get the position of the node in maze
                nodeLocation = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])

                #Check if the node position is within the maze dimensions
                if (nodeLocation[0] > (rowCount - 1) or nodeLocation[0] < 0 or nodeLocation[1] > (columnCount - 1) or nodeLocation[1] < 0):
                    continue
                else:
                    neighbourNodes.append(nodeLocation)
                    currentNode.N = currentNode.N + 1
                    if maze[nodeLocation[0]][nodeLocation[1]] == 1:
                        currentNode.C = currentNode.C + 1  # Updating no.of neighbors that are sensed to be blocked
                    currentNode = UpdateNodeStatus(currentNode, nodeLocation)
            # For loop ends here

            inferredNodes = []
            #Infer current node here
            InferenceOnSingleNodeForAgent4(currentNode, neighbourNodes, nodesWithUndeterminedNeighbours, inferredNodes)

            currentNode.visited = True
            currentNode.confirmed = True

            #Infer previous nodes with H>0 here
            for pathPosition in nodesWithUndeterminedNeighbours.keys():
                neighbourNodesofPreviousNode = set(FindNeighborsIn_8_Directions(pathPosition))
                previousNode = Node(None, tuple(pathPosition))
                previousNode.N = nodeStaus[pathPosition]['N']
                previousNode.C = nodeStaus[pathPosition]['C']
                for neighbour in neighbourNodesofPreviousNode:
                    previousNode = UpdateNodeStatus(previousNode, neighbour)
                InferenceOnSingleNodeForAgent4(previousNode, neighbourNodesofPreviousNode, nodesWithUndeterminedNeighbours, inferredNodes)
                nodeStaus[previousNode.position]['B'] = previousNode.B
                nodeStaus[previousNode.position]['E'] = previousNode.E
                nodeStaus[previousNode.position]['H'] = previousNode.H

            currentNode.E = 0
            currentNode.B = 0
            currentNode.H = 0
            for neighbour in neighbourNodes:
                currentNode = UpdateNodeStatus(currentNode, neighbour)
            InferenceOnSingleNodeForAgent4(currentNode, neighbourNodes, nodesWithUndeterminedNeighbours, inferredNodes)

            nodeStaus[currentNode.position] = {}
            nodeStaus[currentNode.position]['N'] = currentNode.N
            nodeStaus[currentNode.position]['C'] = currentNode.C
            nodeStaus[currentNode.position]['B'] = currentNode.B
            nodeStaus[currentNode.position]['E'] = currentNode.E
            nodeStaus[currentNode.position]['H'] = currentNode.H
            nodeStaus[currentNode.position]['visited'] = currentNode.visited
            nodeStaus[currentNode.position]['confirmed'] = currentNode.confirmed

            if currentNode.H > 0 :
                undeterminedNeighbours = set()
                for node in neighbourNodes:
                    if discoveredGrid[node[0]][node[1]] == -1:
                        undeterminedNeighbours.add(node)
                nodesWithUndeterminedNeighbours[currentNode.position] = undeterminedNeighbours

            for inferredNode in inferredNodes:
                del nodesWithUndeterminedNeighbours[inferredNode]
            inferredNodes = []

            InferenceRulesForAgent4(nodesWithUndeterminedNeighbours, nodeStaus)
            
            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] == 1:
                source = path[index]
                break

            # After inference, when we want to proceed to next node in path, 
            # Update values of nodes in the directions of movement.
            valueOfNextNodeInGivenPath = maze[path[index + 1][0]][path[index + 1][1]]
            discoveredGrid[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            discoveredGridWithRestBlockages[path[index + 1][0]][path[index + 1][1]] = valueOfNextNodeInGivenPath
            if currentNode.position in nodesWithUndeterminedNeighbours and path[index + 1] in nodesWithUndeterminedNeighbours[currentNode.position]:
                nodesWithUndeterminedNeighbours[currentNode.position].remove(path[index + 1])
            currentNode.E = 0
            currentNode.B = 0
            currentNode.H = 0
            for neighbour in neighbourNodes:
                currentNode = UpdateNodeStatus(currentNode, neighbour)
            InferenceOnSingleNodeForAgent4(currentNode, neighbourNodes, nodesWithUndeterminedNeighbours, inferredNodes)
            
            for inferredNode in inferredNodes:
                del nodesWithUndeterminedNeighbours[inferredNode]

            # if it is blocked node, call A star with updated start node.
            if discoveredGrid[path[index + 1][0]][path[index + 1][1]] == 1:
                source = path[index]
                break

    return resultPath

def InferenceRulesForAgent4(nodesWithUndeterminedNeighbours, nodeStaus):
    undeterminedNodes = list(nodesWithUndeterminedNeighbours.keys())
    for i in undeterminedNodes:
        removed = False
        removeNodes = set()
        for neighbor in nodesWithUndeterminedNeighbours[i]:
                if discoveredGrid[neighbor[0]][neighbor[1]] != -1:
                    removed = True
                    removeNodes.add(neighbor)
        if removed:
            nodesWithUndeterminedNeighbours[i] = nodesWithUndeterminedNeighbours[i]-(removeNodes)
        if len(nodesWithUndeterminedNeighbours[i]) == 0:
            del nodesWithUndeterminedNeighbours[i]

    undeterminedNodes = list(nodesWithUndeterminedNeighbours.keys())

    if len(undeterminedNodes) >= 2:
        latestNodePosition = undeterminedNodes[-1] 
        previousNodePosition = undeterminedNodes[-2] 
                  
        commonUndeterminedNodes = (nodesWithUndeterminedNeighbours[latestNodePosition]).intersection(nodesWithUndeterminedNeighbours[previousNodePosition])
        if len(commonUndeterminedNodes) > 0:
            updated = False
            latestNodeStaus = nodeStaus[latestNodePosition]
            undeterminedBlocksLatest = latestNodeStaus['C'] - latestNodeStaus['B']
            undeterminedCellsNotCommon = latestNodeStaus['H'] - len(commonUndeterminedNodes)
            if undeterminedBlocksLatest > undeterminedCellsNotCommon:
                # Common nodes has (above difference) number of blocks for sure
                blocksInCommon = undeterminedBlocksLatest - undeterminedCellsNotCommon
                previousNodeStaus = nodeStaus[previousNodePosition]
                undeterminedBlocksPrevious = previousNodeStaus['C'] - previousNodeStaus['B']
                if undeterminedBlocksPrevious <= blocksInCommon:
                    # Rest of undetermined nodes not in common are empty
                    removedNeighbors = []
                    for neighbour in nodesWithUndeterminedNeighbours[previousNodePosition]:
                        if neighbour not in commonUndeterminedNodes:
                            discoveredGrid[neighbour[0]][neighbour[1]] = 0
                            discoveredGridWithRestBlockages[neighbour[0]][neighbour[1]] = 0
                            removedNeighbors.append(neighbour)
                            updated = True
                    if latestNodePosition in nodesWithUndeterminedNeighbours:
                        for neighbour in removedNeighbors:
                            nodesWithUndeterminedNeighbours[previousNodePosition].remove(neighbour)
            else:
                undeterminedEmptyCellsLatest = latestNodeStaus['N'] - latestNodeStaus['C'] - latestNodeStaus['E']
                if undeterminedEmptyCellsLatest > undeterminedCellsNotCommon:
                    # Common nodes has (above difference) number of empty cells for sure
                    emptyCellsInCommon = undeterminedEmptyCellsLatest - undeterminedCellsNotCommon
                    previousNodeStaus = nodeStaus[previousNodePosition]
                    undeterminedEmptyCellsPrevious = previousNodeStaus['N'] - previousNodeStaus['C'] - previousNodeStaus['E']
                    if undeterminedEmptyCellsPrevious <= emptyCellsInCommon:
                        # Rest of undetermined nodes not in common are blocked
                        removedNeighbors = []
                        for neighbour in nodesWithUndeterminedNeighbours[previousNodePosition]:
                            if neighbour not in commonUndeterminedNodes:
                                discoveredGrid[neighbour[0]][neighbour[1]] = 1
                                discoveredGridWithRestBlockages[neighbour[0]][neighbour[1]] = 1
                                removedNeighbors.append(neighbour)
                                updated = True
                        if latestNodePosition in nodesWithUndeterminedNeighbours:
                            for neighbour in removedNeighbors:
                                nodesWithUndeterminedNeighbours[previousNodePosition].remove(neighbour)
            if updated:
                    previousNode = Node(None, tuple(previousNodePosition))
                    previousNode.N = nodeStaus[previousNodePosition]['N']
                    previousNode.C = nodeStaus[previousNodePosition]['C']
                    previousNode.E = 0
                    previousNode.B = 0
                    previousNode.H = 0
                    neighbourNodesofPreviousNode = set(FindNeighborsIn_8_Directions(previousNodePosition))
                    for neighbour in neighbourNodesofPreviousNode:
                        previousNode = UpdateNodeStatus(previousNode, neighbour)

                    inferredNodes = []   
                    InferenceOnSingleNodeForAgent4(previousNode, neighbourNodesofPreviousNode, nodesWithUndeterminedNeighbours, inferredNodes)

                    for inferredNode in inferredNodes:
                        del nodesWithUndeterminedNeighbours[inferredNode]

    return nodesWithUndeterminedNeighbours, nodeStaus

def InferenceOnSingleNodeForAgent4(node, neighbourNodes, nodesWithUndeterminedNeighbours, inferredNodes):
    global discoveredGrid, discoveredGridWithRestBlockages
    updated = False
    # Exit inference if H = 0
    if node.H != 0:
        # Inference
        if node.C == node.B:
            # make rest as empty
            for nodeLocation in neighbourNodes:
                if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == -1:
                    updated = True
                    discoveredGrid[nodeLocation[0]][nodeLocation[1]] = 0
                    discoveredGridWithRestBlockages[nodeLocation[0]][nodeLocation[1]] = 0
                    if node.position in nodesWithUndeterminedNeighbours:
                        nodesWithUndeterminedNeighbours[node.position].remove(nodeLocation)
        elif node.N - node.C == node.E:
            # make rest as blocked
            for nodeLocation in neighbourNodes:
                if discoveredGrid[nodeLocation[0]][nodeLocation[1]] == -1:
                    updated = True
                    discoveredGrid[nodeLocation[0]][nodeLocation[1]] = 1
                    discoveredGridWithRestBlockages[nodeLocation[0]][nodeLocation[1]] = 1
                    if node.position in nodesWithUndeterminedNeighbours:
                        nodesWithUndeterminedNeighbours[node.position].remove(nodeLocation)
        if updated:
            node.E = 0
            node.B = 0
            node.H = 0
            for neighbour in neighbourNodes:
                node = UpdateNodeStatus(node, neighbour)
            if node.H == 0 and node.position in nodesWithUndeterminedNeighbours:
                inferredNodes.append(node.position)          

    return node, nodesWithUndeterminedNeighbours, inferredNodes

    # This is code for comparision of Agent 1, 2, 3 and 4- comparing trajectory length, total planning time, tajectory length for path in final discoverd girdworld
    # Generating random maze for dimension 101
    # with probabilities ranging from 0 to 1 in interval of 0.033
def PlotPerformanceComparisionForAgents1_2_3_4():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells
    dim = 5
    iterations = 5
    # po value from question 4
    maxProb = 0.33
    # Manhattan heuristic - Best heauristic from question 5
    heuristic = 0 
    problist = []
    
    averageNodesVisitedForAgent1 = []
    averageNodesVisitedForAgent2 = []
    averageNodesVisitedForAgent3 = []
    averageNodesVisitedForAgent4 = []

    averageTimeTakenForAgent1 = []
    averageTimeTakenForAgent2 = []
    averageTimeTakenForAgent3 = []
    averageTimeTakenForAgent4 = []

    averageProcessedCellsForAgent1 = []
    averageProcessedCellsForAgent2 = []
    averageProcessedCellsForAgent3 = []
    averageProcessedCellsForAgent4 = []

    averagePathLengthInFinalDiscoveredGridforAgent1 = []
    averagePathLengthInFinalDiscoveredGridforAgent2 = []
    averagePathLengthInFinalDiscoveredGridforAgent3 = []
    averagePathLengthInFinalDiscoveredGridforAgent4 = []

    for p in (np.arange(0, maxProb + 0.01, 0.033)):
        nodesVisitedForAgent1 = []
        nodesVisitedForAgent2 = []
        nodesVisitedForAgent3 = []
        nodesVisitedForAgent4 = []

        timeTakenForAgent1 = []
        timeTakenForAgent2 = []
        timeTakenForAgent3 = []
        timeTakenForAgent4 = []

        ProcessedCellsForAgent1 = []
        ProcessedCellsForAgent2 = []
        ProcessedCellsForAgent3 = []
        ProcessedCellsForAgent4 = []

        pathLengthInFinalDiscoveredGridforAgent1 = []
        pathLengthInFinalDiscoveredGridforAgent2 = []
        pathLengthInFinalDiscoveredGridforAgent3 = []
        pathLengthInFinalDiscoveredGridforAgent4 = []

        for y in range(iterations):
            solvableGrid = False
            while solvableGrid == False: 
                maze = RandomGridGenerator(p, dim)
                rowCount, columnCount = np.shape(maze)
                start = [0, 0]  # starting position
                end = [rowCount - 1, columnCount - 1]  # ending position
                pathInFullGrid = AStarSearch(maze, start, end, heuristic)
                solvableGrid = solvable
                if solvableGrid:
                    path = RepeatedAStarSearch(maze, start, end, heuristic, True)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent1.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent1.append(totalPlanningTime)
                    nodesVisitedForAgent1.append(len(path))
                    ProcessedCellsForAgent1.append(processedCells)

                    path = RepeatedAStarSearch(maze, start, end, heuristic, False)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent2.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent2.append(totalPlanningTime)
                    nodesVisitedForAgent2.append(len(path))
                    ProcessedCellsForAgent2.append(processedCells)

                    path = RepeatedAStarSearchWithAgent3(maze, start, end, heuristic)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent3.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent3.append(totalPlanningTime)
                    nodesVisitedForAgent3.append(len(path))
                    ProcessedCellsForAgent3.append(processedCells)

                    path = RepeatedAStarSearchWithAgent4(maze, start, end, heuristic)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent4.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent4.append(totalPlanningTime)
                    nodesVisitedForAgent4.append(len(path))
                    ProcessedCellsForAgent4.append(processedCells)

        problist.append(p)

        averageNodesVisitedForAgent1.append(sum(nodesVisitedForAgent1)/len(nodesVisitedForAgent1))
        averageNodesVisitedForAgent2.append(sum(nodesVisitedForAgent2)/len(nodesVisitedForAgent2))
        averageNodesVisitedForAgent3.append(sum(nodesVisitedForAgent3)/len(nodesVisitedForAgent3))
        averageNodesVisitedForAgent4.append(sum(nodesVisitedForAgent4)/len(nodesVisitedForAgent4))

        averageProcessedCellsForAgent1.append(sum(ProcessedCellsForAgent1)/len(ProcessedCellsForAgent1))
        averageProcessedCellsForAgent2.append(sum(ProcessedCellsForAgent2)/len(ProcessedCellsForAgent2))
        averageProcessedCellsForAgent3.append(sum(ProcessedCellsForAgent3)/len(ProcessedCellsForAgent3))
        averageProcessedCellsForAgent4.append(sum(ProcessedCellsForAgent4)/len(ProcessedCellsForAgent4))
        
        averageTimeTakenForAgent1.append(sum(timeTakenForAgent1)/len(timeTakenForAgent1))
        averageTimeTakenForAgent2.append(sum(timeTakenForAgent2)/len(timeTakenForAgent2))
        averageTimeTakenForAgent3.append(sum(timeTakenForAgent3)/len(timeTakenForAgent3))
        averageTimeTakenForAgent4.append(sum(timeTakenForAgent4)/len(timeTakenForAgent4))

        averagePathLengthInFinalDiscoveredGridforAgent1.append(sum(pathLengthInFinalDiscoveredGridforAgent1)/len(pathLengthInFinalDiscoveredGridforAgent1))
        averagePathLengthInFinalDiscoveredGridforAgent2.append(sum(pathLengthInFinalDiscoveredGridforAgent2)/len(pathLengthInFinalDiscoveredGridforAgent2))
        averagePathLengthInFinalDiscoveredGridforAgent3.append(sum(pathLengthInFinalDiscoveredGridforAgent3)/len(pathLengthInFinalDiscoveredGridforAgent3))
        averagePathLengthInFinalDiscoveredGridforAgent4.append(sum(pathLengthInFinalDiscoveredGridforAgent4)/len(pathLengthInFinalDiscoveredGridforAgent4))

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Trajectory Length")
    ax.plot(problist, averageNodesVisitedForAgent1, color="red", label= "blindfolded agent")
    ax.plot(problist, averageNodesVisitedForAgent2, color="blue", label= "agent with fov 4")
    ax.plot(problist, averageNodesVisitedForAgent3, color="green", label= "agent 3")
    ax.plot(problist, averageNodesVisitedForAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Processed cells")
    ax.plot(problist, averageProcessedCellsForAgent1, color="red", label= "blindfolded agent")
    ax.plot(problist, averageProcessedCellsForAgent2, color="blue", label= "agent with fov 4")
    ax.plot(problist, averageProcessedCellsForAgent3, color="green", label= "agent 3")
    ax.plot(problist, averageProcessedCellsForAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Total planning time")
    ax.plot(problist, averageTimeTakenForAgent1, color="red", label= "blindfolded agent")
    ax.plot(problist, averageTimeTakenForAgent2, color="blue", label= "agent with fov 4")
    ax.plot(problist, averageTimeTakenForAgent3, color="green", label= "agent 3")
    ax.plot(problist, averageTimeTakenForAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Trajectory length through final discoverd girdworld")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent1, color="red", label= "blindfolded agent")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent2, color="blue", label= "agent with fov 4")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent3, color="green", label= "agent 3")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    # This is code for comparision of Agent 1, 2, 3 - comparing trajectory length, total planning time, tajectory length for path in final discoverd girdworld
    # Generating random maze for dimension 101
    # with probabilities ranging from 0 to 1 in interval of 0.033
def PlotPerformanceComparisionForAgents1_2_3():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells
    dim = 15
    iterations = 10
    # po value from question 4
    maxProb = 0.33
    # Manhattan heauristic - Best heauristic from question 5
    heuristic = 0 

    problist = []
    averageNodesVisitedForAgent1 = []
    averageNodesVisitedForAgent2 = []
    averageNodesVisitedForAgent3 = []

    averageTimeTakenForAgent1 = []
    averageTimeTakenForAgent2 = []
    averageTimeTakenForAgent3 = []

    averageProcessedCellsForAgent1 = []
    averageProcessedCellsForAgent2 = []
    averageProcessedCellsForAgent3 = []
    averageProcessedCellsForAgent4 = []

    averagePathLengthInFinalDiscoveredGridforAgent1 = []
    averagePathLengthInFinalDiscoveredGridforAgent2 = []
    averagePathLengthInFinalDiscoveredGridforAgent3 = []
    averagePathLengthInFinalDiscoveredGridforAgent4 = []

    for p in np.arange(0, maxProb + 0.01, 0.033):
        nodesVisitedForAgent1 = []
        nodesVisitedForAgent2 = []
        nodesVisitedForAgent3 = []

        timeTakenForAgent1 = []
        timeTakenForAgent2 = []
        timeTakenForAgent3 = []

        ProcessedCellsForAgent1 = []
        ProcessedCellsForAgent2 = []
        ProcessedCellsForAgent3 = []
        ProcessedCellsForAgent4 = []


        pathLengthInFinalDiscoveredGridforAgent1 = []
        pathLengthInFinalDiscoveredGridforAgent2 = []
        pathLengthInFinalDiscoveredGridforAgent3 = []
        pathLengthInFinalDiscoveredGridforAgent4 = []

        for y in range(iterations):
            solvableGrid = False
            while solvableGrid == False: 
                maze = RandomGridGenerator(p, dim)
                rowCount, columnCount = np.shape(maze)
                start = [0, 0]  # starting position
                end = [rowCount - 1, columnCount - 1]  # ending position
                pathInFullGrid = AStarSearch(maze, start, end, heuristic)
                solvableGrid = solvable
                if solvableGrid:
                    path = RepeatedAStarSearch(maze, start, end, heuristic, True)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent1.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent1.append(totalPlanningTime)
                    nodesVisitedForAgent1.append(len(path))
                    ProcessedCellsForAgent1.append(processedCells)

                    path = RepeatedAStarSearch(maze, start, end, heuristic, False)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent2.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent2.append(totalPlanningTime)
                    nodesVisitedForAgent2.append(len(path))
                    ProcessedCellsForAgent2.append(processedCells)

                    path = RepeatedAStarSearchWithAgent3(maze, start, end, heuristic)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent3.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent3.append(totalPlanningTime)
                    nodesVisitedForAgent3.append(len(path))
                    ProcessedCellsForAgent3.append(processedCells)

        problist.append(p)
        averageNodesVisitedForAgent1.append(sum(nodesVisitedForAgent1)/len(nodesVisitedForAgent1))
        averageNodesVisitedForAgent2.append(sum(nodesVisitedForAgent2)/len(nodesVisitedForAgent2))
        averageNodesVisitedForAgent3.append(sum(nodesVisitedForAgent3)/len(nodesVisitedForAgent3))

        averageProcessedCellsForAgent1.append(sum(ProcessedCellsForAgent1)/len(ProcessedCellsForAgent1))
        averageProcessedCellsForAgent2.append(sum(ProcessedCellsForAgent2)/len(ProcessedCellsForAgent2))
        averageProcessedCellsForAgent3.append(sum(ProcessedCellsForAgent3)/len(ProcessedCellsForAgent3))

        averageTimeTakenForAgent1.append(sum(timeTakenForAgent1)/len(timeTakenForAgent1))
        averageTimeTakenForAgent2.append(sum(timeTakenForAgent2)/len(timeTakenForAgent2))
        averageTimeTakenForAgent3.append(sum(timeTakenForAgent3)/len(timeTakenForAgent3))

        averagePathLengthInFinalDiscoveredGridforAgent1.append(sum(pathLengthInFinalDiscoveredGridforAgent1)/len(pathLengthInFinalDiscoveredGridforAgent1))
        averagePathLengthInFinalDiscoveredGridforAgent2.append(sum(pathLengthInFinalDiscoveredGridforAgent2)/len(pathLengthInFinalDiscoveredGridforAgent2))
        averagePathLengthInFinalDiscoveredGridforAgent3.append(sum(pathLengthInFinalDiscoveredGridforAgent3)/len(pathLengthInFinalDiscoveredGridforAgent3))

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Trajectory Length")
    ax.plot(problist, averageNodesVisitedForAgent1, color="red", label= "blindfolded agent", marker="o")
    ax.plot(problist, averageNodesVisitedForAgent2, color="blue", label= "agent with fov 4", marker="o")
    ax.plot(problist, averageNodesVisitedForAgent3, color="green", label= "agent 3", marker="o")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Processed cells")
    ax.plot(problist, averageProcessedCellsForAgent1, color="red", label= "blindfolded agent")
    ax.plot(problist, averageProcessedCellsForAgent2, color="blue", label= "agent with fov 4")
    ax.plot(problist, averageProcessedCellsForAgent3, color="green", label= "agent 3")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Total planning time")
    ax.plot(problist, averageTimeTakenForAgent1, color="red", label= "blindfolded agent", marker="o")
    ax.plot(problist, averageTimeTakenForAgent2, color="blue", label= "agent with fov 4", marker="o")
    ax.plot(problist, averageTimeTakenForAgent3, color="green", label= "agent 3", marker="o")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Trajectory length through final discoverd girdworld")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent1, color="red", label= "blindfolded agent", marker="o")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent2, color="blue", label= "agent with fov 4", marker="o")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent3, color="green", label= "agent 3", marker="o")
    plt.legend()
    plt.show()

    # This is code for comparision of Agent 3, 4 - comparing trajectory length, total planning time, tajectory length for path in final discoverd girdworld
    # Generating random maze for dimension 101
    # with probabilities ranging from 0 to 1 in interval of 0.033
def PlotPerformanceComparisionForAgents_3_4():
    global rowCount, columnCount, discoveredGrid, discoveredGridWithRestBlockages, processedCells
    dim = 30
    iterations = 20
    # po value from question 4
    maxProb = 0.33
    # Manhattan heauristic - Best heauristic from question 5
    heuristic = 0 

    problist = []
    averageNodesVisitedForAgent3 = []
    averageNodesVisitedForAgent4 = []

    averageTimeTakenForAgent3 = []
    averageTimeTakenForAgent4 = []

    averageProcessedCellsForAgent1 = []
    averageProcessedCellsForAgent2 = []
    averageProcessedCellsForAgent3 = []
    averageProcessedCellsForAgent4 = []

    averagePathLengthInFinalDiscoveredGridforAgent3 = []
    averagePathLengthInFinalDiscoveredGridforAgent4 = []

    for p in np.arange(0, maxProb + 0.01, 0.033):

        nodesVisitedForAgent3 = []
        nodesVisitedForAgent4 = []

        ProcessedCellsForAgent1 = []
        ProcessedCellsForAgent2 = []
        ProcessedCellsForAgent3 = []
        ProcessedCellsForAgent4 = []

        pathLengthInFinalDiscoveredGridforAgent3 = []
        pathLengthInFinalDiscoveredGridforAgent4 = []

        timeTakenForAgent3 = []
        timeTakenForAgent4 = []

        for y in range(iterations):
            solvableGrid = False
            while solvableGrid == False: 
                maze = RandomGridGenerator(p, dim)
                rowCount, columnCount = np.shape(maze)
                start = [0, 0]  # starting position
                end = [rowCount - 1, columnCount - 1]  # ending position
                pathInFullGrid = AStarSearch(maze, start, end, heuristic)
                solvableGrid = solvable
                if solvableGrid:

                    path = RepeatedAStarSearchWithAgent3(maze, start, end, heuristic)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent3.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent3.append(totalPlanningTime)
                    nodesVisitedForAgent3.append(len(path))
                    ProcessedCellsForAgent3.append(processedCells)
                    
                    path = RepeatedAStarSearchWithAgent4(maze, start, end, heuristic)
                    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, start, end, heuristic)
                    pathLengthInFinalDiscoveredGridforAgent4.append(len(pathInFinalDiscoveredGrid))
                    timeTakenForAgent4.append(totalPlanningTime)
                    nodesVisitedForAgent4.append(len(path))
                    ProcessedCellsForAgent4.append(processedCells)

        problist.append(p)

        averageNodesVisitedForAgent3.append(sum(nodesVisitedForAgent3)/len(nodesVisitedForAgent3))
        averageNodesVisitedForAgent4.append(sum(nodesVisitedForAgent4)/len(nodesVisitedForAgent4))

        averageProcessedCellsForAgent3.append(sum(ProcessedCellsForAgent3)/len(ProcessedCellsForAgent3))
        averageProcessedCellsForAgent4.append(sum(ProcessedCellsForAgent4)/len(ProcessedCellsForAgent4))

        averageTimeTakenForAgent3.append(sum(timeTakenForAgent3)/len(timeTakenForAgent3))
        averageTimeTakenForAgent4.append(sum(timeTakenForAgent4)/len(timeTakenForAgent4))

        averagePathLengthInFinalDiscoveredGridforAgent3.append(sum(pathLengthInFinalDiscoveredGridforAgent3)/len(pathLengthInFinalDiscoveredGridforAgent3))
        averagePathLengthInFinalDiscoveredGridforAgent4.append(sum(pathLengthInFinalDiscoveredGridforAgent4)/len(pathLengthInFinalDiscoveredGridforAgent4))

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Trajectory Length")
    ax.plot(problist, averageNodesVisitedForAgent3, color="green", label= "agent 3")
    ax.plot(problist, averageNodesVisitedForAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Processed Cells")
    ax.plot(problist, averageProcessedCellsForAgent3, color="green", label= "agent 3")
    ax.plot(problist, averageProcessedCellsForAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Total planning time")
    ax.plot(problist, averageTimeTakenForAgent3, color="green", label= "agent 3")
    ax.plot(problist, averageTimeTakenForAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

    fig,ax=plt.subplots()
    ax.set_xlabel("Probability")
    ax.set_ylabel("Trajectory length through final discoverd girdworld")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent3, color="green", label= "agent 3")
    ax.plot(problist, averagePathLengthInFinalDiscoveredGridforAgent4, color="orange", label= "agent 4")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    processedCells = 0
    totalPlanningTime = 0

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
    discoveredGrid = []
    discoveredGridWithRestBlockages = []

    # The below 4 function calls are for questions 4,5,6,7 - uncomment and run

    # PlotDensityVsSolvabilityForDim()
    # PlotDimensionVsTimeTakenFor3Distances()
    # PlotPerformanceForDensity()
    # PlotPerformanceForDensity(True)

    #PlotPerformanceComparisionForAgents1_2_3()
    #PlotPerformanceComparisionForAgents_3_4()
    PlotPerformanceComparisionForAgents1_2_3_4()

    # The below code is for testing single case
    maze = [[0, 0, 0, 1, 1], 
            [1, 1, 0, 0, 1], 
            [0, 0, 1, 0, 0], 
            [0, 0, 0, 0, 1], 
            [0, 0, 0, 0, 0]]

    #maze = RandomGridGenerator(0.25, 20)

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
    pathOfAgent1 = RepeatedAStarSearch(maze, source, destination, heuristic, True)
    endTime = time.time()  # End time
    print("Path given by agent 1 is :" , pathOfAgent1)
    if solvable:
        print("Length of path is", len(pathOfAgent1))
    print("Processed cells for agent 1(blindfolded) is", processedCells)
    print("Elapsed time for Agent 1 is {}".format(endTime - startTime), "\n")

    startTime = time.time()  # Start time
    pathOfAgent2 = RepeatedAStarSearch(maze, source, destination, heuristic)
    endTime = time.time()  # End time
    timeForRepeatedForwardAStar = endTime - startTime
    print("Path given by agent 2 is :", pathOfAgent2)
    if solvable:
        print("Length of path is", len(pathOfAgent2))
    print("Processed cells for agent 2 is", processedCells)
    print("Elapsed time for Agent 2 is  {}".format(timeForRepeatedForwardAStar), "\n")
    if solvable:
        TrajectoryLength = len(pathOfAgent2)
    processedCellCountOfRepeatedAStar = processedCells

    startTime = time.time()  # Start time
    pathOfAgent3 = RepeatedAStarSearchWithAgent3(maze, source, destination, heuristic)
    endTime = time.time()  # End time
    print("Path given by agent 3 is :", pathOfAgent3)
    if solvable:
        print("Length of path is", len(pathOfAgent3))
    print("Processed cells for agent 3 is", processedCells)
    print("Elapsed time for Agent 3 is {}".format(endTime - startTime), "\n")

    startTime = time.time()  # Start time
    pathOfAgent4 = RepeatedAStarSearchWithAgent4(maze, source, destination, heuristic)
    endTime = time.time()  # End time
    print("Path given by agent 4 is :", pathOfAgent4)
    if solvable:
        print("Length of path is", len(pathOfAgent4))
    print("Processed cells for agent 4 is", processedCells)
    print("Elapsed time for Agent 4 is {}".format(endTime - startTime), "\n")


    # Length of Shortest Path in Final Discovered Gridworld
    startTime = time.time()  # Start time
    pathInFinalDiscoveredGrid = AStarSearch(discoveredGridWithRestBlockages, source, destination, heuristic)
    endTime = time.time()  # End time
    timeForAStarWithFinalDiscoveredGrid = endTime - startTime
    print("Path given by A* with Final discoverd grid is :", pathInFinalDiscoveredGrid)
    
    if solvable:
        lengthInFinalDiscoveredGrid = len(pathInFinalDiscoveredGrid)
        print("Length of path is", len(pathInFinalDiscoveredGrid))
    print("Processed cells for agent 2 in final discovered grid is", processedCells)
    print("Elapsed time for A* with Final discoverd grid is  {}".format(timeForAStarWithFinalDiscoveredGrid), "\n")

    startTime = time.time()  # Start time, 
    pathInFullGrid = AStarSearch(maze, source, destination, heuristic)
    endTime = time.time()  # End time
    timeForAStarWithFullGrid = endTime - startTime
    print("Path given by A* with Full discoverd grid is :", pathInFullGrid)
   
    if solvable:
        lengthInFullGrid = len(pathInFullGrid) - 1
        print("Length of path is", len(pathInFullGrid) - 1)
    print("Processed cells for agent 2 in full discovered grid is", processedCells)
    print("Elapsed time for A* with Full discoverd grid is  {}".format(timeForAStarWithFullGrid), "\n")

    
