import heapq

# TO RUN: type python3 ./InformedSearch.py
# OUTPUTS: the number of visited states and the path to get there, if no path,
#          outputs "Failure"


# Start state of the program - change to whatever state you want to test
startState = [5,10,2,3,8,7,9,6,1,4]

# Goal state based on the spec
goalState = [10,9,8,7,6,5,4,3,2,1]

# List for visited states
# Note: this is a dangerously inefficient implementation of how to check this,
# but with the gap heuristic, the average number of visited states was less
# than 300 according to the PDF, so I allowed this inefficiency.
visited = []

# Implementation of a priority queue class, uses a min heap
class HQ:
    def __init__(self):
        self.queue = []
        heapq.heapify(self.queue)
    
    # Checks if the PQ is empty
    def isEmpty(self):
        return len(self.queue) == 0
        
    # Inserts a node into the PQ
    def insert(self, node):
        heapq.heappush(self.queue, node)
    
    # Removes a node from the PQ
    def remove(self):
        if self.isEmpty():
            print("Empty Queue")
            return 0
        nextState = heapq.heappop(self.queue)
        return nextState
        
    # Checks if a state is in the PQ, returns its index if it is, otherwise
    # returns -1 (false)
    def contains(self, state):
        for i in range(len(self.queue)):
            if self.queue[i].state == state:
                return i
        return -1
    
    # Prints the PQ. Used for debugging
    def printQueue(self):
        for i in range(len(self.queue)):
            print(self.queue[i].state, self.queue[i].cost)

# Class definition of a node in the search space
class Node:
    def __init__(self, state):
        self.state = state
        self.numFlips = 0
        self.cost = 0
        self.neighbors = []
        self.prev = None
    
    # Overloading the greater than operator so that node comparisons can be made
    # Compares based on the cost of the state    
    def __gt__(self,other):
        if self.cost > other.cost:
            return True
        else:
            return False
    
    # Generating neighbors for a given state.
    def generateNeighbors(self):
        # Starts with the index to flip at
        for i in range(0, 10):
            neighbor = Node([])
            # Up until that index, state is the same
            for k in range(0, i):
                neighbor.state.append(self.state[k])
            # After the flip index, prints the rest of the state in reverse
            for j in range(9, i - 1, -1):
                neighbor.state.append(self.state[j])
            # Calculating node costs
            neighbor.numFlips = 1 + self.numFlips
            neighbor.cost = neighbor.totalCost()
            neighbor.prev = self
            self.neighbors.append(neighbor)
            
    # Generating neighbors for a given state for UCS.
    def generateNeighborsUCS(self):
        # Starts with the index to flip at
        for i in range(0, 10):
            neighbor = Node([])
            # Up until that index, state is the same
            for k in range(0, i):
                neighbor.state.append(self.state[k])
            # After the flip index, prints the rest of the state in reverse
            for j in range(9, i - 1, -1):
                neighbor.state.append(self.state[j])
            # Calculating node costs for UCS (no heuristic)
            neighbor.cost = 1 + self.cost
            neighbor.prev = self
            self.neighbors.append(neighbor)
    
    # Method to print neighbors of a node. Used for debugging
    def printNeighbors(self):
        for n in range(len(self.neighbors)):
            print(self.neighbors[n].state)
    
    # The total cost function. Adds the backwards cost and the forwards cost.
    def totalCost(self):
        return self.numFlips + gapCost(self.state)

# Implementation of A* based on lecture slides
def Astar(state):
    startNode = Node(startState)
    pq = HQ()
    pq.insert(startNode)
    while True:
        # Checking if the priority queue is empty. In other words, making sure
        # there are nodes to visit.
        if pq.isEmpty():
            print("Failure")
            break
        n = pq.remove()
        # "Marking" (adding to a list) the state as visited
        visited.append(n.state)
        # Checks if the current state is the goal state
        if n.state == goalState:
            print("Done!")
            print("Num states visited:",len(visited))
            print("Path steps:")
            return generateSolution(n)
        # Generating neighbors of the current state and adding them to the PQ
        # if not visited already
        n.generateNeighbors()
        for i in range(len(n.neighbors)):
            # Checking if the neighbor was visited or already in the PQ
            if (wasVisited(n.neighbors[i]) == False) and (pq.contains(n.neighbors[i].state) == -1):
                pq.insert(n.neighbors[i])
            # Updating cost if necessary
            elif pq.contains(n.neighbors[i].state) > -1:
                index =  pq.contains(n.neighbors[i].state)
                newCost = n.neighbors[i].cost
                currentCost = pq.queue[index].cost
                if newCost < currentCost:
                    pq.queue[index] = n.neighbors[i]
    print("reached end")

# Implementation of UCS for the same problem based on lecture slides. Uses a PQ
# and is the same as the A* Implementation except it doesn't 
def UCS(state):
    startNode = Node(startState)
    pq = HQ()
    pq.insert(startNode)
    while True:
        # Checking if the priority queue is empty. In other words, making sure
        # there are nodes to visit.
        if pq.isEmpty():
            print("Failure")
            break
        n = pq.remove()
        # "Marking" (adding to a list) the state as visited
        visited.append(n.state)
        # Checks if the current state is the goal state
        if n.state == goalState:
            print("Done!")
            print("Num states visited:", len(visited))
            return generateSolution(n)
        # Generating neighbors of the current state and adding them to the PQ
        # if not visited already
        n.generateNeighborsUCS()
        for i in range(len(n.neighbors)):
            # Checking if the neighbor was visited or already in the PQ
            if wasVisited(n.neighbors[i]) == False and pq.contains(n.neighbors[i].state) == -1:
                pq.insert(n.neighbors[i])
            # Updating cost if necessary
            elif pq.contains(n.neighbors[i].state) > -1:
                index = pq.contains(n.neighbors[i].state)
                newCost = n.neighbors[i].cost
                currentCost = pq.queue[index].cost
                if newCost < currentCost:
                    pq.queue[index] = n.neighbors[i]
    print("reached end")

# Gap heuristic for A*, finds the number of gaps in a state. Provided by the
# spec.
def gapCost(state):
    gc = 0
    for i in range(9):
        if abs(state[i] - state[i+1]) > 1:
            gc += 1
    return gc
    
# Generates the path taken from the start node to the parameter node (typically
# the goal state)
def generateSolution(node):
    solution = []
    while(node.state != startState):
        solution.append(node.state)
        node = node.prev
    solution.append(startState)
    for i in range(len(solution)-1, -1, -1):
        print("Step "+str(len(solution) - i)+":",solution[i])

# Checks if a state was visited, compares to the list of visited nodes
def wasVisited(node):
    for i in range(len(visited)):
        if node.state == visited[i]:
            return True
    return False
    
def main():
    # Choose whether to run A* or UCS, make sure one is commented out 
    # as running one changes the start state for the second one
    Astar(startState)
    #UCS(startState)

if __name__ == '__main__':
    main()