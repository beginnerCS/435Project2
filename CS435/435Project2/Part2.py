from random import *

class newNode():
    def __init__(self, val):
        self.val = val
        self.visited = False
        self.parents = 0

class DirectedGraph():
    def __init__(self):
        self.graphSet = set([])
        self.graphDict = {}

    def addNode(self, nodeVal):
        node = newNode(nodeVal)
        self.graphSet.add(node)
        self.graphDict[node] = ()

    def addDirectedEdge(self, first, second):
        if(first in self.graphDict[second] or second in self.graphDict[first] or first is second):
            return
        else:
            self.graphDict[first] += (second,)
            second.parents+=1

    def removeDirectedEdge(self, first, second):
        listF = list(self.graphDict[first])
        listF.remove(second)
        self.graphDict[first] = tuple(listF)

    def getAllNodes(self):
        return self.graphSet

    def reset(self):
        for node in self.graphSet:
            node.visited = False

def createRandomDAGIter(num):
    randomG = DirectedGraph()
    for i in range(0,num):
        randomG.addNode(i+1)
    for i in range(0, num*6):
        node1 = sample(randomG.getAllNodes(), 1)[0]
        node2 = sample(randomG.getAllNodes(), 1)[0]
        if(node1.val > node2.val):
            randomG.addDirectedEdge(node1, node2)
    return randomG

class TopSort():

    def Kahns(self, graph):
        q = []
        visitedNodes = 0
        topSort = []
        for node in graph.graphSet:
            if node.parents is 0:
                q.append(node)
        while len(q) != 0:
             curr = q.pop(0)
             visitedNodes+=1
             topSort.append(curr)
             for val in graph.graphDict[curr]:
                 val.parents-=1
                 if(val.parents is 0):
                     q.append(val)
        if visitedNodes != len(graph.graphSet):
            return None
        else:
            return topSort

    def mDFS(self, graph):
        stack = []
        for node in graph.graphSet:
            if node.visited is False:
                self.mDFSHelper(node, stack, graph)
        return stack

    def mDFSHelper(self, node, stack, graph):
        node.visited = True
        for neighbor in graph.graphDict[node]:
            if neighbor.visited is False:
                self.mDFSHelper(neighbor, stack, graph)
        stack.append(node)

class WeightedGraph():

    def __init__(self):
        self.graphSet = set([])
        self.graphDict = {}
        self.graphWeights = {}

    def addNode(self, nodeVal):
        node = newNode(nodeVal)
        self.graphSet.add(node)
        self.graphDict[node] = []
        self.graphWeights[node] = []
        return node

    def addWeightedEdge(self, first, second, weight):
        if(second in self.graphDict[first]):
            return
        else:
            self.graphDict[first].append(second)
            self.graphWeights[first].append(weight)
            second.parents+=1

    def removeDirectedEdge(self, first, second):
        self.graphDict[first].remove(second)

    def getAllNodes(self):
        return self.graphSet

def createRandomCompleteWeightedGraph(num):
    randomG = WeightedGraph()
    randWeight = 0
    randWeightList = []
    allNodes = randomG.getAllNodes()
    for i in range(0,num):
        randomG.addNode(i+1)
    for i in range(0, 100):
        node1 = sample(allNodes, 1)[0]
        allNodes.remove(node1)
        node2 = sample(allNodes, 1)[0]
        allNodes.add(node1)
        randWeight = randint(1,100)
        while(randWeight in randWeightList):
            randWeight=randint(1,100)
        randWeightList.append(randWeight)
        randomG.addWeightedEdge(node1, node2, randWeight)
    return randomG

def createLinkedList(num):
    linked = WeightedGraph()
    prevNode = None
    for i in range(0,num):
        node = linked.addNode(i+1)
        if(prevNode is None):
            prevNode = node
        else:
            linked.addWeightedEdge(prevNode, node, 1)
            prevNode = node
    return linked

#graph input used to get neighbor nodes and size of graph
def dijkstras(start, graph):
    minVals = {}
    q = []
    minVals[start]=0
    for key in graph.graphDict:
        if(key!=start):
            minVals[key] = 10000
        q.append(key)
    while len(q)>0:
        vert = getMin(minVals, q, graph)
        q.remove(vert)
        i = 0
        for node in graph.graphDict[vert]:
            newDist = minVals[vert] + graph.graphWeights[vert][i]
            if(newDist < minVals[node]):
                minVals[node] = newDist
            i+=1
    return minVals

def getMin(minVals, q, graph):
    minDist = 1000
    for node in q:
        i=0
        if minVals[node] < minDist:
            minDist = graph.graphWeights[node][i]
            minNode = node
            i+=1
    return minNode

class newGridNode():
    def __init__(self, x, y, val):
        self.val = val
        self.x = x
        self.y = y

class GridGraph():
    def __init__(self):
        self.graphSet = set([])
        self.graphDict = {}

    def addGridNode(self, x, y, val):
        node = newGridNode(x, y, val)
        self.graphSet.add(node)
        self.graphDict[node] = []
        return node

    def addUndirectedEdge(self, first, second):
        xDist = abs(first.x - second.x)
        yDist = abs(first.y-second.y)
        if(xDist <= 1 and yDist <=1 and not (xDist == 1 and yDist == 1)):
            if(second not in self.graphDict[first]):
                self.graphDict[first].append(second)
            if(first not in self.graphDict[second]):
                self.graphDict[second].append(first)

    def removeUndirectedEdge(self, first, second):
        if(len(self.graphDict[first]) is 0):
            return
        self.graphDict[first].remove(second)
        self.graphDict[second].remove(first)

    def getAllNodes(self):
        return self.graphSet

def createRandomGridGraph(num):
    g = GridGraph()
    x = 0
    y = 0
    randSet = sample(range(1,100),num*num)
    for i in range(0,num*num):
        g.addGridNode(x, y, randSet[i])
        if(x!=num-1):
            x+=1
        else:
            y+=1
            x=0
    graphList = list(g.graphDict.keys())
    listIndex = 0
    for node in g.graphDict:
        connectL = randint(0,2)
        connectR = randint(0,2)
        connectU = randint(0,2)
        connectD = randint(0,2)
        if(node.x == 0 and node.y == 0):
            connectL = 0
            connectU = 0
        elif(node.x > 0 and node.x < num-1 and node.y == 0):
            connectU = 0
        elif(node.x == num-1 and node.y == 0):
            connectU = 0
            connectR = 0
        elif(node.x == 0 and node.y > 0 and node.y < num-1):
            connectL = 0
        elif(node.x == num-1 and node.y > 0 and node.y < num-1):
            connectR = 0
        elif(node.x == 0 and node.y == num-1):
            connectL = 0
            connectD = 0
        elif(node.x > 0 and node.x < num-1 and node.y == num-1):
            connectD = 0
        elif(node.x == num-1 and node.y == num-1):
            connectR = 0
            connectD = 0
        if(connectL == 1):
            g.addUndirectedEdge(node, graphList[listIndex-1])
        if(connectR == 1):
            g.addUndirectedEdge(node, graphList[listIndex+1])
        if(connectU == 1):
            g.addUndirectedEdge(node, graphList[listIndex-num])
        if(connectD == 1):
            g.addUndirectedEdge(node, graphList[listIndex+num])


        listIndex+=1

    return g

#heuristic is manhattan distance
def  astar(sourceNode, destNode, graph):
    path = []
    visited = []
    visited.append(sourceNode)
    path.append(sourceNode)
    xDelt = destNode.x - sourceNode.x
    yDelt = destNode.y - sourceNode.y
    prevNode = 0
    allAdj = []
    while(True):
        if(xDelt == 0 and yDelt == 0):
            return path
        adj = graph.graphDict[sourceNode]
        lnode = 0
        rnode = 0
        unode = 0
        dnode = 0
        allVisited = True
        if(len(adj)==0):
            return []
        for node in adj:
            if node not in visited:
                if(node.x-sourceNode.x>0):
                    rnode = node
                    allAdj.append(rnode)
                elif(node.x - sourceNode.x<0):
                    lnode = node
                    allAdj.append(lnode)
                elif(node.y-sourceNode.y>0):
                    dnode = node
                    allAdj.append(dnode)
                elif(node.y-sourceNode.y<0):
                    unode = node
                    allAdj.append(unode)
        for node in allAdj:
            if node not in visited:
                allVisited = False
        if(allVisited):
            return []
        if(rnode == 0 and lnode == 0 and unode == 0 and dnode == 0):
            path.pop()
            sourceNode = path.pop()
            path.append(sourceNode)
            continue
        elif(xDelt > 0):
            if(rnode!=0):
                sourceNode = rnode
                path.append(sourceNode)
            elif(yDelt > 0 and dnode != 0):
                sourceNode = dnode
                path.append(sourceNode)
            elif(yDelt < 0 and unode !=0):
                sourceNode = unode
                path.append(sourceNode)
            elif(rnode == 0 and unode == 0 and dnode == 0 and lnode !=0):
                sourceNode = lnode
                path.append(sourceNode)
            else:
                return []
        elif(yDelt>0):
            if(dnode!=0):
                sourceNode = dnode
                path.append(sourceNode)
            elif(xDelt > 0 and rnode != 0):
                sourceNode = rnode
                path.append(sourceNode)
            elif(xDelt < 0 and lnode !=0):
                sourceNode = lnode
                path.append(sourceNode)
            elif(rnode == 0 and lnode == 0 and dnode == 0 and unode !=0):
                sourceNode = unode
                path.append(sourceNode)
            else:
                return []
        visited.append(sourceNode)
        xDelt = destNode.x - sourceNode.x
        yDelt = destNode.y - sourceNode.y
    return path
