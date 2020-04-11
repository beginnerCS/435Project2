from random import *

class newNode():
    def __init__(self, val):
        self.val = val
        self.visited = False

class Graph():
    def __init__(self):
        self.graphSet = set([])
        self.graphDict = {}

    def addNode(self, nodeVal):
        node = newNode(nodeVal)
        self.graphSet.add(node)
        self.graphDict[node] = ()

    def addReturnedNode(self, nodeVal):
        node = newNode(nodeVal)
        self.graphSet.add(node)
        self.graphDict[node] = ()
        return node

    def addUndirectedEdge(self, first, second):
        if(second not in self.graphDict[first]):
            self.graphDict[first] += (second,)
        if(first not in self.graphDict[second]):
            self.graphDict[second] += (first,)

    def addLinkedEdge(self, first, second):
        self.graphDict[first] += (second,)

    def removeUndirectedEdge(self, first, second):
        listF = list(self.graphDict[first])
        listS = list(self.graphDict[second])
        listF.remove(second)
        listS.remove(first)
        self.graphDict[first] = tuple(listF)
        self.graphDict[second] = tuple(listS)

    def getAllNodes(self):
        return self.graphSet

    def reset(self):
        for node in self.graphSet:
            node.visited = False


def createRandomUnweightedGraphIter(num):
        randomG = Graph()
        randVal = 0
        for i in range(0,num):
            randVal = randint(1,100)
            randomG.addNode(randVal)
        for i in range(0, num*2):
            node1 = sample(randomG.getAllNodes(), 1)[0]
            node2 = sample(randomG.getAllNodes(), 1)[0]
            randomG.addUndirectedEdge(node1, node2)
        return randomG

def createLinkedList(num):
    linked = Graph()
    randVal = 0
    prevNode = None
    for i in range(0,num):
        randVal = randint(1,100)
        node = linked.addReturnedNode(randVal)
        if(prevNode is None):
            prevNode = node
        else:
            linked.addLinkedEdge(prevNode, node)
            prevNode = node
    return linked

class GraphSearch():

    def __init__(self, graph):
        self.DFSRecList = []
        self.DFSIterList = []
        self.BFTRecList = []
        self.BFTIterList = []
        self.graph = graph
        self.BFTq = []
        self.BFTstck = []

    def DFSRec(self, start, end):
        if(start.val is end.val):
            self.DFSRecList.append(start)
            return self.DFSRecList
        else:
            self.BFTstck.append(start)
            self.DFSRecList.append(start)
            start.visited = True
            for val in self.graph.graphDict[start]:
                if(val.visited is False):
                    self.BFTstck.append(start)
                    return(self.DFSRec(val, end))
            if(len(self.BFTstck) > 1):
                top = self.BFTstck.pop()
                return(self.DFSRec(self.BFTstck.pop(), end))
            else:
                return None

    def DFSIter(self, start, end):
        stack = []
        stack.append(start)
        while(len(stack)>0):
            top = stack[-1]
            if(top.val is end.val):
                self.DFSIterList.append(top)
                return self.DFSIterList
            stack.pop()
            if(top.visited is False):
                self.DFSIterList.append(top)
                top.visited = True
            for node in self.graph.graphDict[top]:
                if(node.visited is False):
                    stack.append(node)
        return None

    def BFTRec(self, graph):
        if(len(self.BFTq) is 0 and len(self.BFTRecList) is 0):
            self.BFTq.append(list(graph.graphDict.keys())[0])
            return(self.BFTRec(graph))
        if(len(self.BFTq) is 0 and len(self.BFTRecList) > 0):
            return self.BFTRecList
        front = self.BFTq.pop(0)
        self.BFTRecList.append(front)
        for node in graph.graphDict[front]:
            if(node.visited is False):
                node.visited = True
                self.BFTq.append(node)
        return(self.BFTRec(graph))

    def BFTIter(self, graph):
        q = []
        q.append(list(graph.graphDict.keys())[0])
        while(len(q) > 0):
            front = q.pop(0)
            self.BFTIterList.append(front)
            for node in graph.graphDict[front]:
                if(node.visited is False):
                    node.visited = True
                    q.append(node)
        return self.BFTIterList

def BFTRecLinkedList(graph):
    search = GraphSearch(graph)
    bftr = search.BFTRec(graph)

def BFTIterLinkedList(graph):
    search = GraphSearch(graph)
    bfti = search.BFTIter(graph)
