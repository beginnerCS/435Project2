from random import *

class node():
    def __init__(self, val):
        self.val = val
        self.visited = False

class Graph():
    def __init__(self):
        self.vertices = set([])
        self.edges = {}

    #Returns the node so that you may use it
    def addNode(self, nodeVal):
        node1 = node(nodeVal)
        self.vertices.add(node1)
        self.edges[node1] = []
        return node1

    def addUndirectedEdge(self, first, second):
        if(second not in self.edges[first]):
            self.edges[first].append(second)
        if(first not in self.edges[second]):
            self.edges[second].append(first)

    def addDirectedEdge(self, first, second):
        self.edges[first] += (second,)

    def removeUndirectedEdge(self, first, second):
        if(len(self.edges[first]) is 0):
            return
        self.edges[first].remove(second)
        self.edges[second].remove(first)

    def getAllNodes(self):
        return self.vertices

    #Resets the visited value for each node in a graph in case you want to use it again
    def reset(self):
        for node in self.vertices:
            node.visited = False


def createRandomUnweightedGraphIter(num):
        randomG = Graph()
        for i in range(0,num):
            randomG.addNode(i+1)
        allNodes = randomG.getAllNodes()
        for i in range(0, num):
            node1 = sample(allNodes, 1)[0]
            node2 = sample(allNodes, 1)[0]
            randomG.addUndirectedEdge(node1, node2)
        return randomG

def createLinkedList(num):
    linked = Graph()
    prevNode = None
    for i in range(0,num):
        node = linked.addNode(i+1)
        if(prevNode is None):
            prevNode = node
        else:
            linked.addDirectedEdge(prevNode, node)
            prevNode = node
    return linked

class GraphSearch():

    def __init__(self, graph):
        self.DFSRecList = []
        self.BFTRecList = []
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
            for val in self.graph.edges[start]:
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
        DFSIterList = []
        stack.append(start)
        while(len(stack)>0):
            top = stack.pop()
            if(top.val is end.val):
                DFSIterList.append(top)
                return DFSIterList
            if(top.visited is False):
                DFSIterList.append(top)
                top.visited = True
            for node in self.graph.edges[top]:
                if(node.visited is False):
                    stack.append(node)
        return None

    def BFTRec(self, graph):
        if(len(self.BFTq) is 0 and len(self.BFTRecList) is 0):
            self.BFTq.append(list(graph.edges.keys())[0])
            (list(graph.edges.keys())[0]).visited = True
            return(self.BFTRec(graph))
        if(len(self.BFTq) is 0 and len(self.BFTRecList) > 0):
            return self.BFTRecList
        front = self.BFTq.pop(0)
        self.BFTRecList.append(front)
        for node in graph.edges[front]:
            if(node.visited is False):
                node.visited = True
                self.BFTq.append(node)
        allVisited = True
        for node in graph.vertices:
            if node.visited == False:
                allVisited = False
                notVisited = node
        if(len(graph.edges[front]) == 0 and allVisited is False):
            notVisited.visited = True
            self.BFTq.append(notVisited)
        return(self.BFTRec(graph))

    def BFTIter(self, graph):
        BFTIterList = []
        queue = []
        queue.append(list(graph.edges.keys())[0])
        allVisited = False
        while(allVisited == False):
            allVisited = True
            while(len(queue)>0):
                front = queue.pop(0)
                front.visited = True
                BFTIterList.append(front)
                for node in graph.edges[front]:
                    if(node.visited is False):
                        node.visited = True
                        queue.append(node)
            for node in graph.vertices:
                if node.visited is False:
                    allVisited = False
            if(allVisited is False and len(queue) == 0):
                for node in graph.vertices:
                    if(node.visited is False):
                        node.visited = True
                        queue.append(node)
                        continue
        return BFTIterList

def BFTRecLinkedList(graph):
    search = GraphSearch(graph)
    bftr = search.BFTRec(graph)
    return bftr

def BFTIterLinkedList(graph):
    search = GraphSearch(graph)
    bfti = search.BFTIter(graph)
    return bfti
