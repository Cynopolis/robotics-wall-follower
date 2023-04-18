from Graph.Vertex import Vertex
from Graph.Edge import Edge

class Graph:
    def __init__(self):
        self.adjacency_list = {}
        self.current_vertex = None
    
    def addVertex(self, vertex : Vertex):
        self.adjacency_list[vertex] = []
    
    def addEdge(self, vertex1 : Vertex, vertex2 : Vertex, weight = 1):
        if vertex1 not in self.adjacency_list:
            self.addVertex(vertex1)
        
        if vertex2 not in self.adjacency_list:
            self.addVertex(vertex2)
        
        edge = Edge(vertex1, vertex2, weight)
        
        self.adjacency_list[vertex1].append(edge)
    
    def getOutEdges(self, vertex : Vertex):
        # get all outgoing edges from the given vertex
        return self.adjacency_list[vertex]
    
    def getInEdges(self, vertex : Vertex):
        out_edges = []
        for vertex in self.adjacency_list:
            for edge in self.adjacency_list[vertex]:
                if edge.target is vertex and edge not in out_edges:
                    out_edges.append(edge)
        
        return out_edges
    
    def printGraph(self):
        # print the graph
        for vertex in self.adjacency_list:
            print(vertex.name, end=': ')
            for edge in self.adjacency_list[vertex]:
                print(edge.target.name, end=' ')
            print()
    
    def getVertex(self, name : str)->Vertex:
        for vertex in self.adjacency_list:
            if vertex.name == name:
                return vertex
        return None
    
    def setCurrentVertex(self, vertex : Vertex):
        self.current_vertex = vertex
    
    def getCurrentVertex(self)->Vertex:
        return self.current_vertex