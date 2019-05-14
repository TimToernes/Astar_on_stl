# -*- coding: utf-8 -*-
"""
Created on Tue May 14 08:54:49 2019

@author: Tim Pedersen
@contact: timtoernes@gmail.com

"""

import numpy as np
from stl import mesh
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import heapq

# Queue class used in Dikjkstra. 
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
      
# Grapg class
class StlGraph:
    def __init__(self,E,v):
        self.E = E
        self.v = v
        self.weights = {}
        
    def cost(self,from_node, to_node):
        a = np.asarray(from_node)
        b = np.asarray(to_node)
        return np.linalg.norm(a-b)
        
    def neighbors(self,id):
        mask = np.all(self.v[:,:]== id , axis=1)
        vertex_nr = np.flatnonzero(mask)[0]
        
        E_mask = E==vertex_nr
        neighbor_vertex = E[E_mask[:,0],1]
        neighbor_vertex = np.append(neighbor_vertex,E[E_mask[:,1],0])
        results = []
        for i in range(len(neighbor_vertex)):
            results.append(tuple(v[neighbor_vertex[i],:]))
        return results
        
# Function to import stl file and convert to graph 
def stl_to_graph(stl_mesh):
    stl_vector = stl_mesh.vectors
    # index 1 = face number
    # index 2 = node number
    # index 3 = coordinate number (x,y,z)
    
    v = np.array([[0,0,0]]) #Vertices
    E = np.array([[0,0]]) #Edges
    node_idx = 0
    
    for i in range(stl_vector[:,0,0].size):
        face_vector = stl_vector[i,:,:]
        # Loop over vertices on face
        for j in range(face_vector[:,0].size):
            
            v = np.append(v,[face_vector[j,:]],axis=0)
            if j == 0:
                E = np.append(E,[[node_idx+1,node_idx+2]],axis=0)
            if j == 1:
                E = np.append(E,[[node_idx-1,node_idx+1]],axis=0)
            if j == 2:
                E = np.append(E,[[node_idx-2,node_idx-1]],axis=0)
                
            node_idx = node_idx + 1
            
    v = v[1:,:]
    E = E[1:,:]
    
    [v_uni,index] = np.unique(v,axis=0,return_inverse=True)
    
    E_uni = np.copy(E)
    
    for i in range(len(index)):
        idx = E == i
        E_uni[idx] = index[i]
    
    E_uni = np.sort(E_uni)
    E_uni = np.unique(E_uni,axis=0)
    
    return E_uni,v_uni

# Function plotting stl graph and optimal path
def plot_graph(E,v,path):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for i in range(len(E)):
        x = v[[E[i]],0].flatten()
        y = v[[E[i]],1].flatten()
        z = v[[E[i]],2].flatten()
        ax.plot(x,y,z, zdir='z', c='blue', linewidth=0.5)
        
    path_array = np.asarray(path)
    x = path_array[:,0]
    y = path_array[:,1]
    z = path_array[:,2]        
    ax.plot(x,y,z, zdir='z', c='red', linewidth=2, marker='o', markerfacecolor='red')        
    return

# Dijkstra path planning algorithm
def dijkstra_search(graph, start, goal):
    
    def reconstruct_path(came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start) # optional
        path.reverse() # optional
        return path
    
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
        
    path = reconstruct_path(came_from, start, goal)
    return path


""" Main  """

# Using an existing stl file:
stl_mesh = mesh.Mesh.from_file('Part3.stl')

# Generate graph vectors
E,v = stl_to_graph(stl_mesh)

# Define start and end point on stl file
start = tuple(v[0])
goal = tuple(v[len(v)-1])

# Create graph element 
graph = StlGraph(E,v)

# Calculate optimal path using Dijkstra
path = dijkstra_search(graph, start, goal)

# Plot result
plot_graph(E,v,path)
    



    
    
    
    
    