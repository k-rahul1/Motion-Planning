#Author
#Rahul Kumar

import numpy as np
from math import sqrt
from queue import PriorityQueue
from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter

#Function to calculating distance between two points
def dist(a,b):
    return sqrt((b[0]-a[0])*(b[0]-a[0])+(b[1]-a[1])*(b[1]-a[1]))

#Function to recover the path from source to goal 
def recoverPath(s,g,prev):
    path = []
    path.append(g)
    v = prev[g] #prev contains the parent of each node
    while(v != s):
        v = prev[v]
        path.append(v)
    path.reverse()
    return path

#function to find the free neighbor around a vertex v
def neighbor(v,M):
    r,c = v
    #print(r,c,v)
    n = []
    h,w = len(M),len(M[0])
    for i in [-1,0,1]:
        for j in [-1,0,1]:
            if (i,j) == (0,0):
                continue
            elif (M[(r-i)][(c-j)] and 0<=r<=h and 0<=c<=w): #only free space neighbor are appended 
                n.append((r-i,c-j))
    return n

#funciton to create the occupancy grid using image
def map(filename):
    with Image.open(filename) as img: #opening the image file
        M = np.array(img.convert("1")) #creating an occupancy grid with array having free space as 1
    return M


def AStar(V,s,g,M):
    #creating dictionary
    CostTo = {} #store cost to reach vertex v from source
    EstTotalCost = {} #store cost to reach goal from source through vertex v
    prev = {} #stores the previous connected vertex on shortest path

    #Initializing the cost to reach goal as infinity from each vertex
    for v in V:
        CostTo[tuple(v)] = np.inf
        EstTotalCost[tuple(v)] = np.inf
    
    CostTo[s] = 0 #setting cost to reach source as 0
    EstTotalCost[s] = dist(s,g) #dist will return distance/cost from s to g

    #creating a priority queue and initialize with cost from source to goal
    pq = PriorityQueue()
    pq.put((EstTotalCost[s],s))

    while(not pq.empty()): #until priority queue is not null, loop will continue
        v=pq.get()

        #checking if goal is reached
        if v[1]==g:
            return recoverPath(s,g,prev)
        
        #checking neighbor of vertex v
        N = neighbor(v[1],M)

        #traversing each neighbor to find the optimal edge
        for i in N:
            pvi = CostTo[v[1]] + dist(v[1],i) #cost to reach i through v

            if pvi < CostTo[i]: #if new path is better than previous one to i, then update
                prev[i] = v[1]
                CostTo[i] = pvi

                #if vertex i is present in priority queue with larger cost,
                #then replacing it with updated cost
                for vertex in list(pq.queue):
                    if(vertex[1] == i):
                        pq.queue.remove((EstTotalCost[i],i))
                EstTotalCost[i] = pvi + dist(i,g)
                pq.put((EstTotalCost[i],i))
    return 0
    

if __name__ == "__main__":
    s = (480,85) #start point
    g = (105,510) #goal point

    M = map("occupancy_map.png") #reading the map

    V = np.transpose(np.nonzero(M)) #creating a list of vertices from free space of map
    #print(V)

    path = AStar(V,s,g,M)  #Astar is called
    #print(path)

    #calculating total path length
    length = 0
    for i in range(len(path)-1):
        length += dist(path[i+1],path[i])
    
    print(f"Total path length: {length}")

    #Ploting the path
    plt.imshow(M, cmap="gray")
    plt.plot(s[1],s[0],'go',markersize=5)
    plt.plot(g[1],g[0],'go',markersize=5)
    plt.show(block=False)
    for i in range(len(path)-1):
        r1,c1 = path[i]
        r2,c2 = path[i+1]
        plt.plot([c1,c2],[r1,r2],'-r')
        plt.pause(0.001)

    plt.show()



