#Author
#Rahul Kumar

import networkx as nx
import numpy as np
from math import sqrt
from PIL import Image
import matplotlib.pyplot as plt

#Function to sample vertex in the map
def sampleVertex(M):
    r,c = M.shape
    while True:
        row = np.random.randint(0,r)
        column = np.random.randint(0,c)
        if M[row][column] == 1: #keep sampling until point is sampled in free space
            return (row,column)

#Function to calculate distance between two points
def dist(a,b):
    return sqrt((b[0]-a[0])*(b[0]-a[0])+(b[1]-a[1])*(b[1]-a[1]))

#funciton to create the grid map using image
def map(filename):
    with Image.open(filename) as img: #opening the image file
        M = np.array(img.convert("1")) #creating an occupancy grid with array having free space as 1
    return M

#Function to check the whether path between two points is collision-free
def validityCheck(M,v1,v2):
    p = np.linspace(0,1,75) #generating numbers to be used for sampling on line
    r1,c1 = v1
    r2,c2 = v2
    #print(v1,v2)
    for i in p:
        #sampling points along the line joining the points
        if(c1!=c2):
            c = c1 + i*(c2-c1)
            r = r1 + ((r2-r1)*(c-c1)/(c2-c1))
        else:
            c = c1
            r = r1 + i*(r2-r1)
        #print(r,c)
        r = int(r)
        c = int(c)
        if M[r][c] == 0: #checking the sampled point is collision free
            return False
    return True

#Function to add vertex to the graph
def addVertex(M,G,v,d):
    G.add_node(v) #adding vertex
    for vertex in list(G.nodes):
        if(vertex!=v and dist(vertex,v)<=d):
            if(validityCheck(M,vertex,v)):
                G.add_edge(v,vertex,weight=dist(v,vertex)) #adding edge
                #plt.plot(v[1],v[0],'ro',markersize=1)
                #plt.plot([v[1],vertex[1]],[v[0],vertex[0]],'-b')
                #plt.pause(0.00001)
    return G

#Function to create a graph
def PRM(M,N,d):
    G = nx.Graph()
    for i in range(N):
        v = sampleVertex(M)
        G = addVertex(M,G,v,d)
    return G

if __name__ == "__main__":
    M = map("occupancy_map.png") #creating the map
    N = 2500 #No. of sample points
    d = 75 #max distance between each connected points

    plt.imshow(M,cmap="gray")
    plt.show(block=False)
    G = PRM(M,N,d)
    # print(G.nodes())
    # print(G.number_of_nodes())
    # print(M.shape)
    # print(list(G.nodes))

    s = (480,85) #start point
    g = (105,510) #goal point

    G = addVertex(M,G,s,d) #adding start node to the graph
    G = addVertex(M,G,g,d) #adding goal node to the graph

    path = nx.astar_path(G,s,g)
    print(path)

    length = 0
    #calculating total path length
    for i in range(len(path)-1):
        length += dist(path[i+1],path[i])
    
    print(f"Total path length: {length}")

    #Ploting the path

    plt.plot(s[1],s[0],'go',markersize=5)
    plt.plot(g[1],g[0],'go',markersize=5)
    plt.show(block=False)
    for v in list(G.nodes):
        r,c = v
        plt.plot(c,r,'ro',markersize=1.5)
        plt.pause(0.000001)
    for i in range(len(path)-1):
        r1,c1 = path[i]
        r2,c2 = path[i+1]
        plt.plot([c1,c2],[r1,r2],'-g')
        plt.pause(1)
    # for e in G.edges():
    #     e1,e2 = e
    #     plt.plot([e1[1],e2[1]],[e1[0],e2[0]],'-b')
    plt.show()