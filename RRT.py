#Author
#Rahul Kumar

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from PIL import Image

#creating a tree node
class treeNode:
    def __init__(self,r,c):
        self.r = r
        self.c = c
        self.parent_r = []
        self.parent_c = []

#creating the grid array from the map image
def map(filename):
    with Image.open(filename) as img:
        M = np.array(img.convert("1"))
    return M

#function to sample the node in the free space of map/configuration space
def sampleNode(M):
    r,c = M.shape
    while True:
        row = np.random.randint(0,r-1)
        column = np.random.randint(0,c-1)
        if M[row][column] == 1:
            return (row,column)

#Function to calculate distance between two points
def dist(a1,a2,b1,b2):
    return sqrt((b1-a1)*(b1-a1)+(b2-a2)*(b2-a2))

#function to get the closest node
def closestNode(node,node_list):
    distance = []
    for i in range(len(node_list)):
        distance.append(dist(node[0],node[1],node_list[i].r,node_list[i].c))
    return distance.index(min(distance))


#function to find the new node at a distance d from nearest node in line joining nearest to random node
def steerTonearestNode(rNode,nNode,d):
    if ((rNode[0]==int(nNode[0]) and rNode[1]==int(nNode[1])) or dist(rNode[0],rNode[1],nNode[0],nNode[1])<d):
        return rNode
    else:
        lineVector = (rNode[0]-nNode[0],rNode[1]-nNode[1])
        unitVector = lineVector/np.linalg.norm(lineVector)
        #print(rNode,nNode)
        #print("direc",unitVector, np.linalg.norm(lineVector))
        newNode = nNode + d * unitVector
        #print("newnode",newNode)
        newNode[0] = int(newNode[0])
        newNode[1] = int(newNode[1])
        return newNode

#function to check the edge joining two nodes is collision free or not
def validityCheck(M,d,r1,c1,r2,c2):
    p = np.linspace(0,1,d) #for generating intermediate points between two nodes
    for i in p:
        if(c1!=c2):
            c = c1 + i*(c2-c1)
            r = r1 + ((r2-r1)*(c-c1)/(c2-c1))
        else:
            c = c1
            r = r1 + i*(r2-r1)
        r = int(r)
        c = int(c)
        #print(r1,c1,r2,c2,r,c)
        if M[r][c] == 0:
            return False #notValid/Collision
    return True #valid/no-collision

#function to check goal is reached
def goalReached(n,g,radius):
    if dist(n[0],n[1],g[0],g[1])<radius:
        if validityCheck(M,radius,n[0],n[1],g[0],g[1]):
            return True

#RRT is implemented
def RRT(M,s,g,d,iter):
    #creating list to store all the nodes
    node_list = [0]
    node_list[0] = treeNode(s[0],s[1])
    node_list[0].parent_r.append(s[0])
    node_list[0].parent_c.append(s[1])

    i=1 #variable to track number of iterations
    plt.imshow(M, cmap="gray")
    plt.plot(s[1],s[0],'go',markersize=5)
    plt.plot(g[1],g[0],'go',markersize=5)
    plt.show(block=False)
    while(i<iter):
        #generating randomNode in the configutation space/Map
        randomNode = sampleNode(M)

        #finding nearest node from the existing tree to random node
        nearestNodeIndex = closestNode(randomNode,node_list)
        nearestNode = (node_list[nearestNodeIndex].r,node_list[nearestNodeIndex].c)

        #finding node in d distance/stepSize of nearest node
        newNode = steerTonearestNode(randomNode,nearestNode,d)

        if validityCheck(M,d,newNode[0],newNode[1],nearestNode[0],nearestNode[1]):
            node_list.append(i)
            node_list[i] = treeNode(newNode[0],newNode[1])
            node_list[i].parent_r = node_list[nearestNodeIndex].parent_r.copy()
            node_list[i].parent_c = node_list[nearestNodeIndex].parent_c.copy()
            node_list[i].parent_r.append(nearestNode[0])
            node_list[i].parent_c.append(nearestNode[1])

            plt.plot(newNode[1],newNode[0],'co',markersize=2)
            plt.plot([newNode[1],nearestNode[1]],[newNode[0],nearestNode[0]],'-r')
            #plt.show()
            plt.pause(0.001)

            if goalReached(newNode,g,100):
                length = 0
                #print(node_list[i].parent_r)
                for j in range(len(node_list[i].parent_r)-1):
                    length += dist(node_list[i].parent_r[j],node_list[i].parent_c[j],node_list[i].parent_r[j+1],node_list[i].parent_c[j+1])
                    plt.plot([node_list[i].parent_c[j],node_list[i].parent_c[j+1]],[node_list[i].parent_r[j],node_list[i].parent_r[j+1]],'-b')
                    plt.pause(0.001)
                #plotting the path
                plt.plot([newNode[1],nearestNode[1]],[newNode[0],nearestNode[0]],'-b')
                plt.plot([newNode[1],g[1]],[newNode[0],g[0]],'-b')
                #plt.show()
                print("Goal reached")
                print("No. of iteration done: ", i)
                print('No. of nodes in path: ',len(node_list[i].parent_r))
                print(f"Total path length: {length}")  
                break
            i=i+1
    if(i==iter):
        print("No path found")
    plt.show()


if __name__ == "__main__":
    M = map("occupancy_map.png") #creating the map
    iter = 2000 #setting the max iteration
    d = 70 #step size
    s = (480,85) #start point
    g = (105,510) #goal point

    RRT(M,s,g,d,iter) #calling the RRT function



