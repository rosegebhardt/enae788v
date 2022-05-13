import numpy as np
from heapq import heappush, heappop

# A* Algorithm
def optimizeTime(G,startNode,goalNode):

    Q = []
    heappush(Q,(0,startNode))
    startNode.timeStatus = 1
    while(len(Q) != 0):
        thisNode = heappop(Q)[1]
        for edge in thisNode.outgoingEdges:
            neighborNode = edge.endNode
            if((neighborNode.timeStatus==0) or (neighborNode.timeCostToStart > thisNode.timeCostToStart + edge.timeCost)):
                neighborNode.timeStatus = 1
                neighborNode.timeParentNode = thisNode
                neighborNode.timeCostToStart = thisNode.timeCostToStart+edge.timeCost
                heuristic = np.sqrt((goalNode.x-neighborNode.x)**2 + (goalNode.y-neighborNode.y)**2)/(G.Vmax+G.maxFluidVel)
                keyValue = neighborNode.timeCostToStart + heuristic
                heappush(Q,(keyValue,neighborNode))
        
        thisNode.timeStatus = 2

        if(thisNode == goalNode):
            if(thisNode.timeCostToStart == np.Inf):
                print('No path possible in this flow field.')
            else:
                print('Goal Reached!')
            break
    
    if(len(Q) == 0):
        print('No path found.')

# Dijkstra's Algorithm
def optimizeNRG(G,startNode,goalNode):

    Q = []
    heappush(Q,(0,startNode))
    startNode.nrgStatus = 1
    while(len(Q) != 0):
        thisNode = heappop(Q)[1]
        for edge in thisNode.outgoingEdges:
            neighborNode = edge.endNode
            if((neighborNode.nrgStatus==0) or (neighborNode.nrgCostToStart > thisNode.nrgCostToStart + edge.nrgCost)):
                neighborNode.nrgStatus = 1
                neighborNode.nrgParentNode = thisNode
                neighborNode.nrgCostToStart = thisNode.nrgCostToStart+edge.nrgCost
                keyValue = neighborNode.nrgCostToStart
                heappush(Q,(keyValue,neighborNode))
        
        thisNode.nrgStatus = 2

        if(thisNode == goalNode):
            if(thisNode.timeCostToStart == np.Inf):
                print('No path possible in this flow field.')
            else:
                print('Goal Reached!')
            break
    
    if(len(Q) == 0):
        print('No path found.')