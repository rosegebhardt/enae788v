import numpy as np
from operator import attrgetter

# Called in each iteration of cost map computation
def candidateCost(NF):
    simplexSet = [(a, b) for idx, a in enumerate(NF) for b in NF[idx + 1:]]
    V = []
    for simplex in simplexSet:
        xj = simplex[0][0]; xk = simplex[1][0]
        uj = xj.costToGo; uk = xk.costToGo
        cj = simplex[0][1]; ck = simplex[1][1]
        zeta = np.linspace(0,1,100)
        val = np.min(zeta*(cj + uj) + (1-zeta)*(ck + uk))
        if np.isnan(val):
            val = np.Inf
        V.append(val)
    if (len(V) > 0):
        candidate  = np.min(V)
    else:
        candidate = np.Inf
    return candidate

# Function for computing cost map
def wavefrontPropogation(G,goalNode):
    # 1. Start with all the mesh points in Far
        # this is how the graph is initialized
    G.acceptedFront.append(goalNode)
    goalNode.status = 2
    goalNode.costToGo = 0
    # 2. Move the mesh points on the bdy to accepted
    for frontEdge in goalNode.outgoingEdges:
        frontNode = frontEdge.endNode
        G.acceptedFront.append(frontNode)
        frontNode.status = 2
        frontNode.costToGo = frontEdge.edgeCost
        # 3. Move all mesh points x adjacent to the boundary into considered and evaluate tentative values
        for consideredEdge in frontNode.outgoingEdges:
            consideredNode = consideredEdge.endNode
            if (consideredNode.status == 0):
                G.considered.append(consideredNode)
                consideredNode.status = 1
                nearFront = []
                for edge in consideredNode.outgoingEdges:
                    if (edge.endNode.status == 2):
                        nearFront.append((edge.endNode,edge.edgeCost))
                consideredNode.candidateCost = candidateCost(nearFront)

    while(len(G.considered) > 0):
        # 4. Find the grid point with the smallest values of V(x) among all the considered
        newAF = min(G.considered, key=attrgetter('candidateCost'))
        # print(newAF.candidateCost)

        # 5. Move the new point to accepted and update the accepted front
        newAF.status = 2
        newAF.costToGo = newAF.candidateCost
        G.considered.remove(newAF)
        G.acceptedFront.append(newAF)
    
        # 6. Move the far grid points adjacent to the new point into considered
        for consideredEdge in newAF.outgoingEdges:
            consideredNode = consideredEdge.endNode
            if (consideredNode.status == 0):
                G.considered.append(consideredNode)
                consideredNode.status = 1
            # 7. Recompute the value for all considerd nodes neighboring newAF
            if (consideredNode.status == 1):
                nearFront = []
                for edge in consideredNode.outgoingEdges:
                    if (edge.endNode.status == 2):
                        nearFront.append((edge.endNode,edge.edgeCost))
                V = candidateCost(nearFront)
                if (consideredNode.candidateCost > V):
                    consideredNode.candidateCost = V
