import numpy as np
from operator import attrgetter

# Small changes to original graph structure to store 
# info needed for wavefront propagation algorithm

class Node:
    def __init__(self,x,y,u,v):

        self.x = x
        self.y = y
        self.u = u
        self.v = v

        self.numOutgoingEdges = 0
        self.outgoingEdges = []

        self.costToGo = np.Inf
        self.candidateCost = np.Inf
        self.status = 0     # 0 = Far, 1 = Considered, 2 = Accepted

class Edge:
    def __init__(self,startNode,endNode,cost):

        # Pointers to startpoint and endpoint of edge
        self.startNode = startNode
        self.endNode = endNode
        self.edgeCost = cost

class OUMGraph:
    def __init__(self,X,Y,U,V,collisionChecker,Vmax,Kappa):

        [self.y_N,self.x_N] = X.shape
        self.nodeArray = np.array([[Node(X[i][j],Y[i][j],U[i][j],V[i][j]) for j in range(self.x_N)] for i in range(self.y_N)])

        self.nodes = self.nodeArray.flatten()
        self.numNodes = self.x_N*self.y_N
        self.edges = []
        self.numEdges = 0

        self.collisionChecker = collisionChecker

        self.acceptedFront = []
        self.considered = []

        self.Vmax = Vmax
        self.Kappa = Kappa

        self.createEdges()

    def createEdges(self):

        # Iterate over nodes
        for j in range(self.x_N):
            for i in range(self.y_N):

                # See which directions are valid
                up1free = (i > 0); up2free = (i > 1)
                down1free = (i < self.y_N-1); down2free = (i < self.y_N-2)
                left1free = (j > 0); left2free = (j > 1)
                right1free = (j < self.x_N-1); right2free = (j < self.x_N-2)

                # Node of interest
                thisNode = self.nodeArray[i][j]

                # Two rows above current node
                if(up2free):

                    if(left1free):
                        neighborNode = self.nodeArray[i-2][j-1]
                        self.newEdge(thisNode,neighborNode)

                    if(right1free):
                        neighborNode = self.nodeArray[i-2][j+1]
                        self.newEdge(thisNode,neighborNode)
                
                # One row above current node
                if(up1free):

                    if(left2free):
                        neighborNode = self.nodeArray[i-1][j-2]
                        self.newEdge(thisNode,neighborNode)

                    if(left1free):
                        neighborNode = self.nodeArray[i-1][j-1]
                        self.newEdge(thisNode,neighborNode)

                    neighborNode = self.nodeArray[i-1][j]
                    self.newEdge(thisNode,neighborNode)

                    if(right1free):
                        neighborNode = self.nodeArray[i-1][j+1]
                        self.newEdge(thisNode,neighborNode)

                    if(right2free):
                        neighborNode = self.nodeArray[i-1][j+2]
                        self.newEdge(thisNode,neighborNode)
                
                # Same row as current node
                if(left1free):
                    neighborNode = self.nodeArray[i][j-1]
                    self.newEdge(thisNode,neighborNode)

                if(right1free):
                    neighborNode = self.nodeArray[i][j+1]
                    self.newEdge(thisNode,neighborNode)
                
                # One row below current node
                if(down1free):

                    if(left2free):
                        neighborNode = self.nodeArray[i+1][j-2]
                        self.newEdge(thisNode,neighborNode)

                    if(left1free):
                        neighborNode = self.nodeArray[i+1][j-1]
                        self.newEdge(thisNode,neighborNode)

                    neighborNode = self.nodeArray[i+1][j]
                    self.newEdge(thisNode,neighborNode)

                    if(right1free):
                        neighborNode = self.nodeArray[i+1][j+1]
                        self.newEdge(thisNode,neighborNode)

                    if(right2free):
                        neighborNode = self.nodeArray[i+1][j+2]
                        self.newEdge(thisNode,neighborNode)

                # Two rows below current node
                if(down2free):

                    if(left1free):
                        neighborNode = self.nodeArray[i+2][j-1]
                        self.newEdge(thisNode,neighborNode)

                    if(right1free):
                        neighborNode = self.nodeArray[i+2][j+1]
                        self.newEdge(thisNode,neighborNode)

    def newEdge(self,startNode,endNode):

        if(self.collisionChecker(startNode,endNode)):
            pass

        cost = self.getCost(startNode,endNode)
        newEdge = Edge(startNode,endNode,cost)

        startNode.outgoingEdges.append(newEdge)
        self.edges.append(newEdge)
        self.numEdges = self.numEdges + 1
        startNode.numOutgoingEdges = startNode.numOutgoingEdges + 1

    def getCost(self,startNode,endNode):

        u = startNode.u; v = startNode.v
        x1 = startNode.x; x2 = endNode.x
        y1 = startNode.y; y2 = endNode.y

        velocity = np.array([[u],[v]])
        vel = np.linalg.norm(velocity)
        displacement = np.array([[x2-x1],[y2-y1]])
        if (vel == 0):
            v1Hat = np.array([[1],[0]])
        else:
            v1Hat = velocity/vel
        v2Hat = np.array([[-v1Hat[1]],[v1Hat[0]]])

        dx = np.dot(v1Hat.transpose(),displacement)[0][0]
        dy = np.dot(v2Hat.transpose(),displacement)[0][0]

        if (self.Vmax**2*(dx**2+dy**2) - vel**2*dy**2 < 0):
            timeCost = np.Inf
        elif (np.abs(vel) == np.abs(self.Vmax)):
            if ((dx**2 + dy**2)/(2*vel*dx) < 0):
                timeCost = np.Inf
            else:
                timeCost = (dx**2 + dy**2)/(2*vel*dx)
        else:
            if((vel*dx - np.sqrt(self.Vmax**2*(dx**2+dy**2) - vel**2*dy**2))/(vel**2-self.Vmax**2) < 0):
                timeCost = np.Inf
            else:
                timeCost = (vel*dx - np.sqrt(self.Vmax**2*(dx**2+dy**2) - vel**2*dy**2))/(vel**2-self.Vmax**2)

        return timeCost

    # Fall down the gradient
    def optimalPath(self,startNode,goalNode):

        if (startNode.costToGo == np.Inf):
            print('No path possible in this flow field.')

        x = [startNode.x]
        y = [startNode.y]

        thisNode = startNode

        while (thisNode != goalNode):
            neighbors = []
            for edge in thisNode.outgoingEdges:
                neighbors.append(edge.endNode)
            nextNode = min(neighbors, key=attrgetter('costToGo'))
            x = np.append(x,nextNode.x)
            y = np.append(y,nextNode.y)

            thisNode = nextNode
        
        x = np.append(x,goalNode.x)
        y = np.append(y,goalNode.y)

        bestPath = np.vstack((x,y))
        print('Goal Reached!')

        return bestPath