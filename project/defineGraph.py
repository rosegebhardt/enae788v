import numpy as np

class Node:
    def __init__(self,x,y,u,v):

        self.x = x
        self.y = y
        self.u = u
        self.v = v

        self.numOutgoingEdges = 0
        self.outgoingEdges = []

        self.timeCostToStart = 0
        self.nrgCostToStart = 0

        self.timeParentNode = None
        self.nrgParentNode = None

        # 0 = not visited yet, 1 = in open list, 2 = in closed list
        self.timeStatus = 0
        self.nrgStatus = 0

class Edge:
    def __init__(self,startNode,endNode,timeCost,nrgCost):

        # Pointers to startpoint and endpoint of edge
        self.startNode = startNode
        self.endNode = endNode
        self.timeCost = timeCost
        self.nrgCost = nrgCost

class Graph:
    def __init__(self,X,Y,U,V,collisionChecker,Vmax,Kappa):

        [self.y_N,self.x_N] = X.shape
        self.nodeArray = np.array([[Node(X[i][j],Y[i][j],U[i][j],V[i][j]) for j in range(self.x_N)] for i in range(self.y_N)])

        self.nodes = self.nodeArray.flatten()
        self.numNodes = self.x_N*self.y_N
        self.edges = []
        self.numEdges = 0

        self.collisionChecker = collisionChecker

        self.Vmax = Vmax
        self.Kappa = Kappa
        self.maxFluidVel = np.max(np.sqrt(U[U < 1000000]**2 + V[V < 1000000]**2))
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
        
        if(not self.collisionChecker(startNode,endNode)):
            [timeCost,nrgCost] = self.getCosts(startNode,endNode)
            newEdge = Edge(startNode,endNode,timeCost,nrgCost)

            startNode.outgoingEdges.append(newEdge)
            self.edges.append(newEdge)
            self.numEdges = self.numEdges + 1
            startNode.numOutgoingEdges = startNode.numOutgoingEdges + 1

    def getCosts(self,startNode,endNode):

        u = startNode.u; v = startNode.v
        x1 = startNode.x; x2 = endNode.x
        y1 = startNode.y; y2 = endNode.y

        velocity = np.array([[u],[v]])
        vel = np.linalg.norm(velocity)
        displacement = np.array([[x2-x1],[y2-y1]])
        if (vel == 0):
            v1Hat = np.array([[1],[0]]) # direction doesn't matter in this case
        else:
            v1Hat = velocity/vel
        v2Hat = np.array([[-v1Hat[1]],[v1Hat[0]]])

        dx = np.dot(v1Hat.transpose(),displacement)[0][0]
        dy = np.dot(v2Hat.transpose(),displacement)[0][0]

        if (self.Vmax**2*(dx**2+dy**2) - vel**2*dy**2 < 0):
            timeCost = np.Inf
            nrgCost = np.Inf
        elif (np.abs(vel) == np.abs(self.Vmax)):
            if ((dx**2 + dy**2)/(2*vel*dx) < 0):
                timeCost = np.Inf
                nrgCost = np.Inf
            else:
                timeCost = (dx**2 + dy**2)/(2*vel*dx)
                nrgCost = 2*self.Kappa*vel*(np.sqrt(dx**2+dy**2) - dx)
        else:
            if((vel*dx - np.sqrt(self.Vmax**2*(dx**2+dy**2) - vel**2*dy**2))/(vel**2-self.Vmax**2) < 0):
                timeCost = np.Inf
                nrgCost = np.Inf
            else:
                timeCost = (vel*dx - np.sqrt(self.Vmax**2*(dx**2+dy**2) - vel**2*dy**2))/(vel**2-self.Vmax**2)
                nrgCost = 2*self.Kappa*vel*(np.sqrt(dx**2+dy**2) - dx)

        return [timeCost,nrgCost]

    def timeOptimalPath(self,goalNode):
        x = [goalNode.x]
        y = [goalNode.y]
        thisNode = goalNode.timeParentNode
        while(thisNode != None):
            x = np.append(x,thisNode.x)
            y = np.append(y,thisNode.y)
            thisNode = thisNode.timeParentNode
            # print(thisNode.outgoingEdges[0].nrgCost)
        timeBestPath = np.vstack((x,y))
        return timeBestPath
        # np.savetxt(pathFile,bestPath,delimiter=',')

    def nrgOptimalPath(self,goalNode):
        x = [goalNode.x]
        y = [goalNode.y]
        thisNode = goalNode.nrgParentNode
        while(thisNode != None):
            x = np.append(x,thisNode.x)
            y = np.append(y,thisNode.y)
            thisNode = thisNode.nrgParentNode
        nrgBestPath = np.vstack((x,y))
        return nrgBestPath

    # def savePath(self,goalNode,pathFile): # fix these fcns or get rid of them (better option)
    #     x = [goalNode.x]
    #     y = [goalNode.y]
    #     thisNode = goalNode.parentNode
    #     while(thisNode != None):
    #         x = np.append(x,thisNode.x)
    #         y = np.append(y,thisNode.y)
    #         thisNode = thisNode.parentNode
    #     bestPath = np.vstack((x,y))
    #     np.savetxt(pathFile,bestPath,delimiter=',')

    # def saveEdges(self,edgeFile):
    #     x = [np.NaN]
    #     y = [np.NaN]
    #     for edge in self.edges:
    #         x = np.append(x,edge.startNode.x)
    #         x = np.append(x,edge.endNode.x)
    #         x = np.append(x,np.NaN)
    #         y = np.append(y,edge.startNode.y)
    #         y = np.append(y,edge.endNode.y)    
    #         y = np.append(y,np.NaN)
    #     graphEdges = np.vstack((x,y))
    #     np.savetxt(edgeFile,graphEdges,delimiter=',')