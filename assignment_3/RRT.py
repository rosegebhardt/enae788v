import numpy as np
from numpy import genfromtxt
import os

# Given problem parameters
delta_t = 0.1       # ensures collision detection happens before delta > 0.5
eps = [10,5,5,3,2]  
x_start = [30,40,30,-50,-30]
y_start = [-35,-40,40,-30,-35]
theta_start = [np.pi/2,0,3*np.pi/2,np.pi/2,np.pi/4]
goal_region = [[0,0,5],[30,40,5],[-50,-30,5],[30,-35,5],[-35,30,5]]

# Parse obstacle file given
obsFile = 'obstacles.txt'
obs = np.asarray(genfromtxt(obsFile, delimiter=',',skip_header=True))
numObs = obs.shape[0]

# Parse vehicle file given
vehicleFile = 'H3_robot.txt'
vehicle = np.asarray(genfromtxt(vehicleFile, delimiter=','))
numVehiclePts = vehicle.shape[0]
vehicleRadius = 1 # max radial distance from center for quick collision detection

# Waypoints searched by RRT
class Node:
    def __init__(self,x,y,theta,v,omega,t,nearestNode):

        # State variables
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        self.t = t

        # Data structure maintainace
        self.parentNode = nearestNode
        self.numOutgoingEdges = 0
        self.outgoingEdges = []
        self.inPath = False

# Dynamics and trajectories between waypoints
class Edge:
    def __init__(self,startNode,endNode,a,alpha,xTraj,yTraj,thetaTraj,vTraj,omegaTraj,tTraj):

        # Pointers to startpoint and endpoint of edge
        self.startNode = startNode
        self.endNode = endNode
        self.inPath = False

        # Control inputs - constant over edge
        self.a = a
        self.alpha = alpha
        
        # Store state values
        self.xTraj = xTraj
        self.yTraj = yTraj
        self.thetaTraj = thetaTraj
        self.vTraj = vTraj
        self.omegaTraj = omegaTraj
        self.tTraj = tTraj

# Store waypoints and dynamics in graph structure
class Graph:
    def __init__(self,x_start,y_start,theta_start):
        self.nodes = [Node(x_start,y_start,theta_start,0,0,0,None)]
        self.edges = []
        self.numNodes = 1
        self.numEdges = 0
    
    # Find existing node with smallest Euclidean distance to (x,y)
    def findNearestNode(self,x,y):
        distances = []
        for node in self.nodes:
            dist = np.sqrt((x-node.x)**2 + (y-node.y)**2)
            distances.append(dist)
        minIndex = np.argmin(distances)
        return self.nodes[minIndex]

    # Store a new node and edge
    def newNode(self,x,y,theta,v,omega,t,a,alpha,xTraj,yTraj,thetaTraj,vTraj,omegaTraj,tTraj,closeNode):
        newestNode = Node(x,y,theta,v,omega,t,closeNode)
        self.nodes.append(newestNode)
        self.edges.append(Edge(closeNode,newestNode,a,alpha,xTraj,yTraj,thetaTraj,vTraj,omegaTraj,tTraj))

        closeNode.numOutgoingEdges = closeNode.numOutgoingEdges + 1
        closeNode.outgoingEdges.append(newestNode)

        self.numNodes = self.numNodes + 1
        self.numEdges = self.numEdges + 1
    
    # Change attribute of nodes and edges in the feasible path found
    def definePath(self,goalNode):
        thisNode = goalNode
        while(thisNode != None):
            thisNode.inPath = True
            thisNode = thisNode.parentNode
        for edge in self.edges:
            if (edge.startNode.inPath & edge.endNode.inPath):
                edge.inPath = True
    
    # Save the feasible path to a .csv file
    def savePath(self,pathFile,x_start,y_start):
        x = [x_start]
        y = [y_start]
        for edge in self.edges:
            if(edge.inPath):
                x = np.append(x,edge.xTraj)
                y = np.append(y,edge.yTraj)
        path = np.vstack((x,y))
        np.savetxt(pathFile,path,delimiter=',')
    
    # Save the paths searched to a .csv file
    def saveSearchTraj(self,trajFile):
        x = [np.NaN]
        y = [np.NaN]
        for edge in self.edges:
            x = np.append(x,edge.xTraj)
            x = np.append(x,np.NaN)
            y = np.append(y,edge.yTraj)
            y = np.append(y,np.NaN)        
        searchTraj = np.vstack((x,y))
        np.savetxt(trajFile,searchTraj,delimiter=',')
    
    # Save the search tree in the format specified by the problem set
    def saveSearchTree(self,treeFile):
        search = np.zeros([self.numEdges,8])
        i = 0
        for edge in self.edges:
            search[i][0] = edge.startNode.t
            search[i][1] = edge.startNode.x
            search[i][2] = edge.startNode.y
            search[i][3] = edge.startNode.theta
            search[i][4] = edge.startNode.v
            search[i][5] = edge.startNode.omega
            search[i][6] = edge.a
            search[i][7] = edge.alpha
            i = i+1
        np.savetxt(treeFile,search,delimiter=',')

# Full collision checker - checks point by point
def isFree(x,y,theta):

    # Rotate and translate the vehicle
    c = np.cos(theta)
    s = np.sin(theta)
    rotMat = np.array(((c, s), (-s, c)))
    transMat = np.hstack((x*np.ones((numVehiclePts,1)),y*np.ones((numVehiclePts,1))))
    rotVehicle = np.dot(vehicle,rotMat)
    shiftVehicle = rotVehicle + transMat

    # Check for collision with each obstacle at each point
    for i in range(numVehiclePts):
        for j in range(numObs):
            if((shiftVehicle[i][0]-obs[j][0])**2 + (shiftVehicle[i][1]-obs[j][1])**2 < (obs[j][2])**2):
                return False
    return True

# Quick collision checker - collision reported if the obstacle is within maximum radius of the vehicle
def isFreeQuick(x,y,thetaQuick=0):

    # Check for collision with each obstacle
    for i in range(numObs):
        if((x-obs[i][0])**2 + (y-obs[i][1])**2 < (obs[i][2] + vehicleRadius)**2):
            return False
    return True

# Determine if end condition has been met
def reachedGoal(x,y,x_goal,y_goal,r_goal):
    if ((x-x_goal)**2 + (y-y_goal)**2 < r_goal**2):
        return True
    else:
        return False

# Starting from nearestNode, generate a feasible trajectory and store it in the graph
def newTrajectory(G,nearestNode,eps):

    # Preallocate space for trajectories
    x_traj = []
    y_traj = []
    theta_traj = []
    v_traj = []
    omega_traj = []
    t_traj = []

    # State at nearestNode
    x_i = nearestNode.x
    y_i = nearestNode.y
    theta_i = nearestNode.theta
    v_i = nearestNode.v
    omega_i = nearestNode.omega
    t_i = nearestNode.t
    
    # To reduce the chance of exceeding maximum velocities, accelarations are
    # chosen 'randomly' to reduce the magnitude of the velocities

    # Choose a 'random' feasible linear accelaration
    if(v_i > 2.5):
        a = np.random.uniform(-2.0,0,1)
    elif(v_i < -2.5):
        a = np.random.uniform(0,2.0,1)
    else:
        a = np.random.uniform(-2.0,2.0,1)

    # Choose a 'random' feasible rotational accelaration
    if(omega_i > np.pi/4):
        alpha = np.random.uniform(-np.pi/2,0,1)
    elif(omega_i < -np.pi/4):
        alpha = np.random.uniform(0,np.pi/2,1)
    else:
        alpha = np.random.uniform(-np.pi/2,np.pi/2,1)

    # Do not exceed a Euclidean distance of epsilon between waypoints
    while((x_i-nearestNode.x)**2 + (y_i-nearestNode.y)**2 < eps**2):

        # Apply Euler's method to approximate next state
        x_i1 = x_i + v_i*np.cos(theta_i)*delta_t
        y_i1 = y_i + v_i*np.sin(theta_i)*delta_t
        theta_i1 = theta_i + omega_i*delta_t
        v_i1 = v_i + a*delta_t
        omega_i1 = omega_i + alpha*delta_t
        t_i1 = t_i + delta_t
        
        # Break conditions
        if(abs(v_i1) > 5.0):
            break
        if(abs(omega_i1) > np.pi/2):
            break
        if(not isFree(x_i1,y_i1,theta_i1)):
            break

        # Update values
        x_i = float(x_i1)
        y_i = float(y_i1)
        theta_i = float(theta_i1)
        v_i = float(v_i1)
        omega_i = float(omega_i1)
        t_i = float(t_i1)

        # Store trajectory
        x_traj.append(x_i)
        y_traj.append(y_i)
        theta_traj.append(theta_i)
        v_traj.append(v_i)
        omega_traj.append(omega_i)
        t_traj.append(t_i)

    if(not (x_i-nearestNode.x)**2 + (y_i-nearestNode.y)**2 == 0): # only add node if vehicle can move
        G.newNode(x_i,y_i,theta_i,v_i,omega_i,t_i,a,alpha,x_traj,y_traj,theta_traj,v_traj,omega_traj,t_traj,nearestNode)
    else:
        pass

# RRT algorithm with vehicle dynamics and volume
def rrt():
    # Iterate over 5 problems given
    for j in range(5):

        # Initialize graph
        G = Graph(x_start[j],y_start[j],theta_start[j])

        # Limit runtime
        maxNodes = 20000
        for i in range(maxNodes):

            # Chose a random point in the plane
            u_x = np.random.uniform(-50.0,50.0,1)
            u_y = np.random.uniform(-50.0,50.0,1)

            # Choose the node closest to the random point
            nearNode = G.findNearestNode(u_x,u_y)

            # Start at the node, generate a random feasible trajectory, and store it in the graph
            newTrajectory(G,nearNode,eps[j])
            
            # End the search if the goal region has been reached
            if reachedGoal(G.nodes[-1].x,G.nodes[-1].y,goal_region[j][0],goal_region[j][1],goal_region[j][2]):
                print("Goal reached!")
                G.definePath(G.nodes[-1]) # Keep track of nodes and edges in feasbile path
                break
        
        # Warning statement if goal was not reached
        if(not reachedGoal(G.nodes[-1].x,G.nodes[-1].y,goal_region[j][0],goal_region[j][1],goal_region[j][2])):
            print("Timed out: goal not reached! \n")
        
        # File names for RRT search
        outputTree = "outputs/output_tree_" + str(j+1) + ".txt"
        outputPath = "outputs/output_path_" + str(j+1) + ".txt"
        outputTraj = "outputs/output_traj_"+ str(j+1) + ".txt"

        G.saveSearchTraj(outputTraj)                    # Save paths explored
        G.saveSearchTree(outputTree)                    # Save tree in form specified by problem set
        G.savePath(outputPath,x_start[j],y_start[j])    # Save feasible path found

# The important part
def main():
    rrt()

if __name__ == '__main__':
    main()