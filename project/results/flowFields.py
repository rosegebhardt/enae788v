import numpy as np
import matplotlib.pyplot as plt
from defineGraph import Graph
from optimize import optimizeTime, optimizeNRG

from wavefrontGraph import OUMGraph
from wavefront import wavefrontPropogation

# System parameters
Vmax = 2; Kappa = 2; r0 = 8

# Velocity field definitions
def uniformVelocity(X,Y,u0):
    U = u0*np.ones(X.shape)
    V = np.zeros(Y.shape)
    return U,V

def cylinderFreeStream(X,Y,u0,r0,Gamma):
    U = u0 - u0*r0**2*(X**2 - Y**2)/((X**2 + Y**2)**2) + Gamma*Y/(X**2 + Y**2)
    V = -2*u0*r0**2*(X*Y)/((X**2 + Y**2)**2) - Gamma*X/(X**2 + Y**2)
    U[X**2 + Y**2 < r0**2] = 1000000
    V[X**2 + Y**2 < r0**2] = 1000000
    return U,V

def karmanVortexStreet(X,Y,a,h,gamma):
    zv = 0 - 1.5*1j
    Z = X + 1j*Y
    dFdz = (1j*gamma/(2*np.pi)) * (1/(np.tan(np.pi*(Z-zv)/a)) - 1/(np.tan(np.pi*(Z-zv-(a/2 + 1j*h))/a)))
    U = np.real(dFdz); V = -np.imag(dFdz)
    return U,V

# Define collision checkers to prevent invalid edges
def uniformCollisionChecker(startNode,endNode):
    return False

# Reference: https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
def cylinderCollisionChecker(startNode,endNode):
    v_x = startNode.x; v_y = startNode.y
    w_x = endNode.x; w_y = endNode.y
    vw_x = w_x - v_x; vw_y = w_y - v_y
    wc_x = 0 - w_x
    wc_y = 0 - w_y
    vc_x = 0 - v_x
    vc_y = 0 - v_y

    vw_dot_wc = vw_x*wc_x + vw_y*wc_y
    vw_dot_vc = vw_x*vc_x + vw_y*vc_y

    if(vw_dot_wc > 0):
        minDistance = np.sqrt(wc_x**2 + wc_y**2)
    elif(vw_dot_vc < 0):
        minDistance = np.sqrt(vc_x**2 + vc_y**2)
    else:
        vwLength = np.sqrt(vw_x**2 + vw_y**2)
        minDistance = np.abs(vw_x*vc_y - vw_y*vc_x)/vwLength

    if(minDistance < r0):
        return True
    else:
        return False

def karmanCollisionChecker(startNode,endNode):
    return False

# Functions for displaying solutions to time and energy optimization
def uniformSolution(X,Y,u0,timePath,nrgPath):
    Z = X + 1j*Y
    F = u0*Z
    Psi = np.imag(F)

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    plt.contourf(X,Y,Psi,75,cmap = 'coolwarm')
    ax.plot(timePath[0,:],timePath[1,:],'k-',linewidth = 2)
    ax.plot(nrgPath[0,:],nrgPath[1,:],'k--',linewidth = 2)
    ax.scatter(timePath[0,0],timePath[1,0],c='black',s=100,marker='x')
    ax.scatter(timePath[0,-1],timePath[1,-1],c='black',s=100,marker='o')
    ax.legend(['Time Optimal Path', 'Energy Optimal Path'],fontsize = 10)
    plt.show()

def cylinderSolution(X,Y,u0,r0,Gamma,timePath,nrgPath):
    Z = X + 1j*Y
    F = u0*(Z + r0**2/Z) + 1j*Gamma*np.log(Z)
    F[X**2 + Y**2 < r0**2] = 0
    Psi = np.imag(F)

    fig, ax = plt.subplots()
    draw_circle = plt.Circle((0,0),r0,color='black')
    ax.set_aspect(1)
    ax.add_artist(draw_circle)
    plt.contourf(X,Y,Psi,50,cmap = 'coolwarm')
    ax.plot(timePath[0,:],timePath[1,:],'k-',linewidth = 2)
    ax.plot(nrgPath[0,:],nrgPath[1,:],'k--',linewidth = 2)
    ax.scatter(timePath[0,0],timePath[1,0],c='black',s=100,marker='x')
    ax.scatter(timePath[0,-1],timePath[1,-1],c='black',s=100,marker='o')
    ax.legend(['Time Optimal Path', 'Energy Optimal Path'],fontsize = 10)
    plt.show()

def karmanSolution(X,Y,a,h,gamma,timePath,nrgPath):
    zv = 0 - 0.75*1j
    Z = X + 1j*Y
    F = 1j*(gamma/(2*np.pi))*(np.log(np.sin(np.pi*(Z-zv)/a))-np.log(np.sin(np.pi*(Z-(a/2 + 1j*h)-zv)/a)))
    Psi = np.imag(F)

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    plt.contourf(X,Y,Psi,75,cmap = 'coolwarm')
    ax.plot(timePath[0,:],timePath[1,:],'k-',linewidth = 2)
    ax.plot(nrgPath[0,:],nrgPath[1,:],'k--',linewidth = 2)
    ax.scatter(timePath[0,0],timePath[1,0],c='black',s=100,marker='x')
    ax.scatter(timePath[0,-1],timePath[1,-1],c='black',s=100,marker='o')
    ax.legend(['Time Optimal Path', 'Energy Optimal Path'],fontsize = 10)
    plt.show()

# Functions for displaying solutions to A* and wavefront methods
def uniformSolutionOUM(X,Y,u0,timePath,pathOUM):
    Z = X + 1j*Y
    F = u0*Z
    Psi = np.imag(F)

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    plt.contourf(X,Y,Psi,50,cmap = 'coolwarm')
    ax.plot(timePath[0,:],timePath[1,:],'k-',linewidth = 2)
    ax.plot(pathOUM[0,:],pathOUM[1,:],'k--',linewidth = 2)
    ax.scatter(pathOUM[0,0],pathOUM[1,0],c='black',s=100,marker='o')
    ax.scatter(pathOUM[0,-1],pathOUM[1,-1],c='black',s=100,marker='x')
    ax.legend(['A* Algorithm Path', 'Wavefront Propogation Path'],fontsize = 10)
    plt.show()

def cylinderSolutionOUM(X,Y,u0,r0,Gamma,timePath,pathOUM):
    Z = X + 1j*Y
    F = u0*(Z + r0**2/Z) + 1j*Gamma*np.log(Z)
    F[X**2 + Y**2 < r0**2] = 0
    Psi = np.imag(F)

    fig, ax = plt.subplots()
    draw_circle = plt.Circle((0,0),r0,color='black')
    ax.set_aspect(1)
    ax.add_artist(draw_circle)
    plt.contourf(X,Y,Psi,50,cmap = 'coolwarm')
    ax.plot(timePath[0,:],timePath[1,:],'k-',linewidth = 2)
    ax.plot(pathOUM[0,:],pathOUM[1,:],'k--',linewidth = 2)
    ax.scatter(pathOUM[0,0],pathOUM[1,0],c='black',s=100,marker='o')
    ax.scatter(pathOUM[0,-1],pathOUM[1,-1],c='black',s=100,marker='x')
    ax.legend(['A* Algorithm Path', 'Wavefront Propogation Path'],fontsize = 10)
    plt.show()

def karmanSolutionOUM(X,Y,a,h,gamma,timePath,pathOUM):
    zv = 0 - 0.75*1j
    Z = X + 1j*Y
    F = 1j*(gamma/(2*np.pi))*(np.log(np.sin(np.pi*(Z-zv)/a))-np.log(np.sin(np.pi*(Z-(a/2 + 1j*h)-zv)/a)))
    Psi = np.imag(F)

    fig, ax = plt.subplots()
    ax.set_aspect(1)
    plt.contourf(X,Y,Psi,75,cmap = 'coolwarm')
    ax.plot(timePath[0,:],timePath[1,:],'k-',linewidth = 2)
    ax.plot(pathOUM[0,:],pathOUM[1,:],'k--',linewidth = 2)
    ax.scatter(pathOUM[0,0],pathOUM[1,0],c='black',s=100,marker='o')
    ax.scatter(pathOUM[0,-1],pathOUM[1,-1],c='black',s=100,marker='x')
    ax.legend(['A* Algorithm Path', 'Wavefront Propogation Path'],fontsize = 10)
    plt.show()


## UNIFORM CALULATIONS

# Define grid
x_min = -20; x_max = 20; x_N = 100
y_min = -20; y_max = 20; y_N = 100
x = np.linspace(x_min,x_max,x_N)
y = np.linspace(y_min,y_max,y_N)
[X,Y] = np.meshgrid(x,y)

# Get velocity field for given parameters
u0 = 1
U_uniform,V_uniform = uniformVelocity(X,Y,u0)

# Define graph
G_uniform = Graph(X,Y,U_uniform,V_uniform,uniformCollisionChecker,Vmax,Kappa)

# Define start and goal locations
startIndices = [80,1]; startNode = G_uniform.nodeArray[startIndices[0]][startIndices[1]]
goalIndices = [20,98]; goalNode = G_uniform.nodeArray[goalIndices[0]][goalIndices[1]]

# Run Djikstra's and A* optimization
optimizeTime(G_uniform,startNode,goalNode)
optimizeNRG(G_uniform,startNode,goalNode)

# Get optimized paths
timePath = G_uniform.timeOptimalPath(goalNode)
nrgPath = G_uniform.nrgOptimalPath(goalNode)

# View solutions
uniformSolution(X,Y,u0,timePath,nrgPath)

## END UNIFORM CALCULATIONS


'''
## CYLINDER CALCULATIONS

# Define grid
x_min = -20; x_max = 20; x_N = 100
y_min = -20; y_max = 20; y_N = 100
x = np.linspace(x_min,x_max,x_N)
y = np.linspace(y_min,y_max,y_N)
[X,Y] = np.meshgrid(x,y)

# Get velocity field for given parameters
u0 = 1; Gamma = 0
cylinderVelocities = cylinderFreeStream(X,Y,u0,r0,Gamma)
U_cylinder = cylinderVelocities[0]; V_cylinder = cylinderVelocities[1]

# Define graph
G_cylinder = Graph(X,Y,U_cylinder,V_cylinder,cylinderCollisionChecker,Vmax,Kappa)

# Define start and goal locations
startIndices = [20,98]; startNode = G_cylinder.nodeArray[startIndices[0]][startIndices[1]]
goalIndices = [20,1]; goalNode = G_cylinder.nodeArray[goalIndices[0]][goalIndices[1]]

# Run Djikstra's and A* optimization
optimizeTime(G_cylinder,startNode,goalNode)
optimizeNRG(G_cylinder,startNode,goalNode)

# Get optimized paths
timePath = G_cylinder.timeOptimalPath(goalNode)
nrgPath = G_cylinder.nrgOptimalPath(goalNode)

# View solutions
cylinderSolution(X,Y,u0,r0,Gamma,timePath,nrgPath)

# END CYLINDER CALCULATIONS
'''

'''
## KARMAN STREET CALCULATIONS

# Define grid
x_min = -6; x_max = 6; x_N = 100
y_min = -4; y_max = 4; y_N = 100
x = np.linspace(x_min,x_max,x_N)
y = np.linspace(y_min,y_max,y_N)
[X,Y] = np.meshgrid(x,y)

# Get velocity field for given parameters
gamma = 3; h = 1.5; a = 3
U_karman,V_karman = karmanVortexStreet(X,Y,a,h,gamma)

# Define graph
G_karman = Graph(X,Y,U_karman,V_karman,karmanCollisionChecker,Vmax,Kappa)

# Define start and goal locations
startIndices = [49,98]; startNode = G_karman.nodeArray[startIndices[0]][startIndices[1]]
goalIndices = [49,1]; goalNode = G_karman.nodeArray[goalIndices[0]][goalIndices[1]]

# Run Djikstra's and A* optimization
optimizeTime(G_karman,startNode,goalNode)
optimizeNRG(G_karman,startNode,goalNode)

# Get optimized paths
karmanTimePath = G_karman.timeOptimalPath(goalNode)
karmanNRGPath = G_karman.nrgOptimalPath(goalNode)

# View solutions
karmanSolution(X,Y,a,h,gamma,karmanTimePath,karmanNRGPath)

## END KARMAN STREET CALCULATIONS
'''

'''
## UNIFORM WAVEFRONT PROPOGATION

# Define grid
x_min = -20; x_max = 20; x_N = 100
y_min = -20; y_max = 20; y_N = 100
x = np.linspace(x_min,x_max,x_N)
y = np.linspace(y_min,y_max,y_N)
[X,Y] = np.meshgrid(x,y)

# Get velocity field for given parameters
u0 = 1
U_uniform,V_uniform = uniformVelocity(X,Y,u0)

# Define graphs
G_uniform = Graph(X,Y,U_uniform,V_uniform,uniformCollisionChecker,Vmax,Kappa)
G_uniformOUM = OUMGraph(X,Y,U_uniform,V_uniform,uniformCollisionChecker,Vmax,Kappa)

# Define start and goal locations
startIndices = [20,1]
goalIndices = [80,98]
startNode = G_uniform.nodeArray[startIndices[0]][startIndices[1]]
goalNode = G_uniform.nodeArray[goalIndices[0]][goalIndices[1]]
startNodeOUM = G_uniformOUM.nodeArray[startIndices[0]][startIndices[1]]
goalNodeOUM = G_uniformOUM.nodeArray[goalIndices[0]][goalIndices[1]]

# Run time optimization algorithms
optimizeTime(G_uniform,startNode,goalNode)
wavefrontPropogation(G_uniformOUM,goalNodeOUM)

# Get optimized paths
timePath = G_uniform.timeOptimalPath(goalNode)
pathOUM = G_uniformOUM.optimalPath(startNodeOUM,goalNodeOUM)

# View solutions
uniformSolutionOUM(X,Y,u0,timePath,pathOUM)

## END KARMAN WAVEFRONT PROPOGATION
'''

'''
## CYLINDER WAVEFRONT PROPOGATION

# Define grid
x_min = -20; x_max = 20; x_N = 100
y_min = -20; y_max = 20; y_N = 100
x = np.linspace(x_min,x_max,x_N)
y = np.linspace(y_min,y_max,y_N)
[X,Y] = np.meshgrid(x,y)

# Get velocity field for given parameters
u0 = 1; Gamma = 0
cylinderVelocities = cylinderFreeStream(X,Y,u0,r0,Gamma)
U_cylinder = cylinderVelocities[0]; V_cylinder = cylinderVelocities[1]

# Define graphs
G_cylinder = Graph(X,Y,U_cylinder,V_cylinder,cylinderCollisionChecker,Vmax,Kappa)
G_cylinderOUM = OUMGraph(X,Y,U_cylinder,V_cylinder,cylinderCollisionChecker,Vmax,Kappa)

# Define start and goal locations
startIndices = [2,98]
goalIndices = [98,2]
startNode = G_cylinder.nodeArray[startIndices[0]][startIndices[1]]
goalNode = G_cylinder.nodeArray[goalIndices[0]][goalIndices[1]]
startNodeOUM = G_cylinderOUM.nodeArray[startIndices[0]][startIndices[1]]
goalNodeOUM = G_cylinderOUM.nodeArray[goalIndices[0]][goalIndices[1]]

# Run time optimization algorithms
optimizeTime(G_cylinder,startNode,goalNode)
wavefrontPropogation(G_cylinderOUM,goalNodeOUM)

# Get optimized paths
timePath = G_cylinder.timeOptimalPath(goalNode)
pathOUM = G_cylinderOUM.optimalPath(startNodeOUM,goalNodeOUM)

# View solutions
cylinderSolutionOUM(X,Y,u0,r0,Gamma,timePath,pathOUM)

## END CYLINDER WAVEFRONT PROPOGATION
'''

'''
## KARMAN WAVEFRONT PROPOGATION

# Define grid
x_min = -6; x_max = 6; x_N = 100
y_min = -4; y_max = 4; y_N = 100
x = np.linspace(x_min,x_max,x_N)
y = np.linspace(y_min,y_max,y_N)
[X,Y] = np.meshgrid(x,y)

# Get velocity field for given parameters
gamma = 3; h = 1.5; a = 3
U_karman,V_karman = karmanVortexStreet(X,Y,a,h,gamma)

# Define graphs
G_karman = Graph(X,Y,U_karman,V_karman,karmanCollisionChecker,Vmax,Kappa)
G_karmanOUM = OUMGraph(X,Y,U_karman,V_karman,karmanCollisionChecker,Vmax,Kappa)

# Define start and goal locations
startIndices = [35,1]
goalIndices = [65,98]
startNode = G_karman.nodeArray[startIndices[0]][startIndices[1]]
goalNode = G_karman.nodeArray[goalIndices[0]][goalIndices[1]]
startNodeOUM = G_karmanOUM.nodeArray[startIndices[0]][startIndices[1]]
goalNodeOUM = G_karmanOUM.nodeArray[goalIndices[0]][goalIndices[1]]

# Run time optimization algorithms
optimizeTime(G_karman,startNode,goalNode)
wavefrontPropogation(G_karmanOUM,goalNodeOUM)

# Get optimized paths
timePath = G_karman.timeOptimalPath(goalNode)
pathOUM = G_karmanOUM.optimalPath(startNodeOUM,goalNodeOUM)

# View solutions
karmanSolutionOUM(X,Y,a,h,gamma,timePath,pathOUM)

## END KARMAN WAVEFRONT PROPOGATION
'''