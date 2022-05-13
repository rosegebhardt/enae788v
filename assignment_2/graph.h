#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <iterator> 

using namespace std;

struct Node;    // preallocate struct

// Edge between nodes for 2D unweighted graph
struct Edge 
{ 
    Node* startNode;    // start vertex of directed edge
    Node* endNode;      // end vertex of directed edge

    // Edge constructor
    Edge(Node* startpt, Node* endpt)
    {
        startNode = startpt;
        endNode = endpt;
    }
}; 

// Node for a 2D graph
struct Node
{
    double x;                       // Cartesian x-coordinate
    double y;                       // Cartesian y-corrdinate
    int numOutgoingEdges;           // Number of outgoing edges
    vector<Edge*> outgoingEdges;    // Empty vector of outgoing edges
    Node* parentNode;               // Parent node (will have only one)

    // Node constructor
    Node(double pos_x, double pos_y, Node* nearestNode)
    {
        x = pos_x;
        y = pos_y;
        numOutgoingEdges = 0;
        parentNode = nearestNode;
    }

};

// Data structure to store vertices and edges
class Graph 
{ 
  public:
    int numNodes;           // Number of nodes in graph
    vector<Node*> nodes;    // Vector of nodes
    int numEdges;           // Number of edges in graph
    vector<Edge*> edges;    // Vector of edges

    // Graph constructor
    Graph(double x_start, double y_start)
    {
        numNodes = 1;
        numEdges = 0;
        Node* startNode = new Node(x_start,y_start,nullptr);
        nodes.push_back(startNode);
    }

    // Deleted destructor: may want to add it back later

    // Iterate through nodes and find the one closest to (x,y)
    Node* nearestNode(double x, double y)
    {
        // Keep vector of distances
        vector<double> distances;

        // Store distance from each node in graph
        for (vector<Node*>::iterator it = nodes.begin(); it != nodes.end(); ++it)
        {
            double dist = sqrt(pow((x - (*it)->x), 2) + pow((y - (*it)->y), 2));
            distances.push_back(dist);
        }

        // Return node with smallest distance to (x,y)
        int minElementIndex = min_element(distances.begin(),distances.end()) - distances.begin();
        return nodes[minElementIndex];
    }

    // Add node and corresponding edge to the graph
    void newNode(double x, double y, Node* nearestNode)
    {
        // Create the new node and edge
        Node* newNode = new Node(x,y,nearestNode);
        Edge* newEdge = new Edge(nearestNode,newNode);

        // Change parameters of parent node
        nearestNode->numOutgoingEdges++;
        nearestNode->outgoingEdges.push_back(newEdge);

        // Update number of nodes and edges
        numNodes++;
        numEdges++;

        // Add node and edge to the vectors
        nodes.push_back(newNode);
        edges.push_back(newEdge);
    }

    // Save path to text file (can be read as .csv)
    bool savePath(const char* pathFilename, Node* goalNode)
    {
        // Open a new file
        ofstream pathFile(pathFilename);

        // Start with final node and trace back to start
        Node* thisNode = goalNode;
        while(thisNode != nullptr)
        {
            // Write path node positions to text file
            if(pathFile.is_open())
            {
                pathFile << thisNode->x << ", " << thisNode->y << "\n";
            }
            else cout << "Unable to open file\n";
            thisNode = thisNode->parentNode;
        }
        pathFile.close();

        // Returns true if task completed
        return true;
    }

    // Save search tree to text file (can be read as .csv)
    bool saveSearchTree(const char* treeFilename)
    {
        // Open a new file
        ofstream treeFile(treeFilename);

        // Iterate over the edges vector
        for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); ++it)
        {
            // Write coordinates of start and end to text file
            if(treeFile.is_open())
            {
                treeFile << (*it)->startNode->x << ", " << (*it)->startNode->y << ", " << 
                (*it)->endNode->x << ", " << (*it)->endNode->y << "\n";
            }
            else cout << "Unable to open file\n";
        }
        treeFile.close();

        // Returns true if task completed
        return true;
    }

};