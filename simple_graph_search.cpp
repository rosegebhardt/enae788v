/*
Copyright 2018, Michael Otte

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/* 
Edited by Rose Gebhardt: deleted random walk code, changed "expand" function to "A_star" function, changed comments to match new code
*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <string>

#include "heap.h"
#include "heap.cpp"
#include "graph.h"

// Start and goal nodes given in each problem
int startNodeIDs [] = {9,88,30,38,38};  
int goalNodeIDs [] = {5,9,95,502,5002};

// Admissable heuristic for A*
double heuristic(Node* thisNode, Node* goalNode)
{
  // Euclidean distance between current node and goal node
  double costGo = sqrt(pow((thisNode->x - goalNode->x),2) + pow((thisNode->y - goalNode->y),2));
  return costGo;
}

void A_star(Heap<Node> &H, Node* thisNode, Node* goalNode)
{
  // Iterate over neighboring nodes
  for(int n = 0; n < thisNode->numOutgoingEdges; n++)
  {
    // Define neighboring node and corresponding edge
    Edge* thisEdge = thisNode->outgoingEdges[n];
    Node* neighborNode = thisEdge->endNode;

    // Ignore if there is a better path
    if(neighborNode->status == 0 || neighborNode->costStart > thisNode->costStart + thisEdge->edgeCost)  
    {
      // Define parent node
      neighborNode->parentNode = thisNode;

      // Update cost to start (before or after defining neighbor key?!?!?)
      neighborNode->costStart = thisNode->costStart+thisEdge->edgeCost;

      // Define key from cost to start and cost to go heuristic
      double neighborKey = neighborNode->costStart + heuristic(neighborNode,goalNode);

      // Add neighbor to heap
      H.addToHeap(neighborNode, neighborKey);

      // Change neighbor status to open
      neighborNode->status = 1;
    }
  }

  // Once all neighbors are visited, move to closed list
  thisNode->status = 2;
}

int main()
{
  // Iterate over each 
  for(int index = 0; index < 5; index++)
  {
    // there has to be a better way of doing this....
    string inputNodes = "inputs/nodes_" + to_string(index+1) + ".txt";
    string inputEdges = "inputs/edges_with_costs_" + to_string(index+1) + ".txt";
    const char *InputNodes = inputNodes.c_str();
    const char *InputEdges = inputEdges.c_str();

    Graph G;
    G.readGraphFromFiles(InputNodes,InputEdges);

    // Initialize heap with room for 100 nodes
    Heap<Node> H(100);

    // Define start and end nodes (subtract one because c++ indexs at zero)
    int startNodeIndex = startNodeIDs[index] - 1;
    int goalNodeIndex = goalNodeIDs[index] - 1;

    // Pointers to the start and end nodes
    Node* startNode = &G.nodes[startNodeIndex];
    Node* goalNode = &G.nodes[goalNodeIndex];

    // what should the first key value be?
    H.addToHeap(startNode, heuristic(startNode, goalNode));
    startNode->status = 1;    // now the start node is in the open list

    // Implement A*
    while(H.topHeap() != NULL)
    {
      // Look at next node in priority queue
      Node* thisNode = H.popHeap();
      A_star(H, thisNode, goalNode);

      // End search if goal is reached
      if (thisNode->id == goalNode->id)
      {
        printf("SUCCESS: Path found!\n");
        break;
      }
    }

    // Give warning if no path is found
    if(H.topHeap() == NULL)
    {
      printf("FAILURE: No path found!\n");
    }

    // Save path and search tree to files
    string outputPath = "outputs/output_path_" + to_string(index+1) + ".txt";
    string outputTree = "outputs/search_tree_" + to_string(index+1) + ".txt";
    const char *OutputPath = outputPath.c_str();
    const char *OutputTree = outputTree.c_str();

    G.savePathToFile(OutputPath, goalNode);
    G.saveSearchTreeToFile(OutputTree);
  }

  return 0;
}
