#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <random>
#include "graph.h"
using namespace std;

// Uniform random number generator in [-50,50]
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> uniform_dis(-50.0, 50.0);

// Start and goal nodes given in each problem
double eps [5] = {10,5,5,2,1};  
double x_start [5] = {0,27,45,-16,39};
double y_start [5] = {0,30,-45,10,5};
double goal_region [5][3] = {{-38,20,10},{-48,20,10},{-45,45,15},{18,-45,5},{38,-8,3}};

// See if point has reached the goal
bool reachedGoal(double x, double y, double x_goal, double y_goal, double r_goal)
{
    if(pow((x - x_goal),2) + pow((y - y_goal),2) < pow(r_goal,2))
    {
        return true;
    }
    else 
    {
        return false;
    }
}

// Check that no part of line segment intersects an obstacle
bool isFree(double v_x, double v_y, double w_x, double w_y, const char* obsFilename)
{
    fstream obsFile(obsFilename);
    string obsLine;

    // Get number of obstacles
    getline(obsFile,obsLine);
    int numObs = stoi(obsLine);

    // Create obstacle array
    double obs[numObs][3];

    // Store values line by line (each line is one obstacle)
    for(int i=0; i<numObs; i++)
    {
        getline(obsFile, obsLine);
        stringstream obsVal(obsLine);
        string value;
        for(int j=0; j<3; j++)
        {
            getline(obsVal, value, ',');
            obs[i][j] = stod(value);
        }
    }
    obsFile.close();

    // Check if the point (x,y) is in each obstacle
    for(int i=0; i<numObs; i++)
    {
        // Find minimum distance of obstacle center to line segment using dot product
        // Reference: https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/
        double vw_x = w_x - v_x;
        double vw_y = w_y - v_y;
        double wc_x = obs[i][0] - w_x;
        double wc_y = obs[i][1] - w_y;
        double vc_x = obs[i][0] - v_x;
        double vc_y = obs[i][1] - v_y;

        double vw_dot_wc = vw_x*wc_x + vw_y*wc_y;
        double vw_dot_vc = vw_x*vc_x + vw_y*vc_y;

        double minDistance;

        if(vw_dot_wc > 0)
        {
            minDistance = sqrt(pow(wc_x,2) + pow(wc_y,2));
        }
        else if(vw_dot_vc < 0)
        {
            minDistance = sqrt(pow(vc_x,2) + pow(vc_y,2));
        }
        else
        {
            double vwLength = sqrt(pow(vw_x,2) + pow(vw_y,2));
            minDistance = abs(vw_x*vc_y - vw_y*vc_x)/vwLength;
        }

        // Check if minimum distance to segment is smaller than the obstacle radius
        if(minDistance<obs[i][2])
        {
            return false;
        }
    }

    return true;
}

// Obstacle detection function (only checks endpoints)
bool isFreeLite(double x, double y, const char* obsFilename)
{
    fstream obsFile(obsFilename);
    string obsLine;

    // Get number of obstacles
    getline(obsFile,obsLine);
    int numObs = stoi(obsLine);

    // Create obstacle array
    double obs[numObs][3];

    // Store values line by line (each line is one obstacle)
    for(int i=0; i<numObs; i++)
    {
        getline(obsFile, obsLine);
        stringstream obsVal(obsLine);
        string value;
        for(int j=0; j<3; j++)
        {
            getline(obsVal, value, ',');
            obs[i][j] = stod(value);
        }
    }
    obsFile.close();

    // Check if the point (x,y) is in each obstacle
    for(int i=0; i<numObs; i++)
    {
        if(pow((x - obs[i][0]),2) + pow((y - obs[i][1]),2) < pow(obs[i][2],2))
        {
            // The point intersects an obstacle
            return false;
        }
    }

    // The point does not intersect any of the obstacles
    return true;
}

int main()
{
    for(int j = 0; j < 5; j++)
    {
        // Maximum number of attempts before timing out
        int maxNodes = 5000;

        // Allocate space for u,v,w
        double u_x, u_y, v_x, v_y, w_x, w_y;

        // Initialize graph with start node
        Graph G(x_start[j],y_start[j]);

        // Iterate until maximum nuber of attempts reached
        for(int i = 0; i<maxNodes; i++)
        {
            // Choose point from uniform random distribution
            u_x = uniform_dis(gen); double u_y = uniform_dis(gen);

            // Get node closest to random point
            Node* nearNode = G.nearestNode(u_x,u_y);
            v_x = nearNode->x; v_y = nearNode->y;

            // Episilon step toward the random point (if epsilon is sufficiently small)
            double length = sqrt(pow((u_x - v_x), 2) + pow((u_y - v_y), 2));
            double ratio;
            if (eps[j]<length)
            {
                ratio = eps[j]/length;
            }
            else
            {
                ratio = 1.0;
            }
            w_x = ratio*(u_x-v_x) + v_x;
            w_y = ratio*(u_y-v_y) + v_y;

            // // works approximately
            // if(isFreeLite(w_x,w_y,"obstacles.txt"))
            // {
            //     G.newNode(w_x,w_y,nearNode);
            // }

            // Check that path segment does not intersect obstacles
            if(isFree(v_x,v_y,w_x,w_y,"obstacles.txt"))
            {
                // Add new point to graph
                G.newNode(w_x,w_y,nearNode);

                // End while loop if goal region is reached
                if(reachedGoal(w_x,w_y,goal_region[j][0],goal_region[j][1],goal_region[j][2]))
                {
                    cout << i << "\n";
                    cout << "Goal reached! \n";
                    break;
                }
            }
        }

        // Warning message if goal region is not reached in alotted time
        if(!reachedGoal(w_x,w_y,goal_region[j][0],goal_region[j][1],goal_region[j][2]))
        {
            cout << "Timed out: goal not reached! \n";
        }

        // Save path and search tree to files
        string outputPath = "outputs/output_path_" + to_string(j+1) + ".txt";
        string outputTree = "outputs/output_tree_" + to_string(j+1) + ".txt";
        const char *OutputPath = outputPath.c_str();
        const char *OutputTree = outputTree.c_str();

        // Clear output files if they already exist
        remove(OutputPath);
        remove(OutputTree);

        // Write path and search tree to text files
        G.savePath(OutputPath,G.nodes.back());
        G.saveSearchTree(OutputTree);
    }

    return 0;
}