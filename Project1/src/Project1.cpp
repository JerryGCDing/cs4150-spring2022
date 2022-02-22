#include "Project1.h"
#include <chrono>
#include <cmath>

using namespace std;

// tile position with its cost
struct Tile
{
    int x;
    int y;
    int cost;
};

// format - DiscoverdNode{discovered node[?x, ?y], parent node[?x, ?y], action cost[?c]}
struct DiscoveredNode
{
    // default as not found
    int x;
    int y;
    int pX;
    int pY;
    int cost;
};

// format - PathNode{node[?x, ?y], parent node[?x, ?y], action cost[?c], a star key[?f]}
struct PathNode
{
    int x;
    int y;
    int pX;
    int pY;
    int cost;
    double key;
};

// check if the explored path is goal state
bool isGoal(const int x, const int y, const int goal[2])
{
    return x == goal[0] && y == goal[1];
}

// get all the avaliable frontiers from the current position with action cost
vector<Tile> getFrontiers(const vector<vector<char>>& inputMap, int x, int y)
{
    vector<Tile> outputFrontier;
    // -1 - obstacle OR out of boundary, [1...9] - tile with action cost
    vector<Tile> surroundingTiles;

    // available movement actions
    int action[] = { -1, 0, 1 };
    // iterate through all surrounding tiles
    for (int indX = 0; indX < 3; indX++) {
        for (int indY = 0; indY < 3; indY++) {
            if (indX != 1 || indY != 1) {
                int l_x = x + action[indX];
                int l_y = y + action[indY];
                if (l_x > 0 && l_y > 0 && l_x < inputMap.size() && l_y < inputMap[0].size()) {
                    char frontier = inputMap.at(l_x).at(l_y);
                    if (48 <= frontier && frontier <= 57) {
                        int val = frontier % 48;
                        Tile temp = { l_x, l_y, val };
                        surroundingTiles.push_back(temp);
                    }
                }
                else {
                    Tile temp = { l_x, l_y, -1 };
                    surroundingTiles.push_back(temp);
                }
            }
        }
    }

    int fourDirection[] = { 1, 3, 4, 6 };
    Tile iTile = surroundingTiles.at(fourDirection[0]);
    Tile jTile = surroundingTiles.at(fourDirection[1]);
    // 1, 3
    if (iTile.cost != -1) {
        outputFrontier.push_back(iTile);
    }
    // satisfy diagonal movement
    if (iTile.cost != -1 || jTile.cost != -1) {
        Tile temp = surroundingTiles.at(0);
        if (temp.cost != -1) {
            outputFrontier.push_back(temp);
        }
    }
    // 1, 4
    jTile = surroundingTiles.at(fourDirection[2]);
    if (jTile.cost != -1) {
        outputFrontier.push_back(jTile);
    }
    if (iTile.cost != -1 || jTile.cost != -1) {
        Tile temp = surroundingTiles.at(2);
        if (temp.cost != -1) {
            outputFrontier.push_back(temp);
        }
    }
    // 4, 6
    iTile = surroundingTiles.at(fourDirection[3]);
    if (iTile.cost != -1) {
        outputFrontier.push_back(iTile);
    }
    if (iTile.cost != -1 || jTile.cost != -1) {
        Tile temp = surroundingTiles.at(7);
        if (temp.cost != -1) {
            outputFrontier.push_back(temp);
        }
    }
    // 3, 6
    jTile = surroundingTiles.at(fourDirection[1]);
    if (jTile.cost != -1) {
        outputFrontier.push_back(jTile);
    }
    if (iTile.cost != -1 || jTile.cost != -1) {
        Tile temp = surroundingTiles.at(5);
        if (temp.cost != -1) {
            outputFrontier.push_back(temp);
        }
    }

    return outputFrontier;
}

// Find if the given position is already discovered
DiscoveredNode findInDiscovered(const vector<DiscoveredNode>& discovered, int* currentPos)
{
    for (auto d : discovered) {
        DiscoveredNode pos = d;
        if (pos.x == currentPos[0] && pos.y == currentPos[1]) {
            return pos;
        }
    }

    DiscoveredNode notFound{ -1, -1, -1, -1, -1 };
    return notFound;
}

// Insert the new node to the proper position in the queue
void insertToQueue(vector<PathNode>& pathQueue, DiscoveredNode currentState, int* destination, eHeuristic heuristic)
{
    int dx = currentState.x - destination[0];
    int dy = currentState.y - destination[1];
    int heu;
    PathNode outputNode;
    if (heuristic == StraightLine) {
        // euclid distance
        heu = pow(dx, 2) + pow(dy, 2);

        double f = currentState.cost + pow(heu, 0.5);
        outputNode = { currentState.x, currentState.y, currentState.pX, currentState.pY, currentState.cost, f };
    }
    else {
        // manhattan distance
        heu = abs(dx) + abs(dy);

        double f = currentState.cost + heu;
        outputNode = { currentState.x, currentState.y, currentState.pX, currentState.pY, currentState.cost, f };
        // initial case
        if (pathQueue.size() == 0) {
            pathQueue.push_back(outputNode);
        }
    }

    // initial case
    if (pathQueue.size() == 0) {
        pathQueue.push_back(outputNode);
    }

    // insertion
    int j = pathQueue.size() - 2;
    while (j >= 0 && outputNode.key > pathQueue[j].key)
    {
        pathQueue[j + 1] = pathQueue[j];
        j--;
    }
    pathQueue[j + 1] = outputNode;
}

void PlanPath(const vector<vector<char>>& inputMap, int start[2], int destination[2], eMode mode, eHeuristic heuristic)
{
    // Start the timer
    using namespace chrono;
    steady_clock::time_point clock_begin = steady_clock::now();

    // YOUR CODE STARTS HERE
    // Plan the path
    // vector stores discovered nodes
    // format - vector{discovered node[?x, ?y], parent node[?x, ?y], action cost[?c]}
    vector<DiscoveredNode> discovered;
    // priority queue for path to explore
    // format - vector{node[?x, ?y], parent node[?x, ?y], action cost[?c], heuristic[?h]}
    vector<PathNode> pathQueue;
    // starting point
    DiscoveredNode startState = { start[0], start[1], -1, -1, 0 };
    discovered.push_back(startState);
    PathNode currentState = { startState.x, startState.y, -1, -1, 0, 0 };

    // until the final goal is found
    while (!isGoal(currentState.x, currentState.y, destination)) {
        vector<Tile> frontiers = getFrontiers(inputMap, currentState.x, currentState.y);
        int pastCost = currentState.cost;
        // iterate through the frontier
        for (auto f : frontiers) {
            int node[] = { f.x, f.y };
            int actionCost = pastCost + f.cost;
            // stored information in discovered
            DiscoveredNode explored = findInDiscovered(discovered, node);
            // check if the frontier is already discovered
            if (explored.cost == -1) {
                DiscoveredNode tempNode = { node[0], node[1], currentState.x, currentState.y, actionCost };
                insertToQueue(pathQueue, tempNode, destination, heuristic);
                discovered.push_back(tempNode);
            }
            // update the discovered node if cost is lower
            else if (explored.cost > actionCost) {
                DiscoveredNode tempNode = { node[0], node[1], currentState.x, currentState.y, actionCost };
                insertToQueue(pathQueue, tempNode, destination, heuristic);
                // update parent node
                explored.pX = currentState.pX;
                explored.pY = currentState.pY;
                // update cost
                explored.cost = actionCost;
            }
        }

        // check if have valid choices remaining
        if (pathQueue.size() != 0) {
            currentState = pathQueue.back();
            pathQueue.pop_back();
        }
        else {
            break;
        }
    }


    // LEAVE THESE LINES
    // At this point you should have found a path (or determined that there isn't one), but
    // not yet copied that path onto the output map.

    // Stop the timer
    steady_clock::time_point clock_end = steady_clock::now();
    steady_clock::duration time_span = clock_end - clock_begin;
    double seconds = double(time_span.count()) * steady_clock::period::num / steady_clock::period::den;

    cout << endl;
    cout << "It took " << seconds << " seconds.";
    cout << endl;

    // Annotate & print the output map
    vector<vector<char>> outputMap = inputMap;

    // YOUR CODE RESUMES HERE
    // Annotate the output map with your path, along with expanded nodes and fringe nodes 
    switch (mode) {
    case All:
        for (PathNode p : pathQueue) {
            outputMap[p.x][p.y] = 't';
        }
    case Expanded:
        for (DiscoveredNode d : discovered) {
            outputMap[d.x][d.y] = 'e';
        }
    default:
        DiscoveredNode temp;
        temp = findInDiscovered(discovered, destination);
        while (temp.x != start[0] && temp.y != start[1]) {
            outputMap[temp.x][temp.y] = '+';
            int parent[] = { temp.pX, temp.pY };
            temp = findInDiscovered(discovered, parent);
        }
        outputMap[start[0]][start[1]] = 's';
        outputMap[destination[0]][destination[1]] = 'd';
    }
    // (if the mode calls for them).

    cout << endl;
    cout << "the output map is:";
    cout << endl;
    PrintMap(outputMap);
}

int main()
{
    vector<vector<char>> inputMap;
    string loc;
    string modeType, heuristicType;
    int start[2], end[2];

    cout << "Enter the location of the file:";
    cin >> loc;
    readMap(loc, inputMap);

    cout << endl;
    cout << "the input map is:\n";
    PrintMap(inputMap);

    cout << endl;
    cout << "Enter the mode:";
    cin >> modeType;
    eMode mode = convertMode(modeType);

    cout << endl;
    cout << "Enter the heuristic:";
    cin >> heuristicType;
    eHeuristic heuristic = convertHeuristic(heuristicType);

    cout << endl;
    cout << "Start x coordinate:";
    cin >> start[0];
    cout << "Start y coordinate:";
    cin >> start[1];

    cout << endl;
    cout << "End x coordinate:";
    cin >> end[0];
    cout << "End y coordinate:";
    cin >> end[1];

    PlanPath(inputMap, start, end, mode, heuristic);
}
