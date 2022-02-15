#include "Project1.h"
#include <chrono>

using namespace std;

void PlanPath(const vector<vector<char>>& inputMap, int start[2], int destination[2], eMode mode, eHeuristic heuristic)
{
    // Start the timer
    using namespace chrono;
    steady_clock::time_point clock_begin = steady_clock::now();

    // YOUR CODE STARTS HERE
    // Plan the path
    
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
    // (if the mode calls for them).

    cout << endl;
    cout << "the output map is:";
    cout << endl;
    PrintMap(inputMap);
}

int main()
{
    vector<vector<char>> inputMap;
    string loc;
    string modeType,heuristicType;
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
    cout << "Start coordinate:";
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
