#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;

void static readMap(const std::string& filename, std::vector<std::vector<char>>& PathMap)
{
    // Reading in is a pain, because to get natural PathMap[x][y] reference
    // into the map, I need to flip things around from the order in which 
    // they'll come from the  file.  My outer vector is a vector of columns,
    // and the inner vectors are read in from last entry (highest y, at the 
    // top of the map visually) to first entry (y=0, at the bottom of the map).
    // So, when I read in a row (starting at the top, highest y) I'm creating 
    // a vector inside the outer vector for each entry, and then putting the 
    // new character at the front of that vector.
    std::ifstream file(filename);
    std::string input;
    while (file >> input)
    {
        size_t x = 0;
        for (char c : input)
        {
            if (PathMap.size() <= x)
            {
                PathMap.push_back(std::vector<char>());
            }
            PathMap[x].insert(PathMap[x].begin(), c);
            ++x;
        }
    }

    //To check if there are equal number of values in each row.
    size_t length = PathMap[0].size();
    for (size_t i = 1; i < PathMap.size(); i++) {
        if (length != PathMap[i].size()) {
            printf("Error in input file.");
            PathMap.clear();
            return;
        }
    }
}

void static PrintMap(const std::vector<std::vector<char>>& PathMap) {
    if (PathMap.empty())
        return;

    const int xSize = (int)PathMap.size();
    const int ySize = (int)PathMap[0].size();

    for (int y = ySize - 1; y >= 0; --y)
    {
        for (int x = 0; x < xSize; ++x)
        {
            std::cout << PathMap[x][y];
        }
        std::cout << endl;
    }
}

enum eMode { Standard, Expanded, All };
enum eHeuristic { StraightLine, Manhattan };

eMode convertMode(const std::string str)
{
    if (str == "Standard")
    {
        return Standard;
    }
    else if (str == "Expanded") {
        return Expanded;
    }

    else if (str == "All") {
        return All;
    }
    else throw("illegal value of mode");
}

eHeuristic convertHeuristic(const std::string str)
{
    if (str == "StraightLine")
    {
        return StraightLine;
    }
    else if (str == "Manhattan")
    {
        return Manhattan;
    }

    else throw("Illegal value of heuristic");
}