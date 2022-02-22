**Project 1**

Sorry for the program cannot actually run, I had a hard time trying to overcome some obstacles while debugging.<br />

Since I had previous experiences of implementing A* search in Python and used for solving maze games, the basic logics and principles of how the algorithm works should be mostly fine.<br />

But some of the C++ specific features and errors get in the way for me to further debug the program and see how it works out, like I can get different errors while using different compilers or IDEs, the most frustrating one is a "malloc(): invalid size(unsorted)" when I tried to push a data struct in to a vector.<br />

But still tried my best to implement all the features that should be working if I can figure out what exactly is causing these errors to happen.<br />

Feature list:
	- Basic A* search use heuristic of Euclid distance or Manhattan distance (line 180)
	- Optimal path
	- Support "All", "Expanded" and "Standard" modes (line 256)
	- Handle no solution case (line 228)
	- Can handle large and complex maze based on my previous experiences? Not sure about this time.
	- Support diagonal movement, using constraint check when generating available frontiers. (line 74)
	- Handle tile with a cost of 0 (line 60)
	- Allow user to choose different heuristics (line 140)
 