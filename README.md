# PathFinder
This project implements a graph-based pathfinding tool using Dijkstra's algorithm in C. The program allows you to initialize a graph, add or delete nodes, and find the shortest path between nodes or through an intermediate node.
Usage
Initialize the Graph
The graph is automatically initialized with predefined nodes and edges when the program starts. You can view the initial graph structure by selecting the "Print Graph" option from the menu.

Print the Graph
To print the current state of the graph, select the "Print Graph" option from the menu. The output will display each node and its connected nodes with the corresponding edge weights.

Add a Node
You can add a new node to the graph by selecting the "Add Node" option. The program will prompt you to enter the number of connections and the destination nodes with their respective distances.

Delete a Node
To delete a node, select the "Delete Node" option and enter the node number you wish to remove. The program will remove the node and all its connections from the graph.

Find Shortest Path Between Nodes
To find the shortest path between two nodes, select the "Dijkstra's Algorithm between Nodes" option. Enter the start and end nodes to calculate and display the shortest path and its distance.

Find Shortest Path Through an Intermediate Node
If you need to find the shortest path between two nodes while passing through an intermediate node, select the "Dijkstra's Algorithm through an Intermediate Node" option. Enter the start node, intermediate node, and end node to get the desired path and distance.

Features
Graph Representation: The graph is represented using an adjacency list, which efficiently stores edges and their weights.
Dijkstra's Algorithm: Two implementations of Dijkstra's algorithm are available: one for finding the shortest path between two nodes and another for finding the shortest path through an intermediate node.
Dynamic Graph Manipulation: You can dynamically add and delete nodes in the graph.
User-Friendly Menu: The program provides an interactive menu to navigate through its various features.
Contributing
Contributions are welcome! If you find a bug or want to improve the program, feel free to fork the repository and submit a pull request.

License
This project is open-source and available under the MIT License.

