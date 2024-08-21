#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>

// Structure to represent a node in the adjacency list
struct Node {
    int destination;
    double weight;
    struct Node* next;
};

// Structure to represent the graph
struct Graph {
    int numNodes;
    struct Node** adjacencyList;
};

// Function to add an edge to the graph
void addEdge(struct Graph* graph, int src, int dest, double weight) {
    struct Node* newNode = (struct Node*)malloc(sizeof(struct Node));
    newNode->destination = dest;
    newNode->weight = weight;
    newNode->next = graph->adjacencyList[src];
    graph->adjacencyList[src] = newNode;
}

// Function to initialize the graph based on the provided node map
void initializeGraph(struct Graph* graph) {
    int i, maxNode = 0;

    // Find the maximum node value from the edges
    double edges[][3] = {
        {1, 2, 1.9}, {2, 1, 1.9}, {1, 4, 2.5}, {4, 1, 2.5},
        {2, 3, 1.8}, {3, 2, 1.8}, {3, 5, 5.6}, {5, 3, 5.6},
        {3, 6, 2.2}, {6, 3, 2.2}, {3, 7, 2.7}, {7, 3, 2.7},
        {4, 5, 2.4}, {5, 4, 2.4}, {5, 8, 2.9}, {8, 5, 2.9},
        {7, 8, 3.9}, {8, 7, 3.9}
    };

    for (i = 0; i < sizeof(edges) / sizeof(edges[0]); ++i) {
        if (edges[i][0] > maxNode) {
            maxNode = edges[i][0];
        }
        if (edges[i][1] > maxNode) {
            maxNode = edges[i][1];
        }
    }

    // Set the number of nodes in the graph
    graph->numNodes = maxNode;

    // Initialize adjacency list
    graph->adjacencyList = (struct Node*)malloc((maxNode + 1) * sizeof(struct Node));
    for (i = 0; i <= maxNode; ++i) {
        graph->adjacencyList[i] = NULL;
    }

    // Add edges to the graph based on the provided node map
    for (i = 0; i < sizeof(edges) / sizeof(edges[0]); ++i) {
        addEdge(graph, edges[i][0], edges[i][1], edges[i][2]);
    }
}

// Function to print the graph nodes
void printGraph(struct Graph* graph) {
    int i;
    for (i = 1; i <= graph->numNodes; ++i) {
        if (graph->adjacencyList[i] != NULL) {
            struct Node* currentNode = graph->adjacencyList[i];
            printf("Node %d:", i);
            while (currentNode != NULL) {
                printf(" -> %d (%.1f cms)", currentNode->destination, currentNode->weight);
                currentNode = currentNode->next;
            }
            printf("\n");
        } 
    }
}


// Function to add a new node to the graph
void addNode(struct Graph* graph) {
    int newNodeNumber = graph->numNodes + 1;

    printf("Enter the number of nodes connected to the new node: ");
    int numConnections;
    scanf("%d", &numConnections);

    double (*edges)[2] = malloc(numConnections * sizeof(*edges));

    for (int i = 0; i < numConnections; ++i) {
        printf("Enter the destination node and distance for connection %d (e.g., 2 1.5): ", i + 1);
        scanf("%lf %lf", &edges[i][0], &edges[i][1]);
    }

    // Dynamically allocate memory for the new adjacencyList array
    struct Node** newAdjList = malloc((newNodeNumber + 1) * sizeof(struct Node*));

    // Copy the existing adjacencyList to the newAdjList
    for (int i = 0; i <= graph->numNodes; ++i) {
        newAdjList[i] = graph->adjacencyList[i];
    }

    // Initialize the adjacencyList for the new node
    newAdjList[newNodeNumber] = NULL;

    // Add the new node to the graph
    graph->adjacencyList = newAdjList;

    // Add connections to the new node
    for (int i = 0; i < numConnections; ++i) {
        int destination = (int)edges[i][0];
        double weight = edges[i][1];
        addEdge(graph, newNodeNumber, destination, weight);
        addEdge(graph, destination, newNodeNumber, weight);
    }

    printf("Node %d added to the graph.\n", newNodeNumber);
    graph->numNodes = newNodeNumber;

    free(edges);
}
int minDistance(double dist[], int visited[], int numNodes) {
    double min = DBL_MAX;
    int minIndex;

    for (int v = 1; v <= numNodes; ++v) {
        if (visited[v] == 0 && dist[v] <= min) {
            min = dist[v];
            minIndex = v;
        }
    }

    return minIndex;
}
void printShortestPath(double dist[], int numNodes, int src, int dest, int* predecessor) {
    printf("\nShortest path from Node %d to Node %d: %.1f cms\t Path: ", src, dest, dist[dest]);
    int j = dest;
    printf("%d", j);
    while (predecessor[j] != -1) {
        printf(" <- %d", predecessor[j]);
        j = predecessor[j];
    }
    printf("\n");
}

void dijkstraBetweenNodes(struct Graph* graph, int startNode, int endNode) {
    int numNodes = graph->numNodes;
    double* dist = (double*)malloc((numNodes + 1) * sizeof(double));
    int* visited = (int*)malloc((numNodes + 1) * sizeof(int));
    int* predecessor = (int*)malloc((numNodes + 1) * sizeof(int));

    for (int i = 1; i <= numNodes; ++i) {
        dist[i] = DBL_MAX;
        visited[i] = 0;
        predecessor[i] = -1;
    }

    dist[startNode] = 0;

    for (int count = 1; count <= numNodes - 1; ++count) {
        int u = minDistance(dist, visited, numNodes);
        visited[u] = 1;

        struct Node* currentNode = graph->adjacencyList[u];
        while (currentNode != NULL) {
            int v = currentNode->destination;
            if (!visited[v] && dist[u] != DBL_MAX && dist[u] + currentNode->weight < dist[v]) {
                dist[v] = dist[u] + currentNode->weight;
                predecessor[v] = u; // Store predecessor node
            }
            currentNode = currentNode->next;
        }
    }

    if (dist[endNode] != DBL_MAX) {
        printShortestPath(dist, numNodes, startNode, endNode, predecessor);
    } else {
        printf("\nNo path from Node %d to Node %d\n", startNode, endNode);
    }

    free(dist);
    free(visited);
    free(predecessor);
}

void dijkstraThroughNode(struct Graph* graph, int src, int intermediateNode, int dest) {
    int numNodes = graph->numNodes;
    double* distFromSrc = (double*)malloc((numNodes + 1) * sizeof(double));
    double* distToDest = (double*)malloc((numNodes + 1) * sizeof(double));
    int* visitedFromSrc = (int*)malloc((numNodes + 1) * sizeof(int));
    int* visitedToDest = (int*)malloc((numNodes + 1) * sizeof(int));
    int* predecessorFromSrc = (int*)malloc((numNodes + 1) * sizeof(int));
    int* predecessorToDest = (int*)malloc((numNodes + 1) * sizeof(int));

    for (int i = 1; i <= numNodes; ++i) {
        distFromSrc[i] = DBL_MAX;
        distToDest[i] = DBL_MAX;
        visitedFromSrc[i] = 0;
        visitedToDest[i] = 0;
        predecessorFromSrc[i] = -1;
        predecessorToDest[i] = -1;
    }

    distFromSrc[src] = 0;
    distToDest[dest] = 0;

    // Dijkstra's algorithm from source to intermediate node
    for (int count = 1; count <= numNodes - 1; ++count) {
        int u = minDistance(distFromSrc, visitedFromSrc, numNodes);
        visitedFromSrc[u] = 1;

        struct Node* currentNode = graph->adjacencyList[u];
        while (currentNode != NULL) {
            int v = currentNode->destination;
            if (!visitedFromSrc[v] && distFromSrc[u] != DBL_MAX && distFromSrc[u] + currentNode->weight < distFromSrc[v]) {
                distFromSrc[v] = distFromSrc[u] + currentNode->weight;
                predecessorFromSrc[v] = u; // Store predecessor node
            }
            currentNode = currentNode->next;
        }
    }

    // Dijkstra's algorithm from intermediate node to destination
    for (int count = 1; count <= numNodes - 1; ++count) {
        int u = minDistance(distToDest, visitedToDest, numNodes);
        visitedToDest[u] = 1;

        struct Node* currentNode = graph->adjacencyList[u];
        while (currentNode != NULL) {
            int v = currentNode->destination;
            if (!visitedToDest[v] && distToDest[u] != DBL_MAX && distToDest[u] + currentNode->weight < distToDest[v]) {
                distToDest[v] = distToDest[u] + currentNode->weight;
                predecessorToDest[v] = u; // Store predecessor node
            }
            currentNode = currentNode->next;
        }
    }

    double shortestPathThroughIntermediateNode = distFromSrc[intermediateNode] + distToDest[intermediateNode];

    printf("\nShortest path from Node %d to Node %d through Node %d: %.1f cms\t \nPath: ", src, dest, intermediateNode, shortestPathThroughIntermediateNode);
    printf("\n");
    // Print path from source to intermediate node
    int i = intermediateNode;
    printf("%d", i);
    while (predecessorFromSrc[i] != -1) {
        printf(" <- %d", predecessorFromSrc[i]);
        i = predecessorFromSrc[i];
    }
    printf("\n");

    // Print path from intermediate node to destination
    i = intermediateNode;
    printf("%d", i);
    while (predecessorToDest[i] != -1) {
        printf(" -> %d", predecessorToDest[i]);
        i = predecessorToDest[i];
    }
    printf("\n");

    free(distFromSrc);
    free(distToDest);
    free(visitedFromSrc);
    free(visitedToDest);
    free(predecessorFromSrc);
    free(predecessorToDest);
}
// Function to delete a node from the graph
void deleteNode(struct Graph* graph, int nodeToDelete) {
    // Remove all edges connected to the node
    for (int i = 1; i <= graph->numNodes; ++i) {
        struct Node* current = graph->adjacencyList[i];
        struct Node* prev = NULL;

        while (current != NULL) {
            if (current->destination == nodeToDelete) {
                if (prev == NULL) {
                    // Node to delete is the first in the list
                    graph->adjacencyList[i] = current->next;
                    free(current);
                    current = graph->adjacencyList[i];
                } else {
                    // Node to delete is in the middle or end of the list
                    prev->next = current->next;
                    free(current);
                    current = prev->next;
                }
            } else {
                prev = current;
                current = current->next;
            }
        }
    }

    // Remove the node itself
    free(graph->adjacencyList[nodeToDelete]);
    graph->adjacencyList[nodeToDelete] = NULL;
}



// ... (Previous code remains unchanged)

int main() {
    struct Graph graph;

    // Initialize the graph
    initializeGraph(&graph);
    
    int choice;
    int startNode, endNode, intermediateNode, nodeToDelete;
    
    do {
        printf("\nMenu:");
        printf("\n1. Print Graph");
        printf("\n2. Add Node");
        printf("\n3. Delete Node");
        printf("\n4. Dijkstra's Algorithm between Nodes");
        printf("\n5. Dijkstra's Algorithm through an Intermediate Node");
        printf("\n6. Exit");
        printf("\nEnter your choice: ");
        scanf("%d", &choice);

        switch (choice) {
            case 1:
                printf("\nPrinting the Graph:\n");
                printGraph(&graph);
                break;

            case 2:
                printf("\nAdding a Node:\n");
                addNode(&graph);
                break;

            case 3:
                printf("\nEnter the node to delete: ");
                if (scanf("%d", &nodeToDelete) != 1 || nodeToDelete < 1 || nodeToDelete > graph.numNodes) {
                    printf("Error: Invalid node number.\n");
                } else {
                    deleteNode(&graph, nodeToDelete);
                    printf("Node %d deleted from the graph.\n", nodeToDelete);
                }
                break;

            case 4:
                printf("\nEnter start node for Dijkstra's Algorithm: ");
                if (scanf("%d", &startNode) != 1 || startNode < 1 || startNode > graph.numNodes) {
                    printf("Error: Invalid start node number.\n");
                } else {
                    printf("Enter end node for Dijkstra's Algorithm: ");
                    if (scanf("%d", &endNode) != 1 || endNode < 1 || endNode > graph.numNodes) {
                        printf("Error: Invalid end node number.\n");
                    } else {
                        dijkstraBetweenNodes(&graph, startNode, endNode);
                    }
                }
                break;

            case 5:
                printf("\nEnter start node for Dijkstra's Algorithm: ");
                if (scanf("%d", &startNode) != 1 || startNode < 1 || startNode > graph.numNodes) {
                    printf("Error: Invalid start node number.\n");
                } else {
                    printf("Enter intermediate node: ");
                    if (scanf("%d", &intermediateNode) != 1 || intermediateNode < 1 || intermediateNode > graph.numNodes) {
                        printf("Error: Invalid intermediate node number.\n");
                    } else {
                        printf("Enter end node for Dijkstra's Algorithm: ");
                        if (scanf("%d", &endNode) != 1 || endNode < 1 || endNode > graph.numNodes) {
                            printf("Error: Invalid end node number.\n");
                        } else {
                            dijkstraThroughNode(&graph, startNode, intermediateNode, endNode);
                        }
                    }
                }
                break;

            case 6:
                printf("\nExiting the program.\n");
                break;

            default:
                printf("\nInvalid choice. Please enter a valid option.\n");
                break;
        }
    } while (choice != 6);

    return 0;
}