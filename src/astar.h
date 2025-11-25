#ifndef ASTAR_H
#define ASTAR_H

#include <stddef.h>
#include <stdint.h>

// Forward declarations
typedef struct __ASNeighborList *ASNeighborList;
typedef struct __ASPath *ASPath;

// Path node source configuration
typedef struct {
    size_t nodeSize;
    void (*nodeNeighbors)(ASNeighborList neighbors, void *node, void *context);
    float (*pathCostHeuristic)(void *fromNode, void *toNode, void *context);
    int (*earlyExit)(size_t visitedCount, void *visitingNode, void *goalNode, void *context);
    int (*nodeComparator)(void *node1, void *node2, void *context);
} ASPathNodeSource;

// Grid structures
typedef struct {
    int x;
    int y;
} GridNode;

typedef struct {
    GridNode *nodes;
    int count;
    int capacity;
} GridPath;

typedef struct {
    double *costmap_ptr;
    unsigned int *grid_ptr;
    int grid_size;
} AStarContext;

// Public API
void ASNeighborListAdd(ASNeighborList list, void *node, float edgeCost);
ASPath ASPathCreate(const ASPathNodeSource *source, void *context, void *startNodeKey, void *goalNodeKey);
void ASPathDestroy(ASPath path);
ASPath ASPathCopy(ASPath path);
float ASPathGetCost(ASPath path);
size_t ASPathGetCount(ASPath path);
void *ASPathGetNode(ASPath path, size_t index);

// Grid-specific A* functions
GridPath *find_path_astar(int start_x, int start_y, int goal_x, int goal_y);
void destroy_grid_path(GridPath *path);

// A* callbacks for grid navigation
void grid_node_neighbors(ASNeighborList neighbors, void *node, void *context);
float grid_path_cost_heuristic(void *fromNode, void *toNode, void *context);
int grid_node_comparator(void *node1, void *node2, void *context);
int grid_early_exit(size_t visitedCount, void *visitingNode, void *goalNode, void *context);

#endif // ASTAR_H
