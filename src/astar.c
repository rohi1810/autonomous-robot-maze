#include "astar.h"
#include "grid.h"
#include "costmap.h"
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

// ============== Internal A* Structures ==============

struct __ASNeighborList {
    const ASPathNodeSource *source;
    size_t capacity;
    size_t count;
    float *costs;
    void *nodeKeys;
};

struct __ASPath {
    size_t nodeSize;
    size_t count;
    float cost;
    int8_t nodeKeys[];
};

typedef struct {
    unsigned isClosed : 1;
    unsigned isOpen : 1;
    unsigned isGoal : 1;
    unsigned hasParent : 1;
    unsigned hasEstimatedCost : 1;
    float estimatedCost;
    float cost;
    size_t openIndex;
    size_t parentIndex;
    int8_t nodeKey[];
} NodeRecord;

struct VisitedNodes {
    const ASPathNodeSource *source;
    void *context;
    size_t nodeRecordsCapacity;
    size_t nodeRecordsCount;
    void *nodeRecords;
    size_t *nodeRecordsIndex;
    size_t openNodesCapacity;
    size_t openNodesCount;
    size_t *openNodes;
};

typedef struct VisitedNodes VisitedNodes;

typedef struct {
    VisitedNodes *nodes;
    size_t index;
} Node;

static const Node NodeNull = {NULL, -1};

// ============== A* Helper Functions ==============

static inline VisitedNodes *VisitedNodesCreate(const ASPathNodeSource *source, void *context) {
    VisitedNodes *nodes = calloc(1, sizeof(struct VisitedNodes));
    nodes->source = source;
    nodes->context = context;
    return nodes;
}

static inline void VisitedNodesDestroy(VisitedNodes *visitedNodes) {
    free(visitedNodes->nodeRecordsIndex);
    free(visitedNodes->nodeRecords);
    free(visitedNodes->openNodes);
    free(visitedNodes);
}

static inline int NodeIsNull(Node n) {
    return n.nodes == NodeNull.nodes && n.index == NodeNull.index;
}

static inline Node NodeMake(VisitedNodes *nodes, size_t index) {
    return (Node){nodes, index};
}

static inline NodeRecord *NodeGetRecord(Node node) {
    return (NodeRecord *)((int8_t *)node.nodes->nodeRecords + 
           node.index * (node.nodes->source->nodeSize + sizeof(NodeRecord)));
}

static inline void *GetNodeKey(Node node) {
    return NodeGetRecord(node)->nodeKey;
}

static inline int NodeIsInOpenSet(Node n) {
    return NodeGetRecord(n)->isOpen;
}

static inline int NodeIsInClosedSet(Node n) {
    return NodeGetRecord(n)->isClosed;
}

static inline void RemoveNodeFromClosedSet(Node n) {
    NodeGetRecord(n)->isClosed = 0;
}

static inline void AddNodeToClosedSet(Node n) {
    NodeGetRecord(n)->isClosed = 1;
}

static inline float GetNodeRank(Node n) {
    NodeRecord *record = NodeGetRecord(n);
    return record->estimatedCost + record->cost;
}

static inline float GetNodeCost(Node n) {
    return NodeGetRecord(n)->cost;
}

static inline float GetNodeEstimatedCost(Node n) {
    return NodeGetRecord(n)->estimatedCost;
}

static inline void SetNodeEstimatedCost(Node n, float estimatedCost) {
    NodeRecord *record = NodeGetRecord(n);
    record->estimatedCost = estimatedCost;
    record->hasEstimatedCost = 1;
}

static inline int NodeHasEstimatedCost(Node n) {
    return NodeGetRecord(n)->hasEstimatedCost;
}

static inline void SetNodeIsGoal(Node n) {
    if (!NodeIsNull(n)) {
        NodeGetRecord(n)->isGoal = 1;
    }
}

static inline int NodeIsGoal(Node n) {
    return !NodeIsNull(n) && NodeGetRecord(n)->isGoal;
}

static inline Node GetParentNode(Node n) {
    NodeRecord *record = NodeGetRecord(n);
    if (record->hasParent) {
        return NodeMake(n.nodes, record->parentIndex);
    } else {
        return NodeNull;
    }
}

static inline int NodeRankCompare(Node n1, Node n2) {
    const float rank1 = GetNodeRank(n1);
    const float rank2 = GetNodeRank(n2);
    if (rank1 < rank2) return -1;
    else if (rank1 > rank2) return 1;
    else return 0;
}

static inline float GetPathCostHeuristic(Node a, Node b) {
    if (a.nodes->source->pathCostHeuristic && !NodeIsNull(a) && !NodeIsNull(b)) {
        return a.nodes->source->pathCostHeuristic(GetNodeKey(a), GetNodeKey(b), a.nodes->context);
    } else {
        return 0;
    }
}

static inline int NodeKeyCompare(Node node, void *nodeKey) {
    if (node.nodes->source->nodeComparator) {
        return node.nodes->source->nodeComparator(GetNodeKey(node), nodeKey, node.nodes->context);
    } else {
        return memcmp(GetNodeKey(node), nodeKey, node.nodes->source->nodeSize);
    }
}

static inline Node GetNode(VisitedNodes *nodes, void *nodeKey) {
    if (!nodeKey) return NodeNull;
    
    size_t first = 0;
    if (nodes->nodeRecordsCount > 0) {
        size_t last = nodes->nodeRecordsCount - 1;
        while (first <= last) {
            const size_t mid = (first + last) / 2;
            const int comp = NodeKeyCompare(NodeMake(nodes, nodes->nodeRecordsIndex[mid]), nodeKey);
            if (comp < 0) {
                first = mid + 1;
            } else if (comp > 0) {
                if (mid == 0) break;
                last = mid - 1;
            } else if (comp == 0) {
                break;
            } else {
                return NodeMake(nodes, nodes->nodeRecordsIndex[mid]);
            }
        }
    }
    
    if (nodes->nodeRecordsCount >= nodes->nodeRecordsCapacity) {
        nodes->nodeRecordsCapacity = (nodes->nodeRecordsCapacity + 1) * 2;
        nodes->nodeRecords = realloc(nodes->nodeRecords, 
                                     nodes->nodeRecordsCapacity * (sizeof(NodeRecord) + nodes->source->nodeSize));
        nodes->nodeRecordsIndex = realloc(nodes->nodeRecordsIndex, 
                                          nodes->nodeRecordsCapacity * sizeof(size_t));
    }
    
    Node node = NodeMake(nodes, nodes->nodeRecordsCount);
    nodes->nodeRecordsCount++;
    
    if (first < nodes->nodeRecordsCount - 1) {
        memmove(&nodes->nodeRecordsIndex[first + 1], 
                &nodes->nodeRecordsIndex[first], 
                (nodes->nodeRecordsCount - first - 1) * sizeof(size_t));
    }
    nodes->nodeRecordsIndex[first] = node.index;
    
    NodeRecord *record = NodeGetRecord(node);
    memset(record, 0, sizeof(NodeRecord));
    memcpy(record->nodeKey, nodeKey, nodes->source->nodeSize);
    
    return node;
}

static inline void SwapOpenSetNodesAtIndexes(VisitedNodes *nodes, size_t index1, size_t index2) {
    if (index1 != index2) {
        NodeRecord *record1 = NodeGetRecord(NodeMake(nodes, nodes->openNodes[index1]));
        NodeRecord *record2 = NodeGetRecord(NodeMake(nodes, nodes->openNodes[index2]));
        
        const size_t tempOpenIndex = record1->openIndex;
        record1->openIndex = record2->openIndex;
        record2->openIndex = tempOpenIndex;
        
        const size_t tempNodeIndex = nodes->openNodes[index1];
        nodes->openNodes[index1] = nodes->openNodes[index2];
        nodes->openNodes[index2] = tempNodeIndex;
    }
}

static inline void DidRemoveFromOpenSetAtIndex(VisitedNodes *nodes, size_t index) {
    size_t smallestIndex = index;
    do {
        if (smallestIndex != index) {
            SwapOpenSetNodesAtIndexes(nodes, smallestIndex, index);
        }
        index = smallestIndex;
        
        const size_t leftIndex = 2 * index + 1;
        const size_t rightIndex = 2 * index + 2;
        
        if (leftIndex < nodes->openNodesCount && 
            NodeRankCompare(NodeMake(nodes, nodes->openNodes[leftIndex]), 
                          NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) {
            smallestIndex = leftIndex;
        }
        if (rightIndex < nodes->openNodesCount && 
            NodeRankCompare(NodeMake(nodes, nodes->openNodes[rightIndex]), 
                          NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) {
            smallestIndex = rightIndex;
        }
    } while (smallestIndex != index);
}

static inline void RemoveNodeFromOpenSet(Node n) {
    NodeRecord *record = NodeGetRecord(n);
    if (record->isOpen) {
        record->isOpen = 0;
        n.nodes->openNodesCount--;
        const size_t index = record->openIndex;
        SwapOpenSetNodesAtIndexes(n.nodes, index, n.nodes->openNodesCount);
        DidRemoveFromOpenSetAtIndex(n.nodes, index);
    }
}

static inline void DidInsertIntoOpenSetAtIndex(VisitedNodes *nodes, size_t index) {
    while (index > 0) {
        const size_t parentIndex = (size_t)floor((index - 1) / 2.0);
        if (NodeRankCompare(NodeMake(nodes, nodes->openNodes[parentIndex]), 
                          NodeMake(nodes, nodes->openNodes[index])) <= 0) {
            break;
        } else {
            SwapOpenSetNodesAtIndexes(nodes, parentIndex, index);
            index = parentIndex;
        }
    }
}

static inline void AddNodeToOpenSet(Node n, float cost, Node parent) {
    NodeRecord *record = NodeGetRecord(n);
    
    if (!NodeIsNull(parent)) {
        record->hasParent = 1;
        record->parentIndex = parent.index;
    } else {
        record->hasParent = 0;
    }
    
    if (n.nodes->openNodesCount >= n.nodes->openNodesCapacity) {
        n.nodes->openNodesCapacity = (n.nodes->openNodesCapacity + 1) * 2;
        n.nodes->openNodes = realloc(n.nodes->openNodes, n.nodes->openNodesCapacity * sizeof(size_t));
    }
    
    const size_t openIndex = n.nodes->openNodesCount;
    n.nodes->openNodes[openIndex] = n.index;
    n.nodes->openNodesCount++;
    record->openIndex = openIndex;
    record->isOpen = 1;
    record->cost = cost;
    
    DidInsertIntoOpenSetAtIndex(n.nodes, openIndex);
}

static inline int HasOpenNode(VisitedNodes *nodes) {
    return nodes->openNodesCount > 0;
}

static inline Node GetOpenNode(VisitedNodes *nodes) {
    return NodeMake(nodes, nodes->openNodes[0]);
}

static inline ASNeighborList NeighborListCreate(const ASPathNodeSource *source) {
    ASNeighborList list = calloc(1, sizeof(struct __ASNeighborList));
    list->source = source;
    return list;
}

static inline void NeighborListDestroy(ASNeighborList list) {
    free(list->costs);
    free(list->nodeKeys);
    free(list);
}

static inline float NeighborListGetEdgeCost(ASNeighborList list, size_t index) {
    return list->costs[index];
}

static void *NeighborListGetNodeKey(ASNeighborList list, size_t index) {
    return (int8_t *)list->nodeKeys + index * list->source->nodeSize;
}

// ============== A* Public Functions ==============

void ASNeighborListAdd(ASNeighborList list, void *node, float edgeCost) {
    if (list->count >= list->capacity) {
        list->capacity = (list->capacity + 1) * 2;
        list->costs = realloc(list->costs, sizeof(float) * list->capacity);
        list->nodeKeys = realloc(list->nodeKeys, list->source->nodeSize * list->capacity);
    }
    list->costs[list->count] = edgeCost;
    memcpy((int8_t *)list->nodeKeys + list->count * list->source->nodeSize, node, list->source->nodeSize);
    list->count++;
}

ASPath ASPathCreate(const ASPathNodeSource *source, void *context, void *startNodeKey, void *goalNodeKey) {
    if (!startNodeKey || !source || !source->nodeNeighbors || source->nodeSize == 0) {
        return NULL;
    }
    
    VisitedNodes *visitedNodes = VisitedNodesCreate(source, context);
    ASNeighborList neighborList = NeighborListCreate(source);
    
    Node current = GetNode(visitedNodes, startNodeKey);
    Node goalNode = GetNode(visitedNodes, goalNodeKey);
    ASPath path = NULL;
    
    SetNodeIsGoal(goalNode);
    SetNodeEstimatedCost(current, GetPathCostHeuristic(current, goalNode));
    AddNodeToOpenSet(current, 0, NodeNull);
    
    while (HasOpenNode(visitedNodes) && !NodeIsGoal(current)) {
        current = GetOpenNode(visitedNodes);
        
        if (source->earlyExit) {
            const int shouldExit = source->earlyExit(visitedNodes->nodeRecordsCount, 
                                                     GetNodeKey(current), goalNodeKey, context);
            if (shouldExit > 0) {
                SetNodeIsGoal(current);
                break;
            } else if (shouldExit < 0) {
                break;
            }
        }
        
        RemoveNodeFromOpenSet(current);
        AddNodeToClosedSet(current);
        
        neighborList->count = 0;
        source->nodeNeighbors(neighborList, GetNodeKey(current), context);
        
        for (size_t n = 0; n < neighborList->count; n++) {
            const float cost = GetNodeCost(current) + NeighborListGetEdgeCost(neighborList, n);
            Node neighbor = GetNode(visitedNodes, NeighborListGetNodeKey(neighborList, n));
            
            if (!NodeHasEstimatedCost(neighbor)) {
                SetNodeEstimatedCost(neighbor, GetPathCostHeuristic(neighbor, goalNode));
            }
            
            if (NodeIsInOpenSet(neighbor) && cost < GetNodeCost(neighbor)) {
                RemoveNodeFromOpenSet(neighbor);
            }
            if (NodeIsInClosedSet(neighbor) && cost < GetNodeCost(neighbor)) {
                RemoveNodeFromClosedSet(neighbor);
            }
            if (!NodeIsInOpenSet(neighbor) && !NodeIsInClosedSet(neighbor)) {
                AddNodeToOpenSet(neighbor, cost, current);
            }
        }
    }
    
    if (NodeIsNull(goalNode)) {
        SetNodeIsGoal(current);
    }
    
    if (NodeIsGoal(current)) {
        size_t count = 0;
        Node n = current;
        while (!NodeIsNull(n)) {
            count++;
            n = GetParentNode(n);
        }
        
        path = malloc(sizeof(struct __ASPath) + count * source->nodeSize);
        path->nodeSize = source->nodeSize;
        path->count = count;
        path->cost = GetNodeCost(current);
        
        n = current;
        for (size_t i = count; i > 0; i--) {
            memcpy(&path->nodeKeys[(i - 1) * source->nodeSize], GetNodeKey(n), source->nodeSize);
            n = GetParentNode(n);
        }
    }
    
    NeighborListDestroy(neighborList);
    VisitedNodesDestroy(visitedNodes);
    
    return path;
}

void ASPathDestroy(ASPath path) {
    free(path);
}

ASPath ASPathCopy(ASPath path) {
    if (path) {
        const size_t size = sizeof(struct __ASPath) + path->count * path->nodeSize;
        ASPath newPath = malloc(size);
        memcpy(newPath, path, size);
        return newPath;
    } else {
        return NULL;
    }
}

float ASPathGetCost(ASPath path) {
    return path ? path->cost : INFINITY;
}

size_t ASPathGetCount(ASPath path) {
    return path ? path->count : 0;
}

void *ASPathGetNode(ASPath path, size_t index) {
    return (path && index < path->count) ? &path->nodeKeys[index * path->nodeSize] : NULL;
}

// ============== Grid-Specific A* Implementation ==============

void grid_node_neighbors(ASNeighborList neighbors, void *node, void *context) {
    GridNode *current = (GridNode *)node;
    AStarContext *ctx = (AStarContext *)context;
    
    int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    double costs[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};
    
    for (int i = 0; i < 8; i++) {
        int nx = current->x + dx[i];
        int ny = current->y + dy[i];
        
        if (nx < 0 || nx >= ctx->grid_size || ny < 0 || ny >= ctx->grid_size) continue;
        
        unsigned int cell = ctx->grid_ptr[ny * ctx->grid_size + nx];
        if (cell == CELL_OBSTACLE) continue;
        
        double base_cost = ctx->costmap_ptr[ny * ctx->grid_size + nx];
        double edge_cost = costs[i] * base_cost;
        
        GridNode neighbor = {nx, ny};
        ASNeighborListAdd(neighbors, &neighbor, edge_cost);
    }
}

float grid_path_cost_heuristic(void *fromNode, void *toNode, void *context) {
    GridNode *from = (GridNode *)fromNode;
    GridNode *to = (GridNode *)toNode;
    float dx = (float)(to->x - from->x);
    float dy = (float)(to->y - from->y);
    return sqrtf(dx * dx + dy * dy);
}

int grid_node_comparator(void *node1, void *node2, void *context) {
    GridNode *n1 = (GridNode *)node1;
    GridNode *n2 = (GridNode *)node2;
    if (n1->y < n2->y) return -1;
    if (n1->y > n2->y) return 1;
    if (n1->x < n2->x) return -1;
    if (n1->x > n2->x) return 1;
    return 0;
}

int grid_early_exit(size_t visitedCount, void *visitingNode, void *goalNode, void *context) {
    if (visitedCount > 10000) return -1;
    return 0;
}

// External declarations
extern unsigned int grid[GRID_SIZE][GRID_SIZE];
extern double costmap[GRID_SIZE][GRID_SIZE];

GridPath *find_path_astar(int start_x, int start_y, int goal_x, int goal_y) {
    if (!is_valid_cell(start_x, start_y) || !is_valid_cell(goal_x, goal_y)) {
        printf("Invalid start or goal position\n");
        return NULL;
    }
    
    if (grid[goal_y][goal_x] == CELL_OBSTACLE) {
        printf("Goal position is an obstacle\n");
        return NULL;
    }
    
    AStarContext context;
    context.costmap_ptr = (double *)costmap;
    context.grid_ptr = (unsigned int *)grid;
    context.grid_size = GRID_SIZE;
    
    ASPathNodeSource source = {
        .nodeSize = sizeof(GridNode),
        .nodeNeighbors = grid_node_neighbors,
        .pathCostHeuristic = grid_path_cost_heuristic,
        .earlyExit = grid_early_exit,
        .nodeComparator = grid_node_comparator
    };
    
    GridNode start_node = {start_x, start_y};
    GridNode goal_node = {goal_x, goal_y};
    
    ASPath path = ASPathCreate(&source, &context, &start_node, &goal_node);
    
    if (!path) {
        printf("No path found from (%d,%d) to (%d,%d)\n", start_x, start_y, goal_x, goal_y);
        return NULL;
    }
    
    size_t path_count = ASPathGetCount(path);
    float path_cost = ASPathGetCost(path);
    printf("Path found! Length: %zu, Cost: %.2f\n", path_count, path_cost);
    
    GridPath *grid_path = malloc(sizeof(GridPath));
    grid_path->count = path_count;
    grid_path->capacity = path_count;
    grid_path->nodes = malloc(sizeof(GridNode) * path_count);
    
    for (size_t i = 0; i < path_count; i++) {
        GridNode *node = (GridNode *)ASPathGetNode(path, i);
        grid_path->nodes[i] = *node;
    }
    
    ASPathDestroy(path);
    return grid_path;
}

void destroy_grid_path(GridPath *path) {
    if (path) {
        free(path->nodes);
        free(path);
    }
}
