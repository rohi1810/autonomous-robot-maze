
#include <webots/robot.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>
#include <webots/display.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <webots/range_finder.h> 

// ============== A* IMPLEMENTATION (from AStar.h/AStar.c) ==============
typedef struct __ASNeighborList *ASNeighborList;
typedef struct __ASPath *ASPath;

typedef struct {
    size_t  nodeSize;
    void    (*nodeNeighbors)(ASNeighborList neighbors, void *node, void *context);
    float   (*pathCostHeuristic)(void *fromNode, void *toNode, void *context);
    int     (*earlyExit)(size_t visitedCount, void *visitingNode, void *goalNode, void *context);
    int     (*nodeComparator)(void *node1, void *node2, void *context);
} ASPathNodeSource;

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
    unsigned isClosed:1;
    unsigned isOpen:1;
    unsigned isGoal:1;
    unsigned hasParent:1;
    unsigned hasEstimatedCost:1;
    float estimatedCost;
    float cost;
    size_t openIndex;
    size_t parentIndex;
    int8_t nodeKey[];
} NodeRecord;

struct __VisitedNodes {
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
typedef struct __VisitedNodes *VisitedNodes;

typedef struct {
    VisitedNodes nodes;
    size_t index;
} Node;

static const Node NodeNull = {NULL, -1};

// A* Helper Functions
static inline VisitedNodes VisitedNodesCreate(const ASPathNodeSource *source, void *context) {
    VisitedNodes nodes = calloc(1, sizeof(struct __VisitedNodes));
    nodes->source = source;
    nodes->context = context;
    return nodes;
}

static inline void VisitedNodesDestroy(VisitedNodes visitedNodes) {
    free(visitedNodes->nodeRecordsIndex);
    free(visitedNodes->nodeRecords);
    free(visitedNodes->openNodes);
    free(visitedNodes);
}

static inline int NodeIsNull(Node n) {
    return (n.nodes == NodeNull.nodes) && (n.index == NodeNull.index);
}

static inline Node NodeMake(VisitedNodes nodes, size_t index) {
    return (Node){nodes, index};
}

static inline NodeRecord *NodeGetRecord(Node node) {
    uint8_t *base = (uint8_t*)node.nodes->nodeRecords;
    size_t stride = sizeof(NodeRecord) + node.nodes->source->nodeSize;
    return (NodeRecord*)(base + node.index * stride);
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



static inline Node GetNode(VisitedNodes nodes, void *nodeKey) {
    if (!nodeKey) return NodeNull;

    size_t first = 0;
    if (nodes->nodeRecordsCount > 0) {
        size_t last = nodes->nodeRecordsCount - 1;
        while (first <= last) {
            const size_t mid = (first + last) / 2;
            const int comp = NodeKeyCompare(NodeMake(nodes, nodes->nodeRecordsIndex[mid]), nodeKey);
            if (comp < 0) {
                first = mid + 1;
            } else if (comp > 0 && mid > 0) {
                last = mid - 1;
            } else if (comp > 0) {
                break;
            } else {
                return NodeMake(nodes, nodes->nodeRecordsIndex[mid]);
            }
        }
    }

    // Ensure capacity for a NEW node
    if (nodes->nodeRecordsCount == nodes->nodeRecordsCapacity) {
        const size_t newCap = 1 + (nodes->nodeRecordsCapacity * 2);
        void *newRecords = realloc(nodes->nodeRecords,
            newCap * (sizeof(NodeRecord) + nodes->source->nodeSize));
        size_t *newIndex = realloc(nodes->nodeRecordsIndex, newCap * sizeof(size_t));

        if (!newRecords || !newIndex) {
            // allocation failed: keep old pointers valid
            free(newRecords);
            free(newIndex);
            return NodeNull;
        }

        nodes->nodeRecords = newRecords;
        nodes->nodeRecordsIndex = newIndex;
        nodes->nodeRecordsCapacity = newCap;
    }

    // Insert index into sorted index array
    const size_t used = nodes->nodeRecordsCount;  // count BEFORE increment

    if (first < used) {
        memmove(&nodes->nodeRecordsIndex[first + 1],
                &nodes->nodeRecordsIndex[first],
                (used - first) * sizeof(size_t));
    }

    Node node = NodeMake(nodes, nodes->nodeRecordsCount);
    nodes->nodeRecordsIndex[first] = node.index;
    nodes->nodeRecordsCount++;

    // Write record
    NodeRecord *record = NodeGetRecord(node);
    memset(record, 0, sizeof(NodeRecord));
    memcpy(record->nodeKey, nodeKey, nodes->source->nodeSize);

    return node;
}





static inline void SwapOpenSetNodesAtIndexes(VisitedNodes nodes, size_t index1, size_t index2) {
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

static inline void DidRemoveFromOpenSetAtIndex(VisitedNodes nodes, size_t index) {
    size_t smallestIndex = index;
    do {
        if (smallestIndex != index) {
            SwapOpenSetNodesAtIndexes(nodes, smallestIndex, index);
            index = smallestIndex;
        }
        const size_t leftIndex = (2 * index) + 1;
        const size_t rightIndex = (2 * index) + 2;
        
        if (leftIndex < nodes->openNodesCount && NodeRankCompare(NodeMake(nodes, nodes->openNodes[leftIndex]), NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) {
            smallestIndex = leftIndex;
        }
        
        if (rightIndex < nodes->openNodesCount && NodeRankCompare(NodeMake(nodes, nodes->openNodes[rightIndex]), NodeMake(nodes, nodes->openNodes[smallestIndex])) < 0) {
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

static inline void DidInsertIntoOpenSetAtIndex(VisitedNodes nodes, size_t index) {
    while (index > 0) {
        const size_t parentIndex = floorf((index-1) / 2);
        
        if (NodeRankCompare(NodeMake(nodes, nodes->openNodes[parentIndex]), NodeMake(nodes, nodes->openNodes[index])) < 0) {
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
    
    if (n.nodes->openNodesCount == n.nodes->openNodesCapacity) {
        n.nodes->openNodesCapacity = 1 + (n.nodes->openNodesCapacity * 2);
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

static inline int HasOpenNode(VisitedNodes nodes) {
    return nodes->openNodesCount > 0;
}

static inline Node GetOpenNode(VisitedNodes nodes) {
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
    return list->nodeKeys + (index * list->source->nodeSize);
}

// A* Public Functions
void ASNeighborListAdd(ASNeighborList list, void *node, float edgeCost) {
    if (list->count == list->capacity) {
        list->capacity = 1 + (list->capacity * 2);
        list->costs = realloc(list->costs, sizeof(float) * list->capacity);
        list->nodeKeys = realloc(list->nodeKeys, list->source->nodeSize * list->capacity);
    }
    list->costs[list->count] = edgeCost;
    memcpy(list->nodeKeys + (list->count * list->source->nodeSize), node, list->source->nodeSize);
    list->count++;
}

ASPath ASPathCreate(const ASPathNodeSource *source, void *context, void *startNodeKey, void *goalNodeKey) {
    if (!startNodeKey || !source || !source->nodeNeighbors || source->nodeSize == 0) {
        return NULL;
    }
    
    VisitedNodes visitedNodes = VisitedNodesCreate(source, context);
    ASNeighborList neighborList = NeighborListCreate(source);
    Node current = GetNode(visitedNodes, startNodeKey);
    Node goalNode = GetNode(visitedNodes, goalNodeKey);
    ASPath path = NULL;
    
    SetNodeIsGoal(goalNode);
    SetNodeEstimatedCost(current, GetPathCostHeuristic(current, goalNode));
    AddNodeToOpenSet(current, 0, NodeNull);
    
    while (HasOpenNode(visitedNodes) && !NodeIsGoal((current = GetOpenNode(visitedNodes)))) {
        if (source->earlyExit) {
            const int shouldExit = source->earlyExit(visitedNodes->nodeRecordsCount, GetNodeKey(current), goalNodeKey, context);
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
        
        for (size_t n=0; n<neighborList->count; n++) {
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
        
        path = malloc(sizeof(struct __ASPath) + (count * source->nodeSize));
        path->nodeSize = source->nodeSize;
        path->count = count;
        path->cost = GetNodeCost(current);
        
        n = current;
        for (size_t i=count; i>0; i--) {
            memcpy(path->nodeKeys + ((i - 1) * source->nodeSize), GetNodeKey(n), source->nodeSize);
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
        const size_t size = sizeof(struct __ASPath) + (path->count * path->nodeSize);
        ASPath newPath = malloc(size);
        memcpy(newPath, path, size);
        return newPath;
    } else {
        return NULL;
    }
}

float ASPathGetCost(ASPath path) {
    return path? path->cost : INFINITY;
}

size_t ASPathGetCount(ASPath path) {
    return path? path->count : 0;
}

void *ASPathGetNode(ASPath path, size_t index) {
    return (path && index < path->count)? (path->nodeKeys + (index * path->nodeSize)) : NULL;
}







































































































































// ============== WEBOTS ROBOT CONTROLLER ==============


// Configuration
#define TIME_STEP 32
#define GRID_SIZE 500
#define GRID_RESOLUTION 0.02
#define DISPLAY_WIDTH 500
#define DISPLAY_HEIGHT 500
#define STARTUP_CLEAR_STEPS  30   // ~12*32ms = 0.38s (tune 8..30)
#define DISPLAY_SCALE 3.0

// Grid cell types
#define CELL_UNKNOWN 0
#define CELL_FREE 1
#define CELL_OBSTACLE 2
#define CELL_ROBOT 3
#define CELL_BLUE_PILLAR 4
#define CELL_YELLOW_PILLAR 5

// Temporal filtering thresholds
#define OBSTACLE_THRESHOLD 3
#define FREE_THRESHOLD 2
#define COUNTER_DECAY 1

#define NEARBY_RADIUS_INNER 20
#define NEARBY_RADIUS_OUTER 30

#define MAX_QUEUE 10000
#define MIN_BLOB_SIZE 5
#define OBSTACLE_SHARPNESS 2

// Cost Map
#define OBSTACLE_COST 600
#define FREE_COST 1
#define UNKNOWN_COST 30
#define INFLATION_RADIUS 6

// Frontiers
#define MAX_FRONTIERS 150
#define MIN_FRONTIER_SIZE 10
#define MIN_FRONTIER_CLEARANCE 5
#define FRONTIER_SAFETY_CHECK_RADIUS 5

// Colors
#define COLOR_UNKNOWN 0x404040
#define COLOR_FREE 0xC8C8C8
#define COLOR_OBSTACLE 0x000000
#define COLOR_ROBOT 0x9100FF
#define COLOR_BACKGROUND 0x404040
#define COLOR_BLUE_PILLAR   0x0000FF
#define COLOR_YELLOW_PILLAR 0x30FF00



// --- simple "recently failed frontier" cache ---
#define FAIL_CACHE_MAX 200
#define FAIL_TTL_STEPS 400   
// ~800*32ms ≈ 25s. Tune.


// PID Gain Values
#define KP_ROTATE 2.5
#define KI_ROTATE 0.0001 
#define KD_ROTATE 0.0001

#define KP_TRANSLATE 0.10
#define KD_TRANSLATE 0


#define DRIVE_DIST_THRESH 5
#define DRIVE_ANGLE_THRESH 0.8

#define LIDAR_MAX_RANGE_FACTOR 3.0


#define SLEW_ACC 0.8
#define SHOULD_INTEGRATE_LIDAR_RADIANS 0.6
#define SHOULD_INTEGRATE_LIDAR_MAX_PITCH_RAD  (2.0 * M_PI / 180.0)  // 5.0 deg
#define SHOULD_INTEGRATE_LIDAR_PITCH_RADIANS 0.1
#define WAYPOINT_TOO_CLOSE_DIST 5



// IR Safety Configuration
#define IR_SAFETY_THRESHOLD 0.15  // Stop if any sensor detects obstacle closer than this (in meters)
#define IR_CRITICAL_THRESHOLD 0.08  // Emergency stop threshold

// --- Recovery behavior after emergency stop ---
#define BACKUP_TIME_SEC        2.0     // how long to reverse
#define BACKUP_WHEEL_SPEED     2.0     // wheel rad/s (slow)
#define FRONT_CLEAR_RESUME_M   0.1    // must be > critical to resume

#define NAV_REPLAN_EVERY 80   // ~1.3s at 32ms



// ===== IR Safety -> Virtual Obstacle Marking =====
#define SAFETY_MARK_ENABLE            1
#define SAFETY_MARK_DIST_M            0.25   // <-- fixed distance ahead (meters)
#define SAFETY_MARK_RADIUS_CELLS      1      // disk radius in grid cells
#define SAFETY_MARK_EVERY_N_TICKS     2      // paint rate (1=every tick, 2=every 2 ticks)





// ===== GREEN DETECTION PARAMS =====
#define GREEN_FOV_HORIZONTAL        1.04
#define GREEN_STEP_PIXELS           30
#define GREEN_BASE_RADIUS_CELLS     1
#define GREEN_DISTANCE_SCALE        1.0
#define GREEN_MIN_DEPTH_M           0.15
#define GREEN_MAX_DEPTH_M           2.5
#define GREEN_ROI_Y0_FRAC           0.35
#define GREEN_ROI_Y1_FRAC           0.90
#define GREEN_CONST_MARK_DIST_M     0.3


// ===== RED DETECTION PARAMS =====
#define RED_FOV_HORIZONTAL        1.04
#define RED_STEP_PIXELS           30
#define RED_DISTANCE_SCALE        1.0
#define RED_MIN_DEPTH_M           0.2
#define RED_MAX_DEPTH_M           2.0
#define RED_SEGMENT_LEN_M         0.35
#define RED_ROI_X0_FRAC           0.4
#define RED_ROI_X1_FRAC           0.6
#define RED_ROI_Y0_FRAC          -16   // as used in your call


// ===== PILLAR-PIXEL SAFETY BYPASS (NO DEPTH / NO DISTANCE) =====
#define PILLAR_BYPASS_ENABLE         1
#define PILLAR_BYPASS_STEP_PIX       5        // sample every N pixels (1=full, 2=fast)
#define PILLAR_BYPASS_BLUE_PIXELS    700      // tune
#define PILLAR_BYPASS_YELLOW_PIXELS  300      // tune

#define PILLAR_MIN_BLUE_PIXELS    500      // tune
#define PILLAR_MIN_YELLOW_PIXELS  300      // tune

// Optional ROI to focus on “close-ish” pillars (bigger in image).
// If you truly want whole image, set 0.0 .. 1.0
#define PILLAR_BYPASS_ROI_Y0_FRAC    0.2
#define PILLAR_BYPASS_ROI_Y1_FRAC    0.7



// Only do FALLBACK when pillar is *very likely* close (big in image and/or IR says close)
#define PILLAR_FALLBACK_ENABLE          1
#define PILLAR_FALLBACK_REQUIRE_IR      1      // 1 = must have IR-close, 0 = IR is optional
#define PILLAR_FALLBACK_IR_MAX_M        0.22   // "close enough" per IR to allow fallback
#define PILLAR_FALLBACK_MIN_CY_FRAC     0.1   // centroid must be in lower part of image (bigger-looking)




#define FALLBACK_DIST_M 0.42










// ================= Adaptive Inflation (Global Params) =================

// enable/disable adaptive behavior
#define ADAPT_INFLATION_ENABLE        1

// window around robot to estimate clearance (cells)
#define ADAPT_CLEAR_WIN_RADIUS        60   // 30 cells * 0.02 = 0.60 m

// sample step inside window (bigger = faster)
#define ADAPT_CLEAR_SAMPLE_STEP       3

// which statistic to use from samples (0..100)
#define ADAPT_CLEAR_PERCENTILE        70   // 25th percentile = "tight-ish" clearance

// clamp distances (cells) used for inflation shaping
#define INFLATION_LETHAL_MIN_CELLS    4
#define INFLATION_LETHAL_MAX_CELLS    5

#define INFLATION_SOFT_MIN_CELLS      8
#define INFLATION_SOFT_MAX_CELLS      14

// map estimated clearance -> radii
#define CLEAR_TIGHT_CELLS             6.0   // <=4 cells means very tight
#define CLEAR_OPEN_CELLS              12.0  // >=18 cells means open

// cost gradient tuning for soft band
#define COST_SCALE_MIN                0.04
#define COST_SCALE_MAX                0.20

// if a cell is within lethal band, we set cost close to obstacle
#define LETHAL_COST                   700.0


#define PILLAR_RADIUS_CELLS  4   // try 2..6


#define STANDOFF 6
#define SEARCH   10



// ====== Structures =======

// State Machine
typedef enum {
    STATE_START,
    STATE_SCAN_COLORS,          // NEW: stop + rotate 360 + mark blue/yellow
    STATE_FRONTIER_EXPLORE,
    STATE_MARKING_PILLARS,
    STATE_NAV_TO_BLUE,
    STATE_NAV_TO_YELLOW,
    STATE_COLLISION,
    STATE_WANDER
} State;



// Point structure
typedef struct {
    int x;
    int y;
} Point;

// Grid Node for A*
typedef struct {
    int x;
    int y;
} GridNode;

// Path structure
typedef struct {
    GridNode* nodes;
    int count;
    int capacity;
} GridPath;

// A* Context
typedef struct {
    double* cost_map_ptr;
    unsigned int* grid_ptr;
    int grid_size;
} AStarContext;

// Frontier structure
typedef struct {
    int x;
    int y;
    int size;
    double score;
} FrontierCentroid;


typedef enum {
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_YELLOW
} TargetColor;

typedef enum {
  MARK_SQUARE,
  MARK_DISK
} MarkShape;

typedef struct {
  bool  valid;
  float cx;      // centroid x in pixels
  float cy;      // centroid y in pixels
  int   count;   // number of matching pixels
} Centroid2D;

typedef struct {
  bool  valid;
  float dist;    // depth/range at centroid (meters)
} DepthAtCentroid;

typedef struct {
  bool  valid;
  int   gx;
  int   gy;
} GridHit;

// Global variables
static unsigned int grid[GRID_SIZE][GRID_SIZE];
static unsigned int obstacle_counter[GRID_SIZE][GRID_SIZE];
static unsigned int free_counter[GRID_SIZE][GRID_SIZE];
static double cost_map[GRID_SIZE][GRID_SIZE];
static unsigned char frontier_edge_map[GRID_SIZE][GRID_SIZE];
static WbDeviceTag display;
static WbDeviceTag lidar;
static WbDeviceTag motors[4];
static WbNodeRef robot_node;
static WbDeviceTag camera_rgb;
static WbDeviceTag camera_depth;

static WbDeviceTag ir_sensors[4];

//static const char* ir_sensor_names[4] = {"fl_range", "fr_range", "rl_range", "rr_range"};
static const char* ir_sensor_names[2] = {"fl_range", "fr_range"};



static bool ir_safety_triggered = false;

// Path following variables
static GridPath* current_path = NULL;
static int current_waypoint = 0;
static int path_update_counter = 0;


static double pid_rotate_integral = 0.0;
static double pid_rotate_prev_error = 0.0;
static double pid_translate_integral = 0.0;
static double pid_translate_prev_error = 0.0;



static int blue_cooldown_ticks = 0;           // countdown in simulation steps
static double scan_speed = 0.5;      // wheel rad/s (tune)

static bool   scan_active = false;
static double scan_prev_theta = 0.0;
static double scan_accumulated = 0.0;   // radians accumulated
static int    yellow_cooldown_ticks = 0;      // call detect every N ticks



// ===== Target flags + cached target cells =====
static bool blue_seen = false;
static bool yellow_seen = false;
static bool blue_reached = false;
static bool yellow_reached = false;

static int blue_target_gx = -1, blue_target_gy = -1;
static int yellow_target_gx = -1, yellow_target_gy = -1;

// replanning throttles (ticks)

static int nav_replan_counter = 0;



typedef struct {
  int x, y;
  int expire_frame;
} FailedFrontier;

static FailedFrontier failed_frontiers[FAIL_CACHE_MAX];
static int failed_frontiers_count = 0;




// ============ INITIALIZATION ===============

void init_grid() {
    memset(grid, CELL_UNKNOWN, sizeof(grid));
    memset(obstacle_counter, 0, sizeof(obstacle_counter));
    memset(free_counter, 0, sizeof(free_counter));
}

void init_motors() {
    motors[0] = wb_robot_get_device("fl_wheel_joint");
    motors[1] = wb_robot_get_device("rl_wheel_joint");
    motors[2] = wb_robot_get_device("fr_wheel_joint");
    motors[3] = wb_robot_get_device("rr_wheel_joint");
    
    for (int i = 0; i < 4; i++) {
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);
    }
}


void init_ir_sensors() {
    for (int i = 0; i < 2; i++) {
        ir_sensors[i] = wb_robot_get_device(ir_sensor_names[i]);
        if (ir_sensors[i]) {
            wb_distance_sensor_enable(ir_sensors[i], TIME_STEP);
            printf("IR sensor %s initialized\n", ir_sensor_names[i]);
        } else {
            printf("Warning: IR sensor %s not found!\n", ir_sensor_names[i]);
        }
    }
}





















// =========== UTILITIES =============

void world_to_grid(double wx, double wy, int* gx, int* gy) {
    *gx = (int)((wx / GRID_RESOLUTION) + GRID_SIZE / 2);
    *gy = (int)((wy / GRID_RESOLUTION) + GRID_SIZE / 2);
}

int is_valid_cell(int x, int y) {
    return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
}

double clamp(double val, double min, double max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}


static bool is_failed_frontier(int x, int y, int frame_count) {
  for (int i = 0; i < failed_frontiers_count; i++) {
    if (failed_frontiers[i].expire_frame <= frame_count) continue;
    if (failed_frontiers[i].x == x && failed_frontiers[i].y == y) return true;
  }
  return false;
}

static void mark_failed_frontier(int x, int y, int frame_count) {
  // reuse expired slot if possible
  for (int i = 0; i < failed_frontiers_count; i++) {
    if (failed_frontiers[i].expire_frame <= frame_count) {
      failed_frontiers[i] = (FailedFrontier){x, y, frame_count + FAIL_TTL_STEPS};
      return;
    }
  }

  if (failed_frontiers_count < FAIL_CACHE_MAX) {
    failed_frontiers[failed_frontiers_count++] =
        (FailedFrontier){x, y, frame_count + FAIL_TTL_STEPS};
  } else {
    // overwrite oldest-ish (simple strategy: slot 0)
    failed_frontiers[0] = (FailedFrontier){x, y, frame_count + FAIL_TTL_STEPS};
  }
}


static inline double lerp(double a, double b, double t) {
  return a + (b - a) * t;
}

static inline double clamp01(double t) {
  if (t < 0.0) return 0.0;
  if (t > 1.0) return 1.0;
  return t;
}

// small insertion sort for <= a few thousand samples
static inline void sort_small_double(double *a, int n) {
  for (int i = 1; i < n; i++) {
    double v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
}

static inline double percentile_sorted(const double *a, int n, double pct01) {
  if (n <= 0) return 0.0;
  if (pct01 <= 0.0) return a[0];
  if (pct01 >= 1.0) return a[n - 1];
  double idx = pct01 * (double)(n - 1);
  int i0 = (int)floor(idx);
  int i1 = i0 + 1;
  if (i1 >= n) return a[n - 1];
  double t = idx - (double)i0;
  return a[i0] * (1.0 - t) + a[i1] * t;
}









// ================= SAFETY MANAGER =================

typedef enum {
  SAFETY_OK = 0,
  SAFETY_WARN,
  SAFETY_EMERGENCY_STOP,
  SAFETY_RECOVER_BACKUP,
  SAFETY_RECOVER_WAIT_CLEAR
} SafetyState;

typedef struct {
  SafetyState state;

  // last sensor read (for display/logging)
  double min_dist_m;
  int    min_idx;

  // outputs to motion controller
  double v_scale;     // 0..1
  double w_scale;     // 0..1
  bool   hard_stop;   // if true: force motor stop and skip planning

  // recovery timers (in simulation steps)
  int backup_ticks;
  int wait_ticks;
} SafetyManager;

static SafetyManager safety = {0};

// Utility: set all wheels (reduces redundancy)
static inline void set_wheel_speeds(double wl, double wr) {
  wb_motor_set_velocity(motors[0], wl);
  wb_motor_set_velocity(motors[1], wl);
  wb_motor_set_velocity(motors[2], wr);
  wb_motor_set_velocity(motors[3], wr);
}

static inline void stop_all_motors(void) {
  set_wheel_speeds(0.0, 0.0);
}

static void safety_init(void) {
  safety.state = SAFETY_OK;
  safety.min_dist_m = INFINITY;
  safety.min_idx = -1;
  safety.v_scale = 1.0;
  safety.w_scale = 1.0;
  safety.hard_stop = false;
  safety.backup_ticks = 0;
  safety.wait_ticks = 0;
}

static void safety_read_ir(double *min_dist, int *min_idx) {
  *min_dist = INFINITY;
  *min_idx = -1;

  for (int i = 0; i < 2; i++) {
    if (!ir_sensors[i]) continue;
    double d = wb_distance_sensor_get_value(ir_sensors[i]); // assumed meters
    if (d < *min_dist) { *min_dist = d; *min_idx = i; }
  }
}











// ============== A* PATHFINDING CALLBACKS ==============

void grid_node_neighbors(ASNeighborList neighbors, void *node, void *context) {
    GridNode *current = (GridNode*)node;
    AStarContext *ctx = (AStarContext*)context;
    
    int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    double costs[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};
    
    for (int i = 0; i < 8; i++) {
        int nx = current->x + dx[i];
        int ny = current->y + dy[i];
        
        if (nx < 0 || nx >= ctx->grid_size || ny < 0 || ny >= ctx->grid_size)
            continue;
            
        unsigned int cell = ctx->grid_ptr[ny * ctx->grid_size + nx];
        if (cell == CELL_OBSTACLE)
            continue;

            
        double base_cost = ctx->cost_map_ptr[ny * ctx->grid_size + nx];
        double edge_cost = costs[i] * base_cost;
        
        GridNode neighbor = {nx, ny};
        ASNeighborListAdd(neighbors, &neighbor, edge_cost);
    }
}

float grid_path_cost_heuristic(void *fromNode, void *toNode, void *context) {
    GridNode *from = (GridNode*)fromNode;
    GridNode *to = (GridNode*)toNode;
    
    float dx = (float)(to->x - from->x);
    float dy = (float)(to->y - from->y);
    
    return sqrtf(dx * dx + dy * dy);
}

int grid_node_comparator(void *node1, void *node2, void *context) {
    GridNode *n1 = (GridNode*)node1;
    GridNode *n2 = (GridNode*)node2;
    
    if (n1->y < n2->y) return -1;
    if (n1->y > n2->y) return 1;
    if (n1->x < n2->x) return -1;
    if (n1->x > n2->x) return 1;
    return 0;
}

int grid_early_exit(size_t visitedCount, void *visitingNode, void *goalNode, void *context) {
    if (visitedCount > 250000) { // or even 150K
        return -1;
    }
    return 0;
}

// ============== A* PATHFINDING ==============

GridPath* find_path_astar(int start_x, int start_y, int goal_x, int goal_y) {
    if (!is_valid_cell(start_x, start_y) || !is_valid_cell(goal_x, goal_y)) {
        printf("Invalid start or goal position\n");
        return NULL;
    }
    
    if (grid[goal_y][goal_x] == CELL_OBSTACLE) {
        printf("Goal position is an obstacle\n");
        return NULL;
    }
    
    AStarContext context;
    context.cost_map_ptr = (double*)cost_map;
    context.grid_ptr = (unsigned int*)grid;
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
        printf("No path found from (%d,%d) to (%d,%d)\n", 
               start_x, start_y, goal_x, goal_y);
        return NULL;
    }
    
    size_t path_count = ASPathGetCount(path);
    float path_cost = ASPathGetCost(path);
    
    printf("Path found! Length: %zu, Cost: %.2f\n", path_count, path_cost);
    
    GridPath* grid_path = malloc(sizeof(GridPath));
    grid_path->count = path_count;
    grid_path->capacity = path_count;
    grid_path->nodes = malloc(sizeof(GridNode) * path_count);
    
    for (size_t i = 0; i < path_count; i++) {
        GridNode* node = (GridNode*)ASPathGetNode(path, i);
        grid_path->nodes[i] = *node;
    }
    
    ASPathDestroy(path);
    
    return grid_path;
}

void destroy_grid_path(GridPath* path) {
    if (path) {
        free(path->nodes);
        free(path);
    }
}





static inline bool cell_is_valid_target(int gx, int gy, unsigned int pillar_type) {
  if (!is_valid_cell(gx, gy)) return false;
  return grid[gy][gx] == pillar_type;
}

// If the exact cached cell got overwritten, recover by finding *any* pillar cell (nearest).
static bool find_nearest_cell_of_type(unsigned int pillar_type,
                                      int from_gx, int from_gy,
                                      int *out_gx, int *out_gy)
{
  int best_x = -1, best_y = -1;
  int best_d2 = 2147483647;

  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      if (grid[y][x] != pillar_type) continue;
      int dx = x - from_gx;
      int dy = y - from_gy;
      int d2 = dx*dx + dy*dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_x = x;
        best_y = y;
      }
    }
  }

  if (best_x < 0) return false;
  *out_gx = best_x;
  *out_gy = best_y;
  return true;
}

static void clear_current_path_if_any(void) {
  if (current_path) {
    destroy_grid_path(current_path);
    current_path = NULL;
    current_waypoint = 0;
  }
}

// plan (or replan) to a goal cell
static bool ensure_path_to_goal(int goal_gx, int goal_gy) {
  const double* pos = wb_supervisor_node_get_position(robot_node);
  int rgx, rgy;
  world_to_grid(pos[0], pos[1], &rgx, &rgy);

  // if no path, or periodic replan
  if (!current_path || (nav_replan_counter++ % NAV_REPLAN_EVERY) == 0) {
    clear_current_path_if_any();
    GridPath* p = find_path_astar(rgx, rgy, goal_gx, goal_gy);
    if (!p) return false;
    current_path = p;
    current_waypoint = 0;
  }
  return true;
}



















// =============== ROBOT CONTROL ===============

double current_smooth_speed = 0;


double apply_slew_limiter(double target_speed, double max_accel, double dt) {
    double max_change = max_accel * dt;
    double speed_diff = target_speed - current_smooth_speed;

    // Constrain the change to the maximum allowed step
    if (speed_diff > max_change) {
        speed_diff = max_change;
    } else if (speed_diff < -max_change) {
        speed_diff = -max_change;
    }

    current_smooth_speed += speed_diff;
    return current_smooth_speed;
}

double pid_rotate(double angle_error) { 
    
    double integral_limit = 1.0;  
    pid_rotate_integral += angle_error * (TIME_STEP / 1000.0);
    pid_rotate_integral = clamp(pid_rotate_integral, -integral_limit, integral_limit);
    
    double derivative = (angle_error - pid_rotate_prev_error) / (TIME_STEP / 1000.0);
    pid_rotate_prev_error = angle_error;
    
    double omega = KP_ROTATE * angle_error + KI_ROTATE * pid_rotate_integral + KD_ROTATE * derivative;
    
    return omega;
}

double pid_translate(double distance_error) {
    
    double derivative = (distance_error - pid_translate_prev_error) / (TIME_STEP / 1000.0);
    pid_translate_prev_error = distance_error;
    
    double speed = KP_TRANSLATE * distance_error + KD_TRANSLATE * derivative;
    
    return clamp(speed, 0, 2.0);
}

void reset_pid_controllers() {
    pid_rotate_integral = 0.0;
    pid_rotate_prev_error = 0.0;
    pid_translate_integral = 0.0;
    pid_translate_prev_error = 0.0;
}






double rotate_drive(double rotations,
                    double wheel_speed,      // rad/s (wheel angular speed)
                    double angle_threshold,  // rad
                    double b,                // axle length (m)
                    double r)                // wheel radius (m)
{
  static bool active = false;
  static double prev_theta = 0.0;
  static double turned = 0.0;

  // current yaw
  const double *ori = wb_supervisor_node_get_orientation(robot_node);
  double theta = atan2(ori[3], ori[0]);

  // init on first call
  if (!active) {
    active = true;
    prev_theta = theta;
    turned = 0.0;
  }

  // accumulate absolute yaw turned (robust to wrap)
  double dtheta = normalize_angle(theta - prev_theta);
  prev_theta = theta;
  turned += fabs(dtheta);

  // target & remaining
  double target = fabs(rotations) * 2.0 * M_PI;
  double remaining = target - turned;

  // stop condition
  if (remaining <= angle_threshold) {
    stop_all_motors();
    active = false;
    return 1.0;
  }

  // direction: + => CCW, - => CW
  double dir = (rotations >= 0.0) ? 1.0 : -1.0;

  // constant wheel speeds for in-place spin
  // left backward, right forward => CCW (depending on your wheel convention)
  double wl = -dir * fabs(wheel_speed);
  double wr =  dir * fabs(wheel_speed);

  set_wheel_speeds(wl, wr);
  return 0.0;
}









bool diff_drive(int x_goal, int y_goal, double distance_threshold, 
                double angle_threshold, double b, double r) {
    // IR Safety Check
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    double dx = x_goal - robot_gx;
    double dy = y_goal - robot_gy;
    double distance = sqrt(dx * dx + dy * dy);
    
    double angle_to_goal = atan2(dy, dx);
    double angle_diff = normalize_angle(angle_to_goal - robot_theta);
    
    if (distance < distance_threshold) {
        stop_all_motors();
        return true;
    }
    
    double omega, speed;
    
        // --- Angle hysteresis to prevent chattering ---
    static bool turn_in_place = true;   // start by turning until aligned
    const double ANG_HI = angle_threshold;        // enter/keep turn-in-place
    const double ANG_LO = angle_threshold * 0.6;  // exit turn-in-place (smaller)

    // state transitions
    if (turn_in_place) {
        if (fabs(angle_diff) < ANG_LO) {
            turn_in_place = false;
            // switching to translate: avoid PID "kick"
            pid_rotate_integral = 0.0;
            pid_rotate_prev_error = angle_diff;
            pid_translate_prev_error = distance;
            current_smooth_speed = 0.0; // reset slew state
        }
    } else {
        if (fabs(angle_diff) > ANG_HI) {
            turn_in_place = true;
            // switching to rotate: stop translation smoothly
            pid_translate_prev_error = distance;
            current_smooth_speed = 0.0;
        }
    }

    // commands
    omega = pid_rotate(angle_diff);

    if (turn_in_place) {
        speed = 0.0;
    } else {
        double v_cmd = pid_translate(distance);
        speed = apply_slew_limiter(v_cmd, SLEW_ACC, TIME_STEP / 1000.0);
    }
    
    speed *= safety.v_scale;
    omega *= safety.w_scale;
    
    double omega_l = (speed - omega * b / 2.0) / r;
    double omega_r = (speed + omega * b / 2.0) / r;
    
    set_wheel_speeds(omega_l, omega_r);
    
    return false;
}



static double prev_theta = 0.0;
static double prev_pitch = 0.0;
static bool theta_init = false;

// Gate LiDAR integration if the robot is rotating too fast (yaw)
// OR if it is pitching too fast (e.g., climbing a wall so rays hit the floor).
// Thresholds (rad/s):
//   SHOULD_INTEGRATE_LIDAR_RADIANS        -> yaw rate limit
//   SHOULD_INTEGRATE_LIDAR_PITCH_RADIANS  -> pitch rate limit
static bool should_integrate_lidar(double robot_theta, double robot_pitch) {
  if (!theta_init) {
    prev_theta = robot_theta;
    prev_pitch = robot_pitch;
    theta_init = true;
    return true;
  }

  const double dt = TIME_STEP / 1000.0;

  const double dtheta = normalize_angle(robot_theta - prev_theta);
  const double dpitch = normalize_angle(robot_pitch - prev_pitch);

  prev_theta = robot_theta;
  prev_pitch = robot_pitch;

  const double yaw_rate   = fabs(dtheta) / dt;
  const double pitch_rate = fabs(dpitch) / dt;


  const bool pitch_ok = fabs(robot_pitch) < SHOULD_INTEGRATE_LIDAR_MAX_PITCH_RAD;

  return pitch_ok &&
         (yaw_rate   < SHOULD_INTEGRATE_LIDAR_RADIANS);
/*
  return pitch_ok &&
         (yaw_rate   < SHOULD_INTEGRATE_LIDAR_RADIANS) &&
         (pitch_rate < SHOULD_INTEGRATE_LIDAR_PITCH_RADIANS);
         */
}














bool follow_path() {
    if (!current_path || current_waypoint >= current_path->count) {
        return false;
    }
    
    // Skip waypoints that are too close together
    const double* position = wb_supervisor_node_get_position(robot_node);
    int robot_gx, robot_gy;
    world_to_grid(position[0], position[1], &robot_gx, &robot_gy);
    
    while (current_waypoint < current_path->count - 1) {
        GridNode* next_wp = &current_path->nodes[current_waypoint];
        double dx = next_wp->x - robot_gx;
        double dy = next_wp->y - robot_gy;
        double dist = sqrt(dx * dx + dy * dy);
        
        // Skip if waypoint is very close
        if (dist < WAYPOINT_TOO_CLOSE_DIST) {
            current_waypoint++;
            reset_pid_controllers();
        } else {
            break;
        }
    }
    
    GridNode* waypoint = &current_path->nodes[current_waypoint];
    
    bool reached = diff_drive(
        waypoint->x, waypoint->y,
        DRIVE_DIST_THRESH, DRIVE_ANGLE_THRESH, // dist thresh., angle thresh.
        0.287, 0.0825
    );
    
    if (reached) {
        current_waypoint++;
        printf("Reached waypoint %d/%d\n", current_waypoint, current_path->count);
        
        if (current_waypoint >= current_path->count) {
            printf("Path completed!\n");
            return true;
        }
    }
    
    return false;
}










// =============== MAPPING AND PERCEPTION ===============

void draw_line_on_grid(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (1) {
    if (is_valid_cell(x0, y0)) {

      // --- PROTECT PILLARS: don't let lidar/free/obstacle overwrite them ---
      unsigned int cell = grid[y0][x0];
      if (cell == CELL_BLUE_PILLAR || cell == CELL_YELLOW_PILLAR) {
        // do nothing, keep pillar cell as-is
      } else {

        // beam free space
        if (x0 != x1 || y0 != y1) {
          if (grid[y0][x0] != CELL_OBSTACLE &&
              grid[y0][x0] != CELL_BLUE_PILLAR &&
              grid[y0][x0] != CELL_YELLOW_PILLAR) {
            free_counter[y0][x0]++;
            if (free_counter[y0][x0] >= FREE_THRESHOLD) grid[y0][x0] = CELL_FREE;
            obstacle_counter[y0][x0] = 0;
          }
        } else {
          // beam hit (end cell)
          if (grid[y0][x0] != CELL_BLUE_PILLAR &&
              grid[y0][x0] != CELL_YELLOW_PILLAR) {
            obstacle_counter[y0][x0]++;
            if (obstacle_counter[y0][x0] >= OBSTACLE_THRESHOLD) {
              grid[y0][x0] = CELL_OBSTACLE;
              free_counter[y0][x0] = 0;
            }
          }
        }

      }
    }

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dx) { err += dx; y0 += sy; }
  }
}


void clear_nearby_obstacles(int inner_radius_cells, int outer_radius_cells) {
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);

    for (int dy = -outer_radius_cells; dy <= outer_radius_cells; dy++) {
        for (int dx = -outer_radius_cells; dx <= outer_radius_cells; dx++) {
            int gx = robot_gx + dx;
            int gy = robot_gy + dy;
            if (!is_valid_cell(gx, gy)) continue;

            int dist_sq = dx * dx + dy * dy;
            if (dist_sq > outer_radius_cells * outer_radius_cells) continue;
            if (dist_sq < inner_radius_cells * inner_radius_cells) continue;

            if (grid[gy][gx] == CELL_OBSTACLE) {
                grid[gy][gx] = CELL_UNKNOWN;
            }
        }
    }
}

void filter_connected_components(int min_size) {
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));

    const int dx8[8] = { 1,-1, 0, 0, 1,-1, 1,-1 };
    const int dy8[8] = { 0, 0, 1,-1, 1, 1,-1,-1 };

    Point queue[MAX_QUEUE];

    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {

            if (grid[y][x] != CELL_OBSTACLE || visited[y][x]) continue;

            int head = 0, tail = 0, count = 0;
            Point component[MAX_QUEUE];

            queue[tail++] = (Point){x, y};
            visited[y][x] = true;

            while (head < tail) {
                Point p = queue[head++];
                if (count < MAX_QUEUE) component[count++] = p;

                for (int i = 0; i < 8; i++) {
                    int nx = p.x + dx8[i];
                    int ny = p.y + dy8[i];
                    if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;

                    if (!visited[ny][nx] && grid[ny][nx] == CELL_OBSTACLE) {
                        if (tail < MAX_QUEUE) queue[tail++] = (Point){nx, ny};
                        visited[ny][nx] = true;
                    }
                }
            }

            // 1) Remove small components
            if (count < min_size) {
                for (int i = 0; i < count; i++) {
                    int cx = component[i].x, cy = component[i].y;
                    grid[cy][cx] = CELL_FREE;
                    obstacle_counter[cy][cx] = 0;
                }
                continue;
            }

            // 2) Prune thin / sharp obstacle lines (iterative endpoint & spike removal)
            // Heuristic: line/spike pixels usually have <=2 obstacle neighbors (8-neighborhood).
            if (OBSTACLE_SHARPNESS > 0) {
                const int neigh_max = 2; // "sharp line" criterion (keep this fixed; sharpness controls iterations)
                for (int iter = 0; iter < OBSTACLE_SHARPNESS; iter++) {
                    Point to_remove[MAX_QUEUE];
                    int rm_count = 0;

                    for (int i = 0; i < count; i++) {
                        int cx = component[i].x, cy = component[i].y;

                        // It might have already been removed in a previous iter.
                        if (grid[cy][cx] != CELL_OBSTACLE) continue;

                        int neigh = 0;
                        for (int k = 0; k < 8; k++) {
                            int nx = cx + dx8[k];
                            int ny = cy + dy8[k];
                            if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;
                            if (grid[ny][nx] == CELL_OBSTACLE) neigh++;
                        }

                        if (neigh <= neigh_max) {
                            if (rm_count < MAX_QUEUE) to_remove[rm_count++] = (Point){cx, cy};
                        }
                    }

                    // If nothing to prune, stop early
                    if (rm_count == 0) break;

                    // Apply removals
                    for (int r = 0; r < rm_count; r++) {
                        int rx = to_remove[r].x, ry = to_remove[r].y;
                        // Only remove obstacles (don’t touch pillars/robot; those aren't CELL_OBSTACLE anyway)
                        if (grid[ry][rx] == CELL_OBSTACLE) {
                            grid[ry][rx] = CELL_FREE;
                            obstacle_counter[ry][rx] = 0;
                        }
                    }
                }
            }
        }
    }
}




void process_lidar() {
    const double* position = wb_supervisor_node_get_position(robot_node);
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    
    double robot_x = position[0];
    double robot_y = position[1];
    double robot_theta = atan2(orientation[3], orientation[0]);
    
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    const float* ranges = wb_lidar_get_range_image(lidar);
    int resolution = wb_lidar_get_horizontal_resolution(lidar);
    double fov = wb_lidar_get_fov(lidar);
    double max_range = LIDAR_MAX_RANGE_FACTOR; // wb_lidar_get_max_range(lidar) * 
    
    for (int i = 0; i < resolution; i += 2) {
        double range = ranges[i];
        
        if (range < 0.05 || range > max_range * 0.95) {
            continue;
        }
        
        double ray_angle = fov/2 - (i * fov / resolution);
        double world_angle = robot_theta + ray_angle;
        
        double end_x = robot_x + range * cos(world_angle);
        double end_y = robot_y + range * sin(world_angle);
        
        int end_gx, end_gy;
        world_to_grid(end_x, end_y, &end_gx, &end_gy);
        
        draw_line_on_grid(robot_gx, robot_gy, end_gx, end_gy);
    }
}

void decay_counters() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (grid[y][x] != CELL_OBSTACLE &&
                grid[y][x] != CELL_BLUE_PILLAR &&
                grid[y][x] != CELL_YELLOW_PILLAR &&
                obstacle_counter[y][x] > 0) {

                obstacle_counter[y][x] -= COUNTER_DECAY;
                if (obstacle_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
            
            if (grid[y][x] != CELL_FREE &&
                grid[y][x] != CELL_BLUE_PILLAR &&
                grid[y][x] != CELL_YELLOW_PILLAR &&
                free_counter[y][x] > 0) {

                free_counter[y][x] -= COUNTER_DECAY;
                if (free_counter[y][x] <= 0) {
                    grid[y][x] = CELL_UNKNOWN;
                }
            }
        }
    }
}

inline void relax(double dist_transform[GRID_SIZE][GRID_SIZE],
                  int x, int y, int nx, int ny, double cost)
{
    double new_dist = dist_transform[ny][nx] + cost;
    if (new_dist < dist_transform[y][x])
        dist_transform[y][x] = new_dist;
}

void generate_cost_map(void) {
    static double dist_transform[GRID_SIZE][GRID_SIZE];
    const double INF   = GRID_SIZE * GRID_SIZE * 2.0;
    const double SQRT2 = 1.414213562;

    int x, y, k;

    // ---- Step 1: Initialize EDT seeds ----
    for (y = 0; y < GRID_SIZE; y++) {
        for (x = 0; x < GRID_SIZE; x++) {
            dist_transform[y][x] = (grid[y][x] == CELL_OBSTACLE) ? 0.0 : INF;
        }
    }

    // 8-neighbor offsets + weights (two-pass chamfer EDT)
    const int dx[8]   = {-1,  0, -1,  1,  1,  0,  1, -1};
    const int dy[8]   = { 0, -1, -1, -1,  0,  1,  1,  1};
    const double w8[8]= { 1,   1,  SQRT2, SQRT2, 1,  1, SQRT2, SQRT2 };

    // ---- Step 2: Forward pass ----
    for (y = 0; y < GRID_SIZE; y++) {
        for (x = 0; x < GRID_SIZE; x++) {
            if (dist_transform[y][x] == 0.0) continue;

            for (k = 0; k < 4; k++) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx >= 0 && ny >= 0 && nx < GRID_SIZE && ny < GRID_SIZE) {
                    double nd = dist_transform[ny][nx] + w8[k];
                    if (nd < dist_transform[y][x]) dist_transform[y][x] = nd;
                }
            }
        }
    }

    // ---- Step 3: Backward pass ----
    for (y = GRID_SIZE - 1; y >= 0; y--) {
        for (x = GRID_SIZE - 1; x >= 0; x--) {
            if (dist_transform[y][x] == 0.0) continue;

            for (k = 4; k < 8; k++) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx >= 0 && ny >= 0 && nx < GRID_SIZE && ny < GRID_SIZE) {
                    double nd = dist_transform[ny][nx] + w8[k];
                    if (nd < dist_transform[y][x]) dist_transform[y][x] = nd;
                }
            }
        }
    }

    // ============================
    // Adaptive inflation parameters
    // ============================
    int lethal_cells = INFLATION_RADIUS;  // fallback if adaptive disabled
    int soft_cells   = INFLATION_RADIUS;
    double COST_SCALING = 0.05;           // fallback

#if ADAPT_INFLATION_ENABLE
    // robot position -> grid
    const double* pos = wb_supervisor_node_get_position(robot_node);
    int rgx, rgy;
    world_to_grid(pos[0], pos[1], &rgx, &rgy);

    // collect local clearance samples from FREE cells around robot
    // keep this bounded and fast
    enum { MAX_SAMPLES = 2500 };
    static double samples[MAX_SAMPLES];
    int n = 0;

    int R = ADAPT_CLEAR_WIN_RADIUS;
    int step = ADAPT_CLEAR_SAMPLE_STEP;
    if (step < 1) step = 1;

    int y0 = rgx; (void)y0; // silence warnings if needed

    for (int yy = rgy - R; yy <= rgy + R; yy += step) {
        if (yy < 0 || yy >= GRID_SIZE) continue;
        for (int xx = rgx - R; xx <= rgx + R; xx += step) {
            if (xx < 0 || xx >= GRID_SIZE) continue;

            // only consider known FREE cells for clearance estimate
            if (grid[yy][xx] != CELL_FREE) continue;

            double d = dist_transform[yy][xx];
            if (!isfinite(d) || d <= 0.0 || d >= INF * 0.5) continue;

            samples[n++] = d;
            if (n >= MAX_SAMPLES) goto done_sampling;
        }
    }
done_sampling:

    if (n >= 20) {
        sort_small_double(samples, n);

        double pct = (double)ADAPT_CLEAR_PERCENTILE / 100.0;
        double clear_cells = percentile_sorted(samples, n, pct);

        // map clearance -> t in [0..1]
        // tight => t=0, open => t=1
        double t = (clear_cells - CLEAR_TIGHT_CELLS) / (CLEAR_OPEN_CELLS - CLEAR_TIGHT_CELLS);
        t = clamp01(t);

        // choose radii and scaling based on t
        lethal_cells = (int)lround(lerp((double)INFLATION_LETHAL_MIN_CELLS,
                                        (double)INFLATION_LETHAL_MAX_CELLS, t));
        soft_cells   = (int)lround(lerp((double)INFLATION_SOFT_MIN_CELLS,
                                        (double)INFLATION_SOFT_MAX_CELLS, t));
        COST_SCALING = lerp(COST_SCALE_MIN, COST_SCALE_MAX, t);

        // safety clamps
        if (lethal_cells < 1) lethal_cells = 1;
        if (soft_cells < lethal_cells + 1) soft_cells = lethal_cells + 1;
    } else {
        // not enough samples (early exploration): conservative-ish defaults
        lethal_cells = INFLATION_LETHAL_MIN_CELLS;
        soft_cells   = INFLATION_SOFT_MIN_CELLS;
        COST_SCALING = COST_SCALE_MIN;
    }
#endif

    // ============================
    // Step 4: Convert EDT -> costs
    // ============================
    for (y = 0; y < GRID_SIZE; y++) {
        for (x = 0; x < GRID_SIZE; x++) {

            if (grid[y][x] == CELL_OBSTACLE) {
                cost_map[y][x] = OBSTACLE_COST;
                continue;
            }

            if (grid[y][x] == CELL_UNKNOWN) {
                cost_map[y][x] = UNKNOWN_COST;
                continue;
            }

            double dist = dist_transform[y][x];

            if (!isfinite(dist) || dist >= INF * 0.5) {
                cost_map[y][x] = FREE_COST;
                continue;
            }

            // lethal band: near obstacle -> very high cost
            if (dist <= (double)lethal_cells) {
                cost_map[y][x] = LETHAL_COST;
                continue;
            }

            // soft band: apply smooth repulsive cost up to soft_cells
            if (dist <= (double)soft_cells) {
                // shift so cost starts at lethal boundary smoothly
                double d = dist - (double)lethal_cells;          // 0 .. (soft-lethal)
                double band = (double)(soft_cells - lethal_cells);
                if (band < 1.0) band = 1.0;

                // normalized 0..1
                double u = d / band;

                // exponential-ish repulsion, bounded
                // u=0 => high, u=1 => low
                double factor = exp(-COST_SCALING * (d + 1.0));  // keep finite at 0

                // keep this bounded so A* still uses corridors in tight mazes
                double maxSoft = 120.0; // tune 60..200
                cost_map[y][x] = FREE_COST + factor * (maxSoft - FREE_COST);
                continue;
            }

            cost_map[y][x] = FREE_COST;
        }
    }

    // (optional) debug print every so often
    // printf("[COSTMAP] lethal=%d soft=%d k=%.3f\n", lethal_cells, soft_cells, COST_SCALING);
}























// =============== FRONTIER EXPLORATION ===============

static inline bool is_frontier_edge(int x, int y) {
    if (grid[y][x] != CELL_FREE) return false;
    
    if ((y > 0 && grid[y-1][x] == CELL_UNKNOWN) ||
        (y < GRID_SIZE-1 && grid[y+1][x] == CELL_UNKNOWN) ||
        (x > 0 && grid[y][x-1] == CELL_UNKNOWN) ||
        (x < GRID_SIZE-1 && grid[y][x+1] == CELL_UNKNOWN)) {
        return true;
    }
    return false;
}

void build_frontier_edge_map() {
    memset(frontier_edge_map, 0, sizeof(frontier_edge_map));
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    int scan_radius = 150;
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            if (is_frontier_edge(x, y)) {
                frontier_edge_map[y][x] = 1;
            }
        }
    }
}

bool has_safe_clearance(int cx, int cy, int min_clearance) {
    for (int dy = -min_clearance; dy <= min_clearance; dy++) {
        for (int dx = -min_clearance; dx <= min_clearance; dx++) {
            int check_x = cx + dx;
            int check_y = cy + dy;
            
            if (!is_valid_cell(check_x, check_y)) continue;
            
            if (grid[check_y][check_x] == CELL_OBSTACLE) {
                double dist = sqrt(dx * dx + dy * dy);
                if (dist <= min_clearance) {
                    return false;
                }
            }
        }
    }
    return true;
}

int find_frontier_centroids(FrontierCentroid* centroids, int max_frontiers) {
    build_frontier_edge_map();
    
    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);
    
    static bool visited[GRID_SIZE][GRID_SIZE];
    memset(visited, 0, sizeof(visited));
    
    int num_frontiers = 0;
    
    int scan_radius = 150;
    int min_x = fmax(0, robot_gx - scan_radius);
    int max_x = fmin(GRID_SIZE - 1, robot_gx + scan_radius);
    int min_y = fmax(0, robot_gy - scan_radius);
    int max_y = fmin(GRID_SIZE - 1, robot_gy + scan_radius);
    
    static Point queue[1000];
    
    for (int y = min_y; y <= max_y && num_frontiers < max_frontiers; y++) {
        for (int x = min_x; x <= max_x && num_frontiers < max_frontiers; x++) {
            if (frontier_edge_map[y][x] && !visited[y][x]) {
                int head = 0, tail = 0;
                int sum_x = 0, sum_y = 0, count = 0;
                
                queue[tail++] = (Point){x, y};
                visited[y][x] = true;
                
                while (head < tail && tail < 1000) {
                    Point p = queue[head++];
                    sum_x += p.x;
                    sum_y += p.y;
                    count++;
                    
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            
                            int nx = p.x + dx;
                            int ny = p.y + dy;
                            
                            if (nx >= min_x && nx <= max_x && 
                                ny >= min_y && ny <= max_y &&
                                !visited[ny][nx] && 
                                frontier_edge_map[ny][nx]) {
                                
                                queue[tail++] = (Point){nx, ny};
                                visited[ny][nx] = true;
                                
                                if (tail >= 1000) break;
                            }
                        }
                    }
                }
                
                if (count >= MIN_FRONTIER_SIZE) {
                    int cx = sum_x / count;
                    int cy = sum_y / count;
                    
                    if (has_safe_clearance(cx, cy, MIN_FRONTIER_CLEARANCE)) {
                        centroids[num_frontiers].x = cx;
                        centroids[num_frontiers].y = cy;
                        centroids[num_frontiers].size = count;
                        
                        double dist = sqrt((cx - robot_gx) * (cx - robot_gx) + 
                                         (cy - robot_gy) * (cy - robot_gy));
                        
                        centroids[num_frontiers].score = (count * 2.0) / (1.0 + dist * 0.02);
                        
                        num_frontiers++;
                    }
                }
            }
        }
    }
    
    for (int i = 0; i < num_frontiers - 1; i++) {
        for (int j = 0; j < num_frontiers - i - 1; j++) {
            if (centroids[j].score < centroids[j + 1].score) {
                FrontierCentroid temp = centroids[j];
                centroids[j] = centroids[j + 1];
                centroids[j + 1] = temp;
            }
        }
    }
    
    return num_frontiers;
}

FrontierCentroid* get_best_frontier(FrontierCentroid* frontiers, int num_frontiers) {
    return (num_frontiers > 0) ? &frontiers[0] : NULL;
}

void update_path_to_frontier(int frame_count) {
    static FrontierCentroid frontiers[MAX_FRONTIERS];
    int num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);

    if (num_frontiers == 0) {
        printf("No frontiers found - exploration complete!\n");
        return;
    }

    const double* position = wb_supervisor_node_get_position(robot_node);
    int robot_gx, robot_gy;
    world_to_grid(position[0], position[1], &robot_gx, &robot_gy);

    if (current_path) {
        destroy_grid_path(current_path);
        current_path = NULL;
    }

    // Try best-to-worse until we find a reachable one
    for (int i = 0; i < num_frontiers; i++) {
        FrontierCentroid *f = &frontiers[i];

        // skip recently failed frontiers
        if (is_failed_frontier(f->x, f->y, frame_count)) {
            continue;
        }

        printf("Planning path to frontier #%d at (%d,%d) score=%.2f size=%d\n",
               i, f->x, f->y, f->score, f->size);

        GridPath* p = find_path_astar(robot_gx, robot_gy, f->x, f->y);

        if (p) {
            current_path = p;
            current_waypoint = 0;
            printf("Selected frontier #%d (reachable)\n", i);
            return;
        }

        // mark failed so we don't keep retrying it every update cycle
        printf("Frontier #%d unreachable -> trying next\n", i);
        mark_failed_frontier(f->x, f->y, frame_count);
    }

    // If we reach here: none reachable right now
    printf("All %d frontiers unreachable right now. Spinning to discover openings...\n", num_frontiers);
    blue_cooldown_ticks = (int)(1.0 / (TIME_STEP / 1000.0)); // ~1s scan
}














// =============== DISPLAY RENDERING ===============

// Map cost to color
static unsigned int cost_to_color(double cost) {
    double minC = FREE_COST;
    double maxC = OBSTACLE_COST;
    double norm = (cost - minC) / (maxC - minC);
    
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    int r, g, b;
    double t = norm;

    if (norm < 0.6) {
        // White -> Yellow
        double t = norm / 0.60; // 0 -> 1
        r = 255;
        g = (int)(255 - (1-t)); // 255 constant
        b = (int)(255 * (1 - t)); // 255 -> 0
    } else if (norm < 0.8) {
        // Yellow -> Orange
        double t = (norm - 0.60) / 0.60; // 0 -> 1
        r = 255;
        g = (int)(255 - (255 - 165) * t); // 255 -> 165
        b = 0;
    } else {
        // Orange -> Red
        double t = (norm - 0.6) / 0.34; // 0 -> 1
        r = 255;
        g = (int)(165 * (1 - t)); // 165 -> 0
        b = 0;
    }
    
    
    // Clamp safety
    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}

static unsigned int frontier_score_to_color(double score, double min_score, double max_score) {
    double norm = 0.0;
    if (max_score > min_score) norm = (score - min_score) / (max_score - min_score);
    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    int r, g, b;
    if (norm <= 0.2) {
        double t = (norm / 0.5);
        r = (int)(255.0 * t);
        g = 255;
        b = 0;
    } else {
        double t = (norm - 0.2) / 0.2;
        r = 255;
        g = (int)(255.0 * (1.0 - t));
        b = 0;
    }

    if (r < 0) r = 0; if (r > 255) r = 255;
    if (g < 0) g = 0; if (g > 255) g = 255;
    if (b < 0) b = 0; if (b > 255) b = 255;

    return ((unsigned int)r << 16) | ((unsigned int)g << 8) | (unsigned int)b;
}

void draw_path_on_display(WbDeviceTag display, GridPath* path, int robot_gx, int robot_gy, 
                          double scale, int grid_start_x, int grid_start_y) {
    if (!path) return;
    
    wb_display_set_color(display, 0x00FF00);
    
    for (int i = 0; i < path->count - 1; i++) {
        GridNode* n1 = &path->nodes[i];
        GridNode* n2 = &path->nodes[i + 1];
        
        int x1 = (int)((n1->x - grid_start_x) * scale);
        int y1 = DISPLAY_HEIGHT - (int)((n1->y - grid_start_y + 1) * scale);
        int x2 = (int)((n2->x - grid_start_x) * scale);
        int y2 = DISPLAY_HEIGHT - (int)((n2->y - grid_start_y + 1) * scale);
        
        wb_display_draw_line(display, x1, y1, x2, y2);
    }
    
    wb_display_set_color(display, 0x00FFFF);
    for (int i = 0; i < path->count; i++) {
        GridNode* n = &path->nodes[i];
        int x = (int)((n->x - grid_start_x) * scale);
        int y = DISPLAY_HEIGHT - (int)((n->y - grid_start_y + 1) * scale);
        
        if (i == current_waypoint) {
            wb_display_set_color(display, 0xFF0000);
            wb_display_fill_oval(display, x - 3, y - 3, 6, 6);
        } else {
            wb_display_set_color(display, 0x0000FF);
            wb_display_fill_oval(display, x - 2, y - 2, 2, 2);
        }
    }
}

// Display IR sensor status
void display_ir_status(int frame_count) {
  int status_x = 6;
  int status_y = DISPLAY_HEIGHT - 60;

  if (safety.state == SAFETY_WARN || safety.state == SAFETY_EMERGENCY_STOP ||
      safety.state == SAFETY_RECOVER_BACKUP || safety.state == SAFETY_RECOVER_WAIT_CLEAR) {
    wb_display_set_color(display, 0xFF0000);
    wb_display_fill_rectangle(display, status_x - 2, status_y - 2, 180, 55);
  }

  wb_display_set_color(display, 0xFFFFFF);
  wb_display_draw_text(display, "IR Sensors:", status_x, status_y);

  for (int i = 0; i < 4; i++) {
    if (!ir_sensors[i]) continue;

    double dist = wb_distance_sensor_get_value(ir_sensors[i]); // optional:
    // If you want ZERO extra reads, store all 4 in safety_update_per_tick().
    char buf[64];
    sprintf(buf, "%s: %.3fm", ir_sensor_names[i], dist);

    if (dist < IR_CRITICAL_THRESHOLD) wb_display_set_color(display, 0xFF0000);
    else if (dist < IR_SAFETY_THRESHOLD) wb_display_set_color(display, 0xFFAA00);
    else wb_display_set_color(display, 0x00FF00);

    wb_display_draw_text(display, buf, status_x, status_y + 12 + (i * 10));
  }

  if (safety.state != SAFETY_OK) {
    wb_display_set_color(display, 0xFFFF00);
    wb_display_draw_text(display, "SAFETY ACTIVE", status_x + 100, status_y + 20);
  }
}







void render_display_with_path(int frame_count) {
    wb_display_set_color(display, COLOR_BACKGROUND);
    wb_display_fill_rectangle(display, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);

    const double* position = wb_supervisor_node_get_position(robot_node);
    double robot_x = position[0];
    double robot_y = position[1];
    int robot_gx, robot_gy;
    world_to_grid(robot_x, robot_y, &robot_gx, &robot_gy);

    int viewport_size = (int)(DISPLAY_WIDTH / DISPLAY_SCALE);
    int half_viewport = viewport_size / 2;
    int cell_size = (int)(DISPLAY_SCALE + 1);

    int grid_start_x = robot_gx - half_viewport;
    int grid_end_x = robot_gx + half_viewport;
    int grid_start_y = robot_gy - half_viewport;
    int grid_end_y = robot_gy + half_viewport;

    for (int gy = grid_start_y; gy <= grid_end_y; gy++) {
      for (int gx = grid_start_x; gx <= grid_end_x; gx++) {
        if (!is_valid_cell(gx, gy)) continue;

        unsigned int cell = grid[gy][gx];
        unsigned int color = COLOR_UNKNOWN;

        if (cell == CELL_UNKNOWN) {
          color = COLOR_UNKNOWN;

        } else if (cell == CELL_ROBOT) {
          continue;

        } else if (cell == CELL_OBSTACLE) {
          color = COLOR_OBSTACLE;

        } else if (cell == CELL_BLUE_PILLAR) {
          color = COLOR_BLUE_PILLAR;

        } else if (cell == CELL_YELLOW_PILLAR) {
          color = COLOR_YELLOW_PILLAR;

        } else {
          // free / other -> show cost map color
          double c = cost_map[gy][gx];
          color = cost_to_color(c);
        }

        int screen_x = (int)((gx - grid_start_x) * DISPLAY_SCALE);
        int screen_y = DISPLAY_HEIGHT - (int)((gy - grid_start_y + 1) * DISPLAY_SCALE);

        // draw everything except unknown (unknown is background)
        if (cell != CELL_UNKNOWN) {
          wb_display_set_color(display, color);
          wb_display_fill_rectangle(display, screen_x, screen_y, cell_size, cell_size);
        }
      }
    }


    draw_path_on_display(display, current_path, robot_gx, robot_gy, 
                            DISPLAY_SCALE, grid_start_x, grid_start_y);

    static FrontierCentroid frontiers[MAX_FRONTIERS];
    static int num_frontiers = 0;
    static int frontier_update_counter = 0;
    
    if (++frontier_update_counter >= 20) {
        frontier_update_counter = 0;
        num_frontiers = find_frontier_centroids(frontiers, MAX_FRONTIERS);
    }

    double min_score = 1e9, max_score = -1e9;
    for (int i = 0; i < num_frontiers; i++) {
        if (frontiers[i].score < min_score) min_score = frontiers[i].score;
        if (frontiers[i].score > max_score) max_score = frontiers[i].score;
    }
    if (min_score == 1e9) { min_score = 0.0; max_score = 1.0; }

    for (int i = 0; i < num_frontiers; i++) {
        int fx = frontiers[i].x;
        int fy = frontiers[i].y;
        
        if (fx >= grid_start_x && fx <= grid_end_x &&
            fy >= grid_start_y && fy <= grid_end_y) {
            
            int screen_x = (int)((fx - grid_start_x) * DISPLAY_SCALE);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * DISPLAY_SCALE);
            
            unsigned int fcolor = frontier_score_to_color(frontiers[i].score, min_score, max_score);
            wb_display_set_color(display, fcolor);

            int marker_half = 6;
            wb_display_fill_rectangle(display, screen_x - 1, screen_y - marker_half, 2, marker_half * 2);
            wb_display_fill_rectangle(display, screen_x - marker_half, screen_y - 1, marker_half * 2, 2);
        }
    }

    if (num_frontiers > 0) {
        int fx = frontiers[0].x;
        int fy = frontiers[0].y;
        if (fx >= grid_start_x && fx <= grid_end_x &&
            fy >= grid_start_y && fy <= grid_end_y) {

            int screen_x = (int)((fx - grid_start_x) * DISPLAY_SCALE);
            int screen_y = DISPLAY_HEIGHT - (int)((fy - grid_start_y + 1) * DISPLAY_SCALE);

            wb_display_set_color(display, 0xFFFF00);
            int thick = 2;
            int size = 8;
            wb_display_fill_rectangle(display, screen_x - thick, screen_y - size, thick*2, size*2);
            wb_display_fill_rectangle(display, screen_x - size, screen_y - thick, size*2, thick*2);

            wb_display_set_color(display, 0xFFFF00);
            int ring_diameter = 8;
            int ring_x = screen_x - ring_diameter/2;
            int ring_y = screen_y - ring_diameter/2;
            
            //wb_display_draw_oval(display, ring_x, ring_y, ring_diameter, ring_diameter);
            //wb_display_draw_oval(display, ring_x-1, ring_y-1, ring_diameter+2, ring_diameter+2);
        }
    }
    
    display_ir_status(frame_count);

    int center_x = DISPLAY_WIDTH / 2;
    int center_y = DISPLAY_HEIGHT / 2;
    int robot_size = (int)(DISPLAY_SCALE * 6);
    
    wb_display_set_color(display, COLOR_ROBOT);
    wb_display_fill_rectangle(display, 
                              center_x - robot_size/2, 
                              center_y - robot_size/2, 
                              robot_size, 
                              robot_size);
    
    const double* orientation = wb_supervisor_node_get_orientation(robot_node);
    double robot_theta = atan2(orientation[3], orientation[0]);
    int arrow_length = robot_size;
    int arrow_end_x = center_x + (int)(arrow_length * cos(robot_theta));
    int arrow_end_y = center_y - (int)(arrow_length * sin(robot_theta));
    
    wb_display_set_color(display, 0xFFFF00);
    wb_display_draw_line(display, center_x, center_y, arrow_end_x, arrow_end_y);

    wb_display_set_color(display, 0xFFFFFF);
    wb_display_set_font(display, "Arial", 12, 0);
    wb_display_draw_text(display, "A* Pathfinding + Frontiers", 6, 5);

    char info[256];
    sprintf(info, "Pos: (%d,%d) Waypoint: %d/%d", 
            robot_gx, robot_gy, current_waypoint, 
            current_path ? current_path->count : 0);
    wb_display_draw_text(display, info, 6, 20);
    
    sprintf(info, "Frontiers: %d", num_frontiers);
    wb_display_draw_text(display, info, 6, 35);

    int legend_x = DISPLAY_WIDTH - 110;
    int legend_y = 6;
    int box = 10;
    
    wb_display_set_color(display, cost_to_color(FREE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "free", legend_x + 14, legend_y + 9);

    wb_display_set_color(display, cost_to_color(OBSTACLE_COST));
    wb_display_fill_rectangle(display, legend_x, legend_y + 14, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "obstacle", legend_x + 14, legend_y + 23);

    wb_display_set_color(display, COLOR_UNKNOWN);
    wb_display_fill_rectangle(display, legend_x, legend_y + 28, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "unknown", legend_x + 14, legend_y + 37);
    
    wb_display_set_color(display, 0xFFFF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 42, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "target", legend_x + 14, legend_y + 51);
    
    wb_display_set_color(display, 0x00FF00);
    wb_display_fill_rectangle(display, legend_x, legend_y + 56, box, box);
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_draw_text(display, "path", legend_x + 14, legend_y + 65);
}











// ============ CAMERA FUNCTIONS ================


static inline int clampi(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline bool is_finite_float(float x) {
  return isfinite((double)x);
}

static inline bool color_match(TargetColor c, int r, int g, int b) {
  // Basic "colorfulness" filter (reject gray/white-ish pixels)
  int maxv = r; if (g > maxv) maxv = g; if (b > maxv) maxv = b;
  int minv = r; if (g < minv) minv = g; if (b < minv) minv = b;
  if ((maxv - minv) < 40) return false;

  // Thresholds (tune if needed)
  switch (c) {
    case COLOR_BLUE:   return (b > 120 && r < 70  && g < 90);
    case COLOR_YELLOW: return (r > 120 && g > 120 && b < 50);
    case COLOR_RED:    return (r > 150 && g < 60  && b < 60); 
    case COLOR_GREEN:  return (g > 150 && r < 60  && b < 60);
    default: return false;
  }
}

static Centroid2D find_color_centroid(WbDeviceTag cam, TargetColor target,
                                     int min_pixels, int step) {
  Centroid2D out = {0};
  if (!cam) return out;

  int w = wb_camera_get_width(cam);
  int h = wb_camera_get_height(cam);
  const unsigned char *img = wb_camera_get_image(cam);
  if (!img || w <= 0 || h <= 0) return out;

  long long sumx = 0, sumy = 0;
  int count = 0;

  // step=1 full res, step=2 faster, etc.
  for (int y = 0; y < h; y += step) {
    for (int x = 0; x < w; x += step) {
      int r = wb_camera_image_get_red(img,  w, x, y);
      int g = wb_camera_image_get_green(img,w, x, y);
      int b = wb_camera_image_get_blue(img, w, x, y);

      if (color_match(target, r, g, b)) {
        sumx += x;
        sumy += y;
        count++;
      }
    }
  }

  out.count = count;
  if (count < min_pixels) return out;

  out.valid = true;
  out.cx = (float)sumx / (float)count;
  out.cy = (float)sumy / (float)count;
  return out;
}

static float median9(float *a, int n) {
  // n <= 9, simple insertion sort
  for (int i = 1; i < n; i++) {
    float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
    a[j + 1] = v;
  }
  return a[n / 2];
}

static DepthAtCentroid depth_at_centroid(WbDeviceTag rf, float cx, float cy) {
  DepthAtCentroid out = {0};
  if (!rf) return out;

  int w = wb_range_finder_get_width(rf);
  int h = wb_range_finder_get_height(rf);
  const float *range = wb_range_finder_get_range_image(rf);
  if (!range || w <= 0 || h <= 0) return out;

  int x0 = (int)lroundf(cx);
  int y0 = (int)lroundf(cy);

  // Take a small window around centroid and median it (robust against noise)
  float vals[9];
  int n = 0;

  for (int dy = -1; dy <= 1; dy++) {
    for (int dx = -1; dx <= 1; dx++) {
      int x = clampi(x0 + dx, 0, w - 1);
      int y = clampi(y0 + dy, 0, h - 1);
      float d = range[y * w + x];
      if (is_finite_float(d) && d > 0.001f && d < 1000.0f) {
        vals[n++] = d;
      }
    }
  }

  if (n == 0) return out;

  out.valid = true;
  out.dist = median9(vals, n);
  return out;
}

static void mark_obstacle_disk(int gx, int gy, int radius_cells) {
  int r2 = radius_cells * radius_cells;
  for (int dy = -radius_cells; dy <= radius_cells; dy++) {
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
      if (dx*dx + dy*dy > r2) continue;

      int x = gx + dx;
      int y = gy + dy;
      if (!is_valid_cell(x, y)) continue;

      // don't overwrite robot or pillars
      unsigned int cell = grid[y][x];
      if (cell == CELL_ROBOT) continue;
      if (cell == CELL_BLUE_PILLAR || cell == CELL_YELLOW_PILLAR) continue;

      grid[y][x] = CELL_OBSTACLE;
      obstacle_counter[y][x] = OBSTACLE_THRESHOLD; // lock quickly
      free_counter[y][x] = 0;
    }
  }
}


static void mark_obstacle_square(int gx, int gy, int half_size_cells) {
  for (int dy = -half_size_cells; dy <= half_size_cells; dy++) {
    for (int dx = -half_size_cells; dx <= half_size_cells; dx++) {
      int x = gx + dx;
      int y = gy + dy;
      if (!is_valid_cell(x, y)) continue;
      if (grid[y][x] == CELL_ROBOT) continue;

      grid[y][x] = CELL_OBSTACLE;
      obstacle_counter[y][x] = OBSTACLE_THRESHOLD;
      free_counter[y][x] = 0;
    }
  }
}

// Main function you call from loop
bool detect_color_and_mark_on_grid(
  WbDeviceTag cam_rgb,
  WbDeviceTag rf_depth,
  TargetColor target,
  int min_pixels,
  MarkShape shape,
  int mark_radius_cells,
  double fov_horizontal,
  double distance_scale,       // e.g. 0.85 (mark slightly closer than measured)
  WbNodeRef robot
) {
  if (!cam_rgb || !rf_depth || !robot) return false;

  // 1) centroid in image
  // step=2 is a good default speed/quality tradeoff
  Centroid2D cen = find_color_centroid(cam_rgb, target, min_pixels, 2);
  if (!cen.valid) return false;

  // 2) depth at centroid
  DepthAtCentroid dep = depth_at_centroid(rf_depth, cen.cx, cen.cy);
  if (!dep.valid) return false;

  // 3) compute bearing from centroid x
  int w = wb_camera_get_width(cam_rgb);
  float bearing = -(float)((cen.cx - 0.5f * (float)w) * ((float)fov_horizontal / (float)w));

  // 4) robot pose
  const double *pos = wb_supervisor_node_get_position(robot);
  const double *ori = wb_supervisor_node_get_orientation(robot);
  double robot_x = pos[0];
  double robot_y = pos[1];
  double theta   = atan2(ori[3], ori[0]);

  // 5) project to world (planar assumption)
  double d = (double)dep.dist * distance_scale;

  double world_ang = theta + (double)bearing;
  
  double wx = robot_x + d * cos(world_ang);
  double wy = robot_y + d * sin(world_ang);

  // 6) world -> grid
  int gx, gy;
  world_to_grid(wx, wy, &gx, &gy);
  if (!is_valid_cell(gx, gy)) return false;

  // 7) mark as obstacle (only)
  if (shape == MARK_DISK) {
    mark_obstacle_disk(gx, gy, mark_radius_cells);
  } else {
    mark_obstacle_square(gx, gy, mark_radius_cells);
  }

  // Optional: debug print
  // printf("[COLOR] target=%d cen=(%.1f,%.1f) depth=%.2f -> grid=(%d,%d)\n",
  //        target, cen.cx, cen.cy, dep.dist, gx, gy);

  return true;
}


// Single simple function: GREEN pixels + depth -> obstacles
// Fallback: if GREEN detected but depth missing (too close), mark a "blob" obstacle at min_depth.
void detect_green_to_grid(
    WbDeviceTag cam_rgb,
    WbDeviceTag rf_depth,
    WbNodeRef robot,
    double fov_horizontal,
    int step,
    int base_radius_cells,
    double distance_scale,
    double min_depth_m,
    double max_depth_m,
    double roi_y0_frac,
    double roi_y1_frac,
    double const_mark_dist_m          // NEW: constant distance for bottom region marking (meters)
) {
  if (!cam_rgb || !robot) return;
  if (step <= 0) step = 1;
  if (!isfinite(fov_horizontal) || fov_horizontal <= 0.0) return;

  // We'll still use depth for the upper ROI; but the bottom region ignores depth entirely.
  if (!rf_depth) return;

  int w = wb_camera_get_width(cam_rgb);
  int h = wb_camera_get_height(cam_rgb);
  const unsigned char *img = wb_camera_get_image(cam_rgb);
  if (!img || w <= 0 || h <= 0) return;

  int dw = wb_range_finder_get_width(rf_depth);
  int dh = wb_range_finder_get_height(rf_depth);
  const float *range = wb_range_finder_get_range_image(rf_depth);
  if (!range || dw <= 0 || dh <= 0) return;

  const double *pos = wb_supervisor_node_get_position(robot);
  const double *ori = wb_supervisor_node_get_orientation(robot);
  if (!pos || !ori) return;

  double rx = pos[0], ry = pos[1];
  double theta = atan2(ori[3], ori[0]);

  // Convert ROI fractions to pixel rows
  int y0 = (int)lround(roi_y0_frac * (double)h);
  int y1 = (int)lround(roi_y1_frac * (double)h);
  y0 = clampi(y0, 0, h - 1);
  y1 = clampi(y1, 0, h);           // y1 can be == h

  if (y1 <= y0) return;

  // Bottom section = [y1 .. h)
  int y2 = y1;
  int y3 = h;

  // sanity for constant marking distance
  bool use_const = isfinite(const_mark_dist_m) && const_mark_dist_m > 0.0;
  if (!use_const) {
    // If constant distance is invalid, we simply skip the bottom section behavior.
    // Upper ROI (depth-based) still works.
    y2 = y3; // disables bottom loop
  }

  double tan_half = tan(fov_horizontal * 0.5);

  // =========================
  // 1) Upper ROI: depth-based
  // =========================
  for (int y = y0; y < y1; y += step) {
    for (int x = 0; x < w; x += step) {

      int r = wb_camera_image_get_red(img,   w, x, y);
      int g = wb_camera_image_get_green(img, w, x, y);
      int b = wb_camera_image_get_blue(img,  w, x, y);

      if (!color_match(COLOR_GREEN, r, g, b)) continue;

      // ---- depth at pixel (median 3x3), with res mapping if needed ----
      int dxp = (w > 1) ? (int)lround((double)x * (double)(dw - 1) / (double)(w - 1)) : 0;
      int dyp = (h > 1) ? (int)lround((double)y * (double)(dh - 1) / (double)(h - 1)) : 0;
      dxp = clampi(dxp, 0, dw - 1);
      dyp = clampi(dyp, 0, dh - 1);

      float vals[9];
      int n = 0;
      for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {
          int xx = clampi(dxp + ox, 0, dw - 1);
          int yy = clampi(dyp + oy, 0, dh - 1);
          float d0 = range[yy * dw + xx];
          if (is_finite_float(d0) && d0 > 0.001f && d0 < 1000.0f) vals[n++] = d0;
        }
      }
      if (n == 0) continue;

      float d_med = median9(vals, n);
      double d = (double)d_med * distance_scale;
      if (!isfinite(d) || d < min_depth_m || d > max_depth_m) continue;

      // bearing from pixel x
      double bearing = -((double)x - 0.5 * (double)w) * (fov_horizontal / (double)w);

      // project to world
      double world_ang = theta + bearing;
      double wx = rx + d * cos(world_ang);
      double wy = ry + d * sin(world_ang);

      int gx, gy;
      world_to_grid(wx, wy, &gx, &gy);
      if (!is_valid_cell(gx, gy)) continue;

      // choose radius to cover skipped pixels
      double spacing_m = (2.0 * d * tan_half / (double)w) * (double)step;
      int rad = (int)ceil(0.6 * (spacing_m / GRID_RESOLUTION));
      if (rad < base_radius_cells) rad = base_radius_cells;
      if (rad > 20) rad = 20;

      mark_obstacle_disk(gx, gy, rad);
    }
  }

  // ======================================================
  // 2) Bottom region: constant-distance marking (NO depth)
  //    Only runs if const_mark_dist_m is valid (>0).
  // ======================================================
  if (y2 < y3 && use_const) {

    // Optional: quick trigger check (skip work if no green at all in bottom region)
    bool any_green_bottom = false;
    for (int y = y2; y < y3 && !any_green_bottom; y += step) {
      for (int x = 0; x < w; x += step) {
        int r = wb_camera_image_get_red(img,   w, x, y);
        int g = wb_camera_image_get_green(img, w, x, y);
        int b = wb_camera_image_get_blue(img,  w, x, y);
        if (color_match(COLOR_GREEN, r, g, b)) {
          any_green_bottom = true;
          break;
        }
      }
    }

    if (any_green_bottom) {
      const double d = const_mark_dist_m;

      for (int y = y2; y < y3; y += step) {
        for (int x = 0; x < w; x += step) {

          int r = wb_camera_image_get_red(img,   w, x, y);
          int g = wb_camera_image_get_green(img, w, x, y);
          int b = wb_camera_image_get_blue(img,  w, x, y);

          if (!color_match(COLOR_GREEN, r, g, b)) continue;

          // bearing from pixel x (same as above)
          double bearing = -((double)x - 0.5 * (double)w) * (fov_horizontal / (double)w);

          // project at constant distance along that ray
          double world_ang = theta + bearing;
          double wx = rx + d * cos(world_ang);
          double wy = ry + d * sin(world_ang);

          int gx, gy;
          world_to_grid(wx, wy, &gx, &gy);
          if (!is_valid_cell(gx, gy)) continue;

          // radius based on the constant distance (so coverage is consistent)
          double spacing_m = (2.0 * d * tan_half / (double)w) * (double)step;
          int rad = (int)ceil(0.6 * (spacing_m / GRID_RESOLUTION));
          if (rad < base_radius_cells) rad = base_radius_cells;
          if (rad > 20) rad = 20;

          mark_obstacle_disk(gx, gy, rad);
        }
      }
    }
  }
}

// Segment is drawn from hit -> hit + segment_len_m along the same bearing direction.
//
// Uses your existing: color_match(COLOR_RED,...), draw_line_on_grid(),
// world_to_grid(), clampi(), median9(), is_finite_float(), is_valid_cell()
// Simple RED hit -> draw a short radial segment INWARDS (towards robot), not from robot.
// No fallback: if depth is missing/invalid -> skip.
// ============================================================
// detect_red_to_grid(): RED wall hit -> paint OBSTACLE LINE inward
// - Uses depth for each red pixel sample
// - Computes hit point in world
// - Draws an OBSTACLE segment from hit going inward (towards robot)
// - Marks EVERY cell on the segment as CELL_OBSTACLE (not FREE)
// - No depth -> skip (no fallback)
// ============================================================
void detect_red_to_grid(
    WbDeviceTag cam_rgb,
    WbDeviceTag rf_depth,
    WbNodeRef robot,
    double fov_horizontal,     // e.g. 1.04 rad
    int step,                  // e.g. 8..12
    double distance_scale,     // e.g. 1.0
    double min_depth_m,        // e.g. 0.20
    double max_depth_m,        // e.g. 3.0
    double segment_len_m,      // inward length from hit [m], e.g. 0.60
    double roi_y0_frac,        // e.g. 0.10
    double roi_y1_frac,        // e.g. 0.85
    double depth_x_offset_px      // NEW: + => depth appears shifted RIGHT; we sample at (dxp - offset)
) {
  if (!cam_rgb || !rf_depth || !robot) return;
  if (!(segment_len_m > 0.0) || !isfinite(segment_len_m)) return;

  // ---------- RGB image ----------
  int w = wb_camera_get_width(cam_rgb);
  int h = wb_camera_get_height(cam_rgb);
  const unsigned char *img = wb_camera_get_image(cam_rgb);
  if (!img || w <= 0 || h <= 0) return;

  // ---------- Depth image ----------
  int dw = wb_range_finder_get_width(rf_depth);
  int dh = wb_range_finder_get_height(rf_depth);
  const float *range = wb_range_finder_get_range_image(rf_depth);
  if (!range || dw <= 0 || dh <= 0) return;

  // ---------- Robot pose ----------
  const double *pos = wb_supervisor_node_get_position(robot);
  const double *ori = wb_supervisor_node_get_orientation(robot);
  double rx = pos[0], ry = pos[1];
  double theta = atan2(ori[3], ori[0]);

  // ---------- ROI ----------
  int y0 = (int)lround(roi_y0_frac * h);
  int y1 = (int)lround(roi_y1_frac * h);
  y0 = clampi(y0, 0, h - 1);
  y1 = clampi(y1, 0, h);
  if (y1 <= y0) return;

  // ---------- sample pixels ----------
  for (int y = y0; y < y1; y += step) {
    for (int x = 0; x < w; x += step) {

      // [LABEL] RED PIXEL CHECK
      int r = wb_camera_image_get_red(img,   w, x, y);
      int g = wb_camera_image_get_green(img, w, x, y);
      int b = wb_camera_image_get_blue(img,  w, x, y);
      if (!color_match(COLOR_RED, r, g, b)) continue;

      // [LABEL] RGB->DEPTH PIXEL MAP (scale)
      int dxp = (w > 1) ? (int)lround((double)x * (double)(dw - 1) / (double)(w - 1)) : 0;
      int dyp = (h > 1) ? (int)lround((double)y * (double)(dh - 1) / (double)(h - 1)) : 0;

      // [LABEL] APPLY HORIZONTAL OFFSET COMPENSATION
      // depth cam is left => depth image features appear shifted right,
      // so we must sample depth a bit LEFT: dxp -= offset
      dxp -= depth_x_offset_px;

      dxp = clampi(dxp, 0, dw - 1);
      dyp = clampi(dyp, 0, dh - 1);

      // [LABEL] DEPTH MEDIAN 3x3
      float vals[9];
      int n = 0;
      for (int oy = -1; oy <= 1; oy++) {
        for (int ox = -1; ox <= 1; ox++) {
          int xx = clampi(dxp + ox, 0, dw - 1);
          int yy = clampi(dyp + oy, 0, dh - 1);
          float d0 = range[yy * dw + xx];
          if (is_finite_float(d0) && d0 > 0.001f && d0 < 1000.0f) vals[n++] = d0;
        }
      }

      // [LABEL] NO DEPTH -> SKIP (NO FALLBACK)
      if (n == 0) continue;

      double d = (double)median9(vals, n) * distance_scale;
      if (!isfinite(d)) continue;
      if (d < min_depth_m || d > max_depth_m) continue;

      // [LABEL] BEARING FROM PIXEL X
      double bearing = -((double)x - 0.5 * (double)w) * (fov_horizontal / (double)w);

      // [LABEL] WORLD DIRECTION (ROBOT->HIT)
      double ang = theta + bearing;
      double dirx = cos(ang);
      double diry = sin(ang);

      // [LABEL] HIT POINT WORLD
      double hx = rx + d * dirx;
      double hy = ry + d * diry;

      // [LABEL] INWARD ENDPOINT (TOWARDS ROBOT)
      double back = segment_len_m;
      if (back > d) back = d; // avoid going behind robot (safe)
      double sx = hx - back * dirx;
      double sy = hy - back * diry;

      // [LABEL] WORLD->GRID
      int hgx, hgy, sgx, sgy;
      world_to_grid(hx, hy, &hgx, &hgy);
      world_to_grid(sx, sy, &sgx, &sgy);

      if (!is_valid_cell(hgx, hgy)) continue;
      if (!is_valid_cell(sgx, sgy)) continue;

      // ==========================================================
      // [LABEL] OBSTACLE LINE DRAW (BRESENHAM) - EVERY CELL OBSTACLE
      // ==========================================================
      int x0g = hgx, y0g = hgy;
      int x1g = sgx, y1g = sgy;

      int dx = abs(x1g - x0g);
      int dy = abs(y1g - y0g);
      int sxg = (x0g < x1g) ? 1 : -1;
      int syg = (y0g < y1g) ? 1 : -1;
      int err = dx - dy;

      while (1) {
        if (is_valid_cell(x0g, y0g)) {
          unsigned int cell = grid[y0g][x0g];

          // keep pillars untouched
          if (cell != CELL_BLUE_PILLAR && cell != CELL_YELLOW_PILLAR) {
            obstacle_counter[y0g][x0g]++;
            if (obstacle_counter[y0g][x0g] >= OBSTACLE_THRESHOLD) {
              grid[y0g][x0g] = CELL_OBSTACLE;
              free_counter[y0g][x0g] = 0;
            }
          }
        }

        if (x0g == x1g && y0g == y1g) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0g += sxg; }
        if (e2 <  dx) { err += dx; y0g += syg; }
      }
    }
  }
}




// [LABEL] CLEAR ALL CELLS OF A GIVEN PILLAR TYPE (BLUE or YELLOW)
static void clear_all_pillars_of_type(unsigned int pillar_cell_type) {
  for (int y = 0; y < GRID_SIZE; y++) {
    for (int x = 0; x < GRID_SIZE; x++) {
      if (grid[y][x] == pillar_cell_type) {
        grid[y][x] = CELL_UNKNOWN;
        obstacle_counter[y][x] = 0;
        free_counter[y][x] = 0;
      }
    }
  }
}


// Simple drop-in replacement:
// - Adds `max_pixels` right after `min_pixels`.
// - If *no valid depth* is found around the centroid => returns false (not counted as pillar).
// - Improves near-pillar accuracy by taking a *median depth* from a small patch instead of 1 pixel
//   (reduces jitter when close / partial occlusion / edge mixing).

static bool detect_and_mark_pillar_on_grid(
  WbDeviceTag cam_rgb,
  WbDeviceTag rf_depth,
  TargetColor target_color,
  unsigned int pillar_cell_type,
  int min_pixels,
  int max_pixels,            // NEW: reject too-large blobs
  int centroid_step,
  int pillar_radius_cells,
  double fov_horizontal,
  double distance_scale,
  WbNodeRef robot,
  int *out_gx,
  int *out_gy
) {
  // [LABEL] ARG CHECKS
  if (!cam_rgb || !rf_depth || !robot) return false;
  if (pillar_cell_type != CELL_BLUE_PILLAR && pillar_cell_type != CELL_YELLOW_PILLAR) return false;
  if (min_pixels < 1) min_pixels = 1;
  if (max_pixels < min_pixels) max_pixels = min_pixels;

  // [LABEL] 1) CENTROID IN IMAGE (KEEP YOUR ORIGINAL LOGIC)
  Centroid2D cen = find_color_centroid(cam_rgb, target_color, min_pixels, centroid_step);
  if (!cen.valid) return false;

  // NEW: max-pixels gate (uses the same pixel set that produced the centroid)
  // Change `cen.count` to your actual field name if needed.
  if (cen.count > max_pixels) return false;

  // [LABEL] 2) DEPTH AT CENTROID (IF NO DEPTH => NOT A PILLAR)
  DepthAtCentroid dep = depth_at_centroid(rf_depth, cen.cx, cen.cy);
  if (!dep.valid) return false;  // this enforces “no depth => not counted”

  // [LABEL] 3) BEARING FROM CENTROID X
  int w = wb_camera_get_width(cam_rgb);
  if (w <= 0) return false;
  double bearing = -((double)cen.cx - 0.5 * (double)w) * (fov_horizontal / (double)w);

  // [LABEL] 4) ROBOT POSE
  const double *pos = wb_supervisor_node_get_position(robot);
  const double *ori = wb_supervisor_node_get_orientation(robot);
  double robot_x = pos[0];
  double robot_y = pos[1];
  double theta   = atan2(ori[3], ori[0]);

  // [LABEL] 5) PROJECT TO WORLD
  double d = (double)dep.dist * distance_scale;
  if (!isfinite(d) || d < 0.05) return false;

  double world_ang = theta + bearing;
  double wx = robot_x + d * cos(world_ang);
  double wy = robot_y + d * sin(world_ang);

  // NEW: near-range stabilization (does not change color detection)
  // Strong smoothing when close (where your accuracy degrades).
  double alpha;
  if (d < 0.40)      alpha = 0.15;  // very close => heavy smoothing
  else if (d < 1.80) alpha = 0.25;
  else if (d < 2.50) alpha = 0.40;
  else               alpha = 0.60;

  // separate filters for blue vs yellow
  static bool has_b = false, has_y = false;
  static double fbx = 0.0, fby = 0.0;
  static double fyx = 0.0, fyy = 0.0;

  if (pillar_cell_type == CELL_BLUE_PILLAR) {
    if (!has_b) { fbx = wx; fby = wy; has_b = true; }
    else { fbx = (1.0 - alpha) * fbx + alpha * wx; fby = (1.0 - alpha) * fby + alpha * wy; }
    wx = fbx; wy = fby;
  } else { // CELL_YELLOW_PILLAR
    if (!has_y) { fyx = wx; fyy = wy; has_y = true; }
    else { fyx = (1.0 - alpha) * fyx + alpha * wx; fyy = (1.0 - alpha) * fyy + alpha * wy; }
    wx = fyx; wy = fyy;
  }

  // [LABEL] 6) WORLD -> GRID CENTER
  int gx, gy;
  world_to_grid(wx, wy, &gx, &gy);
  if (!is_valid_cell(gx, gy)) return false;

  // [LABEL] 7) CLEAR ALL OLD PILLARS OF THIS TYPE (ONLY WHEN NEW DETECTED)
  clear_all_pillars_of_type(pillar_cell_type);

  // [LABEL] 8) DRAW NEW PILLAR DISK
  int r = pillar_radius_cells;
  if (r < 1) r = 1;
  int r2 = r * r;

  for (int dy = -r; dy <= r; dy++) {
    for (int dx = -r; dx <= r; dx++) {
      if (dx*dx + dy*dy > r2) continue;

      int x = gx + dx;
      int y = gy + dy;
      if (!is_valid_cell(x, y)) continue;

      if (grid[y][x] == CELL_ROBOT) continue;
      if (grid[y][x] == CELL_OBSTACLE) continue;

      grid[y][x] = pillar_cell_type;
      obstacle_counter[y][x] = 0;
      free_counter[y][x] = 0;
    }
  }

  if (out_gx) *out_gx = gx;
  if (out_gy) *out_gy = gy;
  return true;
}








static void safety_mark_front_obstacle(int frame_count) {
#if SAFETY_MARK_ENABLE
  if (SAFETY_MARK_EVERY_N_TICKS > 1) {
    if ((frame_count % SAFETY_MARK_EVERY_N_TICKS) != 0) return;
  }

  // robot pose
  const double *pos = wb_supervisor_node_get_position(robot_node);
  const double *ori = wb_supervisor_node_get_orientation(robot_node);
  if (!pos || !ori) return;

  double rx = pos[0], ry = pos[1];
  double theta = atan2(ori[3], ori[0]); // your yaw convention

  // fixed point in front along robot forward ray
  double d = SAFETY_MARK_DIST_M;
  if (!isfinite(d) || d <= 0.0) return;

  double wx = rx + d * cos(theta);
  double wy = ry + d * sin(theta);

  int gx, gy;
  world_to_grid(wx, wy, &gx, &gy);
  if (!is_valid_cell(gx, gy)) return;

  mark_obstacle_disk(gx, gy, SAFETY_MARK_RADIUS_CELLS);
#endif
}



static bool pillar_pixels_trigger_bypass(WbDeviceTag cam_rgb) {
#if !PILLAR_BYPASS_ENABLE
  (void)cam_rgb;
  return false;
#else
  if (!cam_rgb) return false;

  int w = wb_camera_get_width(cam_rgb);
  int h = wb_camera_get_height(cam_rgb);
  const unsigned char *img = wb_camera_get_image(cam_rgb);
  if (!img || w <= 0 || h <= 0) return false;

  int step = PILLAR_BYPASS_STEP_PIX;
  if (step < 1) step = 1;

  int y0 = (int)lround(PILLAR_BYPASS_ROI_Y0_FRAC * (double)h);
  int y1 = (int)lround(PILLAR_BYPASS_ROI_Y1_FRAC * (double)h);
  y0 = clampi(y0, 0, h - 1);
  y1 = clampi(y1, 0, h);
  if (y1 <= y0) return false;

  int blue_cnt = 0;
  int yellow_cnt = 0;

  for (int y = y0; y < y1; y += step) {
    for (int x = 0; x < w; x += step) {
      int r = wb_camera_image_get_red(img,   w, x, y);
      int g = wb_camera_image_get_green(img, w, x, y);
      int b = wb_camera_image_get_blue(img,  w, x, y);

      if (color_match(COLOR_BLUE, r, g, b)) {
        blue_cnt++;
        if (blue_cnt >= PILLAR_BYPASS_BLUE_PIXELS) return true;
      }

      if (color_match(COLOR_YELLOW, r, g, b)) {
        yellow_cnt++;
        if (yellow_cnt >= PILLAR_BYPASS_YELLOW_PIXELS) return true;
      }
    }
  }

  return false;
#endif
}



















static void safety_update_per_tick(int frame_count) {
  safety_read_ir(&safety.min_dist_m, &safety.min_idx);

  // ---------- PITCH DETECTION (anti wall-climb) ----------
  // Uses supervisor orientation matrix (3x3 row-major).
  // Pitch (nose up/down) ~= asin(-r20) = asin(-orientation[6]).
  double robot_pitch = 0.0;
  {
    const double *ori = wb_supervisor_node_get_orientation(robot_node);
    if (ori) {
      double s = -ori[6];
      if (s >  1.0) s =  1.0;
      if (s < -1.0) s = -1.0;
      robot_pitch = asin(s);
    }
  }

  // Tune: around 8..15 degrees is a good "starting to climb" trigger.
  // (If you already have a define, use that instead.)
  #ifndef SAFETY_PITCH_STOP_RAD
  #define SAFETY_PITCH_STOP_RAD (12.0 * M_PI / 180.0)
  #endif

  // Small helpers
  const double dt = TIME_STEP / 1000.0;

  // ---------- DEFAULTS ----------
  safety.v_scale = 1.0;
  safety.w_scale = 1.0;
  safety.hard_stop = false;

  // ---------- LOCAL ANTI-STUCK STATE (inside this function only) ----------
  // We avoid adding new globals/struct fields.
  static int wait_cycles = 0;      // how many WAIT_CLEAR cycles we've done without clearing
  static int turn_ticks = 0;       // post-backup "turn away" ticks
  static int turn_dir = 1;         // +1 or -1

  // If we are pitching up, treat it like a critical safety event
  // (prevents climbing / LiDAR-floor weirdness cascade).
  if (fabs(robot_pitch) > SAFETY_PITCH_STOP_RAD) {
    safety.state = SAFETY_RECOVER_BACKUP;
    safety_mark_front_obstacle(frame_count);
    safety.backup_ticks = (int)(BACKUP_TIME_SEC / dt);

    // Choose a deterministic turn direction based on which IR was closest
    turn_dir = (safety.min_idx >= 0 && (safety.min_idx % 2)) ? -1 : +1;
    turn_ticks = (int)(0.35 / dt);   // short turn after backup
    wait_cycles = 0;

    stop_all_motors();
    safety.hard_stop = true;

    // Optional debug
    // printf("PITCH: %.3f rad -> FORCE RECOVER\n", robot_pitch);
    return;
  }

  // ---------- PILLAR BYPASS: keep it, but keep it SAFE ----------
  // We still bypass the state machine, but we do NOT allow full-speed slam.
  bool bypass_ok = false;

  // 1) Near-target bypass
  {
    const double *pos = wb_supervisor_node_get_position(robot_node);
    if (pos) {
      int rgx, rgy;
      world_to_grid(pos[0], pos[1], &rgx, &rgy);

      if (!blue_reached && blue_seen && blue_target_gx >= 0 && blue_target_gy >= 0) {
        int dx = rgx - blue_target_gx;
        int dy = rgy - blue_target_gy;
        if ((dx*dx + dy*dy) <= (NEARBY_RADIUS_INNER * NEARBY_RADIUS_INNER)) bypass_ok = true;
      }

      if (blue_reached && !yellow_reached && yellow_seen &&
          yellow_target_gx >= 0 && yellow_target_gy >= 0) {
        int dx = rgx - yellow_target_gx;
        int dy = rgy - yellow_target_gy;
        if ((dx*dx + dy*dy) <= (NEARBY_RADIUS_INNER * NEARBY_RADIUS_INNER)) bypass_ok = true;
      }
    }
  }

  // 2) Pixel-only bypass
  if (!bypass_ok && pillar_pixels_trigger_bypass(camera_rgb))
    bypass_ok = true;

  if (bypass_ok) {
    // Don’t enter the recovery machine, but keep conservative scaling.
    // And if IR is truly critical, we still recover.
    if (safety.min_idx >= 0 && isfinite(safety.min_dist_m) &&
        safety.min_dist_m < IR_CRITICAL_THRESHOLD) {
      // fall through to normal critical handling below (no early return)
    } else {
      safety.state = SAFETY_OK;
      safety.v_scale = 0.35;   // <= IMPORTANT: prevent wall climb near pillar
      safety.w_scale = 0.80;
      safety.hard_stop = false;
      return;
    }
  }

  // ---------- RECOVERY / ESCAPE STATE MACHINE ----------
  switch (safety.state) {

    // After backing up, optionally do a short turn-away to avoid re-hitting the same thing.
    case SAFETY_RECOVER_BACKUP: {
      set_wheel_speeds(-BACKUP_WHEEL_SPEED, -BACKUP_WHEEL_SPEED);
      safety.hard_stop = true;

      if (--safety.backup_ticks <= 0) {
        if (turn_ticks > 0) {
          // go to turning phase (implemented using WAIT_CLEAR state + local flag)
          safety.state = SAFETY_RECOVER_WAIT_CLEAR;
          safety.wait_ticks = 0;  // we’ll use turn_ticks first
        } else {
          safety.state = SAFETY_RECOVER_WAIT_CLEAR;
          safety.wait_ticks = (int)(0.35 / dt);
        }
      }
      return;
    }

    case SAFETY_RECOVER_WAIT_CLEAR: {
      safety_mark_front_obstacle(frame_count);
      safety.hard_stop = true;

      // If we have a pending "turn away" phase, do it first.
      if (turn_ticks > 0) {
        // Turn in place away from the closest IR side
        const double w = BACKUP_WHEEL_SPEED * 0.85;
        set_wheel_speeds(+turn_dir * w, -turn_dir * w);
        turn_ticks--;
        return;
      }

      // Otherwise stop and wait a bit
      stop_all_motors();

      if (safety.wait_ticks > 0) {
        safety.wait_ticks--;
        return;
      }

      // If clear, exit recovery
      if (safety.min_dist_m > FRONT_CLEAR_RESUME_M) {
        safety.state = SAFETY_OK;
        wait_cycles = 0;
        return;
      }

      // Not clear: anti-stuck escalation
      // After a few failed waits, do another backup + forced short turn.
      wait_cycles++;
      if (wait_cycles >= 3) {
        wait_cycles = 0;

        safety.state = SAFETY_RECOVER_BACKUP;
        safety.backup_ticks = (int)(0.25 / dt);   // shorter "bump back"
        turn_dir = (safety.min_idx >= 0 && (safety.min_idx % 2)) ? -1 : +1;
        turn_ticks = (int)(0.30 / dt);

        return;
      }

      // Otherwise, keep waiting (re-arm a short wait window)
      safety.wait_ticks = (int)(0.20 / dt);
      return;
    }

    default:
      break;
  }

  // ---------- NORMAL MONITORING ----------
  if (safety.min_idx >= 0 && isfinite(safety.min_dist_m)) {

    if (safety.min_dist_m < IR_CRITICAL_THRESHOLD) {
      safety.state = SAFETY_RECOVER_BACKUP;
      safety_mark_front_obstacle(frame_count);

      safety.backup_ticks = (int)(BACKUP_TIME_SEC / dt);

      // plan a turn after backup to escape the same hit
      turn_dir = (safety.min_idx >= 0 && (safety.min_idx % 2)) ? -1 : +1;
      turn_ticks = (int)(0.35 / dt);
      wait_cycles = 0;

      stop_all_motors();
      safety.hard_stop = true;

      printf("CRITICAL: IR %s %.3fm -> EMERGENCY + BACKUP\n",
             ir_sensor_names[safety.min_idx], safety.min_dist_m);
      return;
    }

    if (safety.min_dist_m < IR_SAFETY_THRESHOLD) {
      safety.state = SAFETY_WARN;
      safety_mark_front_obstacle(frame_count);

      double t = (safety.min_dist_m - IR_CRITICAL_THRESHOLD) /
                 (IR_SAFETY_THRESHOLD - IR_CRITICAL_THRESHOLD);
      t = clamp(t, 0.0, 1.0);

      // IMPORTANT: don’t creep forward into walls; bias to turning.
      safety.v_scale = 0.0 + 0.25 * t;   // 0..0.25
      safety.w_scale = 0.40 + 0.60 * t;  // 0.40..1.0
      return;
    }
  }

  safety.state = SAFETY_OK;
}


























































// ============ MAIN ================

int main(int argc, char **argv) {
    wb_robot_init();
    
    robot_node = wb_supervisor_node_get_self();
    
    camera_rgb = wb_robot_get_device("camera rgb");
    if (camera_rgb) wb_camera_enable(camera_rgb, TIME_STEP);
    
    camera_depth = wb_robot_get_device("camera depth");   // <-- use your device name
    if (camera_depth) wb_range_finder_enable(camera_depth, TIME_STEP);
    
    display = wb_robot_get_device("display");
    if (!display) {
        printf("Error: No display device found!\n");
        return 1;
    }
    
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_set_font(display, "Arial", 10, 0);
    
    lidar = wb_robot_get_device("laser");
    if (!lidar) {
        printf("Error: No lidar device found!\n");
        return 1;
    }
    wb_lidar_enable(lidar, TIME_STEP);
    
    init_motors();


    init_ir_sensors();
    safety_init();
    init_grid();
   
    
    int frame_count = 0;
    process_lidar();
    generate_cost_map();
    
    State state = STATE_START;

    
    
    //rotate_drive(2, 0.3, 0.287, 0.0825);












    
    
    
    
    
    int gx, gy;
    
// ======================= MAIN LOOP (REWRITE) =======================
// NOTE: This loop assumes you added the enum value:
//   STATE_MARKING_PILLARS
// somewhere in your State enum.
//
// NO detect_and_mark_pillar_on_grid() is used anymore.
// Pillars are ONLY written to grid when STATE_MARKING_PILLARS finishes.
//
// Reuses ONLY existing globals:
//   blue_cooldown_ticks / yellow_cooldown_ticks   -> cooldown timers (blue / yellow)
//   scan_active                    -> which color we are currently marking (true=blue, false=yellow)
//   scan_prev_theta/scan_accumulated -> sum_wx / sum_wy accumulators
//   current_waypoint               -> sample_count accumulator while marking
//   path_update_counter            -> marking "phase/steps" counter while marking
// (current_path is cleared on entry to marking)

while (wb_robot_step(TIME_STEP) != -1) {
  frame_count++;

  // 1) SAFETY UPDATE (IR)
  safety_update_per_tick(frame_count);

  const double* orientation = wb_supervisor_node_get_orientation(robot_node);
  double robot_theta = atan2(orientation[3], orientation[0]);
  double robot_pitch = asin(-orientation[6]);


  // Safety has priority over everything
  if (safety.hard_stop) {
    process_lidar();
    if (frame_count % 50 == 0) {
      decay_counters();
      filter_connected_components(MIN_BLOB_SIZE);
    }
    generate_cost_map();
    if (frame_count % 5 == 0) render_display_with_path(frame_count);
    continue;
  }

  // 2) LIDAR MAP UPDATE (skip if rotating fast)
  if (should_integrate_lidar(robot_theta, robot_pitch)) process_lidar();
  else decay_counters();

  if (frame_count % 200 == 0) {
    decay_counters();
    filter_connected_components(MIN_BLOB_SIZE);
  }

  generate_cost_map();
  if (frame_count % 5 == 0) render_display_with_path(frame_count);

  // 3) Decrement pillar cooldowns (blue=blue_cooldown_ticks, yellow=yellow_cooldown_ticks)
  if (blue_cooldown_ticks > 0) blue_cooldown_ticks--;
  if (yellow_cooldown_ticks > 0) yellow_cooldown_ticks--;

  // 4) Always keep marking other obstacles
  detect_green_to_grid(
    camera_rgb, camera_depth, robot_node,
    GREEN_FOV_HORIZONTAL, GREEN_STEP_PIXELS, GREEN_BASE_RADIUS_CELLS,
    GREEN_DISTANCE_SCALE, GREEN_MIN_DEPTH_M, GREEN_MAX_DEPTH_M,
    GREEN_ROI_Y0_FRAC, GREEN_ROI_Y1_FRAC, GREEN_CONST_MARK_DIST_M
  );

  detect_red_to_grid(
    camera_rgb, camera_depth, robot_node,
    RED_FOV_HORIZONTAL, RED_STEP_PIXELS,
    RED_DISTANCE_SCALE, RED_MIN_DEPTH_M, RED_MAX_DEPTH_M,
    RED_SEGMENT_LEN_M,
    RED_ROI_X0_FRAC, RED_ROI_X1_FRAC,
    RED_ROI_Y0_FRAC
  );








  // ===================== STATE MACHINE =====================
  switch (state) {

    case STATE_START: {
      if (frame_count <= STARTUP_CLEAR_STEPS) {
        stop_all_motors();
        reset_pid_controllers();

        init_grid();
        clear_current_path_if_any();

        generate_cost_map();
        if (frame_count % 2 == 0) render_display_with_path(frame_count);
        break;
      }

      // Start exploring immediately; marking state will be triggered by pixels.
      state = STATE_FRONTIER_EXPLORE;
      break;
    }

  




case STATE_MARKING_PILLARS: {
  // hard stop while marking
  stop_all_motors();
  reset_pid_controllers();
  printf("PILLAR_STATE\n");

  // Phase init: current_waypoint < 0 means "just entered"
  if (current_waypoint < 0) {
    clear_current_path_if_any();
    nav_replan_counter = 0;

    current_waypoint = 0;   // sample_count
    scan_prev_theta = 0.0;  // sum_wx
    scan_accumulated = 0.0; // sum_wy

    path_update_counter = (int)(1.8 / (TIME_STEP / 1000.0)); // time budget
  }

  // Find centroid for chosen color (pixel-only)
  Centroid2D cen = find_color_centroid(
    camera_rgb,
    scan_active ? COLOR_BLUE : COLOR_YELLOW,
    scan_active ? PILLAR_BYPASS_BLUE_PIXELS : PILLAR_BYPASS_YELLOW_PIXELS,
    2
  );

  // Blob gone or timed out -> exit marking, apply cooldown to avoid thrash
  if (!cen.valid || path_update_counter <= 0) {
    if (scan_active) blue_cooldown_ticks      = (int)(10 / (TIME_STEP / 1000.0));
    else            yellow_cooldown_ticks = (int)(10 / (TIME_STEP / 1000.0));

    path_update_counter = 0;

    if (!blue_reached) state = (blue_seen ? STATE_NAV_TO_BLUE : STATE_FRONTIER_EXPLORE);
    else               state = (yellow_seen ? STATE_NAV_TO_YELLOW : STATE_FRONTIER_EXPLORE);
    break;
  }

  // Depth at centroid
  DepthAtCentroid dep = depth_at_centroid(camera_depth, cen.cx, cen.cy);

  // -------------------------------------------------------
  // NO DEPTH (too close / invalid) FALLBACK:
  // If this pillar type is NOT already on the map, mark ONCE
  // using a constant distance. Then exit marking.
  // If already marked, just exit to avoid staring forever.
  // -------------------------------------------------------
if (!dep.valid) {
  unsigned int pillar_type = scan_active ? CELL_BLUE_PILLAR : CELL_YELLOW_PILLAR;

  // ---------- Decide if this "no depth" looks like TOO-CLOSE (allowed) ----------
  int wimg = wb_camera_get_width(camera_rgb);
  int himg = wb_camera_get_height(camera_rgb);

  // pixel-count thresholds you already have
  const int min_close_pixels = scan_active ? PILLAR_MIN_BLUE_PIXELS : PILLAR_MIN_YELLOW_PIXELS;

  bool big_in_image = (cen.count >= min_close_pixels);

  // centroid lower in image => usually closer/bigger (helps reject "peeking from behind wall")
  bool low_centroid = false;
  if (himg > 0) {
    double cy_frac = (double)cen.cy / (double)himg;
    low_centroid = (cy_frac >= PILLAR_FALLBACK_MIN_CY_FRAC);
  }

  bool ir_close = (safety.min_idx >= 0 &&
                   isfinite(safety.min_dist_m) &&
                   safety.min_dist_m <= PILLAR_FALLBACK_IR_MAX_M);

#if PILLAR_FALLBACK_ENABLE
  bool allow_fallback = big_in_image && low_centroid;
  if (PILLAR_FALLBACK_REQUIRE_IR) allow_fallback = allow_fallback && ir_close;
#else
  bool allow_fallback = false;
#endif

  // If it's NOT close-like, it's probably out-of-range/occluded -> do NOT mark anything.
  if (!allow_fallback) {
    if (scan_active) blue_cooldown_ticks  = (int)(10.0 / (TIME_STEP / 1000.0));
    else            yellow_cooldown_ticks = (int)(10.0 / (TIME_STEP / 1000.0));

    path_update_counter = 0;
    if (!blue_reached) state = (blue_seen ? STATE_NAV_TO_BLUE : STATE_FRONTIER_EXPLORE);
    else               state = (yellow_seen ? STATE_NAV_TO_YELLOW : STATE_FRONTIER_EXPLORE);
    break;
  }

  // ---------- Fallback marking (ONLY when "close-like") ----------
  bool already_marked = false;
  for (int yy = 0; yy < GRID_SIZE && !already_marked; yy++) {
    for (int xx = 0; xx < GRID_SIZE; xx++) {
      if (grid[yy][xx] == pillar_type) { already_marked = true; break; }
    }
  }

  // Only mark if we haven't already marked this pillar type
  if (!already_marked) {
    const double *pos = wb_supervisor_node_get_position(robot_node);
    const double *ori = wb_supervisor_node_get_orientation(robot_node);
    double rx = pos[0], ry = pos[1];
    double theta = atan2(ori[3], ori[0]);

    // bearing from centroid x
    double bearing = -((double)cen.cx - 0.5 * (double)wimg) * (1.04 / (double)wimg);

    // project to world at fallback distance
    double ang = theta + bearing;
    double wx = rx + FALLBACK_DIST_M * cos(ang);
    double wy = ry + FALLBACK_DIST_M * sin(ang);

    int pgx, pgy;
    world_to_grid(wx, wy, &pgx, &pgy);

    if (is_valid_cell(pgx, pgy)) {
      clear_all_pillars_of_type(pillar_type);

      if (scan_active) { blue_seen = true;   blue_target_gx = pgx;   blue_target_gy = pgy; }
      else            { yellow_seen = true;  yellow_target_gx = pgx; yellow_target_gy = pgy; }

      int r = PILLAR_RADIUS_CELLS; if (r < 1) r = 1;
      int r2 = r * r;

      for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
          if (dx*dx + dy*dy > r2) continue;
          int x = pgx + dx, y = pgy + dy;
          if (!is_valid_cell(x, y)) continue;
          if (grid[y][x] == CELL_OBSTACLE) continue;
          if (grid[y][x] == CELL_ROBOT) continue;
          grid[y][x] = pillar_type;
          obstacle_counter[y][x] = 0;
          free_counter[y][x] = 0;
        }
      }
    }
  }

  // exit marking
  if (scan_active) blue_cooldown_ticks  = (int)(10.0 / (TIME_STEP / 1000.0));
  else            yellow_cooldown_ticks = (int)(10.0 / (TIME_STEP / 1000.0));

  path_update_counter = 0;
  if (!blue_reached) state = (blue_seen ? STATE_NAV_TO_BLUE : STATE_FRONTIER_EXPLORE);
  else               state = (yellow_seen ? STATE_NAV_TO_YELLOW : STATE_FRONTIER_EXPLORE);
  break;
}


  // If TOO CLOSE: do NOT align. Just sample (prevents stuck aligning)
  if (dep.dist < 0.25f) {
    // fall through to sampling (no rotation)
  } else {
    // Align centroid to image center horizontally
    int w = wb_camera_get_width(camera_rgb);
    double ex = (double)cen.cx - 0.5 * (double)w; // + => blob right

    if (fabs(ex) > 6.0) {
      double dir = (ex > 0.0) ? -1.0 : 1.0;
      set_wheel_speeds(-dir * 1.2, dir * 1.2);
      path_update_counter--;
      break; // keep aligning
    }
  }

  // --- aligned OR too-close: take a measurement sample ---
  {
    const double *pos = wb_supervisor_node_get_position(robot_node);
    const double *ori = wb_supervisor_node_get_orientation(robot_node);
    double rx = pos[0], ry = pos[1];
    double theta = atan2(ori[3], ori[0]);

    int w = wb_camera_get_width(camera_rgb);
    double bearing = -((double)cen.cx - 0.5 * (double)w) * (1.04 / (double)w);

    double d = (double)dep.dist * 1.0; // distance_scale
    double ang = theta + bearing;
    double wx = rx + d * cos(ang);
    double wy = ry + d * sin(ang);

    scan_prev_theta += wx;
    scan_accumulated += wy;
    current_waypoint++;
  }

  // Finish when enough samples OR timeout nearing end
  if (current_waypoint >= 6 || path_update_counter <= (int)(0.3 / (TIME_STEP / 1000.0))) {
    double wx = scan_prev_theta / (double)current_waypoint;
    double wy = scan_accumulated / (double)current_waypoint;

    int pgx, pgy;
    world_to_grid(wx, wy, &pgx, &pgy);

    if (is_valid_cell(pgx, pgy)) {
      unsigned int pillar_type = scan_active ? CELL_BLUE_PILLAR : CELL_YELLOW_PILLAR;

      // clear old of this type (we only finalize once)
      clear_all_pillars_of_type(pillar_type);

      if (scan_active) {
        blue_seen = true;
        blue_target_gx = pgx;
        blue_target_gy = pgy;
      } else {
        yellow_seen = true;
        yellow_target_gx = pgx;
        yellow_target_gy = pgy;
      }

      int r = PILLAR_RADIUS_CELLS;
      if (r < 1) r = 1;
      int r2 = r * r;

      for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
          if (dx*dx + dy*dy > r2) continue;

          int x = pgx + dx;
          int y = pgy + dy;
          if (!is_valid_cell(x, y)) continue;
          if (grid[y][x] == CELL_OBSTACLE) continue;
          if (grid[y][x] == CELL_ROBOT) continue;

          grid[y][x] = pillar_type;
          obstacle_counter[y][x] = 0;
          free_counter[y][x] = 0;
        }
      }

      // cooldown
      if (scan_active) blue_cooldown_ticks      = (int)(10.0 / (TIME_STEP / 1000.0));
      else            yellow_cooldown_ticks = (int)(10.0 / (TIME_STEP / 1000.0));
    }

    // exit marking
    path_update_counter = 0;

    if (!blue_reached) state = (blue_seen ? STATE_NAV_TO_BLUE : STATE_FRONTIER_EXPLORE);
    else               state = (yellow_seen ? STATE_NAV_TO_YELLOW : STATE_FRONTIER_EXPLORE);
    break;
  }

  // keep sampling for a short while
  stop_all_motors();
  path_update_counter--;
  break;
}


























    // -------------------------------------------------------
    // FRONTIER EXPLORE:
    // - keeps looking for BOTH pillars and triggers marking state
    // -------------------------------------------------------
    case STATE_FRONTIER_EXPLORE: {
        printf("FRONTIER_STATE\n");
      // Trigger marking for BLUE if cooldown over
      if (blue_cooldown_ticks == 0) {
        Centroid2D cB = find_color_centroid(camera_rgb, COLOR_BLUE,
                                            PILLAR_BYPASS_BLUE_PIXELS, 2);
        if (cB.valid) {
          scan_active = true;          // marking BLUE
          path_update_counter = 0;
          current_waypoint = -1;     // init marker phase next tick
          state = STATE_MARKING_PILLARS;
          break;
        }
      }

      // Trigger marking for YELLOW if cooldown over
      if (yellow_cooldown_ticks == 0) {
        Centroid2D cY = find_color_centroid(camera_rgb, COLOR_YELLOW,
                                            PILLAR_BYPASS_YELLOW_PIXELS, 2);
        if (cY.valid) {
          scan_active = false;         // marking YELLOW
          path_update_counter = 0;
          current_waypoint = -1;
          state = STATE_MARKING_PILLARS;
          break;
        }
      }

      // transitions: if blue not reached and known -> go
      if (!blue_reached && blue_seen) {
        clear_current_path_if_any();
        reset_pid_controllers();
        nav_replan_counter = 0;
        state = STATE_NAV_TO_BLUE;
        break;
      }

      if (blue_reached && yellow_seen) {
        clear_current_path_if_any();
        reset_pid_controllers();
        nav_replan_counter = 0;
        state = STATE_NAV_TO_YELLOW;
        break;
      }

      // follow or update frontier path
      if (current_path) {
        if (follow_path()) {
          destroy_grid_path(current_path);
          current_path = NULL; // no-op (keeps “no new variables” intent)
        }
        if (frame_count % 50 == 0) {
          destroy_grid_path(current_path);
          current_path = NULL;
        }
      }

      if (!current_path && frame_count % 50 == 0) {
        path_update_counter++;
        update_path_to_frontier(frame_count);
      }
      break;
    }


















    // -------------------------------------------------------
    // NAV TO BLUE:
    // - keeps looking for BOTH pillars and triggers marking state
    // -------------------------------------------------------
    case STATE_NAV_TO_BLUE: {
    printf("BLUE_STATE\n");
      if (!blue_seen) { state = STATE_FRONTIER_EXPLORE; break; }


      // approach goal selection (same as your previous logic)
      const double* pos = wb_supervisor_node_get_position(robot_node);
      int rgx, rgy;
      world_to_grid(pos[0], pos[1], &rgx, &rgy);

      int pgx = blue_target_gx;
      int pgy = blue_target_gy;

              // If we are already close to the currently known pillar target, do NOT enter marking.
        // Otherwise you stop forever near the pillar.
        int ddx = rgx - pgx;
        int ddy = rgy - pgy;
        int d2  = ddx*ddx + ddy*ddy;

        // tune: 8 cells @ 0.04m = 0.32m
        const int CLOSE_MARK_BLOCK_R = 8;
        bool block_marking = (d2 <= (CLOSE_MARK_BLOCK_R * CLOSE_MARK_BLOCK_R));


        // Trigger marking for BLUE if cooldown over (but not when already close)
        if (!block_marking && blue_cooldown_ticks == 0) {
          Centroid2D cB = find_color_centroid(camera_rgb, COLOR_BLUE,
                                              PILLAR_BYPASS_BLUE_PIXELS, 2);
          if (cB.valid) {
            scan_active = true;
            path_update_counter = 0;
            current_waypoint = -1;
            state = STATE_MARKING_PILLARS;
            break;
          }
        }

        // Trigger marking for YELLOW if cooldown over (but not when already close)
        if (yellow_cooldown_ticks == 0) {
          Centroid2D cY = find_color_centroid(camera_rgb, COLOR_YELLOW,
                                              PILLAR_BYPASS_YELLOW_PIXELS, 2);
          if (cY.valid) {
            scan_active = false;
            path_update_counter = 0;
            current_waypoint = -1;
            state = STATE_MARKING_PILLARS;
            break;
          }
        }


      

      int vx = rgx - pgx;
      int vy = rgy - pgy;
      int sx = (vx > 0) ? 1 : (vx < 0) ? -1 : 0;
      int sy = (vy > 0) ? 1 : (vy < 0) ? -1 : 0;
      if (sx == 0 && sy == 0) sx = 1;

      int agx = -1, agy = -1;
      for (int k = STANDOFF; k < STANDOFF + SEARCH; k++) {
        int tx = pgx + sx * k;
        int ty = pgy + sy * k;
        if (!is_valid_cell(tx, ty)) continue;
        if (grid[ty][tx] != CELL_OBSTACLE) { agx = tx; agy = ty; break; }
      }

      if (agx < 0) { clear_current_path_if_any(); state = STATE_FRONTIER_EXPLORE; break; }

      if (!ensure_path_to_goal(agx, agy)) {
        clear_current_path_if_any();
        state = STATE_FRONTIER_EXPLORE;
        break;
      }

      if (follow_path()) {
        blue_reached = true;
        clear_current_path_if_any();
        reset_pid_controllers();
        nav_replan_counter = 0;

        state = (yellow_seen ? STATE_NAV_TO_YELLOW : STATE_FRONTIER_EXPLORE);
      }

      break;
    }













    // -------------------------------------------------------
    // NAV TO YELLOW:
    // - ONLY look for YELLOW (per your requirement)
    // -------------------------------------------------------
    case STATE_NAV_TO_YELLOW: {
        printf("YELLOW_STATE\n");
      if (!yellow_seen) { state = STATE_FRONTIER_EXPLORE; break; }



      const double* pos = wb_supervisor_node_get_position(robot_node);
      int rgx, rgy;
      world_to_grid(pos[0], pos[1], &rgx, &rgy);

      int pgx = yellow_target_gx;
      int pgy = yellow_target_gy;

        // If we are already close to the currently known pillar target, do NOT enter marking.
        // Otherwise you stop forever near the pillar.
        int ddx = rgx - pgx;
        int ddy = rgy - pgy;
        int d2  = ddx*ddx + ddy*ddy;

        // tune: 8 cells @ 0.04m = 0.32m
        const int CLOSE_MARK_BLOCK_R = 4;
        bool block_marking = (d2 <= (CLOSE_MARK_BLOCK_R * CLOSE_MARK_BLOCK_R));



        /*

        // Trigger marking for YELLOW if cooldown over (but not when already close)
        if (!block_marking && yellow_cooldown_ticks == 0) {
          Centroid2D cY = find_color_centroid(camera_rgb, COLOR_YELLOW,
                                              PILLAR_BYPASS_YELLOW_PIXELS, 2);
          if (cY.valid) {
            scan_active = false;
            path_update_counter = 0;
            current_waypoint = -1;
            state = STATE_MARKING_PILLARS;
            break;
          }
        }*/



      int vx = rgx - pgx;
      int vy = rgy - pgy;
      int sx = (vx > 0) ? 1 : (vx < 0) ? -1 : 0;
      int sy = (vy > 0) ? 1 : (vy < 0) ? -1 : 0;
      if (sx == 0 && sy == 0) sx = 1;

      int agx = -1, agy = -1;
      for (int k = STANDOFF; k < STANDOFF + SEARCH; k++) {
        int tx = pgx + sx * k;
        int ty = pgy + sy * k;
        if (!is_valid_cell(tx, ty)) continue;
        if (grid[ty][tx] != CELL_OBSTACLE) { agx = tx; agy = ty; break; }
      }

      if (agx < 0) { clear_current_path_if_any(); state = STATE_FRONTIER_EXPLORE; break; }

      if (!ensure_path_to_goal(agx, agy)) {
        clear_current_path_if_any();
        state = STATE_FRONTIER_EXPLORE;
        break;
      }

      if (follow_path()) {
        yellow_reached = true;
        clear_current_path_if_any();
        reset_pid_controllers();
        nav_replan_counter = 0;
        state = STATE_FRONTIER_EXPLORE;
      }

      break;
    }

    case STATE_COLLISION:
      state = STATE_WANDER;
      break;

    case STATE_WANDER:
      state = STATE_FRONTIER_EXPLORE;
      break;

    // keep your old scan state if you still have it, otherwise fall back:
    case STATE_SCAN_COLORS:
      state = STATE_FRONTIER_EXPLORE;
      break;
  }
}

    
    wb_robot_cleanup();
    return 0;
}
