#ifndef CONFIG_H
#define CONFIG_H

// Webots Configuration
#define TIMESTEP 32
#define GRID_SIZE 500
#define GRID_RESOLUTION 0.02
#define DISPLAY_WIDTH 500
#define DISPLAY_HEIGHT 500

// Grid cell types
#define CELL_UNKNOWN 0
#define CELL_FREE 1
#define CELL_OBSTACLE 2
#define CELL_ROBOT 3

// Temporal filtering thresholds
#define OBSTACLE_THRESHOLD 2
#define FREE_THRESHOLD 3
#define COUNTER_DECAY 1
#define NEARBY_RADIUS_INNER 5
#define NEARBY_RADIUS_OUTER 15
#define MAX_QUEUE 1000
#define MIN_BLOB_SIZE 8

// Cost Map
#define OBSTACLE_COST 200
#define FREE_COST 1
#define UNKNOWN_COST 30
#define INFLATION_RADIUS 10

// Frontiers
#define MAX_FRONTIERS 100
#define MIN_FRONTIER_SIZE 20
#define MIN_FRONTIER_CLEARANCE 5
#define FRONTIER_SAFETY_CHECK_RADIUS 5

// Colors
#define COLOR_UNKNOWN 0x404040
#define COLOR_FREE 0xC8C8C8
#define COLOR_OBSTACLE 0x000000
#define COLOR_ROBOT 0x0064FF
#define COLOR_BACKGROUND 0x303030

// IR Safety Configuration
#define IR_SAFETY_THRESHOLD 0.10
#define IR_CRITICAL_THRESHOLD 0.05

#endif // CONFIG_H
