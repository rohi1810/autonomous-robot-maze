#ifndef FRONTIER_H
#define FRONTIER_H

#include <stdbool.h>

// Frontier structure
typedef struct {
    int x;
    int y;
    int size;
    double score;
} FrontierCentroid;

// Frontier exploration functions
int find_frontier_centroids(FrontierCentroid *centroids, int max_frontiers);
FrontierCentroid *get_best_frontier(FrontierCentroid *frontiers, int num_frontiers);
void update_path_to_frontier(void);
bool has_safe_clearance(int cx, int cy, int min_clearance);

#endif // FRONTIER_H
