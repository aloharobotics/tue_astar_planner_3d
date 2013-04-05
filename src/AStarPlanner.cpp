/*
 * AStarPlanner.cpp
 *
 *  Created on: Apr 18, 2012
 *      Author: sdries
 */

#include "AStarPlanner.h"

using namespace std;

long AStarPlanner::N_OBJECTS = 0;

long AStarPlanner::CellInfo::N_OBJECTS = 0;

AStarPlanner::AStarPlanner(unsigned int width, unsigned int height) : visited_map_(0) {
    resize(width, height);
    ++N_OBJECTS;
}

AStarPlanner::~AStarPlanner() {
    deleteMap();
    --N_OBJECTS;
}

/*
void AStarPlanner::setCost(int x, int y, double cost) {
    cost_map_[x][y] = cost;
}
*/

void AStarPlanner::setCostmap(const std::vector<double> costmap) {
    costmap_ = costmap;
}

double AStarPlanner::getCost(int x, int y) {
    int k = width_ * y + x;

    const double cost = costmap_[k];

    double new_cost = DBL_MAX;
    /// if the cell is unknown the cost is -1
    /// for the A* search to yield the optimal solution cost > 0
    /// thus set the cost here a little bit under the cost of an obstacle
    /// this biases solutions towards known space however! (and it is really ugly)
    if (cost == Map3DCostValues::get_NO_INFORMATION()) {
        new_cost = Map3DCostValues::get_INSCRIBED_INFLATED_OBSTACLE() - 0.01;
    }
    else if (cost < Map3DCostValues::get_INSCRIBED_INFLATED_OBSTACLE()) {
        new_cost = cost;
    }
    //printf("k = %d, cost = %f\n", k, new_cost);
    return new_cost;
}

void AStarPlanner::resize(unsigned int width, unsigned int height) {
    if (visited_map_) {
        deleteMap();
    }

    width_ = width;
    height_ = height;

    visited_map_ = new double*[width_];
    for(unsigned int i = 0; i < width_; ++i) {
        visited_map_[i] = new double[height_];
    }

    // to make sure the algorithm won't expand off the map, mark all border cells as visited
    // todo: this limits the search space: border cells can now not be used in the solution. FIX
    for(unsigned int x = 0; x < width_; ++x) {
        visited_map_[x][0] = 0;
        visited_map_[x][height_ - 1] = 0;
    }

    for(unsigned int y = 0; y < height_; ++y) {
        visited_map_[0][y] = 0;
        visited_map_[width_ - 1][y] = 0;
    }
}

bool AStarPlanner::plan(int mx_start, int my_start, int mx_goal, int my_goal, std::vector<int>& plan_xs, std::vector<int>& plan_ys) {
    /// - initialize all cells in visited_map to false
    /// - determine minimum cell cost in cost map
    /// this cost will determine how the heuristic cost relates to the cost of traversing a cell!    
    double min_cell_cost = 0.1; //DBL_MAX;
    for(unsigned int x = 1; x < width_ - 1; ++x) {
        for(unsigned int y = 1; y < height_ - 1; ++y) {
            visited_map_[x][y] = DBL_MAX;
            //min_cell_cost = min(getCost(x, y), min_cell_cost);
        }
    }

    priority_queue<CellInfo*, vector<CellInfo*>, compareCellInfos> Q;

    list<CellInfo*> visited_cells;   // remember visited cells to be able to clear memory

    // add start to priority queue and visited map
    CellInfo* start = new CellInfo(mx_start, my_start, 0, calculateHeuristicCost(mx_start, my_start, mx_goal, my_goal, min_cell_cost));
    visited_map_[mx_start][my_start] = 0;
    Q.push(start);

    CellInfo* goal_cell = 0;

    while(!Q.empty() && !goal_cell) {
        CellInfo* c = Q.top();
        visited_cells.push_back(c);
        Q.pop();

        // check if goal is reached
        if (c->x_ == mx_goal && c->y_ == my_goal) {
            // goal reached!
            goal_cell = c;
        } else {
            // direct adjacent expansion
            expandCell(c, -1, 0, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
            expandCell(c, +1, 0, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
            expandCell(c, 0, -1, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
            expandCell(c, 0, +1, 1.0, visited_map_, mx_goal, my_goal, min_cell_cost, Q);

            // diagonal expansion
            expandCell(c, -1, -1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
            expandCell(c, +1, -1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
            expandCell(c, -1, +1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
            expandCell(c, +1, +1, SQRT2, visited_map_, mx_goal, my_goal, min_cell_cost, Q);
        }
    }

    if (goal_cell) {
        while(goal_cell) {
            plan_xs.push_back(goal_cell->x_);
            plan_ys.push_back(goal_cell->y_);
            goal_cell = goal_cell->visited_from_;
        }

        // delete remaining CellInfos in Q
        while(!Q.empty()) {
            delete Q.top();
            Q.pop();
        }
    }

    // delete visted CellInfos
    for(list<CellInfo*>::iterator it = visited_cells.begin(); it != visited_cells.end(); ++it) {
        delete *it;
    }

    return (goal_cell != 0);
}

void AStarPlanner::expandCell(CellInfo* c, int dx, int dy, double cost_factor, double** visited_map,
                              int x_goal, int y_goal, double min_cell_cost,
                              priority_queue<CellInfo*, vector<CellInfo*>, compareCellInfos>& Q) {

    int x = c->x_ + dx;
    int y = c->y_ + dy;

    double g_child = c->g_ + getCost(x, y) * cost_factor;
    if (g_child < visited_map[x][y]) {
        CellInfo* c_child = new CellInfo(x, y, g_child, calculateHeuristicCost(x, y, x_goal, y_goal, min_cell_cost));
        c_child->visited_from_ = c;
        Q.push(c_child);
        visited_map_[x][y] = g_child;
    }
}

double AStarPlanner::calculateHeuristicCost(int x, int y, int x_goal, int y_goal, double min_cell_cost) {
    double dx = (double)(x_goal - x);
    double dy = (double)(y_goal - y);
    return sqrt(dx * dx + dy * dy) * min_cell_cost;
}

void AStarPlanner::deleteMap() {
    for(unsigned int i = 0; i < width_; ++i) {
        delete[] visited_map_[i];
    }

    delete[] visited_map_;

    visited_map_ = 0;
}
