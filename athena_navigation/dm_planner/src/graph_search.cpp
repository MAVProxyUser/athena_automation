#include <cmath>
#include <cstring>
#include <iostream>
#include <dm_planner/graph_search.h>


namespace dmp {

GraphSearch::GraphSearch (const unsigned char *cmap, int nx, int ny)
    : cmap_(cmap), nx_(nx), ny_(ny)
{
    hm_.resize(nx_ * ny_);
    seen_.resize(nx_ * ny_, false);

    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            if (x == 0 && y == 0) continue;
            ns_.push_back(std::vector< int>{x, y});
        }
    }
}
double 
GraphSearch::plan (int xStart, int yStart, int xGoal, int yGoal) {
    pq_.clear();
    path_.clear();
    hm_.resize(nx_ * ny_);
    seen_.resize(nx_ * ny_, false);

    int start_id = coordToId(xStart, yStart);

    xStart_ = xStart;
    yStart_ = yStart;
    xGoal_ = xGoal;
    yGoal_ = yGoal;
    int goal_id = coordToId(xGoal, yGoal);

    StatePtr currNode = std::make_shared< State>(State(start_id, xStart, yStart));
    currNode->g = cmap_[start_id];
    currNode->h = getHeur(xStart, yStart);

    return plan(currNode, start_id, goal_id);
}

std::vector< StatePtr> 
GraphSearch::getOpenSet() const {
    std::vector< StatePtr> ss;
    for (const auto &it : hm_) {
        if (it && it->opened && !it->closed) ss.push_back(it);
    }

    return ss;
}

std::vector< StatePtr> 
GraphSearch::getCloseSet() const {
    std::vector< StatePtr> ss;
    for (const auto &it : hm_) {
        if (it && it->closed) ss.push_back(it);
    }

    return ss;
}

std::vector< StatePtr> 
GraphSearch::getAllSet() const {
    std::vector< StatePtr> ss;
    for (const auto &it : hm_) {
        if (it) ss.push_back(it);
    }

    return ss;
}

double 
GraphSearch::plan (StatePtr &currNode, int start_id, int goal_id) {
    currNode->heapkey = pq_.push(currNode);
    currNode->opened = true;
    hm_[currNode->id] = currNode;
    seen_[currNode->id] = true;

    //int expend_iteration = 0;
    while (true) {
        currNode = pq_.top(); pq_.pop();
        currNode->closed = true;
        currNode->opened = false;

        if (currNode->id == goal_id) {
            break;
        }

        std::vector< int> nei_ids;
        std::vector< double> nei_costs;
        getNeighbors(currNode, nei_ids, nei_costs);

        for (uint8_t s = 0; s < nei_ids.size(); ++s) {
            StatePtr &child = hm_[nei_ids[s]];
            double tentative_gval = currNode->g + nei_costs[s];

            if (tentative_gval < child->g) {
                child->parentId = currNode->id;
                child->g = tentative_gval;

                if (child->opened && !child->closed) {
                    pq_.increase(child->heapkey);
                } else if (child->opened && child->closed) {
                    printf("ASTART ERROR!\n");
                } else {
                    child->heapkey = pq_.push(child);
                    child->opened = true;
                }
            }
        }

        if (pq_.empty()) {
            return std::numeric_limits< double>::infinity();
        }
    }
    
    path_ = recoverPath(currNode, start_id);

    return currNode->g;
}

void 
GraphSearch::getNeighbors (const StatePtr &curr, std::vector< int> &nei_ids, std::vector< double> &nei_costs) {
    for (const auto &d : ns_) {
        int new_x = curr->x + d[0];
        int new_y = curr->y + d[1];
        
        if (!isFree(new_x, new_y)) continue;

        int new_id = coordToId(new_x, new_y);

        if (!seen_[new_id]) {
            hm_[new_id] = std::make_shared< State>(State(new_id, new_x, new_y));
            hm_[new_id]->h = getHeur(new_x, new_y);
            seen_[new_id] = true;
        }

        nei_ids.push_back(new_id);
        nei_costs.push_back(std::hypot(d[0], d[1]) * (cmap_[new_id]));
    }
}

std::vector< StatePtr>
GraphSearch::recoverPath (StatePtr node, int id) {
    std::vector< StatePtr> path;
    path.push_back(node);
    while (node && node->id != id) {
        node = hm_[node->parentId];
        path.push_back(node);
    }

    return path;
}

double
GraphSearch::getHeur (int x, int y) const {
    double dx = std::fabs(x - xGoal_);
    double dy = std::fabs(y - yGoal_);
    double h = (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
    double dxx = std::fabs(xStart_ - xGoal_);
    double dyy = std::fabs(yStart_ - yGoal_);
    double cross = std::abs(dx * dyy - dxx * dy);
    h += cross * 0.0001;
    return h * COST_NEUTRAL;
}


} // dmp
