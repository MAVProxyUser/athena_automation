#include "dm_planner/distance_map_planner.h"

namespace dmp {

DMPlanner::DMPlanner () 
    : map_util_(std::make_shared< dmp::MapUtil>()),
      status_(0), free_radius_(0.), search_radius_(0.)
{}

std::vector< Position>
DMPlanner::getOpenSet () const {
    std::vector< Position> ps;
    const auto ss = graph_search_->getOpenSet();
    for (const auto &it : ss) {
        ps.push_back(map_util_->mapToWorld(Index(it->x, it->y)));
    }

    return ps;
}

std::vector< Position>
DMPlanner::getCloseSet () const {
    std::vector< Position> ps;
    const auto ss = graph_search_->getCloseSet();
    for (const auto &it : ss) {
        ps.push_back(map_util_->mapToWorld(Index(it->x, it->y)));
    }

    return ps;
}

std::vector< Position>
DMPlanner::getAlleSet () const {
    std::vector< Position> ps;
    const auto ss = graph_search_->getAllSet();
    for (const auto &it : ss) {
        ps.push_back(map_util_->mapToWorld(Index(it->x, it->y)));
    }

    return ps;
}

void
DMPlanner::setMap (const CostType * cmap, const Position &ori, const Size &size, double res) {
    map_util_->setMap(cmap, ori, size, res);
}

void
DMPlanner::setPath (const std::vector< Position> &path, double radius, bool dense) {
    prior_path_ = path;

    std::vector< Index> ps;
    if (!dense) {
        for (unsigned int i = 1; i < path.size(); ++i) {
            auto pns = map_util_->rayTrace(path[i - 1], path[i]);
            ps.insert(ps.end(), pns.begin(), pns.end());
            ps.push_back(map_util_->worldToMap(path[i]));
        }
    } else {
        for (const auto &pt : path) 
            ps.push_back(map_util_->worldToMap(pt));
    }

    if (radius <= 0) return;

    std::vector< Index> ns;
    int rn = std::ceil(radius / map_util_->getRes());
    for (int nx = -rn; nx <= rn; ++nx) {
        for (int ny = -rn; ny <= rn; ++ny) {
            if (std::hypot(nx, ny) > rn) continue;
            ns.push_back(Index(nx, ny));
        }
    }

    std::vector< Index> indices;

    for (const auto &it : ps) {
        for (const auto &n: ns) {
            Index pn = it + n;
            if (map_util_->isOutside(pn)) continue;
            indices.push_back(pn);
        }
    }

    map_util_->limitRegion(indices);
}

bool
DMPlanner::computePath (const Position &start, const Position &goal, const std::vector< Position> &path, bool dense) {

    // free goal around free_radius in search map;
    double res = map_util_->getRes();
    int n_step = std::round(free_radius_ / res);
    for (int i = -n_step; i <= n_step; ++i) {
        for (int j = -n_step; j <= n_step; ++j) {
            if (std::hypot(i, j) > n_step) continue;
            map_util_->freeIndex(Position(goal.x() + i * res, goal.y() + j * res));
        }
    }

    setPath(path, search_radius_, dense);
    return plan(start, goal);
}

bool
DMPlanner::plan (const Position start, const Position &goal) {
    path_.clear();
    status_ = 0;

    if (map_util_ == NULL || map_util_->getMap() == NULL) {
        status_ = -1;
        return false;
    }

    const Index start_int = map_util_->worldToMap(start);
    if (!checkAvailability(start_int)) {
        status_ = 1;
        return false;
    }

    const Index goal_int = map_util_->worldToMap(goal);
    if (!checkAvailability(goal_int)) {
        status_ = 2;
        return false;
    }


    const Size size = map_util_->getSize();
    graph_search_ = std::make_shared< dmp::GraphSearch> (
                        map_util_->getMap(), size(0), size(1));
    path_cost_ = graph_search_->plan(start_int(0), start_int(1),
                                     goal_int(0), goal_int(1));


    const auto path = graph_search_->getPath();
    if (path.size() < 1) {
        status_ = -1;
        return false;
    }

    std::vector< Position> ps;
    for (const auto &it : path) {
        ps.push_back(map_util_->mapToWorld(Index(it->x, it->y)));
    }


    path_ = ps;
    std::reverse(std::begin(path_), std::end(path_));

    return true;
}

bool
DMPlanner::checkAvailability (const Index &pn) {
    if (map_util_->isUnknown(pn)) 
        return false;
    return !map_util_->isOccupied(pn);
}

} // dmp
