/**
 * @file distance_map_planner.h
 * @brief distance map planner!
 */

#pragma once

#include "dm_planner/data_type.h"
#include "dm_planner/graph_search.h"
#include "dm_planner/map_util.h"

namespace dmp {

class GraphSearch;

class DMPlanner {
public:
    DMPlanner ();

    void freeGoal (double r = 0.0) { free_radius_ = r; }
    void setSearchRadius (double r = 0.0) { search_radius_ = r; }

    std::shared_ptr< dmp::MapUtil> getMap () {
        return map_util_;
    }

    /**
     * @brief Status of the planner
     * 0 --- exit normally;
     * -1 --- no path found;
     *  1, 2 --- start or goal is not free.
     */
    int status () { return status_; }

    std::vector< Position> getPath () { return path_; }
    std::vector< Position> getPriorPath () { return prior_path_; }

    std::vector< Position> getOpenSet () const;
    std::vector< Position> getCloseSet () const;
    std::vector< Position> getAlleSet () const;

    void setMap (const CostType * cmap, const Position &ori, const Size &size, double res);

    void setPath (const std::vector< Position> &path, double radius, bool dense = true);
    bool computePath (const Position &start, const Position &goal, const std::vector< Position> &path, bool dense = true);

protected:
    
    bool plan (const Position start, const Position &goal);

    bool checkAvailability (const Index &pn);

    std::shared_ptr< dmp::MapUtil> map_util_;
    std::shared_ptr< dmp::GraphSearch> graph_search_;

    int status_;
    double free_radius_;
    double search_radius_;

    double path_cost_;
    std::vector< Position> prior_path_;
    std::vector< Position> path_;

};

} // dmp
