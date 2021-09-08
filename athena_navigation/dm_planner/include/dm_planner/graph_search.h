/**
 * @file graph_search.h
 * @brief backend of graph search for distance map
 */

#pragma once

#include "dm_planner/cost_value.h"

#include <boost/heap/d_ary_heap.hpp>        // boost::heap::d_ary_heap
#include <memory>                           // std::shared_ptr
#include <limits>                           // std::numeric_limts
#include <vector>                           // std::vector
#include <unordered_map>                    // std::unordered_map

namespace dmp
{
    // Heap element comparison
    template < typename T>
    struct compare_state {
        bool operator () (T a1, T a2) const {
            double f1 = a1->g + a1->h;
            double f2 = a2->g + a2->h;
            if ((f1 >= f2 - 1e-6) && (f1 <= f2 + 1e-6)) return a1->g < a2->g;
            return f1 > f2;
        }
    };

    struct State;
    using StatePtr = std::shared_ptr< State>;
    using priorityQueue = boost::heap::d_ary_heap< StatePtr, boost::heap::mutable_<true>,
                          boost::heap::arity< 2>, boost::heap::compare< compare_state< StatePtr> > >;

    struct State {
        int id;
        int x, y;
        int parentId = -1;
        priorityQueue::handle_type heapkey;

        double g = std::numeric_limits< double>::max();
        double h;
        bool opened = false;
        bool closed = false;

        State (int id, int x, int y)
            : id(id), x(x), y(y)
        {}
    }; // State

    class GraphSearch {
    public:
        GraphSearch (const unsigned char *cmap, int nx, int ny);
        double plan (int xStart, int yStart, int xGoal, int yGoal);

        std::vector< StatePtr> getPath () const {
            return path_;
        }
        std::vector< StatePtr> getOpenSet() const;
        std::vector< StatePtr> getCloseSet() const;
        std::vector< StatePtr> getAllSet() const;

    private:
        double plan (StatePtr &currNode, int start_id, int goal_id);

        void getNeighbors (const StatePtr &curr, std::vector< int> &nei_ids, std::vector< double> &nei_costs);

        std::vector< StatePtr> recoverPath (StatePtr node, int id);

        inline int coordToId (int x, int y) const {
            return x + y * nx_;
        }

        bool isFree (int x, int y) const {
            return x >= 0 && x < nx_ && y >= 0 && y < ny_ &&
                cmap_[coordToId(x, y)] < COST_OBS;
        }

        double getHeur (int x, int y) const;

        const unsigned char *cmap_;
        int nx_, ny_;

        int xStart_, yStart_;
        int xGoal_, yGoal_;

        priorityQueue pq_;
        std::vector< StatePtr> hm_;
        std::vector< bool> seen_;

        std::vector< StatePtr> path_;

        std::vector< std::vector< int> > ns_;
    }; // GraphSearch
} // dmp
