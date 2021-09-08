/**
 * @file map_util.h
 * @brief MapUtil classes
 */

#pragma once

#include "dm_planner/data_type.h"
#include "dm_planner/cost_value.h"
#include <iostream>

namespace dmp {

class MapUtil {
public:
    MapUtil () {
        map_ = NULL;

        origin_.setZero();
        size_.setZero();
        res_ = 0;
    }

    ~MapUtil () {
        if (map_ != NULL) delete [] map_;
    }

    CostType* getMap() { return map_; }
    double getRes() { return res_; }
    Size getSize() { return size_; }
    Position getOrigin() { return origin_; }

    int getIndex(const Index& pn) { return pn(0) + size_(0) * pn(1); }

    bool isOutsideXYZ(const Index &n, int i) { return n(i) < 0 || n(i) >= size_(i); }
    bool isOutside(const Index &pn) {
        for(int i = 0; i < 2; i++)
            if (isOutsideXYZ(pn, i)) return true;
        return false;
    }

    bool isFree(int idx) { return map_[idx] == COST_FREE; }
    bool isFree(const Index &pn) { return isOutside(pn) ? false : isFree(getIndex(pn)); }

    bool isOccupied(int idx) { return map_[idx] != COST_UNKNOWN && map_[idx] >= COST_OBS; }
    bool isOccupied(const Index &pn) { return isOutside(pn) ? false : isOccupied(getIndex(pn)); }

    bool isUnknown(int idx) { return map_[idx] == COST_UNKNOWN; }
    bool isUnknown(const Index &pn) { return isOutside(pn) ? false : isUnknown(getIndex(pn)); }

    /**
     * @brief Set map
     *
     * @param map array of cell values
     * @param ori origin position
     * @param size number of cells in each dimension
     * @param res map resolution
     */
    void setMap(const CostType *cmap, const Position &ori, const Size &size, double res) {

        origin_ = ori;
        size_ = size;
        res_ = res;

        unsigned int ns = size_(0) * size_(1);

        if (map_ != NULL) delete [] map_;
        map_ = new CostType[ns];
        for (unsigned int i = 0; i < ns; ++i) {
            CostType cm = COST_OBS;
            auto v = cmap[i];
            if (v == COST_UNKNOWN) {
                //cm = COST_OBS - 1;
                cm = COST_NEUTRAL;
            } else if (v > COST_OBS) {
                cm += 1;
            } else if (v < COST_OBS) {
                cm = COST_NEUTRAL + COST_FACTOR * v;
                if (cm >= COST_OBS) cm -= 1;
            }
            map_[i] = cm;
        }
    }

    void limitRegion (std::vector< Index> indices) {
        Index min = size_, max = Index::Constant(-1);
        for (const auto idx : indices) {
            if (idx.x() > max.x()) max.x() = idx.x();
            if (idx.x() < min.x()) min.x() = idx.x();
            if (idx.y() > max.y()) max.y() = idx.y();
            if (idx.y() < min.y()) min.y() = idx.y();
        }

        Size new_size = max - min + Size::Constant(1);
        int ns = new_size.x() * new_size.y();
        CostType *new_map = new CostType[ns];
        memset(new_map, COST_OBS, sizeof(CostType) * ns);
        for (const auto idx: indices) {
            auto new_idx = idx - min;
            new_map[new_idx(0) + new_size(0) * new_idx(1)] = map_[getIndex(idx)];
        }

        origin_ += min.matrix().template cast< double>() * res_;
        size_ = new_size;
        if (map_ != NULL) delete [] map_;
        map_ = new_map;
    }

    void info() {
        Length range = size_.template cast< double>() * res_;
        std::cout << "MapUtil Info ========================== " << std::endl;
        std::cout << "   res: [" << res_ << "]" << std::endl;
        std::cout << "   origin: [" << origin_.transpose() << "]" << std::endl;
        std::cout << "   range: [" << range << "]" << std::endl;
        std::cout << "   size: [" << size_.transpose() << "]" << std::endl;
    };

    Index worldToMap(const Position &pt) {
        Index pn;
        for(int i = 0; i < 2; i++)
            pn(i) = std::round((pt(i) - origin_(i)) / res_ - 0.5);
        return pn;
    }
    Position mapToWorld(const Index &pn) {
        return (pn.matrix().template cast< double>() + Position::Constant(0.5)) * res_ + origin_;
    }

    ///Raytrace from float point pt1 to pt2
    std::vector< Index> rayTrace(const Position &pt1, const Position &pt2) {
        Length diff = pt2 - pt1;
        double k = 0.8;
        int max_diff = (diff / res_).matrix().norm() / k;
        double s = 1.0 / max_diff;
        Length step = diff * s;

        std::vector< Index> pns;
        Index prev_pn = Index::Constant(-1);
        for (int n = 1; n < max_diff; n++) {
            Position pt = pt1 + step.matrix() * n;
            Index new_pn = worldToMap(pt);
            if (isOutside(new_pn))
                break;
            if (new_pn.matrix() != prev_pn.matrix())
                pns.push_back(new_pn);
            prev_pn = new_pn;
        }
        return pns;
    }

    void freeIndex (const Position &pt) {
        Index pn= worldToMap(pt);
        freeIndex(pn);
    }
    void freeIndex (const Index &pn) {
        if (isOutside(pn)) return;
        map_[getIndex(pn)] = COST_NEUTRAL;
    }

    CostType *map_;
protected:
    double res_;
    Position origin_;
    Size size_;
};
} // dmp
