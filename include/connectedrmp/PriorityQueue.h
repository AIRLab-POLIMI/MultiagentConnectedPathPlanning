/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_RRT_PLANNING_GRID_PRIORITYQUEUE_H_
#define INCLUDE_RRT_PLANNING_GRID_PRIORITYQUEUE_H_

#include "rrt_planning/theta_star/FrontierNode.h"

namespace rrt_planning
{

class PriorityQueue
{
    struct Cmp
    {
        bool operator()(const FrontierNode* a, const FrontierNode* b) const
        {
            return (a->getCost() < b->getCost()) ||
                   ((a->getCost() == b->getCost()) && (a->getNode() < b->getNode()));
        }
    };


public:
    void insert(const Cell& cell, double cost);
    void remove(const Cell& cell);
    bool contains(const Cell& cell) const;
    bool empty() const;
    Cell pop();
    void clear();

    std::set<FrontierNode*, Cmp>::iterator begin();
    std::set<FrontierNode*, Cmp>::iterator end();

    ~PriorityQueue();


private:
    std::set<FrontierNode*, Cmp> open;
    std::map<std::pair<int, int>, FrontierNode*> openMap;

};

}


#endif /* INCLUDE_RRT_PLANNING_GRID_PRIORITYQUEUE_H_ */

