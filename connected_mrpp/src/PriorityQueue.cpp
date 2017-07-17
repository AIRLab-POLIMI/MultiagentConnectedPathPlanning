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

#include "rrt_planning/theta_star/PriorityQueue.h"

#include <cassert>

namespace rrt_planning
{

void PriorityQueue::insert(const Cell& cell, double cost)
{
    FrontierNode *frontierNode = new FrontierNode(cell, cost);
    auto res = open.insert(frontierNode);
    openMap[cell] = frontierNode;

    assert(res.second);

    assert(open.size() == openMap.size());
}

void PriorityQueue::remove(const Cell& cell)
{
    FrontierNode* f = openMap.at(cell);
    open.erase(f);
    openMap.erase(cell);

    delete f;

    assert(open.size() == openMap.size());
}

bool PriorityQueue::contains(const Cell& cell) const
{
    assert(open.size() == openMap.size());
    return openMap.count(cell) == 1;
}

bool PriorityQueue::empty() const
{
    assert(open.size() == openMap.size());
    return open.empty();
}

Cell PriorityQueue::pop()
{
    auto it = open.begin();
    auto ptr = *it;

    open.erase(it);

    Cell cell = ptr->getNode();
    openMap.erase(cell);

    delete ptr;

    assert(open.size() == openMap.size());

    return cell;
}

void PriorityQueue::clear()
{
    for(auto f: open)
        delete f;

    open.clear();
    openMap.clear();

}

std::set<FrontierNode*, PriorityQueue::Cmp>::iterator PriorityQueue::begin()
{
    assert(open.size() == openMap.size());
    return open.begin();
}

std::set<FrontierNode*, PriorityQueue::Cmp>::iterator PriorityQueue::end()
{
    assert(open.size() == openMap.size());
    return open.end();
}

PriorityQueue::~PriorityQueue()
{
    assert(open.size() == openMap.size());
    clear();
}


}
