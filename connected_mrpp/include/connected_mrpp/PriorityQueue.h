/*
 * connected_mrpp,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of connected_mrpp.
 *
 * connected_mrpp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * connected_mrpp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with connected_mrpp.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_CONNECTED_MRPP_GRID_PRIORITYQUEUE_H_
#define INCLUDE_CONNECTED_MRPP_GRID_PRIORITYQUEUE_H_

#include "connected_mrpp/FrontierNode.h"

namespace connected_mrpp
{

template<class T>
struct DefaultCmp
{
    bool operator()(const FrontierNode<T>* a, const FrontierNode<T>* b) const
    {
        return (a->getCost() < b->getCost()) ||
                ((a->getCost() == b->getCost()) &&
                		(a->getDistance() > b->getDistance()))||
				((a->getCost() == b->getCost()) &&
						(a->getDistance() == b->getDistance()) &&
						(a->getNode() < b->getNode()));
    }
};

template<class T, class C=DefaultCmp<T>>
class PriorityQueue
{
public:
	PriorityQueue()
	{
	}

	PriorityQueue(C&& comparator) :
			open(comparator)
	{

	}

    void insert(const T& node, double g, double h)
    {
        FrontierNode<T>* frontierNode = new FrontierNode<T>(node, g, h);
        auto res = open.insert(frontierNode);
        openMap[node] = frontierNode;

        assert(res.second);

        assert(open.size() == openMap.size());
    }

    void remove(const T& node)
    {
        FrontierNode<T>* f = openMap.at(node);
        open.erase(f);
        openMap.erase(node);

        delete f;

        assert(open.size() == openMap.size());
    }

    bool contains(const T& node) const
    {
        assert(open.size() == openMap.size());
        return openMap.count(node) == 1;
    }

    bool empty() const
    {
        assert(open.size() == openMap.size());
        return open.empty();
    }

    T pop()
    {
        auto it = open.begin();
        auto ptr = *it;

        T node = ptr->getNode();

        open.erase(it);
        openMap.erase(node);

        delete ptr;

        assert(open.size() == openMap.size());

        return node;
    }

    void clear()
    {
        for(auto f: open)
            delete f;

        open.clear();
        openMap.clear();

    }

    typename std::set<FrontierNode<T>*, C>::iterator begin()
    {
        assert(open.size() == openMap.size());
        return open.begin();
    }

    typename std::set<FrontierNode<T>*, C>::iterator end()
    {
        assert(open.size() == openMap.size());
        return open.end();
    }

    ~PriorityQueue()
    {
        assert(open.size() == openMap.size());
        clear();
    }


private:
    std::set<FrontierNode<T>*, C> open;
    std::map<T, FrontierNode<T>*> openMap;

};

}


#endif /* INCLUDE_CONNECTED_MRPP_GRID_PRIORITYQUEUE_H_ */

