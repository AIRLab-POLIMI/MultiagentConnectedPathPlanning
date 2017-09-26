/*
 * connected_mrpp,
 *
 *
 * Copyright (C) 2017 Davide Tateo
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

#ifndef INCLUDE_CONNECTED_MRPP_PLANNER_LAZYPLANNER_H_
#define INCLUDE_CONNECTED_MRPP_PLANNER_LAZYPLANNER_H_

#include "connected_mrpp/planner/AbstractPlanner.h"
#include "connected_mrpp/queue/PriorityQueue.h"

namespace connected_mrpp
{

class LazyPlanner: public AbstractPlanner
{
public:
    LazyPlanner(Graph& graph, Objective* cost, Objective* heuristic, std::chrono::duration<double> Tmax);

protected:
    Configuration findBestConfiguration(Configuration& pi);

    virtual void clearInstanceSpecific() override;

private:
    std::vector<PartialConfiguration> successors(PartialConfiguration& a);

private:
    typedef std::map<PartialConfiguration, double> PartialCostMap;
    typedef PriorityQueue<PartialConfiguration> PartialConfQueue;

protected:
    std::set<Configuration> closed;

private:
    std::map<Configuration, PartialCostMap> g_local;
    std::map<Configuration, PartialConfQueue> open_local;

};

}



#endif /* INCLUDE_CONNECTED_MRPP_PLANNER_LAZYPLANNER_H_ */
