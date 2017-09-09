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

#ifndef INCLUDE_CONNECTED_MRPP_PLANNER_ABSTRACTPLANNER_H_
#define INCLUDE_CONNECTED_MRPP_PLANNER_ABSTRACTPLANNER_H_

#include "connected_mrpp/graph/Graph.h"
#include "connected_mrpp/planner/Configuration.h"
#include "connected_mrpp/planner/Objective.h"

#include <map>
#include <chrono>

namespace connected_mrpp
{

class AbstractPlanner
{
public:
	AbstractPlanner(Graph& graph, Objective* cost, Objective* heuristic, std::chrono::duration<double> Tmax);

    bool makePlan(const Configuration& start,
                  const Configuration& goal);

    std::vector<Configuration> getPlan();
    double getElapsedTime();

    virtual ~AbstractPlanner();

protected:
    bool timeOut();

protected:
    virtual double computeCost(const Configuration& pi, const Configuration& pi_n);
    double computeHeuristic(const Configuration& pi);
    bool isOneStepReachable(const Configuration& pi, const Configuration& pi_n);
    bool isConnected(const Configuration& pi);

    void clearInstance();

protected:
    virtual void clearInstanceSpecific() = 0;
    virtual bool makePlanImpl() = 0;

protected:
    static const Configuration PI_NULL;

    Graph& graph;

    Configuration pi_start;
    Configuration pi_goal;

    std::map<Configuration, Configuration> parent;

private:
    std::chrono::steady_clock::time_point t0;
    std::chrono::duration<double> Tmax;
    std::chrono::duration<double> Tcurrent;

    Objective* cost;
    Objective* heuristic;

};

}


#endif /* INCLUDE_CONNECTED_MRPP_PLANNER_ABSTRACTPLANNER_H_ */
