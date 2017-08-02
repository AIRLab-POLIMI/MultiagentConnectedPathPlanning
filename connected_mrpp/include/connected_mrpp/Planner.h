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

#ifndef INCLUDE_CONNECTED_MRPP_THETASTARPLANNER_H_
#define INCLUDE_CONNECTED_MRPP_THETASTARPLANNER_H_

#include <Eigen/Dense>

#include "connected_mrpp/Configuration.h"
#include "connected_mrpp/PriorityQueue.h"
#include "connected_mrpp/graph/Graph.h"

namespace connected_mrpp
{

class Planner
{

public:
	Planner(Graph& grid);

    bool makePlan(const Configuration& start,
                  const Configuration& goal,
                  std::vector<Configuration>& plan);

private:
    bool isConnected(Configuration& pi);
    Configuration findBestConfiguration(Configuration& pi);

    void updateConfiguration(Configuration& pi, Configuration& pi_n);
    double bestLeafCost(Configuration& pi);
    double computeCost(Configuration& pi, Configuration& pi_n);
    double computeHeuristic(Configuration& pi);
    std::vector<PartialConfiguration> successors(PartialConfiguration& a);

    void clearInstance();

private:
    typedef std::map<Configuration, double> CostMap;

    struct ConfComp
    {
    	ConfComp(std::map<Configuration, Configuration>& parent);
        bool operator()(const FrontierNode<Configuration>* a,
        			const FrontierNode<Configuration>* b) const;

    	std::map<Configuration, Configuration>& parent;
    };

    typedef PriorityQueue<Configuration, ConfComp> ConfQueue;
    typedef DefaultCmp<PartialConfiguration> PartialConfComp;
    typedef PriorityQueue<PartialConfiguration, PartialConfComp> PartialConfQueue;

private:
    static const Configuration PI_NULL;

    Graph& graph;

    Configuration pi_start;
    Configuration pi_goal;

    //Principal Routine data structure
    CostMap g;
    ConfQueue open;
    std::map<Configuration, Configuration> parent;
    std::set<Configuration> closed;
    std::map<Configuration, std::vector<Configuration>> sons;

    //Local search data structure
    std::map<Configuration, CostMap> g_local;
    std::map<Configuration, PartialConfQueue> open_local;

};

}

#endif /* INCLUDE_CONNECTED_MRPP_THETASTARPLANNER_H_ */
