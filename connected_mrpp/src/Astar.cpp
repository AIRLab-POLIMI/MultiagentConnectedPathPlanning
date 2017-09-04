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

#include "../include/connected_mrpp/planner/Astar.h"

#include <ros/ros.h>

using namespace std;
using namespace std::rel_ops;
using namespace std::chrono;

//#define DEBUG_CONF

//Default Constructor
namespace connected_mrpp
{

Astar::Astar(Graph& graph, Objective* cost, Objective* heuristic, duration<double> Tmax)
		: LazyPlanner(graph, cost, heuristic, Tmax)
{

}

bool Astar::makePlanImpl()
{
    ROS_INFO("Planner started");

    //Init variables
    g[pi_start] = 0.0;
    open.insert(pi_start, 0, computeHeuristic(pi_start));

    //Compute plan
    while(!open.empty() && !timeOut())
    {
        //Pop the best frontier node
        Configuration pi = open.pop();
    	closed.insert(pi);

#ifdef DEBUG_CONF
        std::cout << "-" << pi << "-" << endl;
#endif

        if(pi == pi_goal)
        {
        	ROS_INFO("Plan found");
        	return true;
        }

        auto pi_n = findBestConfiguration(pi);
        sons[pi].push_back(pi_n);

#ifdef DEBUG_CONF
        std::cout << "{" << pi_n << "}" << endl;
#endif

        if(pi_n != PI_NULL)
        {
        	if(!open.contains(pi_n))
        	{
        		g[pi_n] = std::numeric_limits<double>::infinity();
        	}

        	updateConfiguration(pi, pi_n);
        	double c_best = bestLeafCost(pi);
        	double g_pi = g[pi];
        	open.insert(pi, g_pi , c_best - g_pi);
        }
    }

    ROS_INFO("No plan found");

    return false;
}

void Astar::updateConfiguration(Configuration& pi, Configuration& pi_n)
{
	double d = computeCost(pi, pi_n);

    if(g[pi] + d  < g[pi_n])
    {
        if(open.contains(pi_n))
            open.remove(pi_n);

        double g_pi_n = g[pi] + d;

        g[pi_n] = g_pi_n;
        parent[pi_n] = pi;
        open.insert(pi_n, g_pi_n, computeHeuristic(pi_n));
    }
}

double Astar::bestLeafCost(Configuration& pi)
{
	auto&& sonList = sons[pi];

	double best = numeric_limits<double>::infinity();

	for(auto& son : sonList)
	{
		if(closed.count(son) == 0)
		{
			double current = g[son] + computeHeuristic(son);
			best = min(best, current);
		}
	}

	return best;
}

void Astar::clearInstanceSpecific()
{
	//Clear superclass stuff
	LazyPlanner::clearInstanceSpecific();

	//Clear principal routine data structure
    open.clear();
    g.clear();
    sons.clear();

}

};
