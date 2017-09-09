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

#include "connected_mrpp/planner/DFS_Planner.h"
#include <ros/ros.h>

using namespace std;
using namespace std::rel_ops;
using namespace std::chrono;

namespace connected_mrpp
{

DFS_Planner::DFS_Planner(Graph& graph, Objective* cost, Objective* heuristic, duration<double> Tmax)
		: LazyPlanner(graph, cost, heuristic, Tmax)
{

}

bool DFS_Planner::makePlanImpl()
{
    ROS_INFO("Planner started");

    //Init variables
    stack.push_back(pi_start);


    //Compute plan
    while(!stack.empty() && !timeOut())
    {
        //Pop the last frontier node
        Configuration pi = stack.back();
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

#ifdef DEBUG_CONF
        std::cout << "{" << pi_n << "}" << endl;
#endif

        if(pi_n != PI_NULL)
        {
        	stack.push_back(pi_n);
        	parent[pi_n] = pi;
        }
        else
        {
        	stack.pop_back();
        }
    }

    ROS_INFO("No plan found");

    return false;
}

void DFS_Planner::clearInstanceSpecific()
{
	LazyPlanner::clearInstanceSpecific();
	stack.clear();
}

}
