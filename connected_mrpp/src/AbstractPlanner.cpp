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

#include "connected_mrpp/planner/AbstractPlanner.h"

#include <ros/ros.h>
#include <chrono>

using namespace std;
using namespace std::chrono;
using namespace std::rel_ops;

namespace connected_mrpp
{

const Configuration AbstractPlanner::PI_NULL;


AbstractPlanner::AbstractPlanner(Graph& graph, Objective* cost, Objective* heuristic, duration<double> Tmax)
	: graph(graph), cost(cost), heuristic(heuristic), Tmax(Tmax), Tcurrent(0)
{

}

bool AbstractPlanner::makePlan(const Configuration& start,
        			   const Configuration& goal)
{
    clearInstance();

    t0 = steady_clock::now();

    //Init the position of the special states
    pi_start = start;
    pi_goal = goal;

    //Test starting position
    if(!isConnected(pi_start))
    {
        ROS_INFO("Invalid starting position");
        return false;
    }

    //Test target position
    if(!isConnected(pi_goal))
    {
        ROS_INFO("Invalid target position");
        return false;
    }

    //Init parents
    parent[pi_start] = pi_start;
    parent[pi_goal] = PI_NULL;

	bool result = makePlanImpl();
	steady_clock::now() - t0;
	Tcurrent = steady_clock::now() - t0;

	ROS_INFO_STREAM("Elapsed time: " << Tcurrent.count() << " s");

	return result;
}

vector<Configuration> AbstractPlanner::getPlan()
{
	vector<Configuration> plan;

	auto& current = pi_goal;

	while(current != pi_start && current != PI_NULL)
	{
		plan.push_back(current);
		current = parent[current];
	}

	plan.push_back(pi_start);

	reverse(plan.begin(), plan.end());

	return plan;
}

double AbstractPlanner::getElapsedTime()
{
	return Tcurrent.count();
}

bool AbstractPlanner::timeOut()
{
	auto deltaT = steady_clock::now() - t0;

	if(deltaT > Tmax)
	{
		ROS_INFO("Computational time excedeed");
		return true;
	}
	else
	{
		return false;
	}
}

double AbstractPlanner::computeCost(Configuration& pi, Configuration& pi_n)
{
	return cost->computeValue(pi, pi_n);
}

double AbstractPlanner::computeHeuristic(Configuration& pi)
{
	return heuristic->computeValue(pi, pi_goal);
}

bool AbstractPlanner::isOneStepReachable(Configuration& pi, Configuration& pi_n)
{
	for(int i = 0; i < pi.agent.size(); i++)
	{
		if(!graph.isNeighbor(pi.agent[i], pi_n.agent[i])
					&& pi.agent[i] != pi_n.agent[i])
		{
			return false;
		}
	}

	return true;
}

bool AbstractPlanner::isConnected(Configuration& pi)
{
	return graph.isConnected(pi.agent);
}


void AbstractPlanner::clearInstance()
{
	parent.clear();
	clearInstanceSpecific();
}

AbstractPlanner::~AbstractPlanner()
{

}


}
