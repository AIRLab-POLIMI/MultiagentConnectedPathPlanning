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


AbstractPlanner::AbstractPlanner(Graph& graph, duration<double> Tmax)
	: graph(graph), Tmax(Tmax)
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

	return makePlanImpl();
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
	double cost = 0;
	for(unsigned int i = 0; i < pi.agent.size(); i++)
	{
		auto v = pi.agent[i];
		auto v_n = pi_n.agent[i];
		cost += graph.cost(v, v_n);
	}

	return cost;
}

double AbstractPlanner::computeHeuristic(Configuration& pi)
{
	double h = 0;
	for(unsigned int i = 0; i < pi.agent.size(); i++)
	{
		auto v = pi.agent[i];
		auto v_n = pi_goal.agent[i];
		h += graph.heuristic(v, v_n);
	}

	return h;
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
