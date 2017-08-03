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


#include <ros/ros.h>
#include "connected_mrpp/Planner.h"


using namespace std;
using namespace Eigen;
using namespace std::rel_ops;

//Default Constructor
namespace connected_mrpp
{

const Configuration Planner::PI_NULL;


Planner::Planner(Graph& graph) : graph(graph), open(ConfComp(parent))
{

}

bool Planner::makePlan(const Configuration& start,
        			   const Configuration& goal)
{
    clearInstance();
    //visualizer.clean();

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

    //Init variables
    g[pi_start] = 0.0;
    parent[pi_start] = pi_start;
    open.insert(pi_start, computeHeuristic(pi_start));
    parent[pi_goal] = PI_NULL;

    ROS_INFO("Planner started");

    //Compute plan
    while(!open.empty())
    {
        //Pop the best frontier node
        Configuration pi = open.pop();

        if(pi == pi_goal)
        {
        	ROS_INFO("Plan found");
        	return true;
        }

        auto&& pi_n = findBestConfiguration(pi);

        if(pi_n != PI_NULL)
        {
        	if(!open.contains(pi_n))
        	{
        		g[pi_n] = std::numeric_limits<double>::infinity();
        	}

        	updateConfiguration(pi, pi_n);
        	double c_new = bestLeafCost(pi);
        	open.insert(pi, c_new);
        }
        else
        {
        	closed.insert(pi);
        }
    }

    ROS_INFO("No plan found");

    return false;
}

bool Planner::isConnected(Configuration& pi)
{
	return true;
}

Configuration Planner::findBestConfiguration(Configuration& pi)
{
	if(g_local.count(pi) == 0)
	{
		CostMap costMap;
		costMap[pi] = 0;
		g_local[pi] = costMap;

		PartialConfQueue queue;
		PartialConfiguration pi_a = pi;
		queue.insert(pi_a, computeHeuristic(pi));
		open_local[pi] = queue;
	}

	auto& pi_open = open_local[pi];
	auto& pi_g = g_local[pi];

	while(!pi_open.empty())
	{
		PartialConfiguration a = pi_open.pop();

		if(a.count == a.agent.size()
				&& isConnected(a)
				&& closed.count(a) == 0
				&& static_cast<Configuration>(a) != pi)
		{
			sons[pi].push_back(a);
			return a;
		}

		for(auto a_n : successors(a))
		{
			double g = pi_g[a] + computeCost(a, a_n);
			pi_g[a_n] = g;
			double cost = g + computeHeuristic(a_n);
			pi_open.insert(a, cost);
		}
	}

	return PI_NULL;
}


void Planner::updateConfiguration(Configuration& pi, Configuration& pi_n)
{
	double d = computeCost(pi, pi_n);

    if(g[pi] + d  < g[pi_n])
    {
        if(open.contains(pi_n))
            open.remove(pi_n);

        g[pi_n] = g[pi] + d;

        double frontierCost = g[pi_n] + computeHeuristic(pi_n);

        open.insert(pi_n, frontierCost);
    }
}

double Planner::bestLeafCost(Configuration& pi)
{
	auto&& sonList = sons[pi];

	double best = numeric_limits<double>::infinity();
	for(auto& son : sonList)
	{
		if(closed.count(son) == 0)
		{
			double current = g[son];
			best = min(best, current);
		}
	}

	return best;
}

double Planner::computeCost(Configuration& pi, Configuration& pi_n)
{
	return 0;
}

double Planner::computeHeuristic(Configuration& pi)
{
	return 0;
}

std::vector<PartialConfiguration> Planner::successors(PartialConfiguration& a)
{
	std::vector<PartialConfiguration> succ;

	if(a.count != a.agent.size())
	{
		auto v = a.agent[a.count];

		auto neighbours = graph.getNeighbors(v);
		neighbours.push_back(v);

		for(auto& v_n : neighbours)
		{
			PartialConfiguration a_n = a;
			a_n.count += 1;
			a_n.agent[a.count] = v_n;
			succ.push_back(a_n);
		}
	}

	return succ;
}

void Planner::clearInstance()
{
	//Clear principal routine data structure
    open.clear();
    closed.clear();
    parent.clear();
    g.clear();
    sons.clear();

    //clear local search data structure
    open_local.clear();
    g_local.clear();
}

Planner::ConfComp::ConfComp(map<Configuration, Configuration>& parent) :
    		    		parent(parent)
{

}

bool Planner::ConfComp::operator()(const FrontierNode<Configuration>* a, const FrontierNode<Configuration>* b) const
{
	if(a->getCost() < b->getCost())
	{
		return true;
	}
	else if(a->getCost() == b->getCost())
	{
		if(parent[a->getNode()] == b->getNode())
		{
			return true;
		}
		else if(parent[b->getNode()] == a->getNode())
		{
			return false;
		}
		else
		{
			return a->getNode() < b->getNode();
		}
	}
	else
	{
		return false;
	}
}


};
