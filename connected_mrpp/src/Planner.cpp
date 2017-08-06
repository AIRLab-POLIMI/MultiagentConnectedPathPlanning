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


Planner::Planner(Graph& graph) : graph(graph)
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
    open.insert(pi_start, 0, computeHeuristic(pi_start));
    parent[pi_goal] = PI_NULL;

    ROS_INFO("Planner started");

    //Compute plan
    while(!open.empty())
    {
        //Pop the best frontier node
        Configuration pi = open.pop();
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
        	if(!open.contains(pi_n))
        	{
        		g[pi_n] = std::numeric_limits<double>::infinity();
        	}

        	updateConfiguration(pi, pi_n);
        	double c_best = bestLeafCost(pi);
        	double g_pi = g[pi];
        	open.insert(pi, g_pi , c_best - g_pi);
        }
        else
        {
        	closed.insert(pi);
        }
    }

    ROS_INFO("No plan found");

    return false;
}

vector<Configuration> Planner::getPlan()
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

bool Planner::isConnected(Configuration& pi)
{
	return graph.isConnected(pi.agent);
}

Configuration Planner::findBestConfiguration(Configuration& pi)
{
	if(g_local.count(pi) == 0)
	{
		PartialConfiguration pi_a(pi);

		g_local[pi][pi_a] = 0;
		open_local[pi].insert(pi_a, 0, computeHeuristic(pi));
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

			pi_open.insert(a_n, g, computeHeuristic(a_n));
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

        double g_pi_n = g[pi] + d;

        g[pi_n] = g_pi_n;
        parent[pi_n] = pi;
        open.insert(pi_n, g_pi_n, computeHeuristic(pi_n));
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
			double current = g[son] + computeHeuristic(son);
			best = min(best, current);
		}
	}

	return best;
}

double Planner::computeCost(Configuration& pi, Configuration& pi_n)
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

double Planner::computeHeuristic(Configuration& pi)
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

};
