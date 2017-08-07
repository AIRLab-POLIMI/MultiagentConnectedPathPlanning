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

#include "connected_mrpp/planner/Birk.h"
#include "connected_mrpp/utils/RandomGenerator.h"
#include <cmath>
#include <ros/ros.h>

namespace connected_mrpp
{

Birk::Birk(Graph& graph, unsigned int numSamples)
	: AbstractPlanner(graph), numSamples(numSamples)
{

}

bool Birk::makePlanImpl()
{
	unsigned int m = pi_start.agent.size();

	visited_configs.insert(pi_start);

	Configuration pi = pi_start;


	bool progress = true;

	do
	{
		if(isOneStepReachable(pi, pi_goal))
		{
			parent[pi_goal] = pi;
        	ROS_INFO("Plan found");
			return true;
		}
		else
		{
			std::vector<Configuration> candidates;

			for(int i = 0; i < numSamples && i < maxNextConfigurations(pi) - 1; i++)
			{
				auto&& sample = sampleConfiguration(pi);
				candidates.push_back(sample);
			}

			Configuration pi_best = PI_NULL;
			double best_utility = std::numeric_limits<double>::infinity();

			for(auto& pi_n : candidates)
			{
				double pi_n_utility = computeUtility(pi_n);

				if(pi_n_utility < best_utility)
				{
					pi_best = pi_n;
					best_utility = pi_n_utility;
				}
			}

			if (pi_best == PI_NULL || visited_configs.count(pi_best) == 0)
			{
				progress = false;
			}
			else
			{
				parent[pi_best] = pi;
				pi = pi_best;
				visited_configs.insert(pi);
			}
		}

	}while(progress && !timeOut());

    ROS_INFO("No plan found");

	return false;
}


void Birk::clearInstanceSpecific()
{
	visited_configs.clear();
}

bool Birk::timeOut()
{
	return false;
}

unsigned int Birk::maxNextConfigurations(Configuration& pi)
{
	unsigned int counter = 1;
	for(auto v : pi.agent)
	{
		unsigned int n = graph.degree(v);
		counter *= n;
	}

	return counter - 1;
}

Configuration Birk::sampleConfiguration(Configuration& pi)
{
	Configuration pi_n;

	do
	{
		pi_n = pi;
		for(int i = 0; i < pi.agent.size(); i++)
		{
			auto&& neighbours = graph.getNeighbors(pi_n.agent[i]);
			neighbours.push_back(pi_n.agent[i]);
			int index = RandomGenerator::sampleUniform(0, neighbours.size());
			pi_n.agent[i] = neighbours[index];
		}
	} while(pi_n == pi);

	return pi_n;
}

double Birk::computeUtility(Configuration& pi)
{
	if(isConnected(pi))
	{
		return computeHeuristic(pi);
	}
	else
	{
		return std::numeric_limits<double>::infinity();
	}
}

}
