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

#include "connected_mrpp/planner/SampleBasedPlanner.h"

#include "connected_mrpp/utils/RandomGenerator.h"
#include <cmath>
#include <ros/ros.h>

using namespace std::chrono;

namespace connected_mrpp
{

SampleBasedPlanner::SampleBasedPlanner(Graph& graph, Objective* utility, duration<double> Tmax, unsigned int numSamples)
	: AbstractPlanner(graph, nullptr, utility, Tmax), numSamples(numSamples)
{

}

bool SampleBasedPlanner::makePlanImpl()
{
	while(!timeOut())
	{
		visited_configs.clear();
		visited_configs.insert(pi_start);

		Configuration pi = pi_start;

		bool progress = true;

		while(progress && !timeOut())
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

				sampleConfigurations(pi, candidates);

				Configuration pi_best = selectConfiguration(candidates);
				if (pi_best == PI_NULL || visited_configs.count(pi_best) != 0)
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

		}
	}

    ROS_INFO("No plan found");

	return false;
}


void SampleBasedPlanner::clearInstanceSpecific()
{
	visited_configs.clear();
}

unsigned int SampleBasedPlanner::maxNextConfigurations(Configuration& pi)
{
	unsigned int counter = 1;
	for(auto v : pi.agent)
	{
		unsigned int n = graph.degree(v);
		counter *= n;
	}

	return counter - 1;
}

void SampleBasedPlanner::sampleConfigurations(Configuration& pi, std::vector<Configuration>& candidates)
{
	int maxConfs = maxNextConfigurations(pi);
	for(int i = 0; i < numSamples && i < maxConfs; i++)
	{
		auto&& sample = sampleConfiguration(pi);
		candidates.push_back(sample);
	}
}

Configuration SampleBasedPlanner::sampleConfiguration(Configuration& pi)
{
	Configuration pi_n;

	do
	{
		pi_n = pi;
		for(int i = 0; i < pi.agent.size(); i++)
		{
			auto&& neighbours = graph.getNeighbors(pi_n.agent[i]);
			neighbours.push_back(pi_n.agent[i]);
			int index = RandomGenerator::sampleUniform(0, neighbours.size() - 1);
			pi_n.agent[i] = neighbours[index];
		}
	} while(pi_n == pi);

	return pi_n;
}

double SampleBasedPlanner::computeUtility(const Configuration& pi)
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
