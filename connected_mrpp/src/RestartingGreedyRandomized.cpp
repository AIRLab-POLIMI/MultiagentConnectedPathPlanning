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

#include "connected_mrpp/planner/RestartingGreedyRandomized.h"

#include <vector>
#include <ros/ros.h>

namespace connected_mrpp
{

RestartingGreedyRandomized::RestartingGreedyRandomized(Graph& graph, Objective* utility, std::chrono::duration<double> Tmax,
        SamplingStrategy& sampler, double alpha, unsigned int numSamples) :
    SampleBasedPlanner(graph, utility, Tmax, numSamples), sampler(sampler), alpha(alpha)
{

}

bool RestartingGreedyRandomized::makePlanImpl()
{
    unsigned int epoch = 0;

    MaxShortestPathObjective calculator(graph);
    double bottleneck = calculator.computeValue(pi_start, pi_goal);

    while(!timeOut())
    {
        Configuration pi = pi_start;

        unsigned int i = 0;
        std::set<Configuration> visited_configs;

        while(!timeOut() && i < bottleneck * std::exp(1 + alpha * epoch))
        {
            visited_configs.insert(pi);

            if(isOneStepReachable(pi, pi_goal))
            {
                parent[pi_goal] = pi;
                ROS_INFO("Plan found");
                ROS_INFO("Number of epochs: %u", epoch+1);
                return true;
            }
            else
            {
                std::vector<Configuration> candidates;

                sampleConfigurations(pi, candidates);

                Configuration pi_best = selectConfiguration(candidates);
                if (pi_best == PI_NULL)
                {
                    break;
                }
                if (visited_configs.count(pi_best) == 0)
                {
                	parent[pi_best] = pi;
                }
                pi = pi_best;
            }

            i++;
        }

        epoch++;
    }

    ROS_INFO("No plan found");
    ROS_INFO("Number of epochs: %u", epoch);

    return false;
}

void RestartingGreedyRandomized::clearInstanceSpecific()
{

}

Configuration RestartingGreedyRandomized::selectConfiguration(const std::vector<Configuration>& candidates)
{
    std::vector<Configuration> samples;
    std::vector<double> costs;

    for(auto& pi : candidates)
    {
        double c = computeUtility(pi);

        if(c < std::numeric_limits<double>::infinity())
        {
            samples.push_back(pi);
            costs.push_back(c);
        }
    }

    if(samples.size() == 0)
    {
        samples.push_back(PI_NULL);
        costs.push_back(std::numeric_limits<double>::infinity());
    }

    return sampler.sample(samples, costs);
}

}
