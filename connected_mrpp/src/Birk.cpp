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

using namespace std::chrono;

namespace connected_mrpp
{

Birk::Birk(Graph& graph, Objective* utility, duration<double> Tmax, unsigned int numSamples)
    : NoLoopSampleBasedPlanner(graph, utility, Tmax, numSamples)
{

}

Configuration Birk::selectConfiguration(const std::vector<Configuration>& candidates)
{
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

    return pi_best;
}

}
