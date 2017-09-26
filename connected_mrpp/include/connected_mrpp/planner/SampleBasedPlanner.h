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

#ifndef INCLUDE_CONNECTED_MRPP_PLANNER_SAMPLEBASEDPLANNER_H_
#define INCLUDE_CONNECTED_MRPP_PLANNER_SAMPLEBASEDPLANNER_H_
#include "connected_mrpp/planner/AbstractPlanner.h"

#include <set>

namespace connected_mrpp
{

class SampleBasedPlanner : public AbstractPlanner
{
public:
    SampleBasedPlanner(Graph& graph, Objective* utility, std::chrono::duration<double> Tmax, unsigned int numSamples);

protected:
    virtual bool makePlanImpl() override;
    virtual void clearInstanceSpecific() override;

    virtual Configuration selectConfiguration(const std::vector<Configuration>& candidates) = 0;

protected:
    double computeUtility(const Configuration& pi);

private:
    unsigned int maxNextConfigurations(Configuration& pi);
    void sampleConfigurations(Configuration& pi, std::vector<Configuration>& candidates);
    Configuration sampleConfiguration(Configuration& pi);

private:
    std::set<Configuration> visited_configs;

    const unsigned int numSamples;

};

}




#endif /* INCLUDE_CONNECTED_MRPP_PLANNER_SAMPLEBASEDPLANNER_H_ */
