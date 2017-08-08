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

#ifndef INCLUDE_CONNECTED_MRPP_BIRK_BIRK_H_
#define INCLUDE_CONNECTED_MRPP_BIRK_BIRK_H_

#include "connected_mrpp/planner/AbstractPlanner.h"

#include <set>

namespace connected_mrpp
{

class Birk : public AbstractPlanner
{
public:
	Birk(Graph& graph, std::chrono::duration<double> Tmax, unsigned int numSamples = 100);

protected:
    virtual bool makePlanImpl() override;
    virtual void clearInstanceSpecific() override;

private:
    unsigned int maxNextConfigurations(Configuration& pi);
    Configuration sampleConfiguration(Configuration& pi);
    double computeUtility(Configuration& pi);

private:
    std::set<Configuration> visited_configs;

    const unsigned int numSamples;

};

}

#endif /* INCLUDE_CONNECTED_MRPP_BIRK_BIRK_H_ */
