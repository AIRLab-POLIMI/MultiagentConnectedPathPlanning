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

#ifndef INCLUDE_CONNECTED_MRPP_COMMANDLINEPARSER_H_
#define INCLUDE_CONNECTED_MRPP_COMMANDLINEPARSER_H_

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <iostream>
#include <string>

#include "connected_mrpp/graph/GenericGraph.h"
#include "connected_mrpp/planner/AbstractPlanner.h"
#include "connected_mrpp/planner/SamplingStrategy.h"
#include "connected_mrpp/planner/Objective.h"

#include "connected_mrpp/Experiment.h"

#include <boost/graph/graphml.hpp>

namespace connected_mrpp
{

class CommandLineParser
{
public:
    CommandLineParser();

    AbstractPlanner* getPlanner(int argc, char **argv);
    std::string getPlannerName();
    Experiment& getExperiment();

    ~CommandLineParser();

private:
    void printParameters();
    void loadExperiment();
    Objective* getObjective(const std::string& name);
    SamplingStrategy& getStrategy(const std::string& name);
    AbstractPlanner* getPlanner(const std::string& alg);

private:
    boost::program_options::options_description desc;
    boost::program_options::variables_map vm;

    std::vector<Objective*> obj;
    AbstractPlanner* planner;
    SamplingStrategy* strategy;
    Graph* graph;
    Experiment* exp;

};


}


#endif /* INCLUDE_CONNECTED_MRPP_COMMANDLINEPARSER_H_ */
