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


#include <fstream>
#include <iostream>
#include <string>


#include "connected_mrpp/graph/GenericGraph.h"
#include "connected_mrpp/Planner.h"
#include "connected_mrpp/Experiment.h"

#include <boost/graph/graphml.hpp>

using namespace std;
using namespace boost;
using namespace connected_mrpp;

int main(int argc, char** argv)
{
	std::string basePath = "/home/dave/ros/src/connectedmrpp/connected_mrpp/data/";
    std::string expFile = "prova.exp";

    Experiment exp(basePath, expFile);

    cout << exp.getPhysGraph() << endl;
    cout << exp.getCommGraph() << endl;

	std::ifstream fsP(exp.getPhysGraph());
	std::ifstream fsC(exp.getCommGraph());

	GraphStructure physical;
	GraphStructure comunication;

	dynamic_properties dpP(ignore_other_properties), dpC(ignore_other_properties);
	read_graphml(fsP, physical, dpP);
	read_graphml(fsC, comunication, dpC);

	GenericGraph graph(physical, comunication);

	Planner planner(graph);

	Configuration start(exp.getStartConfig());
	Configuration goal(exp.getGoalConfig());

	planner.makePlan(start, goal);

}
