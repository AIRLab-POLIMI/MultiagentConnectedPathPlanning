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
#include "connected_mrpp/planner/Planner.h"
#include "connected_mrpp/planner/Birk.h"
#include "connected_mrpp/Experiment.h"

#include <boost/graph/graphml.hpp>

using namespace std;
using namespace boost;
using namespace connected_mrpp;

int main(int argc, char** argv)
{
	if(argc < 4)
	{
		cout << "Missing experimentFile algorithm and deadline" << endl;
		return -1;
	}

	std::string expFile(argv[1]);
	std::string alg(argv[2]);
	std::chrono::duration<double> Tmax(stod(argv[3]));

	std::string expName = expFile.substr(expFile.find_last_of("/"), expFile.length());
	std::string dataPath = expFile.substr(0, expFile.find_last_of("/"));
	std::string basePath = dataPath.substr(0, dataPath.find_last_of("/"));
	std::string logPath = basePath + "/logs";


    cout << "Loading experiment: " << expFile << endl;
    cout << "Base path:          " << basePath << endl;
    cout << "Data path:          " << dataPath << endl;
    cout << "Log path:           " << logPath << endl;

    Experiment exp(dataPath, expName);

    cout << "physical graph:     " << exp.getPhysGraph() << endl;
    cout << "comunication graph: " << exp.getCommGraph() << endl;
    cout << "Time deadline:      " << Tmax.count() << " s" << std::endl;

	std::ifstream fsP(exp.getPhysGraph());
	std::ifstream fsC(exp.getCommGraph());

	GraphStructure physical;
	GraphStructure comunication;

	dynamic_properties dpP(ignore_other_properties), dpC(ignore_other_properties);
	read_graphml(fsP, physical, dpP);
	read_graphml(fsC, comunication, dpC);


	GenericGraph graph(physical, comunication);


	AbstractPlanner* planner;

	if(alg == "planner")
	{
		planner = new Planner(graph, Tmax);
	}
	else if (alg == "birk")
	{
		planner = new Birk(graph, Tmax);
	}
	else
	{
		cout << alg << " not implemented" << endl;
		return -1;
	}

	Configuration start(exp.getStartConfig());
	Configuration goal(exp.getGoalConfig());

	bool found = planner->makePlan(start, goal);

	if(found)
	{
		std::ofstream ofs(logPath +  "/" + expName.substr(0, expName.find_last_of("."))+".log");
		std::cout << "Computed plan: " << std::endl;
		auto&& plan = planner->getPlan();

		for(int i = 0; i < plan.size(); i++)
		{
			std::cout <<  "[" << plan[i] << "]" << std::endl;
			ofs <<  "s" << i << " " << plan[i] << std::endl;
		}
	}

}
