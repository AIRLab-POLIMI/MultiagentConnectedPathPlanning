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
#include "connected_mrpp/planner/Astar.h"
#include "connected_mrpp/planner/DFS_Planner.h"
#include "connected_mrpp/planner/Birk.h"
#include "connected_mrpp/planner/GreedyRandomized.h"
#include "connected_mrpp/Experiment.h"

#include <boost/graph/graphml.hpp>

using namespace std;
using namespace boost;
using namespace connected_mrpp;

int main(int argc, char** argv)
{
	if(argc < 6)
	{
		cout << "Missing command line arguments" << endl;
		return -1;
	}

	std::string expFile(argv[1]);
	std::string alg(argv[2]);
	std::string cost_type(argv[3]);
	std::string heuristic_type(argv[4]);
	std::chrono::duration<double> Tmax(stod(argv[5]));


	std::string expName = expFile.substr(expFile.find_last_of("/")+1, expFile.length());
	std::string dataPath = expFile.substr(0, expFile.find_last_of("/"));
	std::string basePath = dataPath.substr(0, dataPath.find_last_of("/"));
	std::string logPath = basePath + "/logs";


	if(argc >= 7)
	{
		std::string logSubfolder(argv[6]);
		logPath += "/" + logSubfolder;
	}


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


	string objectives_types[] = {cost_type, heuristic_type};
	Objective* objectives[] = {nullptr, nullptr};

	for (int i = 0; i < 2; i++)
	{
		auto o = objectives_types[i];
		if(o == "zero")
		{
			objectives[i] = new ZeroObjective();
		}
		else if(o == "step")
		{
			objectives[i] = new StepObjective();
		}
		else if(o == "distance")
		{
			objectives[i] = new DistanceObjective(graph);
		}
		else if(o == "shortest_path")
		{
			objectives[i] = new ShortestPathObjective(graph);
		}
		else if(o == "sum_shortest_path")
		{
			objectives[i] = new SumShortestPathObjective(graph);
		}
		else if(o == "null")
		{
			objectives[i] = nullptr;
		}
		else
		{
			std::cout << "Error! objective not recognized" << std::endl;
			return -1;
		}
	}

	AbstractPlanner* planner;
	SamplingStrategy* strategy;

	if(alg == "astar")
	{
		planner = new Astar(graph, objectives[0], objectives[1], Tmax, 1000);
	}
	else if(alg == "birk")
	{
		planner = new Birk(graph, objectives[1],  Tmax);
	}
	else if(alg == "dfs")
	{
		planner = new DFS_Planner(graph, objectives[0], objectives[1],  Tmax);
	}
	else if(alg == "grlog")
	{
		strategy = new LogarithmicSamplingStrategy();
		planner = new GreedyRandomized(graph, objectives[1], Tmax, *strategy);
	}
	else if(alg == "grlin")
	{
		strategy = new PolynomialSamplingStrategy(1.0);
		planner = new GreedyRandomized(graph, objectives[1], Tmax, *strategy);
	}
	else if(alg == "gr2")
	{
		strategy = new PolynomialSamplingStrategy(2.0);
		planner = new GreedyRandomized(graph, objectives[1], Tmax, *strategy);
	}
	else
	{
		cout << alg << " not implemented" << endl;
		return -1;
	}

	Configuration start(exp.getStartConfig());
	Configuration goal(exp.getGoalConfig());

	bool found = planner->makePlan(start, goal);

	std::ofstream ofs(logPath +  "/" + expName.substr(0, expName.find_last_of(".")) + "_" + alg +".log");
	std::cout << "Computed plan: " << std::endl;

	if(found)
	{
		auto&& plan = planner->getPlan();

		for(int i = 0; i < plan.size(); i++)
		{
			std::cout <<  "[" << plan[i] << "]" << std::endl;
			ofs <<  "s" << i << " " << plan[i] << std::endl;
		}
	}
	else
	{
		std::cout <<  "NO_PLAN_FOUND_WITHIN_DEADLINE" << std::endl;
		ofs <<  "NO_PLAN_FOUND_WITHIN_DEADLINE" << std::endl;
	}

	ofs << "t " << planner->getElapsedTime() << " s" << std::endl;

	for (int i = 0; i < 2; i++)
	{
		if(objectives[i])
			delete objectives[i];
	}

	if(strategy)
		delete strategy;

	if(planner)
		delete planner;

}
