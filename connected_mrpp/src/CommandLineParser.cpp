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

#include "connected_mrpp/CommandLineParser.h"

#include "connected_mrpp/planner/Astar.h"
#include "connected_mrpp/planner/DFS_Planner.h"
#include "connected_mrpp/planner/Birk.h"
#include "connected_mrpp/planner/GreedyRandomized.h"
#include "connected_mrpp/planner/RestartingGreedyRandomized.h"

using namespace std;
using namespace boost;

namespace connected_mrpp
{

CommandLineParser::CommandLineParser()
{
    desc.add_options() //
    ("help,h", "produce help message") //
    ("exp_file,f", boost::program_options::value<std::string>(), "set the experiment file") //
    ("subfolder,S", boost::program_options::value<std::string>(), "set the log subfolder") //
    ("algorithm,a", boost::program_options::value<std::string>(), "set the algorithm") //
    ("deadline,d", boost::program_options::value<double>()->default_value(300.0), "set the deadline") //
    ("cost,c", boost::program_options::value<std::string>(), "set the cost") //
    ("heuristic,H", boost::program_options::value<std::string>(), "set the heuristic") //
    ("epsilon,e", boost::program_options::value<double>()->default_value(1.0), "set the psilon for the A* algorithm")
    ("alpha,A", boost::program_options::value<double>(), "set the alpha parameter for the RRSB algorithm") //
    ("beta,B", boost::program_options::value<double>(), "set the beta parameter for the RRSB algorithm")
    ("strategy,s", boost::program_options::value<std::string>(), "set the sampling strategy for the RSB algorithm") //
    ("exponent,e", boost::program_options::value<double>()->default_value(3.0), "set the exponent for the polynomial sampling startegy for the RSB algorithm");

    planner = nullptr;
    strategy = nullptr;
    graph = nullptr;
    exp = nullptr;
}

AbstractPlanner* CommandLineParser::getPlanner(int argc, char **argv)
{
    try
    {
        store(parse_command_line(argc, argv, desc), vm);
        if(vm.count("help"))
        {
            std::cout << desc << std::endl;
            exit(0);
        }
        notify(vm);

        printParameters();
        loadExperiment();
        return getPlanner(vm["algorithm"].as<std::string>());
    }
    catch (boost::program_options::error& e)
    {
        std::cout << e.what() << std::endl;
        std::cout << "Usage:" << std::endl;
        std::cout << desc << std::endl;
        exit(0);
    }
}

std::string CommandLineParser::getPlannerName()
{
    return vm["algorithm"].as<std::string>();
}

Experiment& CommandLineParser::getExperiment()
{
    return *exp;
}

CommandLineParser::~CommandLineParser()
{
    for(auto o : obj)
    {
        delete o;
    }

    if(planner)
    {
        delete planner;
    }

    if(graph)
    {
        delete graph;
    }

    if(exp)
    {
        delete exp;
    }
}

void CommandLineParser::printParameters()
{
    std::cout << "Experiment:    " << vm["exp_file"].as<std::string>() << std::endl;
    std::cout << "Algorithm:     " << vm["algorithm"].as<std::string>() << std::endl;
    std::cout << "Time deadline: " << vm["deadline"].as<double>() << std::endl;
}

void CommandLineParser::loadExperiment()
{
    auto expFile = vm["exp_file"].as<std::string>();


    std::string expName = expFile.substr(expFile.find_last_of("/")+1, expFile.length());
    std::string dataPath = expFile.substr(0, expFile.find_last_of("/"));
    std::string basePath = dataPath.substr(0, dataPath.find_last_of("/"));
    std::string logPath = basePath + "/logs";

    if(vm.count("subfolder") != 0)
    {
        logPath += "/" + vm["subfolder"].as<std::string>();
    }

    exp = new Experiment(dataPath, expName, logPath);

    std::ifstream fsP(exp->getPhysGraph());
    std::ifstream fsC(exp->getCommGraph());

    GraphStructure physical;
    GraphStructure comunication;

    dynamic_properties dpP(ignore_other_properties), dpC(ignore_other_properties);
    read_graphml(fsP, physical, dpP);
    read_graphml(fsC, comunication, dpC);

    graph = new GenericGraph(physical, comunication);
}

inline Objective* CommandLineParser::getObjective(const std::string& name)
{
    Objective* o;
    if(name == "zero")
    {
        o = new ZeroObjective();
    }
    else if(name == "step")
    {
        o = new StepObjective();
    }
    else if(name == "distance")
    {
        o = new DistanceObjective(*graph);
    }
    else if(name == "max_shortest_path")
    {
        o = new MaxShortestPathObjective(*graph);
    }
    else if(name == "mean_shortest_path")
    {
        o = new MeanShortestPathObjective(*graph);
    }
    else if(name == "sum_shortest_path")
    {
        o = new SumShortestPathObjective(*graph);
    }
    else
    {
        throw boost::program_options::error("The " + name + " objective is not implemented");
    }

    obj.push_back(o);

    return o;
}

SamplingStrategy& CommandLineParser::getStrategy(const std::string& name)
{
    if(name == "log")
    {
        strategy = new LogarithmicSamplingStrategy();
    }
    else if(name == "poly")
    {
        double exponent = vm["exponent"].as<double>();
        strategy = new PolynomialSamplingStrategy(exponent);
    }
    else
    {
        throw boost::program_options::error("The " + name + " sampling strategy is not implemented");
    }

    return *strategy;
}

AbstractPlanner* CommandLineParser::getPlanner(const std::string& alg)
{
    std::chrono::duration<double> Tmax(vm["deadline"].as<double>());

    planner = nullptr;

    if(alg == "astar")
    {
        auto cost = vm["cost"].as<std::string>();
        auto heuristic = vm["heuristic"].as<std::string>();
        auto epsilon = vm["epsilon"].as<double>();

        planner = new Astar(*graph, getObjective(cost), getObjective(heuristic), Tmax, epsilon);
    }
    else if(alg == "sb")
    {
        auto heuristic = vm["heuristic"].as<std::string>();
        planner = new Birk(*graph, getObjective(heuristic), Tmax);
    }
    else if(alg == "dfs")
    {
        auto cost = vm["cost"].as<std::string>();
        auto heuristic = vm["heuristic"].as<std::string>();
        planner = new DFS_Planner(*graph, getObjective(cost), getObjective(heuristic), Tmax);
    }
    else if(alg == "rsb")
    {
        auto heuristic = vm["heuristic"].as<std::string>();
        auto strategyName = vm["strategy"].as<std::string>();

        planner = new GreedyRandomized(*graph, getObjective(heuristic), Tmax, getStrategy(strategyName));
    }
    else if(alg == "rrsb")
    {
        auto heuristic = vm["heuristic"].as<std::string>();
        auto strategyName = vm["strategy"].as<std::string>();
        auto alpha = vm["alpha"].as<double>();

        planner = new RestartingGreedyRandomized(*graph, getObjective(heuristic), Tmax, getStrategy(strategyName), alpha);
    }
    else
    {
        throw boost::program_options::error("The " + alg + " algorithm is not implemented");
    }

    return planner;
}

}
