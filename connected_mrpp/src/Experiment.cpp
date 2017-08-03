/*
 * connected_mrpp,
 *
 *
 * Copyright (C) 2016 Davide Tateo
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

#include "connected_mrpp/Experiment.h"

//Default Constructor
namespace connected_mrpp
{

Experiment::Experiment(std::string basePath, std::string filename)
{
    std::ifstream experiment(basePath + filename);
    std::string line;
    Experiment::Field field;
    
    while(std::getline(experiment, line))
    {
        field = Experiment::NONE;
        std::istringstream iss(line);
        for(std::string s; iss >> s; )
        {
            if(field == Experiment::NONE){
                if (s == "phys_graph"){
                    field = Experiment::PHYS_GRAPH;
                }
                else if (s == "comm_graph"){
                    field = Experiment::COMM_GRAPH;
                }
                else if (s == "start"){
                    field = Experiment::START;
                }
                else if (s == "goal"){
                    field = Experiment::GOAL;
                }
                else{
                    std::cerr << "Parser error 1! Aborting.\n";
                    exit(1);
                }
            }
            else{
                switch(field){
                    case Experiment::PHYS_GRAPH:
                        physGraph = basePath + s;
                        break;
                    case Experiment::COMM_GRAPH:
                        commGraph = basePath + s;
                        break;
                    case Experiment::START:
                        startConfig.push_back(atoi(s.c_str()));
                        break;
                    case Experiment::GOAL:
                        goalConfig.push_back(atoi(s.c_str()));
                        break;
                    default:
                        std::cerr << "Parser error 2! Aborting.\n";
                        exit(1);
                }
            }
        }           
    }
    std::cout << "Experiment successfully read." <<std::endl;

}

std::string Experiment::getPhysGraph()
{
	return physGraph;
}

std::string Experiment::getCommGraph()
{
	return commGraph;
}

std::vector<int>& Experiment::getStartConfig()
{
	return startConfig;
}

std::vector<int>& Experiment::getGoalConfig()
{
	return goalConfig;
}

};
