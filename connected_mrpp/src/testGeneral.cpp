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

#include "connected_mrpp/CommandLineParser.h"


using namespace connected_mrpp;

int main(int argc, char** argv)
{

    CommandLineParser parser;

    auto planner = parser.getPlanner(argc, argv);
    auto& exp = parser.getExperiment();

    auto alg = parser.getPlannerName();
    auto logPath = exp.getLogPath();
    auto expName = exp.getExpName();

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

}
