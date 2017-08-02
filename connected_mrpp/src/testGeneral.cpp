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
#include <string>
#include <lemon/dimacs.h>

#include "connected_mrpp/graph/GenericGraph.h"

using namespace std;
using namespace connected_mrpp;

int main(int argc, char** argv)
{
	std::string basePath = "/home/dave/ros-extra/src/connectedmrpp/connected_mrpp/data/";
	std::ifstream fsP(basePath+"offices_phys_uniform_grid_11_range_150.net");
	std::ifstream fsC(basePath+"offices_comm_uniform_grid_11_range_150.net");

	lemon::ListDigraph gP;
	lemon::ListDigraph gC;
	lemon::readDimacsMat(fsP, gP);
	lemon::readDimacsMat(fsP, gC);

	GenericGraph graph(gP, gC);

}
