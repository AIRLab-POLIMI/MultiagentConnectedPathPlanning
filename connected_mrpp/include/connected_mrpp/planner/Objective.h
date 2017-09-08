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

#ifndef INCLUDE_CONNECTED_MRPP_PLANNER_OBJECTIVE_H_
#define INCLUDE_CONNECTED_MRPP_PLANNER_OBJECTIVE_H_

#include "connected_mrpp/planner/Configuration.h"
#include "connected_mrpp/graph/Graph.h"

namespace connected_mrpp
{

class Objective
{
public:
	virtual double computeValue(Configuration& pi, Configuration& pi_n) = 0;
	virtual ~Objective();

};

class ZeroObjective : public Objective
{
public:
	virtual double computeValue(Configuration& pi, Configuration& pi_n);
};

class StepObjective : public Objective
{
public:
	virtual double computeValue(Configuration& pi, Configuration& pi_n);
};

class GraphObjective : public Objective
{
public:
	GraphObjective(Graph& graph);
protected:
	Graph& graph;
};

class DistanceObjective : public GraphObjective
{
public:
	DistanceObjective(Graph& graph, double epsilon = 1);
	virtual double computeValue(Configuration& pi, Configuration& pi_n);

private:
	double epsilon;

};

class ShortestPathObjective : public GraphObjective
{
public:
	ShortestPathObjective(Graph& graph);
	virtual double computeValue(Configuration& pi, Configuration& pi_n);
};

class SumShortestPathObjective : public GraphObjective
{
public:
	SumShortestPathObjective(Graph& graph);
	virtual double computeValue(Configuration& pi, Configuration& pi_n);
};

}


#endif /* INCLUDE_CONNECTED_MRPP_PLANNER_OBJECTIVE_H_ */
