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

#include "connected_mrpp/planner/Objective.h"

namespace connected_mrpp
{

Objective::~Objective()
{

}

double ZeroObjective::computeValue(Configuration& pi, Configuration& pi_n)
{
	return 0;
}


double StepObjective::computeValue(Configuration& pi, Configuration& pi_n)
{
	return 1;
}

GraphObjective::GraphObjective(Graph& graph) : graph(graph)
{

}


DistanceObjective::DistanceObjective(Graph& graph, double epsilon) : GraphObjective(graph), epsilon(epsilon)
{

}

double DistanceObjective::computeValue(Configuration& pi, Configuration& pi_n)
{
	double J = 0;
	for(unsigned int i = 0; i < pi.agent.size(); i++)
	{
		auto v = pi.agent[i];
		auto v_n = pi_n.agent[i];
		J += graph.cost(v, v_n);
	}

	return J*epsilon;
}


ShortestPathObjective::ShortestPathObjective(Graph& graph) : GraphObjective(graph)
{

}

double ShortestPathObjective::computeValue(Configuration& pi, Configuration& pi_n)
{
	double J = 0;
	for(unsigned int i = 0; i < pi.agent.size(); i++)
	{
		auto v = pi.agent[i];
		auto v_n = pi_n.agent[i];
		double Jnew = graph.heuristic(v, v_n);

		J = std::min(J, Jnew);
	}

	return J;
}


SumShortestPathObjective::SumShortestPathObjective(Graph& graph) : GraphObjective(graph)
{

}

double SumShortestPathObjective::computeValue(Configuration& pi, Configuration& pi_n)
{
	double J = 0;
	for(unsigned int i = 0; i < pi.agent.size(); i++)
	{
		auto v = pi.agent[i];
		auto v_n = pi_n.agent[i];
		J += graph.heuristic(v, v_n);
	}

	return J;
}

}
