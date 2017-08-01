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

#include "connected_mrpp/graph/GenericGraph.h"

#include <lemon/connectivity.h>
#include <lemon/dijkstra.h>

using namespace lemon;

namespace connected_mrpp
{

GenericGraph::GenericGraph(lemon::ListDigraph& physicalGraph,
						   lemon::ListDigraph& comunicationGraph) :
									physicalGraph(physicalGraph),
									comunicationGraph(comunicationGraph)
{
}

double GenericGraph::GenericGraph::cost(int v, int v_next)
{
	return v == v_next ? 0 : 1; //TODO in teoria bisogna cercare se sono vicini...
}

double GenericGraph::heuristic(int v, int v_next)
{
	if(costmaps.count(v_next) == 0)
	{
		ListDigraph::ArcMap<double> len(physicalGraph, 1.0);

		auto* distMap = new ListDigraph::NodeMap<double>(physicalGraph);

		costmaps[v_next] = distMap;

		auto n = ListDigraph::nodeFromId(v_next);
		dijkstra(physicalGraph, len).distMap(*distMap).run(n);
	}

	auto n = ListDigraph::nodeFromId(v);

	return (*costmaps[v_next])[n];
}

std::vector<int> GenericGraph::getNeighbors(int v)
{
	std::vector<int> neighbours;

	auto n = ListDigraph::nodeFromId(v);
	for (ListDigraph::OutArcIt arcIt(physicalGraph, n); arcIt != INVALID; ++arcIt)
	{
		ListDigraph::Node n2 = physicalGraph.oppositeNode(n, arcIt);
		neighbours.push_back(ListDigraph::id(n2));
	}

	return neighbours;
}

bool GenericGraph::isConnected(std::vector<int>& v_list)
{
	ListDigraph::NodeMap<bool> nf(comunicationGraph, false);

	for(auto v : v_list)
	{
		nf[ListDigraph::nodeFromId(v)] = true;
	}

	return connected(undirector(filterNodes(comunicationGraph, nf)));
}

GenericGraph::~GenericGraph()
{
	for(auto& v : costmaps)
	{
		delete v.second;
	}
}

}
