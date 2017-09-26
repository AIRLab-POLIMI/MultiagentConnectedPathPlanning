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

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

using namespace boost;

namespace connected_mrpp
{

GenericGraph::GenericGraph(GraphStructure& physicalGraph,
                           GraphStructure& comunicationGraph) :
    physicalGraph(physicalGraph),
    comunicationGraph(comunicationGraph)
{
}

double GenericGraph::GenericGraph::cost(int v, int v_next)
{
    return v == v_next ? 0 : 1;
}

double GenericGraph::heuristic(int v, int v_next)
{
    if(costmaps.count(v_next) == 0)
    {
        std::vector<size_t> d(num_vertices(physicalGraph));

        auto n = vertex(v_next, physicalGraph);
        auto distanceVisitor = make_bfs_visitor(record_distances(&d[0], boost::on_tree_edge()));
        breadth_first_search(physicalGraph, n, visitor(distanceVisitor));

        costmaps[v_next] = d;
    }

    return costmaps[v_next][v];
}

std::vector<int> GenericGraph::getNeighbors(int v)
{
    std::vector<int> neighbours;

    auto iterator = adjacent_vertices(v, physicalGraph);

    for(auto v_next = iterator.first; v_next != iterator.second; v_next++)
    {
        neighbours.push_back(*v_next);
    }

    return neighbours;
}

bool GenericGraph::isNeighbor(int v, int v_next)
{
    return edge(v, v_next, physicalGraph).second;
}

unsigned int GenericGraph::degree(int v)
{
    return boost::degree(v, physicalGraph);
}


bool GenericGraph::isConnected(const std::vector<int>& v_list)
{
    std::set<unsigned int> v_set(v_list.begin(), v_list.end());


    auto subgraph = make_vertex_subset_filter(comunicationGraph, v_set);

    unsigned int components[num_vertices(subgraph)];
    return connected_components(subgraph, components) == 1;
}

GenericGraph::~GenericGraph()
{

}

}
