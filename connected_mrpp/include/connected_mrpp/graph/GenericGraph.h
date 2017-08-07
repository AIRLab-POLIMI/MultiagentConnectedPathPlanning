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

#ifndef INCLUDE_CONNECTED_MRPP_GENERICGRAPH_H_
#define INCLUDE_CONNECTED_MRPP_GENERICGRAPH_H_

#include "connected_mrpp/graph/Graph.h"

#include <boost/graph/adjacency_list.hpp>

#include <map>

namespace connected_mrpp
{

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS,
		boost::property<boost::vertex_color_t, boost::default_color_type>> GraphStructure;

class GenericGraph : public Graph
{
public:
	GenericGraph(GraphStructure& physicalGraph,
				GraphStructure& comunicationGraph);

    virtual double cost(int v, int v_next) override;
    virtual double heuristic(int v, int v_next) override;
    virtual std::vector<int> getNeighbors(int v) override;
    virtual bool isNeighbor(int v, int v_next) override;
    virtual unsigned int degree(int v);
    virtual bool isConnected(std::vector<int>& v_list) override;

    virtual ~GenericGraph();

private:
    GraphStructure& physicalGraph;
    GraphStructure& comunicationGraph;

    std::map<int, std::vector<size_t>> costmaps;
};

}


#endif /* INCLUDE_CONNECTED_MRPP_GENERICGRAPH_H_ */
