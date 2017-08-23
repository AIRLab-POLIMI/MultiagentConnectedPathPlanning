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

#include "connected_mrpp/planner/LazyPlanner.h"


using namespace std;
using namespace std::rel_ops;
using namespace std::chrono;

namespace connected_mrpp
{

LazyPlanner::LazyPlanner(Graph& graph, duration<double> Tmax) : AbstractPlanner(graph, Tmax)
{

}

Configuration LazyPlanner::findBestConfiguration(Configuration& pi)
{
	if(g_local.count(pi) == 0)
	{
		PartialConfiguration pi_a(pi);

		g_local[pi][pi_a] = 0;
		open_local[pi].insert(pi_a, 0, computeHeuristic(pi));
	}

	auto& pi_open = open_local[pi];
	auto& pi_g = g_local[pi];

	while(!pi_open.empty())
	{
		PartialConfiguration a = pi_open.pop();

		if(a.count == a.agent.size()
				&& isConnected(a)
				&& closed.count(a) == 0
				&& static_cast<Configuration>(a) != pi)
		{
			return a;
		}

		for(auto a_n : successors(a))
		{
			double g = pi_g[a] + computeCost(a, a_n);
			pi_g[a_n] = g;

			pi_open.insert(a_n, g, computeHeuristic(a_n));
		}
	}

	return PI_NULL;
}


std::vector<PartialConfiguration> LazyPlanner::successors(PartialConfiguration& a)
{
	std::vector<PartialConfiguration> succ;

	if(a.count != a.agent.size())
	{
		auto v = a.agent[a.count];

		auto neighbours = graph.getNeighbors(v);
		neighbours.push_back(v);

		for(auto& v_n : neighbours)
		{
			PartialConfiguration a_n = a;
			a_n.count += 1;
			a_n.agent[a.count] = v_n;
			succ.push_back(a_n);
		}
	}

	return succ;
}

void LazyPlanner::clearInstanceSpecific()
{
	//Clear principal routine data structure
	closed.clear();

	//clear local search data structure
	open_local.clear();
	g_local.clear();
}


}
