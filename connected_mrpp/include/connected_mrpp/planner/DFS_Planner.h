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

#ifndef INCLUDE_CONNECTED_MRPP_PLANNER_DFS_PLANNER_H_
#define INCLUDE_CONNECTED_MRPP_PLANNER_DFS_PLANNER_H_

#include "connected_mrpp/planner/LazyPlanner.h"

#include <list>

namespace connected_mrpp
{

class DFS_Planner : public LazyPlanner
{
public:
	DFS_Planner(Graph& graph, std::chrono::duration<double> Tmax);


protected:
    virtual bool makePlanImpl() override;
    virtual void clearInstanceSpecific() override;

protected:
    virtual double computeCost(Configuration& pi, Configuration& pi_n) override;

private:
    std::list<Configuration> stack;

};



}

#endif /* INCLUDE_CONNECTED_MRPP_PLANNER_DFS_PLANNER_H_ */
