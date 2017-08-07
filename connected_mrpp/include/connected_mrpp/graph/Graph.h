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

#ifndef INCLUDE_CONNECTED_MRPP_GRID_GRID_H_
#define INCLUDE_CONNECTED_MRPP_GRID_GRID_H_

#include <vector>

namespace connected_mrpp
{

class Graph
{

public:
    virtual double cost(int v, int v_next) = 0;
    virtual double heuristic(int v, int v_next) = 0;
    virtual std::vector<int> getNeighbors(int v) = 0;
    virtual bool isNeighbor(int v, int v_next) = 0;
    virtual unsigned int degree(int v) = 0;
    virtual bool isConnected(std::vector<int>& v_list) = 0;

    virtual ~Graph() { }

};

}

#endif /* INCLUDE_CONNECTED_MRPP_GRID_GRID_H_ */
