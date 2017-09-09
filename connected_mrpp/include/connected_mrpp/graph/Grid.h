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

#ifndef INCLUDE_CONNECTED_MRPP_GRID_H_
#define INCLUDE_CONNECTED_MRPP_GRID_H_

#include "connected_mrpp/graph/Graph.h"
#include "connected_mrpp/map/Map.h"

#include <Eigen/Dense>

namespace connected_mrpp
{

class Grid : public Graph
{
public:
	Grid(Map& map, double gridResolution);

    virtual double cost(int v, int v_next) override;
    virtual double heuristic(int v, int v_next) override;
    virtual std::vector<int> getNeighbors(int v) override;
    virtual bool isConnected(const std::vector<int>& v_list) override;

    virtual ~Grid();

public:
    std::vector<int> getObstacles(int v);
    bool isFree(int v);

public:
    void convert(int v, int& x, int& y);
    int convert(int x, int y);
	//int convertPose(const geometry_msgs::PoseStamped& msg);
    Eigen::VectorXd toMapPose(int X, int Y);

private:
	Map& map;

    double gridResolution;
    unsigned int maxX;
    unsigned int maxY;

};

}

#endif /* INCLUDE_CONNECTED_MRPP_GRID_H_ */
