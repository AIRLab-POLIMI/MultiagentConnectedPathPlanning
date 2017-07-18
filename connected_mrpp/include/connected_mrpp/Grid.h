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
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include "connected_mrpp/map/Map.h"
#include "connected_mrpp/Cell.h"

namespace connected_mrpp
{

class Grid
{

public:
    Grid(Map& map, double gridResolution);
    double cost(const Cell& s, const Cell& s_next);
    double heuristic(const Cell& s, const Cell& s_next);
    bool lineOfSight(const Cell& s, const Cell& s_next);
    std::vector<Cell> getNeighbors(const Cell& s);
    std::vector<Cell> getObstacles(const Cell& s);
    Cell convertPose(const geometry_msgs::PoseStamped& msg);
    bool isFree(const Cell& s);
    Eigen::VectorXd toMapPose(int X, int Y);

private:
    Map& map;

    double gridResolution;	// Cell edges in meters
    unsigned int maxX;
    unsigned int maxY;
};

}

#endif /* INCLUDE_CONNECTED_MRPP_GRID_GRID_H_ */
