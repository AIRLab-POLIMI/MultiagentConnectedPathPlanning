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

#include "connected_mrpp/graph/Grid.h"

using namespace std;

namespace connected_mrpp
{

Grid::Grid(Map& map, double gridResolution): map(map),
    gridResolution(gridResolution)
{
    Bounds bounds = map.getBounds();

    maxX = floor((bounds.maxX - bounds.minX) / gridResolution);
    maxY = floor((bounds.maxY - bounds.minY) / gridResolution);
}


vector<int> Grid::getNeighbors(int v)
{
    int X, Y;
    convert(v, X, Y);

    vector<int> neighbors;

    //Given (X,Y), retrive all the eight-connected free ints
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
            if(i == 0 && j == 0) continue;
            if(X+i < 0 || Y+j < 0 ||
                    X+i > maxX || Y+j > maxY)
                continue;

            auto&& pos = toMapPose(X+i, Y+j);

            if(map.isFree(pos))
                neighbors.push_back(convert(X+i, Y+j));
        }

    return neighbors;
}

std::vector<int> Grid::getObstacles(int v)
{
	int X, Y;
	convert(v, X, Y);

    vector<int> obstacles;

    //Given (X,Y), retrive all the eight-connected free ints
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
            if(i == 0 && j == 0) continue;
            if(X+i < 0 || Y+j < 0 ||
                    X+i > maxX || Y+j > maxY)
                continue;

            auto&& pos = toMapPose(X+i, Y+j);

            if(!map.isFree(pos))
                obstacles.push_back(convert(X+i, Y+j));
        }

    return obstacles;
}


double Grid::cost(int v, int v_next)
{
    int X1, Y1;
    int X2, Y2;
    convert(v, X1, Y1);
    convert(v_next, X2, Y2);

    return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


double Grid::heuristic(int v, int v_next)
{
    int X1, Y1;
    int X2, Y2;
    convert(v, X1, Y1);
    convert(v_next, X2, Y2);

    return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}

bool Grid::isConnected(std::vector<int>& v_list)
{
	//TODO implement
	return true;
}

Grid::~Grid()
{

}


/*int Grid::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& t_ros = msg.pose.position;

    Bounds bounds = map.getBounds();

    int X_index = floor( (t_ros.x - bounds.minX) / gridResolution );
    int Y_index = floor( (t_ros.y - bounds.minY) / gridResolution );

    return convert(X_index, Y_index);
}*/


Eigen::VectorXd Grid::toMapPose(int X, int Y)
{
    Bounds bounds = map.getBounds();

    Eigen::VectorXd pos(2);

    pos(0) = (0.5 + X) * gridResolution + bounds.minX;
    pos(1) = (0.5 + Y) * gridResolution + bounds.minY;

    return pos;
}


bool Grid::isFree(int v)
{
	int X, Y;
	convert(v, X, Y);

    Eigen::VectorXd pos = toMapPose(X, Y);

    return map.isFree(pos);
}

void Grid::convert(int v, int& x, int& y)
{
	x = v % maxX;
	y = v / maxX;
}

int Grid::convert(int x, int y)
{
	return x + maxX*y;
}

}
