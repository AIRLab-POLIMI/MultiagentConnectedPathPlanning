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

#include "connected_mrpp/Grid.h"
#include <cmath>
#include <Eigen/Dense>

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


vector<Cell> Grid::getNeighbors(const Cell& s)
{
    int X = s.first;
    int Y = s.second;

    vector<Cell> neighbors;

    //Given (X,Y), retrive all the eight-connected free cells
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
            if(i == 0 && j == 0) continue;
            if(X+i < 0 || Y+j < 0 ||
                    X+i > maxX || Y+j > maxY)
                continue;

            auto&& pos = toMapPose(X+i, Y+j);

            if(map.isFree(pos))
                neighbors.push_back(make_pair(X+i, Y+j));
        }

    return neighbors;
}

std::vector<Cell> Grid::getObstacles(const Cell& s)
{
    int X = s.first;
    int Y = s.second;

    vector<Cell> obstacles;

    //Given (X,Y), retrive all the eight-connected free cells
    for(int i = -1; i <= 1; i++)
        for(int j = -1; j <= 1; j++)
        {
            if(i == 0 && j == 0) continue;
            if(X+i < 0 || Y+j < 0 ||
                    X+i > maxX || Y+j > maxY)
                continue;

            auto&& pos = toMapPose(X+i, Y+j);

            if(!map.isFree(pos))
                obstacles.push_back(make_pair(X+i, Y+j));
        }

    return obstacles;
}


double Grid::cost(const Cell& s, const Cell& s_next)
{
    //TODO no obstacles (8-connected)?

    int X1 = s.first;
    int Y1 = s.second;

    int X2 = s_next.first;
    int Y2 = s_next.second;

    return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


double Grid::heuristic(const Cell& s, const Cell& s_next)
{
    int X1 = s.first;
    int Y1 = s.second;

    int X2 = s_next.first;
    int Y2 = s_next.second;

    return sqrt( (X2-X1)*(X2-X1) + (Y2-Y1)*(Y2-Y1) );
}


bool Grid::lineOfSight(const Cell& s, const Cell& s_next)
{
    int X1 = s.first;
    int Y1 = s.second;

    int X2 = s_next.first;
    int Y2 = s_next.second;

    //Determine how steep the line is
    bool is_steep = abs(Y2-Y1) > abs(X2-X1);

    //Possibly rotate the line
    if(is_steep)
    {
        swap(X1, Y1);
        swap(X2, Y2);
    }

    //Possibly swap start and end
    bool swapped = false;

    if(X1 > X2)
    {
        swap(X1, X2);
        swap(Y1, Y2);
    }

    int error = (X2 - X1) / 2;
    int ystep = Y1 < Y2 ? 1 : -1;

    int y = Y1;

    //Check for obstalces through the line
    for(int x = X1; x <= X2; x++)
    {
        Eigen::VectorXd pos;

        if(is_steep)
            pos = toMapPose(y, x);
        else
            pos = toMapPose(x, y);

        if(!map.isFree(pos)) return false;

        error -= abs(Y2 - Y1);
        if(error < 0)
        {
            y += ystep;
            error += (X2 - X1);
        }
    }

    return true;
}


Cell Grid::convertPose(const geometry_msgs::PoseStamped& msg)
{
    auto& t_ros = msg.pose.position;

    Bounds bounds = map.getBounds();

    int X_index = floor( (t_ros.x - bounds.minX) / gridResolution );
    int Y_index = floor( (t_ros.y - bounds.minY) / gridResolution );

    return make_pair(X_index, Y_index);
}


Eigen::VectorXd Grid::toMapPose(int X, int Y)
{
    Bounds bounds = map.getBounds();

    Eigen::VectorXd pos(2);

    pos(0) = (0.5 + X) * gridResolution + bounds.minX;
    pos(1) = (0.5 + Y) * gridResolution + bounds.minY;

    return pos;
}


bool Grid::isFree(const Cell& s)
{
    Eigen::VectorXd pos = toMapPose(s.first, s.second);

    return map.isFree(pos);
}

}
