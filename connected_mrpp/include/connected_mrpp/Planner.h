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

#ifndef INCLUDE_CONNECTED_MRPP_THETASTARPLANNER_H_
#define INCLUDE_CONNECTED_MRPP_THETASTARPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "connected_mrpp/PriorityQueue.h"


namespace connected_mrpp
{

class Planner
{

public:
	Planner(Grid& grid);

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    void updateVertex(Cell s, Cell s_next);
    void computeCost(Cell s, Cell s_next);
    void clearInstance();

    void publishPlan(std::vector<Eigen::VectorXd>& path,
    				 std::vector<geometry_msgs::PoseStamped>& plan,
					 const ros::Time& stamp,
					 const geometry_msgs::PoseStamped& start,
					 const geometry_msgs::PoseStamped& goal);

    /*void displayClosed();
    void displayOpen();
    void displayNeighbours(const Cell& cell);
    void displayObstacles(const Cell& cell);
    void displayCost(const Cell& cell, double g_old);*/

private:
    static const Cell S_NULL;

    Grid& grid;

    Cell s_start;
    Cell s_goal;

    std::map<Cell, double> g;
    PriorityQueue open;
    std::map<Cell, Cell> parent;
    std::set<Cell> closed;

};

}

#endif /* INCLUDE_CONNECTED_MRPP_THETASTARPLANNER_H_ */
