/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Davide Tateo
 * Versione 1.0
 *
 * This file is part of rrt_planning.
 *
 * rrt_planning is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * rrt_planning is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with rrt_planning.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_
#define INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

#include "rrt_planning/map/ROSMap.h"
#include "rrt_planning/grid/Grid.h"
#include "rrt_planning/theta_star/PriorityQueue.h"
#include "rrt_planning/visualization/Visualizer.h"


namespace rrt_planning
{

class ThetaStarPlanner : public nav_core::BaseGlobalPlanner
{

public:
    ThetaStarPlanner();
    ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

    ~ThetaStarPlanner();

private:
    void updateVertex(Cell s, Cell s_next);
    void computeCost(Cell s, Cell s_next);
    void publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                     const ros::Time& stamp, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal);
    void clearInstance();
    void displayClosed();
    void displayOpen();
    void displayNeighbours(const Cell& cell);
    void displayObstacles(const Cell& cell);
    void displayCost(const Cell& cell, double g_old);

private:

    ros::Publisher pub;


    static const Cell S_NULL;

    ROSMap* map;
    Grid* grid;

    Cell s_start;
    Cell s_goal;

    std::map<Cell, double> g;
    PriorityQueue open;
    std::map<Cell, Cell> parent;
    std::set<Cell> closed;

    Visualizer visualizer;
};

}

#endif /* INCLUDE_RRT_PLANNING_THETASTARPLANNER_H_ */
