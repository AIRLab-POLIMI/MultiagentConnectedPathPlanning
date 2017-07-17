/*
 * rrt_planning,
 *
 *
 * Copyright (C) 2016 Alessandro Riva
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

#include "rrt_planning/ThetaStarPlanner.h"

#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planning::ThetaStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace Eigen;

//Default Constructor
namespace rrt_planning
{

const Cell ThetaStarPlanner::S_NULL = make_pair(-1, -1);

ThetaStarPlanner::ThetaStarPlanner()
{
    grid = nullptr;
    map = nullptr;
}

ThetaStarPlanner::ThetaStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}


void ThetaStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    double discretization;

    //Get parameters from ros parameter server
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("discretization", discretization, 0.2);
    pub = private_nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    map = new ROSMap(costmap_ros);
    grid = new Grid(*map, discretization);

    visualizer.initialize(private_nh);
}

bool ThetaStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
    clearInstance();
    visualizer.clean();

    //Init the position of the special states
    s_start = grid->convertPose(start);
    s_goal = grid->convertPose(goal);

    //Test starting position
    if(!grid->isFree(s_start))
    {
        ROS_INFO("Invalid starting position");
        return false;
    }

    //Test target position
    if(!grid->isFree(s_goal))
    {
        ROS_INFO("Invalid target position");
        return false;
    }

    //Init variables
    g[s_start] = 0.0;
    parent[s_start] = s_start;
    open.insert(s_start, grid->heuristic(s_start, s_goal));
    parent[s_goal] = S_NULL;

    ROS_INFO("Planner started");

    //Compute plan
    while(!open.empty())
    {
        //Pop the best frontier node
        Cell s = open.pop();

        closed.insert(s);

        if(s == s_goal) break;

        for(auto s_next: grid->getNeighbors(s))
            if(closed.count(s_next) == 0)
            {
                if(!open.contains(s_next))
                {
                    g[s_next] = std::numeric_limits<double>::infinity();
                    parent[s_next] = S_NULL;
                }

                updateVertex(s, s_next);
            }
    }

    //Publish plan
    vector<VectorXd> path;
    auto state = s_goal;
    path.push_back(grid->toMapPose(state.first, state.second));
    do
    {
        state = parent.at(state);

        if(state == S_NULL)
        {
            ROS_INFO("Invalid plan");
            return false;
        }

        path.push_back(grid->toMapPose(state.first, state.second));
    }
    while(state != s_start);

    reverse(path.begin(), path.end());
    publishPlan(path, plan, start.header.stamp, start, goal);
    visualizer.displayPlan(plan);

    return true;
}


void ThetaStarPlanner::updateVertex(Cell s, Cell s_next)
{
    double g_old = g.at(s_next);

    computeCost(s, s_next);

    if(g.at(s_next) < g_old)
    {
        if(open.contains(s_next))
            open.remove(s_next);

        double frontierCost = g.at(s_next) + grid->heuristic(s_next, s_goal);

        open.insert(s_next, frontierCost);
    }
}


void ThetaStarPlanner::computeCost(Cell s, Cell s_next)
{

    if(grid->lineOfSight(parent.at(s), s_next))
    {
        //Path 2
        if(g.at(parent.at(s)) + grid->cost(parent.at(s), s_next) <= g.at(s_next))
        {
            parent.at(s_next) = parent.at(s);
            g.at(s_next) = g.at(parent.at(s)) + grid->cost(parent.at(s), s_next);
        }
    }
    else
    {
        //Path 1
        if(g.at(s) + grid->cost(s, s_next) <= g.at(s_next))
        {
            parent.at(s_next) = s;
            g.at(s_next) = g.at(s) + grid->cost(s, s_next);
        }
    }
}

void ThetaStarPlanner::publishPlan(std::vector<Eigen::VectorXd>& path, std::vector<geometry_msgs::PoseStamped>& plan,
                                   const ros::Time& stamp, const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    plan.push_back(start);

    for(int i = 1; i < path.size() - 1; i++)
    {
        auto&& p1 = path[i];
        auto&& p2 = path[i+1];

        geometry_msgs::PoseStamped msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";

        msg.pose.position.x = p1(0);
        msg.pose.position.y = p1(1);
        msg.pose.position.z = 0;

        double angle = atan2(p2(1) - p1(1), p2(0) - p1(0));

        Matrix3d m;
        m = AngleAxisd(angle, Vector3d::UnitZ())
            * AngleAxisd(0, Vector3d::UnitY())
            * AngleAxisd(0, Vector3d::UnitX());

        Quaterniond q(m);

        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        plan.push_back(msg);
    }

    plan.push_back(goal);
}


void ThetaStarPlanner::clearInstance()
{
    open.clear();
    closed.clear();
    parent.clear();
    g.clear();
}


void ThetaStarPlanner::displayOpen()
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "thetastar";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    for(auto& s : open)
    {
        geometry_msgs::Point p;

        VectorXd pos = grid->toMapPose(s->getNode().first, s->getNode().second);

        p.x = pos(0);
        p.y = pos(1);
        p.z = 0;

        marker.points.push_back(p);
    }

    pub.publish(marker);
}

void ThetaStarPlanner::displayNeighbours(const Cell& cell)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "thetastar";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    for(auto& s : grid->getNeighbors(cell))
    {
        geometry_msgs::Point p;

        VectorXd pos = grid->toMapPose(s.first, s.second);

        p.x = pos(0);
        p.y = pos(1);
        p.z = 0;

        marker.points.push_back(p);
    }

    pub.publish(marker);
}

void ThetaStarPlanner::displayObstacles(const Cell& cell)
{

    static visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "thetastar";
    marker.id = 3;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for(auto& s : grid->getObstacles(cell))
    {
        geometry_msgs::Point p;

        VectorXd pos = grid->toMapPose(s.first, s.second);

        p.x = pos(0);
        p.y = pos(1);
        p.z = 0;

        marker.points.push_back(p);
    }

    pub.publish(marker);
}



void ThetaStarPlanner::displayClosed()
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "thetastar";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    for(auto& s : closed)
    {
        geometry_msgs::Point p;

        VectorXd pos = grid->toMapPose(s.first, s.second);

        p.x = pos(0);
        p.y = pos(1);
        p.z = 0;

        marker.points.push_back(p);
    }

    pub.publish(marker);
}

void ThetaStarPlanner::displayCost(const Cell& cell, double g_old)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "thetastar";
    marker.id = 4;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "g: " + std::to_string(g.at(cell)) + " g_old: " + std::to_string(g_old);


    VectorXd pos = grid->toMapPose(cell.first, cell.second);

    marker.pose.position.x = pos(0);
    marker.pose.position.y = pos(1);
    marker.pose.position.z = 0;

    pub.publish(marker);
}


ThetaStarPlanner::~ThetaStarPlanner()
{
    if(grid)
        delete grid;

    if(map)
        delete map;
}


};
