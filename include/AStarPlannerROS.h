/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef TUE_ASTAR_PLANNER_ROS_H_
#define TUE_ASTAR_PLANNER_ROS_H_

#include <ros/ros.h>

#include "AStarPlanner.h"
//#include <costmap_2d/costmap_2d_ros.h>
#include <tue_map_3d/map_3d_cost_values.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

#include <tue_nav_core/base_global_planner.h>

#include <vector>

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

/**
 * @class AmigoGlobalPlanner
 */
class AStarPlannerROS : public tue_nav_core::TueBaseGlobalPlanner {
public:
    /**
     * @brief  Default constructor for the NavFnROS object
     */
    AStarPlannerROS();

    /**
     * @brief  Constructor for the NavFnROS object
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
     */
    AStarPlannerROS(std::string name, Map3D* map_3d);

    ~AStarPlannerROS();

    // **************************************

    /**
     * @brief  Initialization function for the NavFnROS object
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(const std::string& name, const Map3D* map_3d);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    bool getPlan(const geometry_msgs::PoseStamped& start,
                 const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

protected:

    /**
     * @brief Store a copy of the current costmap in a costmap.  Called by makePlan.
     */
    //virtual void getCostmap(costmap_2d::Costmap2D& costmap);

    //void getCostmap(costmap_2d::Costmap2D& costmap, double size_x, double size_y);

    /**
     * @brief  Publish a path for visualization purposes
     */
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

    //void publishCostmap(const costmap_2d::Costmap2D& costmap, int mx_center, int my_center, int size);

    //costmap_2d::Costmap2DROS* costmap_ros_;
    std::string global_frame_id_;

    const Map3D* map_;

    AStarPlanner* planner_;

    double inscribed_radius_, circumscribed_radius_, inflation_radius_;

    //ros::Publisher pub_costmap_;

    ros::Publisher plan_pub_;

    ros::Publisher error_pub_;

    bool initialized_;
    double max_velocity_;


private:

    inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
    }

    void mapToWorld(double mx, double my, double& wx, double& wy);

    //void clearRobotCell(costmap_2d::Costmap2D& costmap, const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);

    //double planner_window_x_, planner_window_y_, default_tolerance_;

    std::string tf_prefix_;

    int smooth_size_;



    //boost::mutex mutex_;
};

#endif
