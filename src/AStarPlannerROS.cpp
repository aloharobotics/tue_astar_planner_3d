/*
 * AStarPlannerROS.cpp
 *
 *  Created on: Apr 18, 2012
 *      Author: sdries
 */

#include "AStarPlannerROS.h"

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

#ifdef ASTAR_MEASURE_TIME
	#include <time.h>
#endif

using namespace std;

//register this planner as a TueBaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(tue_astar_planner_3d, AStarPlannerROS, AStarPlannerROS, tue_nav_core::TueBaseGlobalPlanner)


AStarPlannerROS::AStarPlannerROS()
: map_(NULL),  planner_(), initialized_(false) {}

AStarPlannerROS::AStarPlannerROS(std::string name, Map3D *map_3d)
: map_(NULL),  planner_(), initialized_(false) {
	//initialize the planner
    initialize(name, map_3d);
}

AStarPlannerROS::~AStarPlannerROS() {
	delete planner_;
}

void AStarPlannerROS::initialize(const std::string& name, const Map3D* map_3d){

	if(!initialized_){

        map_ = map_3d;

        planner_ = new AStarPlanner(map_->getSizeX(), map_->getSizeY());

		ros::NodeHandle private_nh("~/" + name);

		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

		double costmap_pub_freq;
		private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

        private_nh.param("smoothing_window_size", smooth_size_, 1);

		//read parameters for the planner
        global_frame_id_ = map_->getGlobalFrameID();

		//we'll get the parameters for the robot radius from the costmap we're associated with
        inscribed_radius_ = map_->getInscribedRadius();
        circumscribed_radius_ = map_->getCircumscribedRadius();
        inflation_radius_ = map_->getInflationRadius();

        //pub_costmap_ = private_nh.advertise<visualization_msgs::Marker>("global_costmap", 2);
        error_pub_ = private_nh.advertise<std_msgs::Bool>("result", 1);

		//get the tf prefix
		ros::NodeHandle prefix_nh;
		tf_prefix_ = tf::getPrefixParam(prefix_nh);

		max_velocity_ = 1.0;

		initialized_ = true;
	}
	else
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

//void AStarPlannerROS::clearRobotCell(Map3D *costmap, const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my){
//	if(!initialized_){
//		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
//		return;
//	}

//	//set the associated costs in the cost map to be free
//    //costmap->setCost(mx, my, Map3DCostValues::FREE_SPACE);

//	//double max_inflation_dist = inflation_radius_ + inscribed_radius_;

//	//make sure to re-inflate obstacles in the affected region
//	//costmap.reinflateWindow(global_pose.getOrigin().x(), global_pose.getOrigin().y(), max_inflation_dist, max_inflation_dist);

//	//just in case we inflate over the point we just cleared
//	//costmap.setCost(mx, my, costmap_2d::FREE_SPACE);

//}

//void AStarPlannerROS::getCostmap(costmap_2d::Costmap2D& costmap) {
//#ifdef ASTAR_MEASURE_TIME
//	timespec t_start, t_end;
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
//#endif

//    costmap_->clearRobotFootprint();
//    costmap_->getCostmapCopy(costmap);

//#ifdef ASTAR_MEASURE_TIME
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
//	printf("getCostmap: %f sec.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);
//	printf("costmap size = %d x %d\n", costmap.getSizeInCellsX(), costmap.getSizeInCellsY());
//#endif
//}

//void AStarPlannerROS::getCostmap(costmap_2d::Costmap2D& costmap, double size_x, double size_y) {
//#ifdef ASTAR_MEASURE_TIME
//	timespec t_start, t_end;
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
//#endif

//    costmap_->clearRobotFootprint();
//    costmap_->getCostmapWindowCopy(size_x, size_y, costmap);

//#ifdef ASTAR_MEASURE_TIME
//	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
//	printf("getCostmap (window): %f sec.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);
//	printf("costmap size = %d x %d\n", costmap.getSizeInCellsX(), costmap.getSizeInCellsY());
//#endif
//}

/// This function can not be used from Map3D because it expects an integer cell value!
/// it is preferred to leave the function as it is in Map3D (and not change it to doubles)
void AStarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = map_->getMapOrigin().position.x + mx * map_->getRes();
    wy = map_->getMapOrigin().position.y + my * map_->getRes();
}

bool AStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

    //boost::mutex::scoped_lock lock(mutex_);

#ifdef ASTAR_MEASURE_TIME
	timespec t_start_total, t_end_total;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start_total);
#endif

	//clear the plan, just in case
	plan.clear();

	ros::NodeHandle n;

	//until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, map_->getGlobalFrameID())){
		ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                tf::resolve(tf_prefix_, map_->getGlobalFrameID()).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
                std_msgs::Bool res;
                res.data = false;
                error_pub_.publish(res);
                return false;
	}

    if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, map_->getGlobalFrameID())){
		ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                tf::resolve(tf_prefix_, map_->getGlobalFrameID()).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
                std_msgs::Bool res;
                res.data = false;
                error_pub_.publish(res);
                return false;
	}


	//double x_size = fabs(start.pose.position.x - goal.pose.position.x) + 1.0;
	//double y_size = fabs(start.pose.position.y - goal.pose.position.y) + 1.0;

    //costmap_2d::Costmap2D costmap;

	bool success = false;
	//getCostmap(costmap, x_size, y_size);
	//success = makePlan(start, goal, costmap, plan);

	if (!success) {
		// if it didn't succeed, get the full costmap, and try again
        //getCostmap(costmap);
        success = getPlan(start, goal, plan);
	}

#ifdef ASTAR_MEASURE_TIME
	timespec t_end_total_before_pub;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end_total_before_pub);
	printf("TOTAL before publishing: %f sec.\n", (t_end_total_before_pub.tv_sec - t_start_total.tv_sec) + double(t_end_total_before_pub.tv_nsec - t_start_total.tv_nsec) / 1e9);
#endif

	//publish the plan for visualization purposes
	publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

    // publish costmap
    //unsigned int mx_start, my_start;
    //if (costmap_.worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    //	publishCostmap(costmap_, mx_start, my_start, 50);
    //}

#ifdef ASTAR_MEASURE_TIME
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end_total);
	printf("TOTAL: %f sec.\n", (t_end_total.tv_sec - t_start_total.tv_sec) + double(t_end_total.tv_nsec - t_start_total.tv_nsec) / 1e9);
#endif

        std_msgs::Bool res;
        res.data = success;
        error_pub_.publish(res);
	return success;
}

bool AStarPlannerROS::getPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

	unsigned int mx_start, my_start;
    if(!map_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)){
		ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		return false;
	}

	unsigned int mx_goal, my_goal;
    if(!map_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)){
		ROS_WARN("The goal sent to the planner is off the global costmap. Planning will always fail to this goal.");
        return false;
	}

    double goal_cell_cost = map_->getCost(mx_goal, my_goal);
    if (goal_cell_cost >= Map3DCostValues::INSCRIBED_INFLATED_OBSTACLE) {
		ROS_WARN("The goal sent to the planner is in an (inflated) obstacle.");
        return false;
	}

	//clear the starting cell within the costmap because we know it can't be an obstacle
	tf::Stamped<tf::Pose> start_pose;
	tf::poseStampedMsgToTF(start, start_pose);

    //clearRobotCell(costmap, start_pose, mx_start, my_start);
    //costmap_->clearRobotFootprint();

#ifdef ASTAR_MEASURE_TIME
	timespec t_start, t_end;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
#endif
	//make sure to resize the underlying array that Navfn uses
    planner_->resize(map_->getSizeX(), map_->getSizeY());

#ifdef ASTAR_MEASURE_TIME
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
	printf("resize: %f sec.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);
#endif

    planner_->setCostmap(map_->getCostmap());

	std::vector<int> plan_xs;
	std::vector<int> plan_ys;

#ifdef ASTAR_MEASURE_TIME
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
#endif

	planner_->plan(mx_start, my_start, mx_goal, my_goal, plan_xs, plan_ys);

#ifdef ASTAR_MEASURE_TIME
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
	printf("planning: %f sec.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);
#endif

    /// smooth plan
	std::vector<double> plan_xs_smooth;
	std::vector<double> plan_ys_smooth;
	plan_xs_smooth.resize(plan_xs.size());
	plan_ys_smooth.resize(plan_ys.size());

    //int smooth_size_ = 2;
    /// loop over all poses of the path
	for(int i = 0; i < (int)plan_xs.size(); ++i) {
		int n = 0;
		double x_total = 0;
		double y_total = 0;
        /// get the previous and next poses within the smoothing window for every pose
        for(int j = max(i - smooth_size_, 0); j < min(i + smooth_size_ + 1, (int)plan_xs.size()); ++j) {
			x_total += plan_xs[j];
			y_total += plan_ys[j];
			++n;
		}
        /// average the pose over the smoothing window
		plan_xs_smooth[i] = x_total / n;
		plan_ys_smooth[i] = y_total / n;

        //ROS_INFO_STREAM("plan_x[i]" << plan_xs[i] << ", plan_xs[i] = " << plan_xs_smooth[i]);
	}

	ros::Time plan_time = ros::Time::now();

	for(int i = (int)plan_xs.size() - 1; i >= 0; --i) {
		double world_x, world_y;
        /// use mapToWorld defined in this class because now the map coordinates are doubles (as they are smoothed and
        /// thus can be inbetween to integer values) instead of the expected int values...
        mapToWorld(plan_xs_smooth[i], plan_ys_smooth[i], world_x, world_y);

		geometry_msgs::PoseStamped pose;
		pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame_id_;
		pose.pose.position.x = world_x;
		pose.pose.position.y = world_y;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		plan.push_back(pose);
	}

	if (plan.empty()) {
		// publish the (empty) plan, such that the former visualized plan is removed
		publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
		return false;
	}

    geometry_msgs::PoseStamped goal_copy = goal;
    goal_copy.header.stamp = ros::Time::now();
    plan.push_back(goal_copy);

	return true;
}

void AStarPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//create a message for the plan
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	if(!path.empty())
	{
		gui_path.header.frame_id = path[0].header.frame_id;
		gui_path.header.stamp = path[0].header.stamp;
	}

	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
	for(unsigned int i=0; i < path.size(); i++){
		gui_path.poses[i] = path[i];
	}

	plan_pub_.publish(gui_path);
}

//void AStarPlannerROS::publishCostmap(const Map3D* costmap, int mx_center, int my_center, int size) {
//	int width = costmap->getSizeX();
//	int height = costmap->getSizeY();

//	visualization_msgs::Marker marker;

//    marker.header.frame_id = costmap_->getGlobalFrameID();
//	marker.header.stamp = ros::Time::now();
//	marker.ns = "costmap_amigo_global";
//	marker.id = 0;
//	marker.type = visualization_msgs::Marker::CUBE_LIST;
//	marker.action = visualization_msgs::Marker::ADD;

//	marker.scale.x = costmap->getResolution();
//	marker.scale.y = costmap->getResolution();
//	marker.scale.z = costmap->getResolution();

//	// Set alpha to 1
//	marker.color.a = 1;

//	marker.lifetime = ros::Duration(0.0);

//	double x, y;
//	for(int mx = max(mx_center - size, 0); mx < min(mx_center + size, width - 1); ++mx) {
//		for(int my = max(my_center - size, 0); my < min(my_center + size, height - 1); ++my) {
//			double cost = (double)costmap.getCost((int)mx, (int)my) / 255;

//			costmap->mapToWorld(mx, my, x, y);

//			geometry_msgs::Point p;
//			p.x = x;
//			p.y = y;
//			p.z = -0.05;

//			std_msgs::ColorRGBA color;
//			color.r = cost;
//			color.g = 0;
//			color.b = 1 - cost;
//			color.a = 0.95;

//			marker.points.push_back(p);
//			marker.colors.push_back(color);
//		}
//	}

//	pub_costmap_.publish(marker);
//}

