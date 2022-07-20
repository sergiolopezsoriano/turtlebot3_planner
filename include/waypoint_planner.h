#include <ros/ros.h>
#include <ros/package.h>
// #include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sqlite3.h>


class Waypoint_planner
{
private:
    ros::NodeHandle nh_;
    std::string path = ros::package::getPath("turtlebot3_planner");
    std::string output_file;
    std::string str = "/output/";
    std::string s;

    ros::Publisher marker_publisher_;
    visualization_msgs::MarkerArray marker_array;

    ros::Subscriber amcl_subs_;
    
    /* tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::PoseStamped odom_goal;
    geometry_msgs::TransformStamped transformStamped; */

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

    std::vector<std::string> trajectory_;
    std::vector<geometry_msgs::PoseStamped> waypoint_list_;
    ros::Time ts_;

    sqlite3* DB;
    int counter = 0;

public:
    Waypoint_planner(ros::NodeHandle* nodehandle);
    ~Waypoint_planner();
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped build_pose(ros::Time stamp, std::string frame_id, double x, double y, double z, double qx, double qy, double qz, double qw);
    void execute_trajectory();
    visualization_msgs::Marker build_marker(geometry_msgs::PoseStamped ps, int i);
    int create_db();
    int insert_db(std::string table);
};