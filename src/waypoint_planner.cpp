#include <waypoint_planner.h>

Waypoint_planner::Waypoint_planner(ros::NodeHandle* nodehandle):nh_(*nodehandle), ac("move_base", true)
{
    nh_.getParam("/waypoint_planner/output_file", output_file);
    s = path.append(str.append(output_file));

    ac.waitForServer();

    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    amcl_subs_ = nh_.subscribe("amcl_pose", 100, &Waypoint_planner::amcl_callback, this);
    marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
    std::string frame_id;
    double x, y, z, qx, qy, qz, qw;
    nh_.getParam("/waypoint_planner/trajectory", trajectory_);
    for (int i=0; i < trajectory_.size(); i++)
    {
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/frame_id", frame_id);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/x", x);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/y", y);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/z", z);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/qx", qx);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/qy", qy);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/qz", qz);
        nh_.getParam("/waypoint_planner/waypoint_list/" + trajectory_[i] + "/qw", qw);
        waypoint_list_.push_back(build_pose(ts_.now(), frame_id, x, y, z, qx, qy, qz, qw));
        marker_array.markers.push_back(build_marker(waypoint_list_[i], i));
    }
    sleep(1);
    marker_publisher_.publish(marker_array);
}

Waypoint_planner::~Waypoint_planner()
{

}

geometry_msgs::PoseStamped Waypoint_planner::build_pose(ros::Time stamp, std::string frame_id, double x, double y, double z, double qx, double qy, double qz, double qw)
{
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = frame_id;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
    ps.pose.orientation.w = qw;
    ps.pose.orientation.x = qx;
    ps.pose.orientation.y = qy;
    ps.pose.orientation.z = qz;
    return ps;
}

visualization_msgs::Marker Waypoint_planner::build_marker(geometry_msgs::PoseStamped ps, int i){
    visualization_msgs::Marker mk;
    mk.header = ps.header;
    mk.ns = "";
    mk.id = i;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose = ps.pose;
    mk.scale.x = 0.3;
    mk.scale.y = 0.3;
    mk.scale.z = 0.3;
    mk.color.a = 0.8;
    mk.color.r = 0;
    mk.color.g = 1;
    mk.color.b = 0;
    mk.lifetime = ros::Duration(0);
    mk.frame_locked = false;
    return mk;
}

void Waypoint_planner::execute_trajectory()
{
    sleep(1);
    for (int wp = 0; wp < waypoint_list_.size(); wp++)
    {   
        /* try{
            // tf_buffer_.transform(waypoint_list_[wp], odom_goal, "odom");
            transformStamped = tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        } */
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = waypoint_list_[wp];
        ROS_INFO("Sending goal!");
        ac.sendGoalAndWait(goal);
        sleep(1);
    }
}

void Waypoint_planner::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg) {
    // ROS_INFO_STREAM("Received pose: " << amcl_msg_->pose);
    int exit = 0;
    exit = sqlite3_open(s.data(), &DB);
    if (exit) {
        std::cerr << "Error open DB " << sqlite3_errmsg(DB) << std::endl;
    }
    char* messaggeError;
    char sql[512];
    sprintf(sql,"insert into amcl_poses values(%f,'%f',%f);",amcl_msg->pose.pose.position.x, amcl_msg->pose.pose.position.y, amcl_msg->header.stamp.toSec());
    exit = sqlite3_exec(DB, sql, NULL, 0, &messaggeError);
    sqlite3_close(DB);
}

int Waypoint_planner::create_db(){
    int exit = 0;
    exit = sqlite3_open(s.data(), &DB);
    if (exit) {
        std::cerr << "Error open DB " << sqlite3_errmsg(DB) << std::endl;
        return(-1);
    }
    std::string sql = "CREATE TABLE amcl_poses("
                        "x REAL NOT NULL, "
                        "y REAL NOT NULL, "
                        "stamp REAL NOT NULL);";
    char* messaggeError;
    try{
        exit = sqlite3_exec(DB, sql.c_str(), NULL, 0, &messaggeError);
    }
    catch(...){
        if( exit != SQLITE_OK ){ // redundant
            fprintf(stderr, "SQL error: %s\n", messaggeError);
            sqlite3_free(messaggeError);
            return(-1);
        }
    }
    sqlite3_close(DB);
    return(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_planner");
    ros::NodeHandle nh;
    Waypoint_planner wp(&nh);
    wp.create_db();
    // wp.execute_trajectory();
    ros::spin();
    return(0);
}