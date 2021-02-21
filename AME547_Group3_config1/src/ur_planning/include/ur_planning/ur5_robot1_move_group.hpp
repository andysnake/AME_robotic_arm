#ifndef UR5_ROBOT1_MOVE_GROUP
#define UR5_ROBOT1_MOVE_GROUP

#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

void moveBox(std::string box_name, double x, double y, double z, double qw, double qx, double qy, double qz);

void moveBoxBySpeed(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string box_name, double startPos_x, double endPos_x, double speed = 0.4);

void moveRobotToJointValue(const std::vector<double> &pos);

void moveRobotToPos(const geometry_msgs::Pose &pos);

void moveRobotToPos(const Eigen::Affine3d& pos);

void moveRobotToPos(double x, double y, double z, double rr, double rp, double ry);

void init();

Eigen::Affine3d getTransform(double x, double y, double z, double roll, double pitch, double yaw);



class RobotTrajectory
{
public:
    void addPoint(double x, double y, double z, double rr, double rp, double ry)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
        pose.orientation.x = Quaternion[0];
        pose.orientation.y = Quaternion[1];
        pose.orientation.z = Quaternion[2];
        pose.orientation.w = Quaternion[3];
        waypoints.push_back(pose);
    }

    void  move();
  

private:
    std::vector<geometry_msgs::Pose> waypoints;
};

    void addplatform(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void addShelf(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void addGround(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);  
    void addbasis(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    void addpackage(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string package_name);
    void updatebox(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string package_name);
    void updateShelf(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

#endif