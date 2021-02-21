#include "ur_planning/ur10_robot2_move_group.hpp"
#include <gazebo_msgs/SetModelState.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#define FREQUENCY 1000
#define BOX_INITIAL_POS_X 2.65
#define BOX_INITIAL_POS_Y -0.6
#define BOX_INITIAL_POS_Z 0.58
#define BOX_END_POX_X 2.05
#define PI 3.1415926536

static const std::string PLANNING_GROUP = "manipulator";
static const std::string ROBOT_DESCRIPTION = "ur10_robot2/robot_description";

static Eigen::Affine3d endEffector_in_box, world_in_base,  tool0_in_endEffector;

moveit::planning_interface::MoveGroupInterfacePtr ur10_robot2_group_ptr;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
robot_state::RobotStatePtr kinematic_state;
ros::Publisher gazebo_model_state_pub;

std::string currentBoxName;

double BEFORE_PICK_POS_Z = 0.5;
double PICK_POS_Z = 0.23;

void jointStatesCallback(const sensor_msgs::JointState &joint_states_current)
{
    std::vector<double> joints_state;
    joints_state.push_back(joint_states_current.position[2]);
    joints_state.push_back(joint_states_current.position[1]);
    joints_state.push_back(joint_states_current.position[0]);
    joints_state.push_back(joint_states_current.position[3]);
    joints_state.push_back(joint_states_current.position[4]);
    joints_state.push_back(joint_states_current.position[5]);

    kinematic_state->setJointGroupPositions("manipulator", joints_state);

    // calculate box picked by the robot while robot is moving
    Eigen::Affine3d box_in_endEffector = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926);
    Eigen::Affine3d endEffector_in_tool0 = getTransform(0, 0, 0.05, 0, 0, 0);
    Eigen::Affine3d base_in_world = getTransform(1.95, 0.4, 0.5, 0, 0, 0);
    Eigen::Affine3d tool0_in_base = kinematic_state->getGlobalLinkTransform("tool0");

    Eigen::Affine3d box_in_world = base_in_world * tool0_in_base * endEffector_in_tool0 * box_in_endEffector;

    geometry_msgs::Pose pose;
    pose.position.x = box_in_world.translation().x();
    pose.position.y = box_in_world.translation().y();
    pose.position.z = box_in_world.translation().z();

    Eigen::Quaterniond quat(box_in_world.rotation());
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    // ros::Duration(100).sleep();
    gazebo_msgs::ModelState model_state;
    model_state.model_name = std::string(currentBoxName);
    model_state.pose = pose;
    model_state.reference_frame = std::string("world");
    gazebo_model_state_pub.publish(model_state);
}

int main(int argc, char **argv)
{ /**************************Initialize*************************************/
    ros::init(argc, argv, "ur10_robot2_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    init();

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, nh};
    ur10_robot2_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = ur10_robot2_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    kinematic_state = std::make_shared<robot_state::RobotState>(ur10_robot2_group_ptr->getRobotModel());
    ros::Rate poll_rate(FREQUENCY);

    //import real environment into RViz
    addGround(planning_scene_interface);
    // addplatform(planning_scene_interface);
    addbasis(planning_scene_interface);
    // addShelf(planning_scene_interface);

    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    while (gazebo_model_state_pub.getNumSubscribers() == 0)
    {
        poll_rate.sleep();
    }

    currentBoxName = "ur10_robot2_box1";

    moveBox(currentBoxName, BOX_INITIAL_POS_X, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0); //initical position of the box
    std::vector<double> robot_pos;
    // initical position of the robot
    // std::vector<double> robot_pos = {2.1638830636965913, -1.9281731851461448, -1.6153735611638558, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    // moveRobotToJointValue(robot_pos);
    //addShelf(planning_scene_interface);
    /////////////////////////////////box moves on the platform//////////////////////////////////////
    ////  set up parameters /////
    Eigen::Affine3d box_in_world;
    Eigen::Affine3d tool0_in_base;
    ros::Subscriber joint_states_sub;

        for (int i = 1; i < 7; i++){
        moveBox(std::string("ur10_robot2_box") + std::to_string(i), BOX_INITIAL_POS_X, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0); //initical position of the box
    }

    ///////////////////////////////////first time/////////////////////////////////////////////////////////////
// //    initical position of the robot
//     moveRobotToPos(0.1, -0.5, BEFORE_PICK_POS_Z, 0, PI, 0);

//     //move box to the end position of the platform
//     moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);

//     RobotTrajectory  robotTrajectory;
//     // before pick
//     moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
//     // pick
//     moveRobotToPos(0.1, -1, PICK_POS_Z, 0, PI, 0);
//     // up
//     joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);
//     moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
    
//     // before place
//     box_in_world = getTransform(1.2, -0.175, 0.9, 0, -3.1415926 / 2, 3.1415926);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);
//     // robot_pos = {-2.487008245541916, -1.2187048534973908, 1.5461895225694722, -0.7884351941775147, 1.241023429027373, -0.3038670107777097};
//     // moveRobotToJointValue(robot_pos);    
//     // place
//     box_in_world = getTransform(0.9, -0.175, 0.9, 0, -3.1415926 / 2, 3.1415926);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);

//     joint_states_sub.shutdown();

//     // leave 
//     box_in_world = getTransform(1.2, -0.175, 0.9, 0, -3.1415926 / 2, 3.1415926);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);

// //////////////////////////////////////////////second time////////////////////////////////////////////////////////////////////////////////

//     currentBoxName = "ur10_robot2_box2";
//     //initical position of the robot
//     moveRobotToPos(0.1, -0.5, BEFORE_PICK_POS_Z, 0, PI, 0);
//     //move box2
//     moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
//    // before pick
//     moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
//     // pick
//     moveRobotToPos(0.1, -1, PICK_POS_Z, 0, PI, 0);
//     // up
//     joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);
//     moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
 
//    // before place
//     box_in_world = getTransform(1.2, 0.175, 0.9, 0, -3.1415926 / 2, 3.1415926);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);
//     // robot_pos = {1.5099995058767846, -2.0199121402158893, -1.4519386586547007, 0.5313963337110366, 0.9550683153304149, -1.3092096436065166};
//     // moveRobotToJointValue(robot_pos);
//     // place
//     box_in_world = getTransform(0.9, 0.175, 0.9, 0, -3.1415926 / 2, 3.1415926);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);

//     joint_states_sub.shutdown();

//     // leave 弄远一点
//     box_in_world = getTransform(1.2, 0.175, 0.9, 0, -3.1415926 / 2, 3.1415926);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);

// ////////////////////////////////////////////////third time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur10_robot2_box3";
    // initical position of the robot
    robot_pos = {-1.7005314225343229, -1.8922720230110004, 2.096122484738439, -1.775551720279946, -1.5716035325502986, -0.13021947796781497};
    moveRobotToJointValue(robot_pos);     
    // moveRobotToPos(0.1, -0.5, BEFORE_PICK_POS_Z, 0, PI, 0);
    //move box2
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
   // before pick
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
    // pick
    moveRobotToPos(0.1, -1, PICK_POS_Z, 0, PI, 0);
    // up
    joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
 
   // before place
    // box_in_world = getTransform(1.2, 0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = {0.7168111519229043, -1.564542791533908, -1.9432369129632692, 0.36550882193140843, 0.85430588439531, -1.5694882472315728};
    moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.9, 0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 
    box_in_world = getTransform(1.2, 0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

// ////////////////////////////////////////////////fourth time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur10_robot2_box4";
    // initical position of the robot
    robot_pos = {-1.7005314225343229, -1.8922720230110004, 2.096122484738439, -1.775551720279946, -1.5716035325502986, -0.13021947796781497};
    moveRobotToJointValue(robot_pos);     
    // moveRobotToPos(0.1, -0.5, BEFORE_PICK_POS_Z, 0, PI, 0);
    //move box2
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
   // before pick
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
    // pick
    moveRobotToPos(0.1, -1, PICK_POS_Z, 0, PI, 0);
    // up
    joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
 
   // before place
    // box_in_world = getTransform(1.2, 0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = { 1.062749423023451, -1.908648664752521, -1.5352541177952919, 0.3004292127717907, 0.5084236918783125, -1.569243454954865};
    moveRobotToJointValue(robot_pos);
    //place
    box_in_world = getTransform(0.9, -0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 
    box_in_world = getTransform(1.2, -0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
////////////////////////////////////////////////fifth time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur10_robot2_box5";
    // initical position of the robot
    robot_pos = {-1.7005314225343229, -1.8922720230110004, 2.096122484738439, -1.775551720279946, -1.5716035325502986, -0.13021947796781497};
    moveRobotToJointValue(robot_pos);     
    // moveRobotToPos(0.1, -0.5, BEFORE_PICK_POS_Z, 0, PI, 0);
    //move box2
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
   // before pick
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
    // pick
    moveRobotToPos(0.1, -1, PICK_POS_Z, 0, PI, 0);
    // up
    joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
 
   // before place
    // box_in_world = getTransform(1.2, 0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = {-3.0239909849126425, -0.14694709936541717, -1.5547965196001563, -1.439191930779213, -1.6885940494328135, -1.5716181703132168};
    moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.9, 0.175, 1.4, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 
    box_in_world = getTransform(1.2, 0.175, 1.4, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);
    
////////////////////////////////////////////////sixth time////////////////////////////////////////////////////////////////////////////////
    currentBoxName = "ur10_robot2_box6";
    // initical position of the robot
    robot_pos = {-1.7005314225343229, -1.8922720230110004, 2.096122484738439, -1.775551720279946, -1.5716035325502986, -0.13021947796781497};
    moveRobotToJointValue(robot_pos);     
    // moveRobotToPos(0.1, -0.5, BEFORE_PICK_POS_Z, 0, PI, 0);
    //move box2
    moveBoxBySpeed(currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
   // before pick
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
    // pick
    moveRobotToPos(0.1, -1, PICK_POS_Z, 0, PI, 0);
    // up
    joint_states_sub = nh.subscribe("/ur10_robot2/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(0.1, -1, BEFORE_PICK_POS_Z, 0, PI, 0);
 
   // before place
    // box_in_world = getTransform(1.2, 0.175, 1.15, 0, -3.1415926 / 2, 3.1415926);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = {-2.5095953723742097, -1.269376977904237, 1.142268725886364, -3.01376200160625, -2.2032285275788155, -1.5703027981177051};
    moveRobotToJointValue(robot_pos);
    // place
    box_in_world = getTransform(0.9, -0.175, 1.4, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // leave 
    box_in_world = getTransform(1.2, -0.175, 1.4, 0, -3.1415926 / 2, 3.1415926);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    ros::Duration(10000).sleep();

    ros::shutdown();
    return 0;
}

void init(){
    endEffector_in_box = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926).inverse();
    world_in_base = getTransform(-1.95, -0.4, -0.5, 0, 0, 0);
    tool0_in_endEffector = getTransform(0, 0, -0.05, 0, 0, 0);
}
void moveBox(std::string box_name, double x, double y, double z, double qw, double qx, double qy, double qz)
{
    gazebo_msgs::ModelState model_state;
    model_state.model_name = box_name;
    model_state.reference_frame = std::string("world");
    model_state.pose.position.x = x; //start point  , end point 
    model_state.pose.position.y = y;
    model_state.pose.position.z = z;

    model_state.pose.orientation.w = qw;
    model_state.pose.orientation.x = qx;
    model_state.pose.orientation.y = qy;
    model_state.pose.orientation.z = qz;
    gazebo_model_state_pub.publish(model_state);
}

void moveBoxBySpeed(std::string box_name, double startPos_x, double endPos_x, double speed)
{
    double currentPos_x = startPos_x;
    ros::Rate poll_rate(FREQUENCY);
    while (ros::ok() && currentPos_x > endPos_x)
    {
        currentPos_x = currentPos_x + speed * 1.0 / FREQUENCY;
        moveBox(box_name, currentPos_x, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0);
        ros::spinOnce();
        poll_rate.sleep();
    }
}

void moveRobotToJointValue(const std::vector<double> &pos)
{
    ur10_robot2_group_ptr->setJointValueTarget(pos);
    bool success = (ur10_robot2_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur10_robot2_group_ptr->move();
}

void moveRobotToPos(const geometry_msgs::Pose &pos)
{
    ur10_robot2_group_ptr->setPoseTarget(pos);
    bool success = (ur10_robot2_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur10_robot2_group_ptr->move();
}

void moveRobotToPos(const Eigen::Affine3d& pos){
    geometry_msgs::Pose pose;
    pose.position.x = pos.translation().x();
    pose.position.y = pos.translation().y();
    pose.position.z = pos.translation().z();

    Eigen::Quaterniond quat(pos.rotation());
    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    moveRobotToPos(pose);
}

void moveRobotToPos(double x, double y, double z, double rr, double rp, double ry){
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
    pose.orientation.x = Quaternion[0];
    pose.orientation.y = Quaternion[1];
    pose.orientation.z = Quaternion[2];
    pose.orientation.w = Quaternion[3];
    moveRobotToPos(pose);
}

Eigen::Affine3d getTransform(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Affine3d trans;
    trans.matrix()(0, 3) = x;
    trans.matrix()(1, 3) = y;
    trans.matrix()(2, 3) = z;

    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    trans.matrix().block<3, 3>(0, 0) = q.matrix();
    return trans;
}

void RobotTrajectory::move(){
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = ur10_robot2_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ur10_robot2_group_ptr->execute(trajectory);
    waypoints.clear();
}

void addplatform(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "platform";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/platform.STL");
	shape_msgs::Mesh platform_mesh;
	shapes::ShapeMsg platform_mesh_msg;
	shapes::constructMsgFromShape(m,platform_mesh_msg);
	platform_mesh = boost::get<shape_msgs::Mesh>(platform_mesh_msg);

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose platform_pose;
	// platform_pose.orientation.w = 0;
	// platform_pose.orientation.x = 0;
	// platform_pose.orientation.y = -0.7071;
	// platform_pose.orientation.z = -0.7071;
	// platform_pose.position.x = 0.5; // origin is 0.0
	// platform_pose.position.y = 1.15; // origin is 0.75;
	// platform_pose.position.z = 0 - 0.3;
    double rr, rp, ry;
    rr = 1.5707963;
    rp = 0.0;
    ry = 0.0;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
    platform_pose.orientation.x = Quaternion[0];
    platform_pose.orientation.y = Quaternion[1];
    platform_pose.orientation.z = Quaternion[2];
    platform_pose.orientation.w = Quaternion[3];    
	platform_pose.position.x = 1.35; // origin is 0.0
	platform_pose.position.y = -0.8; // origin is 0.75;
	platform_pose.position.z = 0 - 0.5;

	collision_object.meshes.push_back(platform_mesh);
	collision_object.mesh_poses.push_back(platform_pose);
	collision_object.operation = collision_object.ADD;
  // color
    std_msgs::ColorRGBA color;
    color.r = 1;
    color.g = 0.55;
    color.b = 0;
    color.a = 1;

    planning_scene_interface.applyCollisionObject(collision_object,color);

	ROS_INFO("Add a platform into the world");
}

void addShelf(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "shelf";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/shelf.STL");
	shape_msgs::Mesh shelf_mesh;
	shapes::ShapeMsg shelf_mesh_msg;
	shapes::constructMsgFromShape(m,shelf_mesh_msg);
	shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

    double rr, rp, ry;
    rr = 1.5707963;
    rp = 0.0;
    ry = 0.0;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
	geometry_msgs::Pose shelf_pose;
    shelf_pose.orientation.x = Quaternion[0];
    shelf_pose.orientation.y = Quaternion[1];
    shelf_pose.orientation.z = Quaternion[2];
    shelf_pose.orientation.w = Quaternion[3];
	shelf_pose.position.x = 0.55;//0.55;
	shelf_pose.position.y = -0.375;
	shelf_pose.position.z = 0 - 0.5;

	collision_object.meshes.push_back(shelf_mesh);
	collision_object.mesh_poses.push_back(shelf_pose);
	collision_object.operation = collision_object.ADD;
  // color
    std_msgs::ColorRGBA color;
	 
    color.r = 0.87;
    color.g = 0.72;
    color.b = 0.53;
    color.a = 1;

    planning_scene_interface.applyCollisionObject(collision_object,color);

	ROS_INFO("Add a shelf into the world");
}

void addGround(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "ground_surface";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/plate.STL");
	shape_msgs::Mesh shelf_mesh;
	shapes::ShapeMsg shelf_mesh_msg;
	shapes::constructMsgFromShape(m,shelf_mesh_msg);
	shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

	/* A pose for the box (specified  relative to frame_id) */
	geometry_msgs::Pose ground_pose;
    // double rr, rp, ry;
    // rr = -1.5707963;
    // rp = -3.1415926;
    // ry = 0;
	ground_pose.orientation.w = 0.7071;
	ground_pose.orientation.x = 0.7071;
	ground_pose.orientation.y = 0;
	ground_pose.orientation.z = 0;
	ground_pose.position.x = -4.0;
	ground_pose.position.y = 4.0;
	ground_pose.position.z = -0.05 - 0.5;

	collision_object.meshes.push_back(shelf_mesh);
	collision_object.mesh_poses.push_back(ground_pose);
	collision_object.operation = collision_object.ADD;
  // color
    std_msgs::ColorRGBA color;
	 
    color.r = 0.87;
    color.g = 0.72;
    color.b = 0.53;
    color.a = 1;

    planning_scene_interface.applyCollisionObject(collision_object,color);

	ROS_INFO("Add a shelf into the world");
}

void addbasis(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "basis2";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/arm_basis2.STL");
	shape_msgs::Mesh shelf_mesh;
	shapes::ShapeMsg shelf_mesh_msg;
	shapes::constructMsgFromShape(m,shelf_mesh_msg);
	shelf_mesh = boost::get<shape_msgs::Mesh>(shelf_mesh_msg);

    double rr, rp, ry;
    rr = 1.5707963;
    rp = 0.0;
    ry = 0.0;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
	geometry_msgs::Pose basis_pose;
    basis_pose.orientation.x = Quaternion[0];
    basis_pose.orientation.y = Quaternion[1];
    basis_pose.orientation.z = Quaternion[2];
    basis_pose.orientation.w = Quaternion[3];
	basis_pose.position.x = 1.85;//0.55;
	basis_pose.position.y = 0.3;
	basis_pose.position.z = 0 - 0.5;

	collision_object.meshes.push_back(shelf_mesh);
	collision_object.mesh_poses.push_back(basis_pose);
	collision_object.operation = collision_object.ADD;
  // color
    std_msgs::ColorRGBA color;
	 
    color.r = 0.87;
    color.g = 0.72;
    color.b = 0.53;
    color.a = 1;

    planning_scene_interface.applyCollisionObject(collision_object,color);

	ROS_INFO("Add a shelf into the world");
}

