#include "ur_planning/ur5_robot1_move_group.hpp"
#include <gazebo_msgs/SetModelState.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2

#define FREQUENCY 1000
#define BOX_INITIAL_POS_X -1.1
#define BOX_INITIAL_POS_Y 0.6
#define BOX_INITIAL_POS_Z 0.58
#define BOX_END_POX_X -0.1
#define PI 3.1415926536

static const std::string PLANNING_GROUP = "manipulator";
static const std::string ROBOT_DESCRIPTION = "ur5_robot1/robot_description";

static Eigen::Affine3d endEffector_in_box, world_in_base,  tool0_in_endEffector;

moveit::planning_interface::MoveGroupInterfacePtr ur5_robot1_group_ptr;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
robot_state::RobotStatePtr kinematic_state;
ros::Publisher gazebo_model_state_pub;

std::string currentBoxName;
geometry_msgs::Pose box_pose;

void jointStatesCallback(const sensor_msgs::JointState &joint_states_current)
{
    std::vector<double> joints_state;
    joints_state.push_back(joint_states_current.position[2]);
    joints_state.push_back(joint_states_current.position[1]);
    joints_state.push_back(joint_states_current.position[0]);
    joints_state.push_back(joint_states_current.position[3]);
    joints_state.push_back(joint_states_current.position[4]);
    joints_state.push_back(joint_states_current.position[5]);

    // calculate the pose of box picked by the robot while robot is moving
    kinematic_state->setJointGroupPositions("manipulator", joints_state);

    Eigen::Affine3d box_in_endEffector = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926);
    Eigen::Affine3d endEffector_in_tool0 = getTransform(0, 0, 0.05, 0, 0, 0);
    Eigen::Affine3d base_in_world = getTransform(0,0, 0.3, 0, 0, 0);
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

    gazebo_msgs::ModelState model_state;
    model_state.model_name = std::string(currentBoxName);
    model_state.pose = pose;
    model_state.reference_frame = std::string("world");
    gazebo_model_state_pub.publish(model_state);
}

int main(int argc, char **argv)
{ /**************************Initialize*************************************/
    ros::init(argc, argv, "ur5_robot1_planning");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    init();

    ros::Subscriber joint_states_sub;
    Eigen::Affine3d box_in_world;
    Eigen::Affine3d tool0_in_base;
    std::vector<double> robot_pos;

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, nh};
    ur5_robot1_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = ur5_robot1_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    kinematic_state = std::make_shared<robot_state::RobotState>(ur5_robot1_group_ptr->getRobotModel());
    ros::Rate poll_rate(FREQUENCY);

   

    gazebo_model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    while (gazebo_model_state_pub.getNumSubscribers() == 0)
    {
        poll_rate.sleep();
    }

    // moveBox(currentBoxName, BOX_INITIAL_POS_X, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0); //initical position of the box

    //////////////////////////////first time//////////////////////////////////////////////////////////

    for (int i = 1; i < 7; i++){
        moveBox(std::string("ur5_robot1_box") + std::to_string(i), BOX_INITIAL_POS_X, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0); //initical position of the box
    }

    addGround(planning_scene_interface);
    addplatform(planning_scene_interface);
    addbasis(planning_scene_interface);
  
    //pos 3
    currentBoxName = "ur5_robot1_box1";
    moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
    // before pick
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    robot_pos = {1.5556195421803336, -1.2402746918971053, 0.8718481924040615, -1.2018217454600704, -1.5710407537138575, -0.015712654397613157};
    moveRobotToJointValue(robot_pos);
    addShelf(planning_scene_interface); 
    // // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
  
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    // joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    
    // before place
    // box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = { 0.4343770321541536, -1.691978396175891, 2.6300749687077616, -0.9372566037408845, 2.004677378496499, 1.5705890517078824};
    moveRobotToJointValue(robot_pos);

    // place
    box_in_world = getTransform(0.65, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // // leave 
    box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

        //pos 4
    currentBoxName = "ur5_robot1_box2";
     moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
    // before pick
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    robot_pos = {1.5556195421803336, -1.2402746918971053, 0.8718481924040615, -1.2018217454600704, -1.5710407537138575, -0.015712654397613157};
    moveRobotToJointValue(robot_pos);
    // // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
  
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    // joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    
    // before place
    // box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = {-1.5224628481723261, -1.6939811695411597, 2.6321827705749214, -0.9468483343943142, 0.047622991433765804, 1.5803030713713841};
    moveRobotToJointValue(robot_pos);

    // place
    box_in_world = getTransform(0.65, -0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // // leave 
    box_in_world = getTransform(0.35, -0.175, 0.4, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);


    //pos 5
    currentBoxName = "ur5_robot1_box3";
     moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
    // before pick
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    robot_pos = {1.5556195421803336, -1.2402746918971053, 0.8718481924040615, -1.2018217454600704, -1.5710407537138575, -0.015712654397613157};
    moveRobotToJointValue(robot_pos);
    // // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
  
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    // joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    
    // before place
    // box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = { -1.6189231750904636, -1.2604615074407555, -2.535010332271052,  0.6665288700529892, 0.04851586917115558, -1.5832525194701912};
    moveRobotToJointValue(robot_pos);

    // place
    box_in_world = getTransform(0.65, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // // leave 
    box_in_world = getTransform(0.35, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);


    //pos 6
    currentBoxName = "ur5_robot1_box4";
     moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
    // before pick
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    robot_pos = {1.5556195421803336, -1.2402746918971053, 0.8718481924040615, -1.2018217454600704, -1.5710407537138575, -0.015712654397613157};
    moveRobotToJointValue(robot_pos);
    // // pick
    moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
  
    // up
    joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    // joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    
    // before place
    // box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    robot_pos = { -1.5228999120662596, -2.093779183393999, 2.1261672828295968, -0.039707742104378774, 0.04855911683073, 1.5783760408838559};
    moveRobotToJointValue(robot_pos);

    // place
    box_in_world = getTransform(0.65, -0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

    joint_states_sub.shutdown();

    // // leave 
    box_in_world = getTransform(0.35, -0.175, 0.65, 0, -3.1415926 / 2, 0);
    tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    moveRobotToPos(tool0_in_base);

   ////////////////////////////////////////////////second time////////////////////////////////////////////////////////////////////////////////

//     currentBoxName = "ur5_robot1_box2";
//     addpackage(planning_scene_interface, currentBoxName);
//     //initial position
//     robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
//     moveRobotToJointValue(robot_pos);
//     //move box2
//     moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
//      // before pick
//     moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
//     // pick
//     moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
//     // up
//     joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
//     moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
//    // before place
//     box_in_world = getTransform(0.35, -0.175, 0.15, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);
//     // place
//     box_in_world = getTransform(0.65, -0.175, 0.15, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);

//     joint_states_sub.shutdown();

//     // leave 
//     box_in_world = getTransform(0.35, -0.175, 0.15, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);

// ////////////////////////////////////////////////third time////////////////////////////////////////////////////////////////////////////////
//     currentBoxName = "ur5_robot1_box3";
//     addpackage(planning_scene_interface, currentBoxName);
//     //initial position
//     robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
//     moveRobotToJointValue(robot_pos);
//     //move box3
//     moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
//      // before pick
//     moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
//     // pick
//     moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
//     // up
//     joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
//     moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
//     // before place
//     box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);
//     // place
//     box_in_world = getTransform(0.65, 0.175, 0.4, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);

//     joint_states_sub.shutdown();

//     // leave 
//     box_in_world = getTransform(0.35, 0.175, 0.4, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);

// ////////////////////////////////////////////////fourth time////////////////////////////////////////////////////////////////////////////////
//     currentBoxName = "ur5_robot1_box4";
//     addpackage(planning_scene_interface, currentBoxName);
//     //initial position
//     robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
//     moveRobotToJointValue(robot_pos);
//     //move box3
//     moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
//      // before pick
//     moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
//     // pick
//     moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
//     // up
//     joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
//     moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
//     // before place
//     box_in_world = getTransform(0.35, -0.175, 0.4, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);
//     // place
//     box_in_world = getTransform(0.65, -0.175, 0.4, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);

//     joint_states_sub.shutdown();

//     // leave 
//     box_in_world = getTransform(0.35, -0.175, 0.4, 0, -3.1415926 / 2, 0);
//     tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
//     moveRobotToPos(tool0_in_base);
//     // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
//     // moveRobotToJointValue(robot_pos);
////////////////////////////////////////////////fifth time////////////////////////////////////////////////////////////////////////////////
    // currentBoxName = "ur5_robot1_box5";
    // //initial position
    // robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    // moveRobotToJointValue(robot_pos);
    // //move box3
    // moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
    
    //  // before pick
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // // arrive pick position
    // moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);

    // pick up the package
    // joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // // before place
    // box_in_world = getTransform(0.35, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    // // place
    // box_in_world = getTransform(0.65, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    // joint_states_sub.shutdown();

    // // leave 

    //box_in_world = getTransform(0.35, 0.175, 0.65, 0, -3.1415926 / 2, 0);
    //tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    //moveRobotToPos(tool0_in_base);

    // // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // // moveRobotToJointValue(robot_pos);
////////////////////////////////////////////////sixth time////////////////////////////////////////////////////////////////////////////////
    // currentBoxName = "ur5_robot1_box6";
    // // addpackage(planning_scene_interface, currentBoxName);
    // //initial position
    // robot_pos = {2.1638830636965913, -1.9281731851461448, 1.8554271998957468, -1.4975848537807899, -1.5702273917826703, 0.59340612948853};
    // moveRobotToJointValue(robot_pos);
    // //move box3
    // moveBoxBySpeed(planning_scene_interface, currentBoxName, BOX_INITIAL_POS_X, BOX_END_POX_X);
    //  // before pick
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // // pick
    // moveRobotToPos(-0.1, 0.6, 0.43, PI, 0, 0);
    // // up
    // joint_states_sub = nh.subscribe("/ur5_robot1/joint_states", 1000, jointStatesCallback);
    // moveRobotToPos(-0.1, 0.6, 0.55, PI, 0, 0);
    // // before place
    // // box_in_world = getTransform(0.35, -0.175, 0.65, 0, -3.1415926 / 2, 0);
    // // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // // moveRobotToPos(tool0_in_base);
    // // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // // moveRobotToJointValue(robot_pos);
    // robot_pos = {-1.5231460457739399, -2.0870857629666304, 2.122891851704818, -0.01648866417023065, 0.04831108374293436, 1.5518637437273393};
    // moveRobotToJointValue(robot_pos);
    // // place
    // box_in_world = getTransform(0.65, -0.175, 0.65, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);

    // joint_states_sub.shutdown();

    // // leave 弄远一点
    // box_in_world = getTransform(0.35, -0.175,0.65, 0, -3.1415926 / 2, 0);
    // tool0_in_base = world_in_base * box_in_world * endEffector_in_box * tool0_in_endEffector;
    // moveRobotToPos(tool0_in_base);
    // // robot_pos = {0.21425946885709557, -1.4303890087576585, 1.4612216939449594, -0.03146524694353836, 1.7850690314487814, 1.5706091041467118};
    // // moveRobotToJointValue(robot_pos);

    ros::Duration(10000).sleep();

    ros::shutdown();
    return 0;
}

void init(){
    endEffector_in_box = getTransform(0, 0, 0.1, 0, 3.1415926, 3.1415926).inverse();
    world_in_base = getTransform(0, 0, -0.3, 0, 0, 0);
    tool0_in_endEffector = getTransform(0, 0, -0.05, 0, 0, 0);
}

void moveBox(std::string box_name, double x, double y, double z, double qw, double qx, double qy, double qz)
{
    gazebo_msgs::ModelState model_state;
    model_state.model_name = box_name;
    model_state.reference_frame = std::string("world");
    model_state.pose.position.x = x; //start point -1.1 , end point -0.05
    model_state.pose.position.y = y;
    model_state.pose.position.z = z;

    model_state.pose.orientation.w = qw;
    model_state.pose.orientation.x = qx;
    model_state.pose.orientation.y = qy;
    model_state.pose.orientation.z = qz;
    gazebo_model_state_pub.publish(model_state);
}

void moveBoxBySpeed(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string box_name, double startPos_x, double endPos_x, double speed)
{
    double currentPos_x = startPos_x;
    ros::Rate poll_rate(FREQUENCY);
    while (ros::ok() && currentPos_x < endPos_x)
    {
        currentPos_x = currentPos_x + speed * 1.0 / FREQUENCY;
        // updatebox(planning_scene_interface,box_name);
        moveBox(box_name, currentPos_x, BOX_INITIAL_POS_Y, BOX_INITIAL_POS_Z, 1, 0, 0, 0);
        ros::spinOnce();
        poll_rate.sleep();
    }
}

void addplatform(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "platform1";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/platform.STL");
	shape_msgs::Mesh platform_mesh;
	shapes::ShapeMsg platform_mesh_msg;
	shapes::constructMsgFromShape(m,platform_mesh_msg);
	platform_mesh = boost::get<shape_msgs::Mesh>(platform_mesh_msg);

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose platform_pose;

    double rr, rp, ry;
    rr = -1.5707963;
    rp = -3.1415926;
    ry = 0.0;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
    platform_pose.orientation.x = Quaternion[0];
    platform_pose.orientation.y = Quaternion[1];
    platform_pose.orientation.z = Quaternion[2];
    platform_pose.orientation.w = Quaternion[3];    
	platform_pose.position.x = 0.2; // origin is 0.0
	platform_pose.position.y = 0.8; // origin is 0.75;
	platform_pose.position.z = 0 - 0.3;

	collision_object.meshes.push_back(platform_mesh);
	collision_object.mesh_poses.push_back(platform_pose);
	collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);

}

void addShelf(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "shelf";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/shelf.STL");
	shape_msgs::Mesh platform_mesh;
	shapes::ShapeMsg platform_mesh_msg;
	shapes::constructMsgFromShape(m,platform_mesh_msg);
	platform_mesh = boost::get<shape_msgs::Mesh>(platform_mesh_msg);

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose platform_pose;

    double rr, rp, ry;
    rr =1.5707963;
    rp =0;
    ry = 0.0;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
    platform_pose.orientation.x = Quaternion[0];
    platform_pose.orientation.y = Quaternion[1];
    platform_pose.orientation.z = Quaternion[2];
    platform_pose.orientation.w = Quaternion[3];    
	platform_pose.position.x = 0.55; 
	platform_pose.position.y = -0.375; 
	platform_pose.position.z = 0 - 0.3;

	collision_object.meshes.push_back(platform_mesh);
	collision_object.mesh_poses.push_back(platform_pose);
	collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);

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

	ground_pose.orientation.w = 0.7071;
	ground_pose.orientation.x = 0.7071;
	ground_pose.orientation.y = 0;
	ground_pose.orientation.z = 0;
	ground_pose.position.x = -4.0;
	ground_pose.position.y = 4.0;
	ground_pose.position.z = -0.05 - 0.3;

	collision_object.meshes.push_back(shelf_mesh);
	collision_object.mesh_poses.push_back(ground_pose);
	collision_object.operation = collision_object.ADD;


    planning_scene_interface.applyCollisionObject(collision_object);

}


void addbasis(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
	// put in collision avoidance data
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id = "world";

	/* The id of the object is used to identify it. */
	collision_object.id = "basis";

	shapes::Mesh* m = shapes::createMeshFromResource("package://process_visualizer/resources/arm_basis1.STL");
	shape_msgs::Mesh platform_mesh;
	shapes::ShapeMsg platform_mesh_msg;
	shapes::constructMsgFromShape(m,platform_mesh_msg);
	platform_mesh = boost::get<shape_msgs::Mesh>(platform_mesh_msg);

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose platform_pose;

    double rr, rp, ry;
    rr = 1.5707963;
    rp = 0;
    ry = 0.0;
    tf::Quaternion Quaternion = tf::createQuaternionFromRPY(rr, rp, ry);
    platform_pose.orientation.x = Quaternion[0];
    platform_pose.orientation.y = Quaternion[1];
    platform_pose.orientation.z = Quaternion[2];
    platform_pose.orientation.w = Quaternion[3];    
	platform_pose.position.x = -0.1; 
	platform_pose.position.y = -0.1; 
	platform_pose.position.z = - 0.3;

	collision_object.meshes.push_back(platform_mesh);
	collision_object.mesh_poses.push_back(platform_pose);
	collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);

}



void moveRobotToJointValue(const std::vector<double> &pos)
{
    ur5_robot1_group_ptr->setJointValueTarget(pos);
    bool success = (ur5_robot1_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur5_robot1_group_ptr->move();
}

void moveRobotToPos(const geometry_msgs::Pose &pos)
{
    ur5_robot1_group_ptr->setPoseTarget(pos);
    bool success = (ur5_robot1_group_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ur5_robot1_group_ptr->move();
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
    double fraction = ur5_robot1_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ur5_robot1_group_ptr->execute(trajectory);
    waypoints.clear();
}

