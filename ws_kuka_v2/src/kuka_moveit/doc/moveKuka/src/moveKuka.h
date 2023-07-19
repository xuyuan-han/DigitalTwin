//All the necessary header documents of moveit to realize plan function
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

//All the necessary header documents of moveit to realize workpiece-interaction function
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//All the necessary header documents of moveit to realize visualzation function
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//All the necessary header documents to include .stl documents
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

//All the necessary header documents of to realize pause function
#include <termios.h>
#include <unistd.h>

#include <iostream>

namespace rvt = rviz_visual_tools;

//Creat the class moveKuka
class moveKuka {

private:
    //Creat a name PLANNING_GROUP
    static const std::string PLANNING_GROUP;

    //Creat the required objects
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_visual_tools::MoveItVisualTools visual_tools;

    Eigen::Isometry3d text_pose;
public:
    moveKuka();
    ~moveKuka();
private:
    void MoveTo(double pos_x, double pos_y, double pos_z, 
        double ori_x, double ori_y, double ori_z, double ori_w);
    void Attach(const std::string& ObjectID);
    void Detach(const std::string& ObjectID);
    void Remove(std::vector<std::string>& object_ids);
    void AddStl(const std::string& stlFile, double pos_x, double pos_y, double pos_z);
    
    int Menu();
    void Scene1();
    void Scene2();
    void Scene3();

    static int WaitKey();
public:
    void Start();
};

//set the grouped joints of KUKA robot as planning_group
const std::string moveKuka::PLANNING_GROUP = "robot_arm";

//The constructor
moveKuka::moveKuka() : move_group(PLANNING_GROUP), visual_tools("base_link")
{    
    //Init step
    visual_tools.deleteAllMarkers();
    //Load remote-control tool that allows users to control program in RViz
    visual_tools.loadRemoteControl();
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "moveKuka", rvt::WHITE, rvt::XLARGE);
    
    //Use batch-publishing-tool to reduce the number of messages being sent to RViz
    visual_tools.trigger();
    //Print the name of the reference frame for this robot
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    //Print a list of all the groups in the robot
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
    std::cout << std::endl;
}

//The destructor
moveKuka::~moveKuka()
{
    ros::shutdown();
}

//Function: pause
int moveKuka::WaitKey()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

    return ch;
}

//Function: add a object from .stl documents
void moveKuka::AddStl(const std::string& stlFile, double pos_x, double pos_y, double pos_z) 
{
    //Define a vector of all the objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    //Define a collision object ROS message
    moveit_msgs::CollisionObject collision_stl;
    collision_stl.header.frame_id = move_group.getPlanningFrame();

    //Creat a id of the object to identify it 
    collision_stl.id = stlFile;

    //Define a pointer to creat the object in RViz
    shapes::Mesh* m = shapes::createMeshFromResource(stlFile);
    shape_msgs::Mesh stl_mesh;
    shapes::ShapeMsg stl_mesh_msg;

    //Use pointer to creat msg
    shapes::constructMsgFromShape(m,stl_mesh_msg);
    stl_mesh = boost::get<shape_msgs::Mesh>(stl_mesh_msg);

    //Define a pose for the object 
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.orientation.x = 0.0;  
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.position.x = pos_x;
    box_pose.position.y = pos_y;
    box_pose.position.z = pos_z;

    collision_stl.meshes.push_back(stl_mesh);
    collision_stl.mesh_poses.push_back(box_pose);
    collision_stl.operation = collision_stl.ADD;

    collision_objects.push_back(collision_stl);

    //Add the new created object into the world
    planning_scene_interface.addCollisionObjects(collision_objects);

    //Show text in RViz of status
    visual_tools.publishText(text_pose, "Add a object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
}

//Function: design, plan, move of the "robot_arm" with collision avoidance and visualization in RViz
void moveKuka::MoveTo(double pos_x, double pos_y, double pos_z, 
        double ori_x, double ori_y, double ori_z, double ori_w)
{
    //Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //Get the current state and set it as start state
    move_group.setStartState(*move_group.getCurrentState());

    //Creat and design the final pose for the end-effector of the group "robot_arm"
    geometry_msgs::Pose object_pose;
    object_pose.position.x = pos_x;
    object_pose.position.y = pos_y;
    object_pose.position.z = pos_z;
    object_pose.orientation.x = ori_x;
    object_pose.orientation.y = ori_y;
    object_pose.orientation.z = ori_z;
    object_pose.orientation.w = ori_w;
    //Set the designed pose as goal_pose
    move_group.setPoseTarget(object_pose);

    //Bulid a variable to judge if the plan succeed with the help of MoveItErrorCode tool
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //Print the result in the RViz
    ROS_INFO_NAMED("tutorial", "move robot to the object %s", success ? "" : "FAILED");
    
    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);

    const robot_state::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    
    //Move the robot in RViz
    move_group.move();
}

//Function: attach the object
void moveKuka::Attach(const std::string& ObjectID)
{
    ROS_INFO_NAMED("tutorial","Attach the object to the robot");
    
    //Attach the object to the KUKA robot
    move_group.attachObject(ObjectID);
    
    //Show text in RViz of status
    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
}

//Function: detach the object
void moveKuka::Detach(const std::string& ObjectID)
{
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    
    //Detach the object to the KUKA robot
    move_group.detachObject(ObjectID);

    //Show text in RViz of status
    visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
}

//Fuction: remove all the boxes
void moveKuka::Remove(std::vector<std::string>& object_ids)
{
    //Remove all the boxes that added into the world
    planning_scene_interface.removeCollisionObjects(object_ids);
}

//Function: start the simulation
void moveKuka::Start()
{
    //Start the menu function and loop until the user want to stop the simulation
    while (Menu());   
}

//Function: show the choice menu
int moveKuka::Menu()
{
    //clear the screen
    system("clear");
   
    //Print the choice menu for user to choose
    puts("[1] scene1 (add a box as target object)");
    puts("[2] scene2 (add a box as target object, add a lamp and a cuboid as collison objects)");
    puts("[3] scene3 (add a cylinder as target object, add a lamp and a cuboid as collison objects)");
    puts("[0] finish");
    puts("\nPlease choose:");

    //User inputs choice
    int choice = 0;
    std::cin >> choice;

    //According to the entered choice deciding the scene
    //When the choice is "0", close the function
    switch (choice) {
    case 1:
        Scene1();
        break;
    case 2:
        Scene2();
        break;
    case 3:
        Scene3();
        break;
    case 0:
        return 0;
        break;
    default:
        std::cout << "please try again" << std::endl;
        break;
    }
    
    return 1;
}

//Fuction: scene1
void moveKuka::Scene1()
{
    //Clear the screen
    system("clear");

    //Include the corresponding .stl document
    const std::string boxFile = "package://kuka_moveit/model/stl/box_xyz_0.085_0.19_0.1m.STL";

    //Show text in RViz of status
    std::cout << "-----------------------" << std::endl;
    std::cout << "[First Step] Add Box." << std::endl;

    //Add the workpiece
    AddStl(boxFile, 0.35 - 0.085 / 2, -0.25 - 0.19 / 2, 1.534 - 0.1 / 2); 
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Second Step] Move to first place." << std::endl;
    
    //Move the KUKA robot to the target location
    MoveTo(0.358829, -0.274889, 1.73522, -0.654404, 0.145868, 0.697154, 0.253879);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Third Step] Attach the object." << std::endl;
    
    //Attach the workpiece to KUKA robot
    Attach(boxFile);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Fourth Step] Move to second place." << std::endl;

    //Move the KUKA robot and workpiece to the target location
    MoveTo(0.846218, -0.990494, 1.45394, 0.429289, 0.555199, -0.518724, 0.488253);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Fifth Step] Detach the Object." << std::endl;
    
    //Detach the piece from KUKA robot
    Detach(boxFile);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Last Step] Remove the Object." << std::endl;
    
    //Remove the object
    std::vector<std::string> object_ids;
    object_ids.push_back(boxFile);
    Remove(object_ids);

    //Wait for next step
    std::cout << "Press any key to go back to the menu..." <<std::endl;
    WaitKey();
}

//Fuction: scene2
void moveKuka::Scene2()
{
    //Clear the screen
    system("clear");

    //Include the all the corresponding .stl documents
    const std::string lampFile = "package://kuka_moveit/model/stl/ceiling_lamp_xyz_1_0.03_0.02m.STL";
    const std::string boxFile = "package://kuka_moveit/model/stl/box_xyz_0.085_0.19_0.1m.STL";
    const std::string collisionFile = "package://kuka_moveit/model/stl/collision_xyz_0.5_0.6_1.5m.STL";

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[First Step] Add Objects." << std::endl;

    //Add the workpiece and collisions step by step
    std::cout << "add a lamp" << std::endl;
    AddStl(lampFile, 0.35, -0.25, 1.886);
    WaitKey();
    WaitKey();
    std::cout << "add a box" << std::endl;
    AddStl(boxFile, 0.35 - 0.085 / 2, -0.25 - 0.19 / 2, 1.534 - 0.1 / 2);
    WaitKey();
    std::cout << "add a cuboid" << std::endl;
    AddStl(collisionFile, 0.01, -1.215, 0);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Second Step] Move to first place." << std::endl;

    //Move the KUKA robot to the target location
    MoveTo(0.358829, -0.274889, 1.73522, -0.654404, 0.145868, 0.697154, 0.253879);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Third Step] Attach the object." << std::endl;

    //Attach the workpiece to KUKA robot
    Attach(boxFile);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Fourth Step] Move to second place." << std::endl;
    
    //Move the KUKA robot and workpiece to the target location
    MoveTo(0.846218, -0.990494, 1.45394, 0.429289, 0.555199, -0.518724, 0.488253);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Fifth Step] Detach the Object." << std::endl;
    
    //Detach the piece from KUKA robot
    Detach(boxFile);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Last Step] Remove the Object." << std::endl;
    
    //Remove the all the objects
    std::vector<std::string> object_ids;
    object_ids.push_back(boxFile);
    object_ids.push_back(collisionFile);
    object_ids.push_back(lampFile);
    Remove(object_ids);
    
    //Wait for next step
    std::cout << "Press any key to go back to the menu..." <<std::endl;
    WaitKey();
}

//Fuction: scene3
void moveKuka::Scene3()
{   
    //Clear the screen
    system("clear");

    //Include the all the corresponding .stl documents
    const std::string lampFile = "package://kuka_moveit/model/stl/ceiling_lamp_xyz_1_0.03_0.02m.STL";
    const std::string cylinderFile = "package://kuka_moveit/model/stl/cylinder_xyz_d0.085_h0.03m.STL";
    const std::string collisionFile = "package://kuka_moveit/model/stl/collision_xyz_0.5_0.6_1.5m.STL";

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[First Step] Add Objects." << std::endl;
    
    //Add the workpiece and collisions step by step
    std::cout << "add a lamp" << std::endl;
    AddStl(lampFile, 0.35, -0.25, 1.886);   // AddObjects();
    WaitKey();
    WaitKey();
    std::cout << "add a cylinder" << std::endl;
    AddStl(cylinderFile, 0.35 - 0.085 / 2, -0.25 - 0.085 / 2, 1.534 - 0.03);
    WaitKey();
    std::cout << "add a cuboid" << std::endl;
    AddStl(collisionFile, 0.01, -1.215, 0);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Second Step] Move to first place." << std::endl;

    //Move the KUKA robot to the target location
    MoveTo(0.358829, -0.274889, 1.73522, -0.654404, 0.145868, 0.697154, 0.253879);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Third Step] Attach the object." << std::endl;

    //Attach the workpiece to KUKA robot
    Attach(cylinderFile);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Fourth Step] Move to second place." << std::endl;
    
    //Move the KUKA robot and workpiece to the target location
    MoveTo(0.846218, -0.990494, 1.45394, 0.429289, 0.555199, -0.518724, 0.488253);
    
    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Fifth Step] Detach the Object." << std::endl;

    //Detach the piece from KUKA robot
    Detach(cylinderFile);

    //Wait for next step
    std::cout << "\nPress any key to continue..." <<std::endl;
    WaitKey();

    //Show text in RViz of status
    std::cout << "\n-----------------------" << std::endl;
    std::cout << "[Last Step] Remove the Object." << std::endl;

    //Remove the all the objects
    std::vector<std::string> object_ids;
    object_ids.push_back(cylinderFile);
    object_ids.push_back(collisionFile);
    object_ids.push_back(lampFile);
    Remove(object_ids);
    
    //Wait for next step
    std::cout << "Press any key to go back to the menu..." <<std::endl;
    WaitKey();
}

