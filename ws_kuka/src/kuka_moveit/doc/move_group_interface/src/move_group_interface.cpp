/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>

double sqrt(double,int);
double hexToDec(char*);
std::string xxxx;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "robot_arm";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface KUKA", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");








//------------------------------------------------First Step-------------------------------------------------------
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  
  std::string answer;
  
  
  while(1){
    std::cout << "\n--------------------------------------------------------------------------------" << std::endl;
    std::cout << "> Do you want to use an already programmed target pose?(yes/no) " << std::endl;
    std::cin >> answer;
    if(answer=="yes"||answer=="y"){
      while(1){
        answer.clear();
        std::cout << "> Which target do you wang to use? \n> Please type in only the sequence number" << std::endl;
        std::cout << ">> 1. arm_pos_get" << std::endl;
        std::cout << ">> 2. arm_pos_work" << std::endl;
        std::cin >> answer;
        if(answer=="1"){        //arm_pos_get
          target_pose1.position.x = 0.81;//0.81
          target_pose1.position.y = -0.96;//-0.96
          target_pose1.position.z = 1.24;//1.24
          target_pose1.orientation.w = 0.49;//0.49
          target_pose1.orientation.x = 0.51;//0.51
          target_pose1.orientation.y = 0.49;//0.49
          target_pose1.orientation.z = -0.51;//-0.51
          answer.clear();
          break;
        }
        else if(answer=="2"){   //arm_pos_work
          target_pose1.position.x = 0.36;
          target_pose1.position.y = -0.75;
          target_pose1.position.z = 1.27;
          target_pose1.orientation.w = 0.2;
          target_pose1.orientation.x = 0.69;
          target_pose1.orientation.y = 0.21;
          target_pose1.orientation.z = -0.66;
          answer.clear();
          break;
        }
        else{
          std::cout << "> !!! Invalid answer, please try again. ";
        }
      }
      break;
    }
    else if (answer=="no"||answer=="n"){
      answer.clear();
      std::cout << "> 请输入目标点x坐标：";
      std::cin >> target_pose1.position.x;
      // std::cout << "您输入目标点x坐标是：" << target_pose1.position.x << std::endl;
      
      std::cout << "> 请输入目标点y坐标：";
      std::cin >> target_pose1.position.y;
      // std::cout << "您输入目标点y坐标是：" << target_pose1.position.y << std::endl;
      
      std::cout << "> 请输入目标点z坐标：";
      std::cin >> target_pose1.position.z;
      // std::cout << "您输入目标点z坐标是：" << target_pose1.position.z << std::endl;
      
      std::cout << "> 请输入目标点w方向：";
      std::cin >> target_pose1.orientation.w;
      // std::cout << "您输入目标点w方向是：" << target_pose1.orientation.w << std::endl;

      std::cout << "> 请输入目标点x方向：";
      std::cin >> target_pose1.orientation.x;
      // std::cout << "您输入目标点x方向是：" << target_pose1.orientation.x << std::endl;
      
      std::cout << "> 请输入目标点y方向：";
      std::cin >> target_pose1.orientation.y;
      // std::cout << "您输入目标点y方向是：" << target_pose1.orientation.y << std::endl;
      
      std::cout << "> 请输入目标点z方向：";
      std::cin >> target_pose1.orientation.z;
      // std::cout << "您输入目标点z方向是：" << target_pose1.orientation.z << std::endl;
      break;
    }
    else{
      std::cout << "> !!! Invalid answer, please try again. ";
      answer.clear();
    }
  }
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  // Uncomment below line when working with a real robot //
  move_group.move();





//                          add new object
// Define a collision object ROS message.
  std::cout << "next" << std::endl;
  std::cin >> xxxx;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.085;//0.095
  primitive.dimensions[1] = 0.19;//0.2
  primitive.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.35;
  box_pose.position.y = -0.25;
  box_pose.position.z = 1.524;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();



  //                                move the robot to the object
  std::cout << "next" << std::endl;
  std::cin >> xxxx;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose object_pose;
  object_pose.orientation.w = 0.263555;//0.263555
  object_pose.orientation.x = -0.547637;//-0.547637
  object_pose.orientation.y = 0.257702;//0.257702
  object_pose.orientation.z = 0.751147;//0.751147
  object_pose.position.x = 0.379765;//0.379765
  object_pose.position.y = -0.24068;//-0.24068
  object_pose.position.z = 1.62333;//1.62333
  move_group.setPoseTarget(object_pose);
  
 
  bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "move robot to the object %s", success2 ? "" : "FAILED");
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan2.trajectory_, joint_model_group);
  visual_tools.trigger();
  move_group.move();
  


  //                             attach the object
  // Now, let's attach the collision object to the robot.
  std::cout << "next" << std::endl;
  std::cin >> xxxx;
  ROS_INFO_NAMED("tutorial","Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();





  //                           detach the object
  // Now, let's detach the collision object from the robot.
  std::cout << "next" << std::endl;
  std::cin >> xxxx;
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();





  ros::shutdown();
  return 0;
}

double sqrt(double sum,int i)
{
  double root = sum;
  while (i>0,i--)
    sum *= root;
  return sum;
}
 
double hexToDec(char *str)
{
  int i = 0;
  float sumd = 0.0;
  double sumf = 0.0;
  bool negative = false;
  bool error = false;
 
  for (; *str; str++) {
    if (*str == '-') {
      negative = true;
      continue;
    }
    if (*str == '.') {
      error = true;
      continue;
    }
    if (error){
      sumf = sumf + (*str - '0')/sqrt(10.0,i);
      i++;
    }
    else {
      sumd = 10.0 * sumd + (*str - '0');
    }
  }
  if (negative)
    sumd = -(sumd + sumf);
  else
    sumd += sumf;
  return sumd;
}


// //--------------------------Adding/Removing Objects and Attaching/Detaching Objects--------------------------------------------
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // Define a collision object ROS message.
//   moveit_msgs::CollisionObject collision_object;
//   collision_object.header.frame_id = move_group.getPlanningFrame();

//   // The id of the object is used to identify it.
//   collision_object.id = "box1";

//   // Define a box to add to the world.
//   shape_msgs::SolidPrimitive primitive;
//   primitive.type = primitive.BOX;
//   primitive.dimensions.resize(3);
//   primitive.dimensions[0] = 1.2; //初始值为0.4
//   primitive.dimensions[1] = 0.1; //初始值为0.1
//   primitive.dimensions[2] = 0.4; //初始值为0.4

//   // Define a pose for the box (specified relative to frame_id)
//   geometry_msgs::Pose box_pose;
//   box_pose.orientation.w = 1.0;
//   box_pose.position.x = 0.4; //初始值为0.4
//   box_pose.position.y = -0.2; //初始值为-0.2
//   box_pose.position.z = 1.0; //初始值为1.0

//   collision_object.primitives.push_back(primitive);
//   collision_object.primitive_poses.push_back(box_pose);
//   collision_object.operation = collision_object.ADD;

//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.push_back(collision_object);

//   // Now, let's add the collision object into the world
//   ROS_INFO_NAMED("tutorial", "Add an object into the world");
//   planning_scene_interface.addCollisionObjects(collision_objects);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   // Wait for MoveGroup to recieve and process the collision object message
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

//   // Now when we plan a trajectory it will avoid the obstacle
//   move_group.setStartState(*move_group.getCurrentState());
//   geometry_msgs::Pose another_pose;
//   another_pose.orientation.w = 1.0;
//   another_pose.position.x = 0.4;
//   another_pose.position.y = -0.4;
//   another_pose.position.z = 0.9;
//   move_group.setPoseTarget(another_pose);

//   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

//   // Visualize the plan in RViz
//   visual_tools.deleteAllMarkers();
//   visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("next step");

// //--------------------------------------Now, let's attach the collision object to the robot.------------------------------
//   ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
//   move_group.attachObject(collision_object.id);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to recieve and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
//                       "robot");

//   // Now, let's detach the collision object from the robot.
//   ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//   move_group.detachObject(collision_object.id);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to recieve and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
//                       "robot");

//   // Now, let's remove the collision object from the world.
//   ROS_INFO_NAMED("tutorial", "Remove the object from the world");
//   std::vector<std::string> object_ids;
//   object_ids.push_back(collision_object.id);
//   planning_scene_interface.removeCollisionObjects(object_ids);

//   // Show text in RViz of status
//   visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for MoveGroup to recieve and process the attached collision object message */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

//   // END_TUTORIAL
