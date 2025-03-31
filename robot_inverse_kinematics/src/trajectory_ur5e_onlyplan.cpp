#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frame_transform/FrameTransform.h>
#include <iostream>

const double tau = 2 * M_PI;

/**
 * Function to display planned trajectory before execution
 */
void displayTrajectory(moveit_msgs::RobotTrajectory& trajectory)
{
    ros::NodeHandle nh;
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory.push_back(trajectory);
    display_publisher.publish(display_trajectory);

    ROS_INFO("Displaying planned trajectory...");
    ros::Duration(3.0).sleep(); // Allow time to visualize in RViz
}

/**
 * Function to ask the user for confirmation before execution
 */
bool askForExecution()
{
    std::string input;
    std::cout << "Do you want to execute the planned motion? (yes/no): ";
    std::cin >> input;
    return (input == "yes" || input == "y");
}

/**
 * Function to plan and optionally execute motion to a target pose
 */
bool moveToPose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& target_pose)
{
    move_group.setPoseTarget(target_pose, "tool0");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        ROS_INFO("Motion plan successful.");

        // Display the planned trajectory
        displayTrajectory(my_plan.trajectory_);

        // Ask user if they want to execute the motion
        if (askForExecution()) {
            ROS_INFO("Executing motion...");
            move_group.execute(my_plan);
        } else {
            ROS_WARN("Motion execution skipped.");
        }
    } else {
        ROS_WARN("Motion plan failed!");
    }

    return success;
}

/**
 * Function to add collision objects to the scene
 */
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(7);  

    // Add table
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions = {1.3, 1.3, 0.2};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = -0.1;

    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI/4);
    collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(q);
    collision_objects[0].operation = collision_objects[0].ADD;

    // Add spheres (random obstacles)
    double sphere_positions[6][3] = {
        {-0.29, -0.11, 0.18},
        {-0.4, 0.1, 0.3},
        {-0.36, 0.01, 0.39},
        {-0.3, -0.3, 0.6},
        {0.1, 0.4, 0.2},
        {-0.2, -0.26, 0.7}
    };

    for (int i = 1; i <= 6; i++)
    {
        collision_objects[i].id = "sphere_" + std::to_string(i);
        collision_objects[i].header.frame_id = "base_link";
        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].SPHERE;
        collision_objects[i].primitives[0].dimensions.resize(1);
        collision_objects[i].primitives[0].dimensions[0] = 0.05;
        collision_objects[i].primitive_poses.resize(1);
        collision_objects[i].primitive_poses[0].position.x = sphere_positions[i-1][0];
        collision_objects[i].primitive_poses[0].position.y = sphere_positions[i-1][1];
        collision_objects[i].primitive_poses[0].position.z = sphere_positions[i-1][2];
        collision_objects[i].operation = collision_objects[i].ADD;
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPoseReferenceFrame("base_link");
    group.setPlanningTime(5.0);
    
    // Set velocity and acceleration scaling
    group.setMaxVelocityScalingFactor(0.2);   
    group.setMaxAccelerationScalingFactor(0.2);   

    // Add collision objects
    addCollisionObjects(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    geometry_msgs::Pose pick_position, place_position;

    // Define pick position
    tf2::Quaternion pick_orientation;
    pick_orientation.setRPY(3.134, 0.043, 2.943);
    pick_position.orientation = tf2::toMsg(pick_orientation);
    pick_position.position.x = -0.462;
    pick_position.position.y = -0.257;
    pick_position.position.z = 0.361;

    // Define place position
    tf2::Quaternion place_orientation;
    place_orientation.setRPY(-3.094, 0.016, 1.255);
    place_position.orientation = tf2::toMsg(place_orientation);
    place_position.position.x = -0.357;
    place_position.position.y = 0.436;
    place_position.position.z = 0.322;

    // Perform pick and place movements
    if (moveToPose(group, pick_position)) {
        ros::WallDuration(2.0).sleep();
        moveToPose(group, place_position);
    }

    ros::waitForShutdown();
    return 0;
}
