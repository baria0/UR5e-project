#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frame_transform/FrameTransform.h>

const double tau = 2 * M_PI;


class PickAndPlace
{
public:
    PickAndPlace()
    {
        client_picking_pose = nh.serviceClient<frame_transform::FrameTransform>("/get_position_base_link");
    }


    // void close_gripper(moveit::planning_interface::MoveGroupInterface& gripper)
    // {
    //     gripper.setMaxVelocityScalingFactor(0.5);
    //     gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.65);
    //     gripper.move();
    // }

    // void open_gripper(moveit::planning_interface::MoveGroupInterface& gripper)
    // {
    //     gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
    //     gripper.move();
    // }

    

    void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {

        tf2::Quaternion orientation; 
        orientation.setRPY(-3.136, -0.101, -0.001); 
        pick_position.orientation = tf2::toMsg(orientation);
        const double eef_offset = 0.2; 
        pick_position.position.x = 0.600;
        pick_position.position.y = 0.070;
        pick_position.position.z = 0.443;
        move_group.setPoseTarget(pick_position, "tool0");
        move_group.move();


    }

    void place(moveit::planning_interface::MoveGroupInterface& move_group_place)
    {
        geometry_msgs::Pose place_position;
        

        tf2::Quaternion orientation;
        orientation.setRPY(-3.131, -0.002, 1.483); 
        place_position.orientation = tf2::toMsg(orientation);
        place_position.position.x = -0.147;
        place_position.position.y = 0.791;
        place_position.position.z = 0.379;
        move_group_place.setPoseTarget(place_position, "tool0");

        move_group_place.move();
    }

    
private:
        ros::NodeHandle nh;
        geometry_msgs::Pose pick_position;
        geometry_msgs::Pose place_position;

        ros::ServiceClient client_picking_pose;
        // Initialize the service
        frame_transform::FrameTransform service;

    
};

    void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
    {
    // add collision objects
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.resize(3);

        // Add the first table
        collision_objects[0].id = "table1";
        collision_objects[0].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 1
        collision_objects[0].primitives.resize(1);
        collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
        collision_objects[0].primitives[0].dimensions.resize(3);
        collision_objects[0].primitives[0].dimensions[0] = 0.608;
        collision_objects[0].primitives[0].dimensions[1] = 2;
        collision_objects[0].primitives[0].dimensions[2] = 1;
        // pose of table 1
        collision_objects[0].primitive_poses.resize(1);
        collision_objects[0].primitive_poses[0].position.x = 0.75;
        collision_objects[0].primitive_poses[0].position.y = 0;
        collision_objects[0].primitive_poses[0].position.z = -0.3;
        collision_objects[0].primitive_poses[0].orientation.w = 1.0;
        // Add tabe 1 to the scene

        collision_objects[0].operation = collision_objects[0].ADD;


        // Add the second table
        collision_objects[1].id = "table2";
        collision_objects[1].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 2
        collision_objects[1].primitives.resize(1);
        collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
        collision_objects[1].primitives[0].dimensions.resize(3);
        collision_objects[1].primitives[0].dimensions[0] = 1.3;
        collision_objects[1].primitives[0].dimensions[1] = 0.8;
        collision_objects[1].primitives[0].dimensions[2] = 1;
        // pose of table 2
        collision_objects[1].primitive_poses.resize(1);
        collision_objects[1].primitive_poses[0].position.x = 0;
        collision_objects[1].primitive_poses[0].position.y = 1;
        collision_objects[1].primitive_poses[0].position.z = -0.3;
        collision_objects[1].primitive_poses[0].orientation.w = 1.0;
        // Add tabe 2 to the scene
        collision_objects[1].operation = collision_objects[1].ADD;


        // add the basement
        collision_objects[2].id = "basement";
        collision_objects[2].header.frame_id = "base_link";

        // Define primitive dimension, position of the table 2
        collision_objects[2].primitives.resize(1);
        collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].CYLINDER;
        collision_objects[2].primitives[0].dimensions.resize(2);
        collision_objects[2].primitives[0].dimensions[0] = 0.8;
        collision_objects[2].primitives[0].dimensions[1] = 0.2;
        
        // pose of table 2
        collision_objects[2].primitive_poses.resize(1);
        collision_objects[2].primitive_poses[0].position.x = 0;
        collision_objects[2].primitive_poses[0].position.y = 0;
        collision_objects[2].primitive_poses[0].position.z = -0.4;
        collision_objects[2].primitive_poses[0].orientation.w = 1.0;
        // Add tabe 2 to the scene
        collision_objects[2].operation = collision_objects[2].ADD;

        //         // Add the first sphere
        // collision_objects[3].id = "sphere1";
        // collision_objects[3].header.frame_id = "base_link";

        // collision_objects[3].primitives.resize(1);
        // collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].SPHERE;
        // collision_objects[3].primitives[0].dimensions.resize(1);
        // collision_objects[3].primitives[0].dimensions[0] = 0.1; // Radius

        // collision_objects[3].primitive_poses.resize(1);
        // collision_objects[3].primitive_poses[0].position.x = 0.5;
        // collision_objects[3].primitive_poses[0].position.y = 0.3;
        // collision_objects[3].primitive_poses[0].position.z = 0.5;
        // collision_objects[3].primitive_poses[0].orientation.w = 1.0;
        // collision_objects[3].operation = collision_objects[3].ADD;

        // // Add the second sphere
        // collision_objects[4].id = "sphere2";
        // collision_objects[4].header.frame_id = "base_link";

        // collision_objects[4].primitives.resize(1);
        // collision_objects[4].primitives[0].type = collision_objects[4].primitives[0].SPHERE;
        // collision_objects[4].primitives[0].dimensions.resize(1);
        // collision_objects[4].primitives[0].dimensions[0] = 0.09; // Radius

        // collision_objects[4].primitive_poses.resize(1);
        // collision_objects[4].primitive_poses[0].position.x = 0.26;
        // collision_objects[4].primitive_poses[0].position.y = 0.56;
        // collision_objects[4].primitive_poses[0].position.z = 0.51;
        // collision_objects[4].primitive_poses[0].orientation.w = 1.0;
        // collision_objects[4].operation = collision_objects[4].ADD;

        // // Add the third sphere
        // collision_objects[5].id = "sphere3";
        // collision_objects[5].header.frame_id = "base_link";

        // collision_objects[5].primitives.resize(1);
        // collision_objects[5].primitives[0].type = collision_objects[5].primitives[0].SPHERE;
        // collision_objects[5].primitives[0].dimensions.resize(1);
        // collision_objects[5].primitives[0].dimensions[0] = 0.08; // Radius

        // collision_objects[5].primitive_poses.resize(1);
        // collision_objects[5].primitive_poses[0].position.x = -0.2;
        // collision_objects[5].primitive_poses[0].position.y = 0.5;
        // collision_objects[5].primitive_poses[0].position.z = 0.3;
        // collision_objects[5].primitive_poses[0].orientation.w = 1.0;
        // collision_objects[5].operation = collision_objects[5].ADD;


        planning_scene_interface.applyCollisionObjects(collision_objects);
    }


    // void setGripperConstraints(moveit::planning_interface::MoveGroupInterface& gripper)
    // {
    //     moveit_msgs::Constraints gripper_constraints;
    //     moveit_msgs::JointConstraint joint_constraint;
    //     joint_constraint.joint_name = "robotiq_85_left_knuckle_joint";
    //     joint_constraint.position = 0.04;
    //     joint_constraint.tolerance_above = 0.001;
    //     joint_constraint.tolerance_below = 0.001;
    //     joint_constraint.weight = 1.0;

    //     gripper_constraints.joint_constraints.push_back(joint_constraint);
    //     gripper.setPathConstraints(gripper_constraints);
    // }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    //ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    // moveit::planning_interface::MoveGroupInterface gripper("gripper");
    group.setPoseReferenceFrame("base_link");
    group.setPlanningTime(45.0);

    // setup the speed and accelleration
    group.setMaxVelocityScalingFactor(1.0);   // added
    group.setMaxAccelerationScalingFactor(1.0);   // added


    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    // If you do the following procedure, the robot goes in collision with the object that has to be picked

    PickAndPlace pick_and_place;
    
    ros::WallDuration(1.0).sleep();
    // setGripperConstraints(gripper);
    pick_and_place.pick(group);
    // gripper.clearPathConstraints();
    ros::WallDuration(2.0).sleep(); // increased to 2
    // pick_and_place.close_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    // setGripperConstraints(gripper);
    pick_and_place.place(group);
    // gripper.clearPathConstraints();
    ros::WallDuration(1.0).sleep();
    // pick_and_place.open_gripper(gripper);
    ros::WallDuration(1.0).sleep();
    

    ros::waitForShutdown();
    return 0;
}