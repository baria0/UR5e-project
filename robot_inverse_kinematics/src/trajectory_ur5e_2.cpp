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

    void pick(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        tf2::Quaternion orientation; 
        orientation.setRPY(3.134, 0.043, 2.943); 
        pick_position.orientation = tf2::toMsg(orientation);
        pick_position.position.x = -0.462; 
        pick_position.position.y = -0.257;
        pick_position.position.z = 0.361;
        move_group.setPoseTarget(pick_position, "tool0");
        move_group.move();
    }

    void place(moveit::planning_interface::MoveGroupInterface& move_group_place)
    {
        geometry_msgs::Pose place_position;
        
        tf2::Quaternion orientation;
        orientation.setRPY(-3.094, 0.016, 1.255); 
        place_position.orientation = tf2::toMsg(orientation);
        place_position.position.x = -0.357;  
        place_position.position.y = 0.436;
        place_position.position.z = 0.322;
        move_group_place.setPoseTarget(place_position, "tool0");
        move_group_place.move();
    }

private:
    ros::NodeHandle nh;
    geometry_msgs::Pose pick_position;
    geometry_msgs::Pose place_position;
    ros::ServiceClient client_picking_pose;
    frame_transform::FrameTransform service;
};

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(7);  

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
    group.setPlanningTime(45.0);

    group.setMaxVelocityScalingFactor(0.2);   
    group.setMaxAccelerationScalingFactor(0.2);   

    addCollisionObjects(planning_scene_interface);
    ros::WallDuration(1.0).sleep();

    PickAndPlace pick_and_place;
    ros::WallDuration(1.0).sleep();
    pick_and_place.pick(group);
    ros::WallDuration(2.0).sleep();
    pick_and_place.place(group);
    ros::WallDuration(1.0).sleep();
    
    ros::waitForShutdown();
    return 0;
}
