#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <frame_transform/FrameTransform.h>
#include <Eigen/Dense>
#include <iostream>
#include <unordered_map>

const double tau = 2 * M_PI;
const double BASE_VOXEL_SIZE = 0.1;
const double GRID_RESOLUTION = 0.1;

struct CollisionPoint {
    Eigen::Vector3d position;
    double intensity;
};

std::vector<CollisionPoint> getCollisionPoints() {
    return {
        {{-0.30, 0.00, 0.20}, 0.6},
        {{-0.33, 0.05, 0.25}, 0.4},
        {{-0.37, -0.02, 0.18}, 0.55},
        {{-0.50, 0.20, 0.30}, 0.5},
        {{-0.55, 0.18, 0.33}, 0.3},
        {{-0.35, -0.10, 0.22}, 0.7},
        {{-0.28, -0.25, 0.18}, 0.3},
        {{-0.32, -0.22, 0.22}, 0.4},
        {{-0.29, -0.30, 0.21}, 0.5},
        {{-0.31, -0.28, 0.20}, 0.6}

    };
}

struct GridKey {
    int x, y, z;
    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std {
    template<>
    struct hash<GridKey> {
        size_t operator()(const GridKey& k) const {
            return ((hash<int>()(k.x) ^ (hash<int>()(k.y) << 1)) >> 1) ^ (hash<int>()(k.z) << 1);
        }
    };
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Table
    moveit_msgs::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";
    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions = {1.3, 1.3, 0.2};
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0;
    table.primitive_poses[0].position.y = 0;
    table.primitive_poses[0].position.z = -0.1;
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI/4);
    table.primitive_poses[0].orientation = tf2::toMsg(q);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // Walls
    double wall_positions[4][3] = {
        {0.47, 0.48, 0.50},
        {-0.47, -0.48, 0.50},
        {-0.47, 0.48, 0.50},
        {0.47, -0.48, 0.50}
    };
    double wall_rotations[4] = {0.79, 0.79, -0.79, -0.79};
    for (int i = 0; i < 4; i++) {
        moveit_msgs::CollisionObject wall;
        wall.id = "wall_" + std::to_string(i + 1);
        wall.header.frame_id = "base_link";
        wall.primitives.resize(1);
        wall.primitives[0].type = wall.primitives[0].BOX;
        wall.primitives[0].dimensions = {0.02, 1.3, 1.0};
        wall.primitive_poses.resize(1);
        wall.primitive_poses[0].position.x = wall_positions[i][0];
        wall.primitive_poses[0].position.y = wall_positions[i][1];
        wall.primitive_poses[0].position.z = wall_positions[i][2];
        tf2::Quaternion q;
        q.setRPY(0, 0, wall_rotations[i]);
        wall.primitive_poses[0].orientation = tf2::toMsg(q);
        wall.operation = wall.ADD;
        collision_objects.push_back(wall);
    }

// Raw collision points
    std::vector<CollisionPoint> raw_points = getCollisionPoints();

    // // Scaling workspace bounds
    // Eigen::Vector3d original_min(DBL_MAX, DBL_MAX, DBL_MAX);
    // Eigen::Vector3d original_max(DBL_MIN, DBL_MIN, DBL_MIN);
    // for (const auto& p : raw_points) {
    //     original_min = original_min.cwiseMin(p.position);
    //     original_max = original_max.cwiseMax(p.position);
    // }

    // Eigen::Vector2d x_bounds(0.25, 0.60);
    // Eigen::Vector2d y_bounds(-0.4, 0.4);
    // Eigen::Vector2d z_bounds(0.15, 0.4);

    // // Scale points
    // std::vector<CollisionPoint> scaled_points;
    // for (const auto& p : raw_points) {
    //     Eigen::Vector3d scaled;
    //     scaled.x() = ((p.position.x() - original_min.x()) / (original_max.x() - original_min.x())) * (x_bounds[1] - x_bounds[0]) + x_bounds[0];
    //     scaled.y() = ((p.position.y() - original_min.y()) / (original_max.y() - original_min.y())) * (y_bounds[1] - y_bounds[0]) + y_bounds[0];
    //     scaled.z() = ((p.position.z() - original_min.z()) / (original_max.z() - original_min.z())) * (z_bounds[1] - z_bounds[0]) + z_bounds[0];
    //     scaled_points.push_back({scaled, p.intensity});
    // }

    std::vector<CollisionPoint> scaled_points = raw_points;

    // Distance-based merging
    const double distance_threshold = 0.1;
    std::vector<bool> used(scaled_points.size(), false);
    int voxel_id = 0;

    for (size_t i = 0; i < scaled_points.size(); ++i) {
        if (used[i]) continue;

        std::vector<CollisionPoint> group = {scaled_points[i]};
        used[i] = true;

        for (size_t j = i + 1; j < scaled_points.size(); ++j) {
            if (!used[j] && (scaled_points[i].position - scaled_points[j].position).norm() <= distance_threshold) {
                group.push_back(scaled_points[j]);
                used[j] = true;
            }
        }

        // Compute average position and max intensity
        Eigen::Vector3d avg_pos(0, 0, 0);
        double max_intensity = 0.0;
        for (const auto& p : group) {
            avg_pos += p.position;
            if (p.intensity > max_intensity)
                max_intensity = p.intensity;
        }
        avg_pos /= static_cast<double>(group.size());

        // Compute dynamic size
        double voxel_size = BASE_VOXEL_SIZE * (0.05 + max_intensity);

        moveit_msgs::CollisionObject voxel;
        voxel.id = "voxel_" + std::to_string(voxel_id++);
        voxel.header.frame_id = "base_link";
        voxel.primitives.resize(1);
        voxel.primitives[0].type = voxel.primitives[0].BOX;
        voxel.primitives[0].dimensions = {voxel_size, voxel_size, voxel_size};
        voxel.primitive_poses.resize(1);
        voxel.primitive_poses[0].position.x = avg_pos.x();
        voxel.primitive_poses[0].position.y = avg_pos.y();
        voxel.primitive_poses[0].position.z = avg_pos.z();
        voxel.primitive_poses[0].orientation.w = 1.0;
        voxel.operation = voxel.ADD;

        collision_objects.push_back(voxel);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);

    // Fake gripper as attached collision object (unchanged)
    moveit_msgs::CollisionObject fake_gripper;
    fake_gripper.id = "fake_gripper";
    fake_gripper.header.frame_id = "tool0";
    fake_gripper.primitives.resize(1);
    fake_gripper.primitives[0].type = fake_gripper.primitives[0].BOX;
    fake_gripper.primitives[0].dimensions = {0.08, 0.13, 0.15};
    fake_gripper.primitive_poses.resize(1);
    fake_gripper.primitive_poses[0].position.z = 0.115;
    fake_gripper.primitive_poses[0].orientation.w = 1.0;
    fake_gripper.operation = fake_gripper.ADD;

    moveit_msgs::AttachedCollisionObject attached_gripper;
    attached_gripper.link_name = "tool0";
    attached_gripper.object = fake_gripper;
    attached_gripper.touch_links = {"tool0"};
    planning_scene_interface.applyAttachedCollisionObject(attached_gripper);
}



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

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            std::cout << "Pick motion plan successful. Execute? (y/n): ";
            char response;
            std::cin >> response;
            if (response == 'y' || response == 'Y') {
                ROS_INFO("Executing pick motion...");
                move_group.execute(my_plan);
            } else {
                ROS_WARN("Pick execution cancelled.");
            }
        } else {
            ROS_WARN("Pick motion plan failed!");
        }
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

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_place.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            std::cout << "Place motion plan successful. Execute? (y/n): ";
            char response;
            std::cin >> response;
            if (response == 'y' || response == 'Y') {
                ROS_INFO("Executing place motion...");
                move_group_place.execute(my_plan);
            } else {
                ROS_WARN("Place execution cancelled.");
            }
        } else {
            ROS_WARN("Place motion plan failed!");
        }
    }

private:
    ros::NodeHandle nh;
    geometry_msgs::Pose pick_position;
    geometry_msgs::Pose place_position;
    ros::ServiceClient client_picking_pose;
    frame_transform::FrameTransform service;
};

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
