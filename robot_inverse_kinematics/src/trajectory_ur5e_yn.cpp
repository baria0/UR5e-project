#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <iostream>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>



const double GRID_RESOLUTION = 0.1;

// Structure representing a point in 3D space with an associated intensity.
// These are used as raw collision input points.
struct CollisionPoint {
    Eigen::Vector3d position;
    double intensity;
};

// Returns a list of manually defined collision points with intensities.
// These represent clustered objects or obstacles in the environment.
std::vector<CollisionPoint> getCollisionPoints() {
    return {

        // {{-0.60, 0.00, 0.20}, 0.8},
        // {{-0.61, 0.02, 0.21}, 0.75},
        // {{-0.59, -0.01, 0.19}, 0.9},   // Group 1 (3 points)
        // {{-0.50, 0.10, 0.30}, 0.5},
        // {{-0.49, 0.11, 0.29}, 0.4},
        // {{-0.48, 0.09, 0.32}, 0.6},    // Group 2 (3 points)   
        // {{-0.70, -0.05, 0.25}, 0.3},
        // {{-0.71, -0.06, 0.24}, 0.1},    // Group 3 (2 points)   
        // {{-0.55, 0.00, 0.40}, 0.9},
        // {{-0.56, -0.02, 0.42}, 0.95},    // Group 4 (2 points)
        // {{-0.47, 0.0805, 0.115}, 0.2},
        // {{-0.455, 0.07, 0.07}, 0.25},
        // {{-0.48, 0.081, 0.09}, 0.3}    // Group 5 (3 points)
        
        {{-0.665, 0.297, 0.380}, 0.80},
        {{-0.651, 0.297, 0.380}, 0.80},
        {{0.104, 0.514, 0.200}, 0.90},
        {{0.099, 0.523, 0.210}, 0.80},
        {{0.099, 0.509, 0.200}, 0.80},
        {{-0.198, 0.651, 0.300}, 0.80},
        {{-0.212, 0.651, 0.310}, 0.70},
        {{-0.194, 0.640, 0.315}, 0.40},
        {{-0.191, 0.643, 0.300}, 0.80},
        {{-0.636, 0.198, 0.390}, 0.80},
        {{-0.640, 0.202, 0.380}, 0.80},
        {{-0.626, 0.202, 0.375}, 0.80},
        {{-0.636, 0.198, 0.375}, 0.80},
        {{-0.509, 0.113, 0.250}, 0.70},
        {{-0.506, 0.103, 0.240}, 0.60},
        {{-0.262, 0.276, 0.165}, 0.50},
        {{-0.283, 0.269, 0.165}, 0.40},
        {{-0.279, 0.265, 0.185}, 0.40}


    };
}

// Clusters collision points and adds axis-aligned box (voxel) collision objects
// to the planning scene, with dimensions adapted based on point spread and intensity.
void addAdaptiveVoxelCollisionObjects(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const std::vector<CollisionPoint>& points,
    ros::Publisher& planning_scene_diff_publisher,
    double base_voxel_size = 0.05,
    double merge_distance = 0.07) // Maximum distance between points to be considered part of the same cluster
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    int voxel_id = 0;
    std::vector<bool> used(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
        if (used[i]) continue;

        std::vector<CollisionPoint> cluster;
        cluster.push_back(points[i]);
        used[i] = true;

        for (size_t j = i + 1; j < points.size(); ++j) {
            if (!used[j] &&
                (points[i].position - points[j].position).norm() <= merge_distance) {
                cluster.push_back(points[j]);
                used[j] = true;
            }
        }

        Eigen::Vector3d center(0, 0, 0);
        double max_intensity = 0.0;
        for (const auto& p : cluster) {
            center += p.position;
            if (p.intensity > max_intensity)
                max_intensity = p.intensity;
        }
        center /= static_cast<double>(cluster.size());

        // Compute anisotropic voxel sizes by accounting for the radius of influence
        double max_dx = 0.0, max_dy = 0.0, max_dz = 0.0;
        for (const auto& p : cluster) {
            Eigen::Vector3d diff = p.position - center;
            double radius = 0.5 * (0.01 + p.intensity * 0.01);  // Estimate radius of the point's influence area based on intensity


            max_dx = std::max(max_dx, std::abs(diff.x()) + radius);
            max_dy = std::max(max_dy, std::abs(diff.y()) + radius);
            max_dz = std::max(max_dz, std::abs(diff.z()) + radius);
        }

        double padding = 0.01;
        double size_x = std::max(base_voxel_size, 2.0 * max_dx + padding); // Compute voxel dimensions to fully enclose the cluster (plus padding)
        double size_y = std::max(base_voxel_size, 2.0 * max_dy + padding);
        double size_z = std::max(base_voxel_size, 2.0 * max_dz + padding);

        ROS_INFO("Voxel %d -> size: [%.3f, %.3f, %.3f] m | intensity: %.2f | cluster size: %lu",
            voxel_id, size_x, size_y, size_z, max_intensity, cluster.size());

        moveit_msgs::CollisionObject voxel;
        voxel.id = "adaptive_voxel_" + std::to_string(voxel_id++);
        voxel.header.frame_id = "base_link";

        voxel.primitives.resize(1);
        voxel.primitives[0].type = voxel.primitives[0].BOX;
        voxel.primitives[0].dimensions = {size_x, size_y, size_z};

        voxel.primitive_poses.resize(1);
        voxel.primitive_poses[0].position.x = center.x();
        voxel.primitive_poses[0].position.y = center.y();
        voxel.primitive_poses[0].position.z = center.z();
        voxel.primitive_poses[0].orientation.w = 1.0;

        voxel.operation = voxel.ADD;
        collision_objects.push_back(voxel);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);

    // === Set color for adaptive voxels ===
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    for (int i = 0; i < voxel_id; ++i) {
        moveit_msgs::ObjectColor color;
        color.id = "adaptive_voxel_" + std::to_string(i);
        color.color.r = 1.0;
        color.color.g = 0.0;
        color.color.b = 0.0;
        color.color.a = 0.40;
        planning_scene_msg.object_colors.push_back(color);
    }

    planning_scene_diff_publisher.publish(planning_scene_msg);
}

// Adds the original collision points to the scene as spheres
// (for visualization/debugging purposes)
void addCollisionPointsToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const std::vector<CollisionPoint>& points) 
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    int point_id = 0;

    for (const auto& point : points) {
    moveit_msgs::CollisionObject obj;
    obj.id = "collision_point_" + std::to_string(point_id++);
    obj.header.frame_id = "base_link";

    obj.primitives.resize(1);
    obj.primitives[0].type = obj.primitives[0].SPHERE;

    //Set the sphere size based on intensity (e.g., range between 0.02m - 0.06m)
    double size = 0.01 + point.intensity * 0.01; // Sample computation
    obj.primitives[0].dimensions = {size};

    obj.primitive_poses.resize(1);
    obj.primitive_poses[0].position.x = point.position.x();
    obj.primitive_poses[0].position.y = point.position.y();
    obj.primitive_poses[0].position.z = point.position.z();
    obj.primitive_poses[0].orientation.w = 1.0;

    obj.operation = obj.ADD;

    collision_objects.push_back(obj);
    }

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Adds static environment objects like the table, stand, walls, and gripper.
// These are modeled as collision objects to constrain robot motion planning.
void addEnvironmentObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Table
    moveit_msgs::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";
    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions = {1.5, 1.5, 0.2};
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.z = -0.17;
    tf2::Quaternion q;
    q.setRPY(0, 0, M_PI / 4);
    table.primitive_poses[0].orientation = tf2::toMsg(q);
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // Stand
    moveit_msgs::CollisionObject stand;
    stand.id = "stand";
    stand.header.frame_id = "base_link";
    stand.primitives.resize(1);
    stand.primitives[0].type = stand.primitives[0].BOX;
    stand.primitives[0].dimensions = {0.2, 0.2, 0.2};
    stand.primitive_poses.resize(1);
    stand.primitive_poses[0].position.z = -0.1;
    q.setRPY(0, 0, M_PI / 4);
    stand.primitive_poses[0].orientation = tf2::toMsg(q);
    stand.operation = stand.ADD;
    collision_objects.push_back(stand);


    // Walls
    double wall_positions[4][3] = {     // Corner positions for placing diagonal walls
        { 1.06 / 2,  1.06 / 2, 0.43},   // Right up
        {-1.06 / 2, -1.06 / 2, 0.43},   // Left down
        {-1.06 / 2,  1.06 / 2, 0.43},   // Left up
        { 1.06 / 2, -1.06 / 2, 0.43}    // Right down
    };
    
    double wall_rotations[4] = {
        M_PI / 4,   // 45°
        M_PI / 4,
        -M_PI / 4,
        -M_PI / 4
    };
    
    for (int i = 0; i < 4; ++i) {
        moveit_msgs::CollisionObject wall;
        wall.id = "wall_" + std::to_string(i + 1);
        wall.header.frame_id = "base_link";
        wall.primitives.resize(1);
        wall.primitives[0].type = wall.primitives[0].BOX;
    
        // Slightly increased wall dimensions to fully cover the corners
        wall.primitives[0].dimensions = {0.02, 1.4, 1.0};
    
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
    

    // Adds a dummy gripper geometry as an attached collision object
    moveit_msgs::CollisionObject fake_gripper;
    fake_gripper.id = "fake_gripper";
    fake_gripper.header.frame_id = "tool0";
    fake_gripper.primitives.resize(1);
    fake_gripper.primitives[0].type = fake_gripper.primitives[0].BOX;
    fake_gripper.primitives[0].dimensions = {0.105, 0.086, 0.225};
    fake_gripper.primitive_poses.resize(1);
    fake_gripper.primitive_poses[0].position.z = 0.115;
    q.setRPY(0, 0, M_PI / 6); 
    fake_gripper.primitive_poses[0].orientation = tf2::toMsg(q);
    fake_gripper.primitive_poses[0].orientation.w = 1.0;
    fake_gripper.operation = fake_gripper.ADD;

    moveit_msgs::AttachedCollisionObject attached_gripper;
    attached_gripper.link_name = "tool0";
    attached_gripper.object = fake_gripper;
    attached_gripper.touch_links = {"tool0"};
    planning_scene_interface.applyAttachedCollisionObject(attached_gripper);

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Class responsible for executing a basic pick-and-place task
class PickAndPlace {
public:
    void executePickAndPlace(moveit::planning_interface::MoveGroupInterface& move_group,    // Performs a pick and then a place motion using the planning group
        ros::Publisher& display_publisher)

    {
        // performMotion(move_group, {-0.462, -0.257, 0.361}, {3.134, 0.043, 2.943}, "Pick", display_publisher); 
        performMotion(move_group, {-0.593, 0.042, 0.285}, {3.134, 0.043, 2.943}, "Pick", display_publisher);   
        ros::WallDuration(2.0).sleep();
        // performMotion(move_group, {-0.357, 0.436, 0.322}, {-3.094, 0.016, 1.255}, "Place", display_publisher); 
        performMotion(move_group, {-0.084, 0.632, 0.415}, {-3.094, 0.016, 1.255}, "Place", display_publisher);    
    }

private:
    // Attempts motion planning to a target pose, trying different Z heights.
    // Selects the plan with the shortest joint-space path.
void performMotion(moveit::planning_interface::MoveGroupInterface& group,
                   const std::vector<double>& pos,
                   const std::vector<double>& rpy,
                   const std::string& action,
                   ros::Publisher& display_publisher)
{
    const int max_attempts = 10;
    const double delta_z = 0.01;

    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(rpy[0], rpy[1], rpy[2]);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = pos[0];
    target_pose.position.y = pos[1];
    target_pose.position.z = pos[2];

    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("manipulator");

    std::vector<std::pair<moveit::planning_interface::MoveGroupInterface::Plan, double>> successful_plans;

    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        moveit::core::RobotStatePtr ik_state(new moveit::core::RobotState(*group.getCurrentState()));
        bool found_ik = ik_state->setFromIK(joint_model_group, target_pose, "tool0", 2.0);

        if (!found_ik) {
            ROS_WARN("%s motion IK not found on attempt %d.", action.c_str(), attempt);
        } else {
            std::vector<double> joint_values;
            ik_state->copyJointGroupPositions(joint_model_group, joint_values);
            group.setJointValueTarget(joint_values);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (success) {
                double total_path_length = 0.0;
                const auto& traj = plan.trajectory_.joint_trajectory;
                for (size_t i = 1; i < traj.points.size(); ++i) {
                    double dist = 0.0;
                    for (size_t j = 0; j < traj.points[i].positions.size(); ++j) {
                        double diff = traj.points[i].positions[j] - traj.points[i - 1].positions[j];
                        dist += diff * diff;
                    }
                    total_path_length += std::sqrt(dist);
                }

                ROS_INFO("%s motion plan successful on attempt %d with path length: %.3f", action.c_str(), attempt, total_path_length);
                successful_plans.emplace_back(plan, total_path_length);
            } else {
                ROS_WARN("%s motion plan failed on attempt %d.", action.c_str(), attempt);
            }
        }

        target_pose.position.z += delta_z;
    }

    if (successful_plans.empty()) {
        ROS_ERROR("%s motion plan failed after %d attempts.", action.c_str(), max_attempts);
        return;
    }

    // Select the best plan with shortest joint path
    auto best_plan = std::min_element(successful_plans.begin(), successful_plans.end(),
        [](const std::pair<moveit::planning_interface::MoveGroupInterface::Plan, double>& a,
           const std::pair<moveit::planning_interface::MoveGroupInterface::Plan, double>& b) {
            return a.second < b.second;
        });

    // Visualize
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    moveit_msgs::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(*current_state, state_msg);
    display_trajectory.trajectory_start = state_msg;
    display_trajectory.trajectory.push_back(best_plan->first.trajectory_);
    display_publisher.publish(display_trajectory);
    ros::Duration(0.5).sleep();

    std::cout << action << " best motion plan selected (path length: " << best_plan->second << "). Execute? (y/n): ";
    char response;
    std::cin >> response;
    if (response == 'y' || response == 'Y') {
        ROS_INFO("Executing %s motion...", action.c_str());
        group.execute(best_plan->first);
    } else {
        ROS_WARN("%s execution cancelled.", action.c_str());
    }
}

// void performMotion(moveit::planning_interface::MoveGroupInterface& group,
//     const std::vector<double>& pos,
//     const std::vector<double>& rpy,
//     const std::string& action,
//     ros::Publisher& display_publisher)

// {
//     const int max_attempts = 10;    // Try up to n Z-offsets for feasible motion plan
//     const double delta_z = 0.02;

//     geometry_msgs::Pose target_pose;
//     tf2::Quaternion orientation;
//     orientation.setRPY(rpy[0], rpy[1], rpy[2]);
//     target_pose.orientation = tf2::toMsg(orientation);
//     target_pose.position.x = pos[0];
//     target_pose.position.y = pos[1];
//     target_pose.position.z = pos[2];

//     std::vector<std::pair<moveit::planning_interface::MoveGroupInterface::Plan, double>> successful_plans;

//     for (int attempt = 1; attempt <= max_attempts; ++attempt) {
//         group.setPoseTarget(target_pose, "tool0");

//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//         if (success) {
//             double total_path_length = 0.0;
//             const auto& traj = plan.trajectory_.joint_trajectory;
//             for (size_t i = 1; i < traj.points.size(); ++i) {
//                 double dist = 0.0;
//                 for (size_t j = 0; j < traj.points[i].positions.size(); ++j) {
//                     double diff = traj.points[i].positions[j] - traj.points[i - 1].positions[j];
//                     dist += diff * diff;
//                 }
//                 total_path_length += std::sqrt(dist);
//             }

//             ROS_INFO("%s motion plan successful on attempt %d with path length: %.3f", action.c_str(), attempt, total_path_length);
//             successful_plans.emplace_back(plan, total_path_length);
//         } else {
//             ROS_WARN("%s motion plan failed on attempt %d.", action.c_str(), attempt);
//         }

//         // Increment the Z height slightly on each attempt
//         target_pose.position.z += delta_z;
//     }

//     if (successful_plans.empty()) {
//         ROS_ERROR("%s motion plan failed after %d attempts.", action.c_str(), max_attempts);
//         return;
//     }

//     // Choose the shortest path for the executing plan
//     auto best_plan = std::min_element(successful_plans.begin(), successful_plans.end(),
//         [](const std::pair<moveit::planning_interface::MoveGroupInterface::Plan, double>& a,
//            const std::pair<moveit::planning_interface::MoveGroupInterface::Plan, double>& b) {
//             return a.second < b.second;
//         });
    
//     moveit_msgs::DisplayTrajectory display_trajectory;
//     moveit::core::RobotStatePtr current_state = group.getCurrentState();
//     moveit_msgs::RobotState state_msg;
//     moveit::core::robotStateToRobotStateMsg(*current_state, state_msg);

//     display_trajectory.trajectory_start = state_msg;

//     display_trajectory.trajectory.push_back(best_plan->first.trajectory_);
//     display_publisher.publish(display_trajectory);
//     ros::Duration(0.5).sleep(); 


//     std::cout << action << " best motion plan selected (path length: " << best_plan->second << "). Execute? (y/n): ";
//     char response;
//     std::cin >> response; // Wait for user confirmation before executing the trajectory
//     if (response == 'y' || response == 'Y') {
//         ROS_INFO("Executing %s motion...", action.c_str());
//         group.execute(best_plan->first);
//     } else {
//         ROS_WARN("%s execution cancelled.", action.c_str());
//     }
// }


};

// Initializes ROS node and interfaces, sets up the planning scene and robot,
// and executes the pick-and-place routine.
int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_pick_place_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    
    ros::Publisher display_publisher =
    nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    
    ros::Publisher planning_scene_diff_publisher =  // Publisher for coloring the adaptive voxels (diff-based planning scene)
    nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPoseReferenceFrame("base_link");
    group.setPlanningTime(15.0);

    // Slow down motion to 10% of maximum speed for safety
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.1);

    auto collision_points = getCollisionPoints();

    addCollisionPointsToScene(planning_scene_interface, collision_points);
    addEnvironmentObjects(planning_scene_interface);
    addAdaptiveVoxelCollisionObjects(planning_scene_interface, collision_points, planning_scene_diff_publisher);
    
    ros::WallDuration(1.0).sleep();

    PickAndPlace pick_and_place;
    pick_and_place.executePickAndPlace(group, display_publisher);

    ros::waitForShutdown();
    return 0;
}