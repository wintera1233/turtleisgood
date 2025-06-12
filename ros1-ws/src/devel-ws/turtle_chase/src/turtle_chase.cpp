
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <vector>
#include <turtlesim/Spawn.h>


struct Pose {
    double x;
    double y;
    double theta;
};

Pose turtleA_pose, turtleB_pose;
bool turtleA_pose_ready = false, turtleB_pose_ready = false;

void turtleACallback(const turtlesim::Pose::ConstPtr& msg) {
    turtleA_pose = {msg->x, msg->y, msg->theta};
    turtleA_pose_ready = true;
}

void turtleBCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtleB_pose = {msg->x, msg->y, msg->theta};
    turtleB_pose_ready = true;
}

// 將 vx, vy 經轉移方程式產生 vx', vy'
void applyTransform(double vx, double vy, double& vx_out, double& vy_out) {
    vx_out = 0.8 * vx + 0.5 * vy;
    vy_out = 0.2 * vx + 0.5 * vy;
}

// 將 vx', vy' 投影到差速控制: linear.x, angular.z
void omniToDiffDrive(double vx, double vy, double theta, geometry_msgs::Twist& cmd) {
    double v_forward = vx * cos(theta) + vy * sin(theta);

    double desired_theta = atan2(vy, vx);
    double angle_error = desired_theta - theta;
    angle_error = atan2(sin(angle_error), cos(angle_error)); // normalize

    double Kp_ang = 2.0;
    cmd.linear.x = v_forward;
    cmd.angular.z = Kp_ang * angle_error;
}

void spawnTurtle2(ros::NodeHandle& nh) {
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    srv.request.x = 1.0;
    srv.request.y = 1.0;
    srv.request.theta = 0.0;
    srv.request.name = "turtle2";
    if (spawn_client.call(srv)) {
        ROS_INFO("Spawned turtle2 at (1.0, 1.0, 0.0)");
    } else {
        ROS_ERROR("Failed to spawn turtle2");
    }
}

// P 控制器
geometry_msgs::Twist goToTarget(const Pose& current, const Pose& target) {
    geometry_msgs::Twist cmd;
    double dx = target.x - current.x;
    double dy = target.y - current.y;

    double distance = sqrt(dx*dx + dy*dy);
    double angle_to_target = atan2(dy, dx);
    double angle_error = angle_to_target - current.theta;
    angle_error = atan2(sin(angle_error), cos(angle_error));

    double K_lin = 1.0;
    double K_ang = 4.0;
    cmd.linear.x = std::min(K_lin * distance, 1.5);
    cmd.angular.z = K_ang * angle_error;
    return cmd;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_square_chase");
    ros::NodeHandle nh;

    ros::Publisher pubA = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pubB = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);

    ros::Subscriber subA = nh.subscribe("/turtle1/pose", 10, turtleACallback);
    ros::Subscriber subB = nh.subscribe("/turtle2/pose", 10, turtleBCallback);

    ros::Rate rate(20);
    
    spawnTurtle2(nh);

    std::vector<Pose> square_points = {
        {5.0, 5.0},
        {7.5, 5.0},
        {7.5, 7.5},
        {5.0, 7.5},
        {5.0, 5.0}
    };

    size_t target_index = 0;
    ros::Time start_time = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();
        if (!turtleA_pose_ready || !turtleB_pose_ready) {
            rate.sleep();
            continue;
        }

        ros::Duration elapsed = ros::Time::now() - start_time;
        geometry_msgs::Twist cmdA, cmdB;

        // === turtle A 的路徑控制 ===
        Pose target = square_points[target_index];
        double dx = target.x - turtleA_pose.x;
        double dy = target.y - turtleA_pose.y;
        double distance = sqrt(dx*dx + dy*dy);

        if (distance < 0.1) {
            ROS_INFO("Reached corner %lu: Current=(%.2f, %.2f), Target=(%.2f, %.2f)",
                     target_index, turtleA_pose.x, turtleA_pose.y, target.x, target.y);
            target_index++;
            if (target_index >= square_points.size()) {
                cmdA.linear.x = 0;
                cmdA.angular.z = 0;
                pubA.publish(cmdA);
                ROS_INFO("Finished square path.");
                break;
            }
        } else {
            // 理想 vx, vy
            double vx = dx;
            double vy = dy;
            // normalize
            double mag = sqrt(vx*vx + vy*vy);
            vx /= mag;
            vy /= mag;

            // apply transform
            double vx_out, vy_out;
            applyTransform(vx, vy, vx_out, vy_out);

            // convert to diff-drive
            omniToDiffDrive(vx_out * 1.0, vy_out * 1.0, turtleA_pose.theta, cmdA);
        }

        pubA.publish(cmdA);

        // === turtle B 的追擊邏輯 ===
        if (elapsed.toSec() > 10.0) {
            double dxB = turtleA_pose.x - turtleB_pose.x;
            double dyB = turtleA_pose.y - turtleB_pose.y;
            double distB = sqrt(dxB*dxB + dyB*dyB);
            if (distB < 0.03) {
                ROS_INFO("Turtle B caught up to A. Distance: %.4f", distB);
                cmdB.linear.x = 0;
                cmdB.angular.z = 0;
            } else {
                Pose targetA = turtleA_pose;
                cmdB = goToTarget(turtleB_pose, targetA);
            }
            pubB.publish(cmdB);
        }

        rate.sleep();
    }

    return 0;
}