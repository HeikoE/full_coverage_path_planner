#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Trigger.h"
#include "full_coverage_path_planner/spiral_stc.h"

class PlannerWrapper {
private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    full_coverage_path_planner::SpiralSTC planner_;
    std::string global_frame_id_;
    std::string robot_base_frame_id_;
    ros::ServiceServer service_;

public:
    PlannerWrapper() : nh_("~") {
        // Initialize frame ids with ROS parameters
        nh_.param("global_frame_id", global_frame_id_, std::string("map"));
        nh_.param("robot_base_frame_id", robot_base_frame_id_, std::string("base_link"));

        // Initialize the planner
        planner_.initialize("spiral_stc_planner", nullptr);

        // Advertise the service
        service_ = nh_.advertiseService("trigger_planning", &PlannerWrapper::planCallback, this);
    }

    bool getCurrentPose(geometry_msgs::PoseStamped& current_pose) {
        tf::StampedTransform transform;
        try {
            tf_listener_.lookupTransform(global_frame_id_, robot_base_frame_id_, ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        current_pose.header.stamp = ros::Time::now();
        current_pose.header.frame_id = global_frame_id_;
        tf::poseTFToMsg(transform, current_pose.pose);

        return true;
    }

    bool planCallback(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res) {
        geometry_msgs::PoseStamped current_pose;
        if (!getCurrentPose(current_pose)) {
            res.success = false;
            res.message = "Failed to get current pose";
            return false;
        }


        geometry_msgs::PoseStamped goal_pose; 
        std::vector<geometry_msgs::PoseStamped> plan;
        if (!planner_.makePlan(current_pose, goal_pose, plan)) {
            res.success = false;
            res.message = "Failed to make plan";
            return false;
        }

        // Optionally, do something with the plan here
        ROS_INFO("Number of poses planned:" ,plan.size());

        res.success = true;
        res.message = "Planning successful";
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "planner_wrapper_node");
    PlannerWrapper wrapper;
    ros::spin();
    return 0;
}