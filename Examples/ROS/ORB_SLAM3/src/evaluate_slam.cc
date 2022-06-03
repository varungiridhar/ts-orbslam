#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class EvaluatePose
{
    public:
        EvaluatePose();
        void Step();
    private:
        
        geometry_msgs::PoseStamped mhe_pose_;
        geometry_msgs::PoseStamped orb_pose_;
        ros::NodeHandle nh_;
        ros::Subscriber mhe_pose_sub_, orb_pose_sub_;
        ros::Subscriber odom_sub_; 
        void UpdateMHEPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
        void UpdateORBSLAMPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
};
    EvaluatePose::EvaluatePose()
    {
        ros::Rate rate(2);

        mhe_pose_sub_ = nh_.subscribe("/terrasentia/mhe_pose", 100, &EvaluatePose::UpdateMHEPose, this);
        orb_pose_sub_ = nh_.subscribe("/orb_pose", 100, &EvaluatePose::UpdateORBSLAMPose, this); 
        
        while (ros::ok()) {
            EvaluatePose::Step();
            rate.sleep();
            ros::spinOnce();   
        }    
    }    

    void EvaluatePose::UpdateMHEPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) 
    {
        mhe_pose_.header = pose_msg->header;
        mhe_pose_.pose = pose_msg->pose; 
    }
    
    void EvaluatePose::UpdateORBSLAMPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
    {
        orb_pose_.header = pose_msg->header;
        orb_pose_.pose = pose_msg->pose;
    
    }

    void EvaluatePose::Step() {
        
        std::cout << "### mhe_pose_ ###" << std::endl;
        std::cout << mhe_pose_.header.stamp << std::endl;       
        std::cout << mhe_pose_.pose.position << std::endl;
         
        std::cout << "### orb_pose_ ###" << std::endl;
        std::cout << orb_pose_.header.stamp << std::endl;       
        std::cout << orb_pose_.pose.position << std::endl;
        
    }
    
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "EvaluatePose");

        EvaluatePose ep; 

        ROS_INFO("evaluate pose is runnin...");


        return 0;
    }

