#include <ros/ros.h>
#include <vector>


#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//namespace plt = matplotlibcpp;

class EvaluatePose
{
    public:
        EvaluatePose();
        void Step();
	void calc();
	float diffx;
	float diffy;
	int count=0;
        std::vector<double> orbx,orby,mhex,mhey;
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
        std::cout << "### Count ###" << std::endl;
        
	std::cout << count << std::endl;
        std::cout << "### mhe_pose_ ###" << std::endl;
        std::cout << mhe_pose_.header.stamp << std::endl;
        std::cout << mhe_pose_.pose.position << std::endl;
	mhex.push_back(mhe_pose_.pose.position.x);
	mhey.push_back(mhe_pose_.pose.position.y);	
        diffx = mhe_pose_.pose.position.x-orb_pose_.pose.position.x;
        diffy = mhe_pose_.pose.position.y-orb_pose_.pose.position.y;
        std::cout << "### orb_pose_ ###" << std::endl;
        std::cout << orb_pose_.header.stamp << std::endl;
        std::cout << orb_pose_.pose.position << std::endl;
	orbx.push_back(orb_pose_.pose.position.x);
        orby.push_back(orb_pose_.pose.position.y);

        
	std::cout << "### Difference ###" << std::endl;
	std::cout << diffx << std::endl;
	
	std::cout << "###Vector ###" << std::endl;
	std::cout << mhex[count] << std::endl;
	std::cout << mhey[count] << std::endl;
	std::cout << orbx[count] << std::endl;
	std::cout << orby[count] << std::endl;
	
	count = count+1;
    }
    
    void EvaluatePose::calc(){
	float max_deviation=0;
	float avg_deviation=0;
        float sumx=0;
	float sumy=0;
	float x,y;
	std::cout << "###Vector in calc func ###" << std::endl;
        std::cout << mhex[count-1] << std::endl;
        std::cout << mhey[count-1] << std::endl;
        std::cout << orbx[count-1] << std::endl;
        std::cout << orby[count-1] << std::endl;
	for(int i=0;i<count;i++){
        	x=(mhex[i]-orbx[i])*(mhex[i]-orbx[i]);
		y=(mhey[i]-orby[i])*(mhey[i]-orby[i]);
		sumx = sumx+x;
		sumy= sumy+y;
	}
	avg_deviation=(sumx+sumy)/count;
        
        std::cout << "###Average Deviation in calc func ###" << std::endl;
        std::cout << avg_deviation << std::endl;

        std::cout << "###Final Values of mhex ###" << std::endl;
        for (int i = 0; i < mhex.size(); i++)
        	std::cout << mhex[i]<<"," << " ";
	
        std::cout << "###Final Values of mhey###" << std::endl;
	for (int i = 0; i < mhex.size(); i++)
       		std:: cout << mhey[i]<<"," << " ";

	std::cout << "###Final Values of orbx ###" << std::endl;
        for (int i = 0; i < mhex.size(); i++)
		std:: cout << orbx[i]<<"," << " ";

	std::cout << "###Final Values of orby ###" << std::endl;
        for (int i = 0; i < mhex.size(); i++)
                std:: cout << orby[i]<<"," << " ";
        


        //plt::plot(mhex,mhey);
        //plt::save();
	
   }

    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "EvaluatePose");

        EvaluatePose ep;
        ep.calc(); 
        ROS_INFO("evaluate pose is runnin...");
        return 0;
    }
