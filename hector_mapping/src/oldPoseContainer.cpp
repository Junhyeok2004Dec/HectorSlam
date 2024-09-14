#include "PoseInfoContainer.h"
std::<vector> data;

void saveToCsv(const std::string& filePath, const std::vector<std::string>& data) {
    std::ofstream outFile(filePath, std::ios::app);
    
    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open file " << filePath << std::endl;
        return;
    }
    
    for (size_t i = 0; i < data.size(); ++i) {
        outFile << data[i];
        if (i != data.size() - 1) { 
            outFile << ",";
        }
    }
    outFile << std::endl; // EOL
    
    outFile.close();
}


void PoseInfoContainer::update(const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov, const ros::Time& stamp, const std::string& frame_id)
{
  
  //Fill stampedPose
  std_msgs::Header& header = stampedPose_.header;
  header.stamp = stamp;
  header.frame_id = frame_id;



  geometry_msgs::Pose& pose = stampedPose_.pose; 
  //ROS_INFO("pose = %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);

  pose.position.x = slamPose.x();
  pose.position.y = slamPose.y();

  //data = new std::vector{pose.position.x, pose.position.y, pose.position.z}  ;

  pose.orientation.w = cos(slamPose.z()*0.5f);
  pose.orientation.z = sin(slamPose.z()*0.5f);
  saveToCsv("test.csv", data );

  //Fill covPose
  //geometry_msgs::PoseWithCovarianceStamped covPose;
  covPose_.header = header;
  covPose_.pose.pose = pose;

  boost::array<double, 36>& cov(covPose_.pose.covariance);

  cov[0] = static_cast<double>(slamCov(0,0));
  cov[7] = static_cast<double>(slamCov(1,1));
  cov[35] = static_cast<double>(slamCov(2,2));

  double xyC = static_cast<double>(slamCov(0,1));
  cov[1] = xyC;
  cov[6] = xyC;

  double xaC = static_cast<double>(slamCov(0,2));
  cov[5] = xaC;
  cov[30] = xaC;

  double yaC = static_cast<double>(slamCov(1,2));
  cov[11] = yaC;
  cov[31] = yaC;

  //Fill tf tansform
  tf::poseMsgToTF(pose, poseTransform_);
}