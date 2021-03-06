/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2016
 *      Author: sleutene
 */

#include <arp/Frontend.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

namespace arp {

Frontend::~Frontend() {
  if(camera_ != nullptr) {
    delete camera_;
  }
}

void Frontend::setCameraParameters(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2)
{
  if(camera_ != nullptr) {
    delete camera_;
  }
  camera_ = new arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>(
          imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV, arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2));
  camera_->initialiseUndistortMaps();
}

// the undistorted camera model used for the estimator (later)
arp::cameras::PinholeCamera<arp::cameras::NoDistortion> 
    Frontend::undistortedCameraModel() const {
  assert(camera_);
  return camera_->undistortedPinholeCamera();
}

int Frontend::detect(const cv::Mat& image, DetectionVec & detections)
{
  // TODO: implement
  cv::Mat greyimg;
  cv::cvtColor(image, greyimg, CV_BGR2GRAY);
  
  cv::Mat destimg;
  camera_->undistortImage(greyimg, destimg);
 
  std::vector<AprilTags::TagDetection> aprilTags = tagDetector_.extractTags(destimg);

  int cnt=0;
  for(int i=0;i!=aprilTags.size();i++){
	  AprilTags::TagDetection currTag=aprilTags[i];
	  int id=currTag.id;
	  if (idToSize_.find(id)!=idToSize_.end()){
		    Detection newdetect;
		    newdetect.T_CT=kinematics::Transformation(currTag.getRelativeTransform(idToSize_[id],camera_->undistortedPinholeCamera().focalLengthU(),
		    camera_->undistortedPinholeCamera().focalLengthV(),camera_->undistortedPinholeCamera().imageCenterU(),camera_->undistortedPinholeCamera().imageCenterV()));
		    newdetect.points<<currTag.p[0].first,currTag.p[1].first,currTag.p[2].first,currTag.p[3].first,
								currTag.p[0].second,currTag.p[1].second,currTag.p[2].second,currTag.p[3].second;
			newdetect.id=id;
			detections.push_back(newdetect);
			cnt++;
	
	}
		
  }
  
  //throw std::runtime_error("not implemented");
  return cnt; // TODO: number of detections...
}

bool Frontend::setTarget(unsigned int id, double targetSizeMeters) {
  idToSize_[id] = targetSizeMeters;
  return true;
}

}  // namespace arp

