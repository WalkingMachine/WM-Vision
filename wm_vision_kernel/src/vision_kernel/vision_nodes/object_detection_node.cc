/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 12/07/2013
 *
 * Programmer: Julien Côté
 *
 * Description: Object detection
 *
 */

#include "../../../include/vision_kernel/vision_nodes/object_detection_node.h"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <boost/lexical_cast.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

Data ObjectDetectionNode::Function(InputData input_data) {
  std::shared_ptr<Data> data = input_data->at("input");
  Data output_data;

  if (parameters()["IsOnCUDA"] == "true") {

  } else {
	  cv::Mat img_object = cv::imread("/home/escaladeur/Pictures/voyage200.jpg", CV_LOAD_IMAGE_GRAYSCALE );
	  cv::Mat img_scene = *(data->data<cv::Mat>());
	  ROS_INFO("%d", img_object.size);
	  ROS_INFO("%d", img_scene.size);
	  //-- Step 1: Detect the keypoints using SURF Detector
	  int minHessian = 400;

	  cv::SurfFeatureDetector detector( minHessian );

	  std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

	  detector.detect( img_object, keypoints_object );
	  detector.detect( img_scene, keypoints_scene );

	  //-- Step 2: Calculate descriptors (feature vectors)
	  cv::SurfDescriptorExtractor extractor;

	  cv::Mat descriptors_object, descriptors_scene;

	  extractor.compute( img_object, keypoints_object, descriptors_object );
	  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

	  //-- Step 3: Matching descriptor vectors using FLANN matcher
	  cv::FlannBasedMatcher matcher;
	  std::vector< cv::DMatch > matches;
	  matcher.match( descriptors_object, descriptors_scene, matches );

	  double max_dist = 0; double min_dist = 100;

	  //-- Quick calculation of max and min distances between keypoints
	  for( int i = 0; i < descriptors_object.rows; i++ )
	  { double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	  }

	  printf("-- Max dist : %f \n", max_dist );
	  printf("-- Min dist : %f \n", min_dist );

	  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	  std::vector< cv::DMatch > good_matches;

	  for( int i = 0; i < descriptors_object.rows; i++ )
	  { if( matches[i].distance < 3*min_dist )
	     { good_matches.push_back( matches[i]); }
	  }

	  cv::Mat img_matches;
	  cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
	               good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	               cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	  //-- Localize the object
	  std::vector<cv::Point2f> obj;
	  std::vector<cv::Point2f> scene;

	  for( int i = 0; i < good_matches.size(); i++ )
	  {
	    //-- Get the keypoints from the good matches
	    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
	    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	  }

	  cv::Mat H = findHomography( obj, scene, CV_RANSAC );

	  //-- Get the corners from the image_1 ( the object to be "detected" )
	  std::vector<cv::Point2f> obj_corners(4);
	  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
	  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
	  std::vector<cv::Point2f> scene_corners(4);

	  perspectiveTransform( obj_corners, scene_corners, H);

	  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
	  cv::line( img_matches, scene_corners[0] + cv::Point2f( img_object.cols, 0), scene_corners[1] + cv::Point2f( img_object.cols, 0), cv::Scalar(0, 255, 0), 4 );
	  cv::line( img_matches, scene_corners[1] + cv::Point2f( img_object.cols, 0), scene_corners[2] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
	  cv::line( img_matches, scene_corners[2] + cv::Point2f( img_object.cols, 0), scene_corners[3] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
	  cv::line( img_matches, scene_corners[3] + cv::Point2f( img_object.cols, 0), scene_corners[0] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );

	  //-- Show detected matches
	  imshow( "Good Matches & Object detection", img_matches );
	  std::shared_ptr<cv::Mat> worst_variable_name_ever = std::shared_ptr<cv::Mat>(new cv::Mat());
	  *worst_variable_name_ever = img_matches;
	  output_data.set_data(worst_variable_name_ever);
  }

  return output_data;
}
