#ifndef RAI_OPENCV
  #define RAI_OPENCV 1
#endif


#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/roscom.h>
#include <Kin/frame.h>

void minimal_use(){

  Var<byteA> rgb;
  Var<floatA> depth;

#if 0 //using ros
  RosCom ROS;
  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb(rgb, "/camera/rgb/image_rect_color");
//  SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA> subDepth(depth, "/camera/depth_registered/image_raw");
#else //using a webcam
  OpencvCamera cam(rgb);
#endif

  //looping images through opencv
  for(uint i=0;i<100;i++){
    cv::Mat img = CV(rgb.get());
    if(img.total()>0){
      cv::imshow("RGB", img);
      cv::waitKey(1);
    }
    rai::wait(.1);
  }
}

//void get_objects_into_configuration(){
//  RosCom ROS;

//  Var<byteA> rgb;
//  Var<floatA> depth;

//  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb(rgb, "/camera/rgb/image_rect_color");
//  SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA> subDepth(depth, "/camera/depth_registered/image_raw");


//  Depth2PointCloud d2p(depth, 1.);

//  rai::KinematicWorld C;
//  C.addFile("model.g");
//  rai::Frame *pcl = C.addFrame("pcl", "camera", "shape:pointCloud");
//  for(uint i=0;i<100;i++){
////    cout <<d2p.points.get()->N <<endl;
//    pcl->shape->mesh().V = d2p.points.get();
//    pcl->shape->mesh().V.reshape(640*480,3);
//    C.watch(false);
//    rai::wait(.1);
//  }

//  rai::wait();
//}

/*
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
*/
#include "opencv2/opencv.hpp"
using namespace cv;

Mat3b getMean(const vector<Mat3b>& images) // average
{
    if (images.empty()) return Mat3b();

    // Create a 0 initialized image to use as accumulator
    Mat m(images[0].rows, images[0].cols, CV_64FC3);
    m.setTo(Scalar(0,0,0,0));

    // Use a temp image to hold the conversion of each input image to CV_64FC3
    // This will be allocated just the first time, since all your images have
    // the same size.
    Mat temp;
    for (int i = 0; i < images.size(); ++i)
    {
        // Convert the input images to CV_64FC3 ...
        images[i].convertTo(temp, CV_64FC3);

        // ... so you can accumulate
        m += temp;
    }

    // Convert back to CV_8UC3 type, applying the division to get the actual mean
    m.convertTo(m, CV_8U, 1. / images.size());
    return m;
}

void red_circle(){

  Var<byteA> rgb;
  Var<floatA> depth;

#if 0 //using ros
  RosCom ROS;
  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb(rgb, "/camera/rgb/image_rect_color");
//  SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA> subDepth(depth, "/camera/depth_registered/image_raw");
#else //using a webcam
  OpencvCamera cam(rgb);
#endif

  //cv::Mat img = CV(rgb.get());
  //cv::Mat avg_img = CV(rgb.get());

  //cv::accumulate(img, avg_img); //No!
  //cout<<"+";

  vector<Mat3b> images;

  //avg_img.convertTo(avg_img, CV_32F);
  //cv::Mat avg_img = cv::Mat::zeros(img.size(), CV_32F); //larger depth to avoid saturation
  cv::Mat img;
  for(uint i=0;i<10;i++){
    //randu(img, Scalar(0), Scalar(256));
    img = CV(rgb.get());
    
    if(img.total()>0){
    //cout<<"in if<<";

    //cv::accumulate(img, avg_img);
    //avg_img = avg_img/2;
    //img.convertTo(img, CV_32F);
    //avg_img.convertTo(avg_img, CV_32F);
    //avg_img += img;
    //cv::accumulate(img, avg_img); //No!
    
    //cv::imshow("avg_img", avg_img);
    //cv::waitKey(1);

    images.push_back(img);
    cv::imshow("img", img);
    cv::waitKey(1);

    }
    rai::wait(.1);
  }

  // Compute the mean
  Mat3b meanImage = getMean(images);

  // Show result
  imshow("Mean image", meanImage); //not working...??
  waitKey();

  //looping images through opencvmake
  for(uint i=0;i<100;i++){
    cv::Mat img = CV(rgb.get());
    //cout<<"start<<";
    if(img.total()>0){
      //cv::imshow("RGB", img);
      //cv::waitKey(1);
      
      //cout<<"in if";   
      
      cv::Mat bgr_image;
      bgr_image = img;

      cv::Mat orig_image = bgr_image.clone(); 

      cv::medianBlur(bgr_image, bgr_image, 3);
      //cv::GaussianBlur(bgr_image, bgr_image, cv::Size(9, 9), 2, 2);

      // Convert input image to GRAY
      cv::Mat gray_sc;
      cv::cvtColor(bgr_image, gray_sc, cv::COLOR_BGR2GRAY); //GRAY
      //cv::imshow("gray_sc", gray_sc);
      //cv::waitKey(1);

      // Convert input image to HSV
      cv::Mat hsv_image;
      cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV); //HSV
      
      // Threshold the HSV image, keep only the red pixels
      cv::Mat lower_red_hue_range;
      cv::Mat upper_red_hue_range;
      cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
      cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

      // Combine the above two images
      cv::Mat red_hue_image;
      cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
      
      //cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
	    //cv::imshow("Combined threshold images", red_hue_image);
      //cv::waitKey(1);

      // add blur
      cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(3, 3), 2, 2); // small is better
      cv::imshow("red_hue_image + GaussianBlur", red_hue_image);
      cv::waitKey(1);
      
	    // Use the Hough transform to detect circles in the combined threshold image
      std::vector<cv::Vec3f> circles;
      //cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/4, 100, 20, 5, red_hue_image.rows/4);
      //cv::HoughCircles(grey_sc, circles, CV_HOUGH_GRADIENT, 1, grey_sc.rows/4, 100, 20, 0, grey_sc.rows/4);
      cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);
      
      // Loop over all detected circles and outline them on the original image
      //if(circles.size() == 0) std::exit(-1); //will exit
      for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
        int radius = std::round(circles[current_circle][2]);

        cv::circle(orig_image, center, radius, cv::Scalar(0, 255, 0), 5);
      }
      
      //cout<<"circle<<";
      cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
	    cv::imshow("Detected red circles on the input image", orig_image);
      cv::waitKey(1);
      

    }
    rai::wait(.1);
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  //minimal_use();
  red_circle(); // ex1, any segments!
  //track_change(); // ex2

  return 0;
}
