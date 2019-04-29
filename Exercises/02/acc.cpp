#include "opencv2/opencv.hpp"

using namespace cv;

int main()
{
    /*
    Mat img = imread("frame1.jpg", 0); // grayscale for simplicity
    Mat acc(img.size(), CV_64F, Scalar(0)); // all black, *double* image

    cv::accumulate(img,acc);
    cv::accumulate(img,acc);
    cv::accumulate(img,acc);
    cv::accumulate(img,acc);
    cv::accumulate(img,acc);
    Mat avg;
    acc.convertTo(avg, CV_8U, 1.0/4); // back to u8 land, divide by count
    imshow("average", avg);
    waitKey();
    */
   /*
    VideoCapture cap(0);
    Mat img; cap >> img;
    Mat acc(img.size(), CV_64F, Scalar(0)); // all black, *double* image
    
    // Create a vector of 100 random images
    for (int i = 0; i < 100; ++i)
    {
        Mat img; //inside loop! ?
        cap >> img;

        cv::accumulate(img,acc);
    }

    Mat avg;
    acc.convertTo(avg, CV_8U, 1.0/100); // back to u8 land, divide by count
    imshow("average", avg);
    waitKey();
    */
    VideoCapture cap(0); // open the default camera
    if (!cap.isOpened())  // check if we succeeded
        return -1;
    cap.set(CV_CAP_PROP_FPS, 15);

    std::vector<cv::Mat> images(100);
    uint framenumb;
    for (framenumb = 0; framenumb < 100; ++framenumb)
    {
        // this is optional, preallocation so there's no allocation
        // during capture
        images[framenumb].create(480, 640, CV_32FC3);
    }
    for (framenumb = 0; framenumb < 100; ++framenumb)
    {
        Mat frame;
        cap >> frame;
        if (frame.empty()) break; // end of video stream
        imshow("webcam", frame);
        if (waitKey(1) == 27) break; // stop capturing by pressing ESC 
        frame.copyTo(images[framenumb]);
    }

    Mat avgImg(480, 640, CV_32FC3, Scalar()); // Create and zero initialize
    for (framenumb = 0; framenumb < 100; ++framenumb)
    {
        avgImg += images[framenumb];
    }

    return 0;
}
