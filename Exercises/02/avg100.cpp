#include "opencv2/opencv.hpp"

using namespace cv;

Mat3b getMean(const vector<Mat3b>& images)
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

int main()
{
    VideoCapture cap(0);
    // Create a vector of 100 random images
    vector<Mat3b> images;
    for (int i = 0; i < 100; ++i)
    {
        //Mat3b img(598, 598);
        //randu(img, Scalar(0), Scalar(256));

        //cv::Mat img = CV(rgb.get());
        Mat img; //inside loop!?
        cap >> img;

        images.push_back(img);
    }

    // Compute the mean
    Mat3b meanImage = getMean(images);

    // Show result
    imshow("Mean image", meanImage);
    waitKey();
    

    return 0;
}
