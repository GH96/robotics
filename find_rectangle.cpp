#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int main()
{
    cv::Mat input = cv::imread("frame2.jpg");
    //cv::Mat input = cv::imread("grid.png");

    // convert to grayscale (you could load as grayscale instead)
    cv::Mat gray;
    cv::cvtColor(input, gray, CV_BGR2GRAY);

    // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
    cv::Mat mask;
    cv::threshold(gray, mask, 100, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

    // find contours (if always so easy to segment as your image, you could just add the black/rect pixels to a vector)
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cout<<"contours.size() "<<contours.size()<<endl;

    /// Draw contours and find biggest contour (if there are other contours in the image, we assume the biggest one is the desired rect)
    // drawing here is only for demonstration!
    int biggestContourIdx = -1;
    float biggestContourArea = 0;
    std::vector<cv::Point> approx; //to detect only rect
    cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar(0, 100, 0);
        drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );

        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.01, true); //Approx a polygonal curve with less vertices, Douglas-Peucker algorithm
        cout<<approx.size()<<endl;
        if (approx.size() == 4 ) //&& (approx.size() <= 6)
        {
            float ctArea = cv::contourArea(contours[i]);
            cout<<"ctArea "<<ctArea<<endl;
            if(ctArea > biggestContourArea)
            {
                biggestContourArea = ctArea;
                biggestContourIdx = i;
                //bounding_rect = boundingRect( contours[i] ); // Find the bounding rectangle for biggest contour
            }
        }
    }

    // if no contour found
    if(biggestContourIdx < 0)
    {
        std::cout << "no contour found" << std::endl;
        return 1;
    }
    else
    {
        drawContours( input, contours, biggestContourIdx, Scalar( 0, 255, 0 ), 2 ); // Draw the largest contour using previously stored index.
        imshow( "result", input );
        waitKey(0);
    }

    /*
    // compute the rotated bounding rect of the biggest contour! (this is the part that does what you want/need)
    cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
    // one thing to remark: this will compute the OUTER boundary box, so maybe you have to erode/dilate if you want something between the ragged lines

    // draw the rotated rect
    cv::Point2f corners[4];
    boundingBox.points(corners);
    cv::line(drawing, corners[0], corners[1], cv::Scalar(255,255,255));
    cv::line(drawing, corners[1], corners[2], cv::Scalar(255,255,255));
    cv::line(drawing, corners[2], corners[3], cv::Scalar(255,255,255));
    cv::line(drawing, corners[3], corners[0], cv::Scalar(255,255,255));

    // display
    cv::imshow("input", input);
    cv::imshow("drawing", drawing);
    cv::waitKey(0);

    //cv::imwrite("rotatedRect.png",drawing);
    */

    return 0;
}
