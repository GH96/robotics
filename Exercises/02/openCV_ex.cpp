/*
#include <opencv2/opencv.hpp>
using namespace cv;
#pragma comment(lib, "opencv_world331.lib") // or whatever version you're using...

#include <iostream>
using namespace std;


int main(void)
{
    Mat master = imread("frame1.jpg");
    Mat current = imread("frame2.jpg");

    if (master.empty() || current.empty())
    {
        cout << "Read error" << endl;
        return -1;
    }

    Mat a;

    absdiff(master, current, a); // absolute difference
 
    imshow("a", a);

    waitKey();

    return 0;
}
*/

#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;
int main() {

    Mat img1 = imread("frame1.jpg");
    Mat img2 = imread("frame2.jpg");

    // calc the difference
    Mat diff;
    absdiff(img1, img2, diff);

    // Get the mask if difference greater than th
    int th = 10;  // 0
    Mat mask(img1.size(), CV_8UC1);
    for(int j=0; j<diff.rows; ++j) {
        for(int i=0; i<diff.cols; ++i){
            cv::Vec3b pix = diff.at<cv::Vec3b>(j,i);
            int val = (pix[0] + pix[1] + pix[2]);
            if(val>th){
                mask.at<unsigned char>(j,i) = 255;
            }
        }
    }

    // get the foreground
    Mat res;
    bitwise_and(img1, img2, res, mask);

    // display
    imshow("res", res);
    waitKey();
    return 0;
}
