#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "color_classifier");
    ros::NodeHandle nh;

    // Create a publisher to publish the color classification result
    ros::Publisher color_pub = nh.advertise<std_msgs::Int32>("/color", 10);

    // Main loop
    ros::Rate rate(10); // Publish at 10 Hz
    while (ros::ok())
    {
        VideoCapture cap(0); // capture the video from web cam

        if (!cap.isOpened()) // if not success, exit program
        {
            cout << "Cannot open the web cam" << endl;
            return -1;
        }
        namedWindow("Control", WINDOW_AUTOSIZE); // create a window called "Control"

        int iLowH = 40;
        int iHighH = 80;

        int iLowS = 40;
        int iHighS = 255;

        int iLowV = 40;
        int iHighV = 255;

        // Create trackbars in "Control" window
        cv::createTrackbar("LowH", "Control", &iLowH, 179); // Hue (0 - 179)
        cv::createTrackbar("HighH", "Control", &iHighH, 179);

        cv::createTrackbar("LowS", "Control", &iLowS, 255); // Saturation (0 - 255)
        cv::createTrackbar("HighS", "Control", &iHighS, 255);

        cv::createTrackbar("LowV", "Control", &iLowV, 255); // Value (0 - 255)
        cv::createTrackbar("HighV", "Control", &iHighV, 255);
        while (true)
        {
            Mat imgOriginal;

            bool bSuccess = cap.read(imgOriginal); // read a new frame from video

            if (!bSuccess) // if not success, break loop
            {
                cout << "Cannot read a frame from video stream" << endl;
                break;
            }

            Mat imgHSV;

            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); // Convert the captured frame from BGR to HSV

            Mat imgThresholded;

            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); // Threshold the image

            // morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

            // morphological closing (fill small holes in the foreground)
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

            imshow("Thresholded Image", imgThresholded); // show the thresholded image
            imshow("Original", imgOriginal);             // show the original image

            // Count the number of green pixels
            int greenPixels = countNonZero(imgThresholded);

            // Determine the color based on the number of green pixels
            int color = (greenPixels > 0) ? 1 : 0;

            // Create a message and set the color classification result
            std_msgs::Int32 msg;
            msg.data = color;

            // Publish the message
            color_pub.publish(msg);

            if (waitKey(30) == 27) // wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
                cout << "esc key is pressed by user" << endl;
                break;
            }

            // Spin once and sleep to maintain the desired publishing rate
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
