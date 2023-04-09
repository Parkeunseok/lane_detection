#pragma once

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>

using namespace cv;

struct pixel_point
{
    double x;
    double y;
};

class Detection
{
private:
    //subscriber
    ros::Subscriber subImg;
    ros::Subscriber state_sub;
    //publisher
    ros::Publisher cmd_pub;
    //data
    cv::Mat mImage;
    cv_bridge::CvImagePtr mImagePtr;
    image_transport::Publisher img_pub;

    geometry_msgs::Twist lane_cmd;
    std_msgs::String state;

    bool is_look_foward_point;

    cv::Mat cropped;
    cv::Mat out_dst;
    cv::Mat erode_dst;
    cv::Mat result;
    cv::Mat bdst;
    cv::Mat cdst;
    cv::Mat img_combine;
    Mat element = getStructuringElement(MORPH_RECT,Size(3,3),Point(-1,-1));
    std::vector<Vec4i> lines;
    std::vector<Vec4i>lines2;
    std::vector<Point> ptptr;
    std::vector<pixel_point> right_point;
    std::vector<pixel_point> left_point;
public:
    Detection();
    Detection(ros::NodeHandle node);

    void subImgCallback(const sensor_msgs::CompressedImage& subImgMsgs);
    void state_callback(const std_msgs::String &msg);
    void run();
    void iamge_transform(cv::Point2f* src_vertices, cv::Point2f* dst_vertices, cv::Mat& ip_image, cv::Mat& ot_image);
    void angle(int x, int y);
    cv::Mat shadow_processing(cv::Mat src);
    Mat filter_colors(Mat src);
    Mat bird_view(Mat &src);
    bool check;
};
