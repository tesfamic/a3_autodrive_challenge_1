#ifndef LANEDETECTOR_DETECTOR_H
#define LANEDETECTOR_DETECTOR_H

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include "aggies_msgs/PathData.h"
#include "aggies_msgs/ControlData.h"
#include "aggies_lib/aggies_param.h"
#include "aggies_lib/aggies_util.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>

struct BoxData{
    int cor_x;
    int cor_y;    
    int avg_pt;
    int pix_count;
    int box_width;
    std::vector<cv::Point2i> pixel_pos;
};

class Detector{
    private:
        // ROS node handle and image transport objects
        ros::NodeHandle nh_;   
        image_transport::ImageTransport it_; 
        ros::Subscriber image_sub_;
        image_transport::Publisher img_with_line_pub_;        
        ros::Publisher center_lane_pub_;
        ros::Publisher lane_left_pub_;
        ros::Publisher lane_right_pub_;
        

        cv::Mat image_;
        cv::Mat img_warped_;
        cv::Mat img_gray_;
        cv::Mat img_gabor_;
        cv::Mat img_filtered_;    
        cv::Mat img_final_;

        int num_of_frames_;  
        UtilMethods util_;

        cv::Mat trans_matrix_;
        cv::Mat inv_trans_matrix_;
        cv::Mat gabor_kernel_;
        cv::Mat erosion_kernel_;


    public:
        Detector();
        ~Detector();

        void ImageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg);

        void InitializeParams();
        void PreprocessImage();
        void AdjustStartPoints(int& argmax_left,int& argmax_right);
        void GetLinePixels(int argmax_left,int argmax_right,
                           std::vector<cv::Point>& l_line_px,
                           std::vector<cv::Point>& r_line_px,
                           std::vector<BoxData>& l_boxes,
                           std::vector<BoxData>& r_boxes);
        void AnalyzeDetection(std::vector<BoxData>& l_boxes,
                                std::vector<BoxData>& r_boxes,
                                bool& l_detected, bool& r_detected,
                                int& l_empty, int& r_empty,
                                cv::Scalar& l_col,cv::Scalar& r_col);
        void AdjustLine(std::vector<cv::Point>& l_pixs,
                          std::vector<cv::Point>& r_pixs,bool left,bool right,
                          std::vector<cv::Point>& l_line,
                          std::vector<cv::Point>& r_line);
        void CorrectParallel(std::vector<cv::Point>& l_line,
                               std::vector<cv::Point>& r_line,
                               std::vector<cv::Point>& l_pix,
                               std::vector<cv::Point>& r_pix,
                               int l_empty,int r_empty);
        
        void PlotLine(std::vector<cv::Point>& line,cv::Scalar color);

        void SubtractAverageIntensity(cv::Mat& img);

        void ApplyImageFiltering();
        void FilterImage(std::vector<cv::Point2i> &indices);
        void GetZeroIndices(int thresh,std::vector<cv::Point2i> &zero_indices);
        void GetSelectIndices(std::vector<cv::Point2i> &indices,
                                          std::vector<cv::Point2i> &selected_indices);
        void GetFinalIndices(std::vector<cv::Point2i> &indices, int thresh, 
                            std::vector<cv::Point2i> &final_indices);
        void GetLineStartPoints(int& left,int& right);
        void FilterLinePixels(int l_start,
                      std::vector<cv::Point2i>& line_pixels, std::vector<BoxData>& line_boxes);
        void FitPolynomial(std::vector<cv::Point2i> &line_pixels, std::vector<cv::Point2i> &line);
        bool CompareCorrectLineError(std::vector<cv::Point2i> &prev_l,
                                std::vector<cv::Point2i> &cur_l,
                                int &error_limit);
        void GetCorrectedLine(cv::Mat &img_filt,std::vector<cv::Point2i> &ref_line,std::vector<cv::Point2i> line);
    
};
#endif //LANEDETECTOR_DETECTOR_H