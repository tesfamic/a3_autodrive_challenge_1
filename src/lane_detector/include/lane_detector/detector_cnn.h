
#ifndef LANEDETECTOR_DETECTORCNN_H_
#define LANEDETECTOR_DETECTORCNN_H_

#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/dnn.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include "aggies_msgs/PathData.h"
#include "aggies_lib/aggies_param.h"
#include "aggies_lib/aggies_util.h"
#include <fstream>

class CnnDetector{
    private:
        // ROS node handle and image transport objects
        ros::NodeHandle nh_;    
        ros::Subscriber image_sub_;
        ros::Subscriber traj_sub_;
        ros::Subscriber left_bound_sub_;
        ros::Subscriber right_bound_sub_;
        ros::Publisher llane_data_pub_;
        ros::Publisher rlane_data_pub_;
        ros::Publisher traj_shift_pub_;

        image_transport::ImageTransport it_;
        //ros::Subscriber gps_sub_;
        //ros::Subscriber imu_sub_;

        int img_w_;
        int img_h_;

        cv::Mat image_;
        cv::Mat image_org_;
        int num_of_frames_;    
        cv::Mat trans_matrix_;
        cv::Mat inv_trans_matrix_;
        int img_scale_;

        aggies_msgs::PathData traj_ref_;
        aggies_msgs::PathData left_b_,right_b_;
        cv::dnn::Net net;        

        std::vector<std::vector<cv::Point>> pts_in_row_ipm_;
        std::vector<cv::Point2d> cur_pts_;
        std::vector<std::vector<cv::Point>> pts_in_row_;

        std::ofstream t_file_;

        UtilMethods util_;

        double azimuth_, yaw_;
        //ExtendedWaypoint current_loc_;
        const unsigned int MAX_PTS = 40;
        std::vector<int> map_left_x, map_left_y, map_right_x, map_right_y;
        Eigen::VectorXd left_ref_,right_ref_;

    public:
        struct LaneD{
            int r_st;
            int c_st;
        };
        CnnDetector();
        ~CnnDetector();

        void ImageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg);
        void TrajCallback(const aggies_msgs::PathData::ConstPtr& traj_msg);
        void LeftBoundCallback(const aggies_msgs::PathData::ConstPtr& lmsg);
        void RightBoundCallback(const aggies_msgs::PathData::ConstPtr& rmsg);

        Eigen::VectorXd GetCurve(std::vector<cv::Point>& ref_line);
        std::vector<int> GetAnchors(int ref_anch);
        std::vector<Eigen::VectorXd> GetOtherCurves(int ref_anch,std::vector<int> anchs,Eigen::VectorXd pol_ref);
        void ToPerspPoly(Eigen::VectorXd coeff,std::vector<cv::Point>& p_pts);

        //void UpdateLane();
        //void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu);
        //void GpsCallback(const novatel_gps_msgs::NovatelPosition msg);
        void ConvertToImageCoord();
        void CalculateTheirCurveFit();
        void ToImageCoord(std::vector<ExtendedWaypoint>&pts,std::vector<int>& x,std::vector<int> &y);
        void PlotToImage(cv::Mat &img);
        //void CalculateIndex();

        cv::Point2d GetAveragePoints(cv::Mat& img,int r_st,int c_st);
        cv::Point2d GetCenterPoint(cv::Mat& img);
        void GetLaneMarkings(cv::Mat& image);
        void GetLines(std::vector<std::vector<cv::Point> > &ret_pts);
        bool NearestPoints(cv::Point pi,
                            const std::vector<cv::Point> &row_pts,
                            std::vector<cv::Point> &near_pts);
        bool NearestPoint(cv::Point pi, const std::vector<cv::Point> &row_pts,
                                cv::Point &near_pt);
        void InitializeParams();

        Eigen::VectorXd Polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order);
        double Polyeval(Eigen::VectorXd coeffs, double x);

       
};
#endif //LANEDETECTOR_DETECTORCNN_H_
