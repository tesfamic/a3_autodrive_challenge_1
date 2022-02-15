#include "detector_cnn.h"
// #include "map_data.h"

CnnDetector::CnnDetector():it_(nh_){
  //// subscriber
  image_sub_ = nh_.subscribe("/camera2/image/compressed", 1,&CnnDetector::ImageCallback, this);    
  traj_sub_ = nh_.subscribe("/trajectory",1,&CnnDetector::TrajCallback,this);
  left_bound_sub_ = nh_.subscribe("/left_boundary",1,&CnnDetector::LeftBoundCallback,this);
  right_bound_sub_ = nh_.subscribe("/right_boundary",1,&CnnDetector::RightBoundCallback,this);

  //// advertise the lane info
  llane_data_pub_ = nh_.advertise<aggies_msgs::PathData>("/left_lane",2);
  rlane_data_pub_ = nh_.advertise<aggies_msgs::PathData>("/right_lane",2);
  
  ///// load trained model
  std::string model_xml = "lane_model.xml";
  std::string model_bin = "lane_model.bin";
  net = cv::dnn::Net::readFromModelOptimizer(model_xml,model_bin);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);//OPENCL);

  trans_matrix_ = cv::Mat::zeros(3,3,CV_32F);
  inv_trans_matrix_ = cv::Mat::zeros(3,3,CV_32F);

  img_scale_ = 1.0;
  InitializeParams();
  num_of_frames_ = 0;
  azimuth_ = 0.0;
  yaw_ = 0.0;
  
}
CnnDetector::~CnnDetector(){
  // t_file_.close();
}
void CnnDetector::TrajCallback(const aggies_msgs::PathData::ConstPtr &traj_msg){
  ////
}
void CnnDetector::LeftBoundCallback(const aggies_msgs::PathData::ConstPtr& lmsg){
  /////
}
void CnnDetector::RightBoundCallback(const aggies_msgs::PathData::ConstPtr& rmsg){
   /////
}
void CnnDetector::ImageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg){
    //cv bridge converts ros message image into opencv format
    cv_bridge::CvImagePtr cv_ptr;
    try{
        //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        image_ = cv::imdecode(cv::Mat(img_msg->data),1);
        //cv::resize(image_,image_org_,cv::Size(1024,768));
    }catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if(img_scale_==1.0){
      cv::resize(image_,image_,cv::Size(1024,768));
    }else{
      cv::resize(image_,image_,cv::Size(512,384));
    }    
    cv::Mat img_warped;// = cv::Mat::zeros(image_.size(),image_.type());
    cv::warpPerspective(image_,img_warped,trans_matrix_,image_.size());

    img_w_ = image_.cols;
    img_h_ = image_.rows;
    num_of_frames_ ++;

    //Lane markings
    GetLaneMarkings(image_);

    //transform points to IPM
    std::vector<cv::Point2d> ipm;
    if(cur_pts_.size()>0){
        cv::perspectiveTransform(cur_pts_,ipm,trans_matrix_);
        for(auto& p:ipm){
          cv::circle(img_warped,cv::Point(int(p.x),int(p.y)),1,cv::Scalar(10,20,250),2);
        }
    }
    
    ///////Load map info /////
    // ConvertToImageCoord();
    // CalculateTheirCurveFit();
    ///////////////////////////////////////
    ///// code logic starts here
    /////
    cv::resize(img_warped,img_warped,cv::Size(int(img_warped.cols/2),int(img_warped.rows/2)));
    cv::imshow("top view",img_warped);

    std::cout<<"\n frame #"<<num_of_frames_<<"\n";
    cv::waitKey(1);

}

Eigen::VectorXd CnnDetector::GetCurve(std::vector<cv::Point>& ref_line){

    Eigen::VectorXd x(ref_line.size()),y(ref_line.size());
    Eigen::VectorXd ref_coef;
    
    return ref_coef;
}

std::vector<int> CnnDetector::GetAnchors(int ref_anch){
    std::vector<int> anch_out;
    
    return anch_out;
}

std::vector<Eigen::VectorXd> CnnDetector::GetOtherCurves(int ref_anch,std::vector<int> anchs,Eigen::VectorXd pol_ref){
    std::vector<Eigen::VectorXd> pols;      
        
    return pols;
}

void CnnDetector::ToPerspPoly(Eigen::VectorXd coeff,std::vector<cv::Point>& p_pts){
    /////
}

void CnnDetector::ConvertToImageCoord(){
  //////////
}

void CnnDetector::CalculateTheirCurveFit(){
    //////
}

void CnnDetector::PlotToImage(cv::Mat& img){
    /////
}
void CnnDetector::ToImageCoord(std::vector<ExtendedWaypoint>& pts,
                                std::vector<int>& x,
                                std::vector<int>& y){
   ////////
}

void CnnDetector::GetLaneMarkings(cv::Mat &image){
    
    /////
}

void CnnDetector::GetLines(std::vector<std::vector<cv::Point> > &ret_pts){
  
}

bool CnnDetector::NearestPoint(cv::Point pi, const std::vector<cv::Point> &row_pts,cv::Point &near_pt){
  double min_dist = std::numeric_limits<double>::max();
  double min_ang = M_PI;
  
  return false;
}
bool CnnDetector::NearestPoints(cv::Point pi,
                                const std::vector<cv::Point> &row_pts,
                                std::vector<cv::Point> &near_pts){

  //////////
  return near_pts.size()>0;
}

void CnnDetector::InitializeParams(){
  cv::Point2f inPt[4];
  cv::Point2f outPt[4];
  //for 
  int row,col;   
  if(img_scale_==1.0){
    row = 768;
    col = 1024;
  }else{
    row = 384; 
    col = 512; 
  }
  inPt[0] = cv::Point2f(0.44 * (col), 0.42 * (row));
  inPt[1] = cv::Point2f(0.56 * (col), 0.42 * (row));
  inPt[2] = cv::Point2f(1 * (col), row - 1);
  inPt[3] = cv::Point2f(0 * (col), row - 1);

  outPt[0] = cv::Point2f(0.4 * (col), 0);
  outPt[1] = cv::Point2f(0.6 * (col), 0);
  outPt[2] = cv::Point2f(0.6 * (col), row - 1);
  outPt[3] = cv::Point2f(0.4 * (col), row - 1);
    
  trans_matrix_ = cv::getPerspectiveTransform(inPt,outPt);
  inv_trans_matrix_ = cv::getPerspectiveTransform(outPt,inPt);
}

cv::Point2d CnnDetector::GetAveragePoints(cv::Mat& img,int r_st,int c_st){
  
  return cv::Point(-1,-1);
}

cv::Point2d CnnDetector::GetCenterPoint(cv::Mat& img){
  int c_avg =0, r_avg = 0;
  int cntr = 0;
 
  return cv::Point(r_avg,c_avg);
}

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd CnnDetector::Polyfit(const Eigen::VectorXd &xvals, const Eigen::VectorXd &yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); ++i) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); ++j) {
        for (int i = 0; i < order; ++i) {
        A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);

    return result;
}

// Evaluate a polynomial.
double CnnDetector::Polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

