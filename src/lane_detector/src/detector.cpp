
#include "detector.h"

Detector::Detector():it_(nh_){
    
    num_of_frames_ = 0;

    trans_matrix_ = cv::Mat::zeros(3,3,CV_32F);
    inv_trans_matrix_ = cv::Mat::zeros(3,3,CV_32F);

    // //restart = true;
    starting_pt_exists = true;
    argmax_left_prev_ = 0;
    argmax_right_prev_ = 0;

    image_sub_ = nh_.subscribe("/camera2/image/compressed", 1,&Detector::ImageCallback, this);
    center_lane_pub_ = nh_.advertise<aggies_msgs::PathData>("/lane_center",1);
    // lane_left_pub_ = nh_.advertise<aggies_msgs::PathData>("/lane_left",1);
    // lane_right_pub_ = nh_.advertise<aggies_msgs::PathData>("/lane_right",1);

    img_with_line_pub_ = it_.advertise("/img_with_line",1);

    InitializeParams();

    cv::setNumThreads(2); 

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    auto tt = std::localtime(&now);
    auto today = std::to_string(1900+ tt->tm_year)+"-";
    today += std::to_string(1+tt->tm_mon)+"-";
    today += std::to_string(tt->tm_mday)+"-";
    today += std::to_string(tt->tm_hour)+"-";
    today += std::to_string(tt->tm_min)+"-";
    today += std::to_string(tt->tm_sec);
    
    std::string dir = "/home/a3/vision_nav/src/lane_detector/log/"+today;
    
    // f_lane_out.open(dir+"-lane_out.txt",std::ios::out);
    // if(!f_lane_out.is_open()) std::cout<<"\n Unable to open log:ref_trj.txt file.\n";

    // out_writer_ = cv::VideoWriter(dir+"-proc.avi",cv::VideoWriter::fourcc('M','J','P','G'),20, cv::Size(512,384),true);
    // raw_writer_ = cv::VideoWriter(dir+"-raw.avi",cv::VideoWriter::fourcc('M','J','P','G'),20, cv::Size(512,384),true);
}
Detector::~Detector(){
    // if(f_lane_out.is_open()) f_lane_out.close();
    // out_writer_.release();
    // raw_writer_.release();
}

void Detector::ImageCallback(const sensor_msgs::CompressedImageConstPtr& img_msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        image_ = cv::imdecode(cv::Mat(img_msg->data),1);
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:%s",e.what());
        return;
    }

    num_of_frames_++;
    cv::resize(image_,image_,cv::Size(512,384));
    ////save raw image
    //raw_writer_.write(image_);
    //cv::imshow("image",image_);

    PreprocessImage();
       
    int argmax_left, argmax_right;  
    GetLineStartPoints(argmax_left,argmax_right);    
    
    // //Publish lane center data
    // aggies_msgs::PathData lane_ctr_msg,lane_left_msg,lane_right_msg;
    
    if(starting_pt_exists){
        AdjustStartPoints(argmax_left,argmax_right);
                  
        // Register pixels positions around the starting points
        std::vector<cv::Point> l_line_pixels,r_line_pixels;
        std::vector<BoxData> l_line_boxes,r_line_boxes;
        GetLinePixels(argmax_left,argmax_right,l_line_pixels,r_line_pixels,
                      l_line_boxes,r_line_boxes);

        //Analysing the number of pixels in each window       
        bool left_line = true;
        bool right_line = true;
        cv::Scalar color_l_line;
        cv::Scalar color_r_line;        
        int l_empty=0,r_empty=0;
        AnalyzeDetection(l_line_boxes,r_line_boxes,left_line,right_line,
                         l_empty,r_empty, color_l_line,color_r_line);

        std::vector<cv::Point> l_line,r_line;         
        if(left_line || right_line){                  
            // move pixels from left to right and vice-versa            
            AdjustLine(l_line_pixels,r_line_pixels,left_line,right_line,l_line,r_line);            
            CorrectParallel(l_line,r_line,l_line_pixels,r_line_pixels, l_empty,r_empty);
            
             /* Calculate the lane center */                    
            middle.clear();  
            if(l_line.size()>0 && r_line.size()>0){  
                int num_pts = std::min(l_line.size(),r_line.size());            
                for(int i=0;i<num_pts;++i){
                    auto pl=l_line[i];
                    auto pr=r_line[i];
                    middle.emplace_back(cv::Point2i(pl.x,int((pl.y+pr.y)*0.5)));                    
                }
            }

            // // plot the lines on the image          
            // PlotLine(l_line,color_l_line);
            // PlotLine(r_line,color_r_line);               
            // //plot the middle line on the image  
            // PlotLine(center,cv::Scalar(255,0,0));              
           
           
            // //Publish lane center data
            // aggies_msgs::PathData lane_ctr_msg,lane_left_msg,lane_right_msg;
            
        }        
        else{
            std::cout<<" \n ...No detection ...";
        }
    }
    // center_lane_pub_.publish(lane_ctr_msg);
    // lane_left_pub_.publish(lane_left_msg);
    // lane_right_pub_.publish(lane_right_msg);

   
    // cv::Mat imgF; 
    // cv::addWeighted(img_warped_,1.0,img_final_,1.0,0.0,imgF);
    // cv::Mat tmp_img; 
    // cv::warpPerspective(img_final_,tmp_img,inv_trans_matrix_,img_final_.size());         
    // cv::addWeighted(image_,1.0,tmp_img,1.0,0.0,image_);
    // //cv::imshow("warp_f",imgF);
    cv::imshow("img final",image_);
    /////save the processed image
    // out_writer_.write(image_);
    // cv::imshow("img warped",img_warped_);    
    cv::waitKey(1);    
    // std::cout<<"\n #frames:"<<num_of_frames_;
    //std::cout<<" ros time:"<<ros::Time::now()<<", clock():"<<clock()<<", chrono:"<<std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
}


void Detector::PlotLine(std::vector<cv::Point>& line,cv::Scalar color){
    if(line.size()>1){
        for(int i=0;i<line.size()-1;++i){
            cv::Point2i p1,p2;
            p1 = line[i];  
            p2 = line[i+1];
            cv::line(img_warped_,cv::Point(p1.y,p1.x),cv::Point(p2.y,p2.x),color,2); 
            cv::line(img_final_,cv::Point(p1.y,p1.x),cv::Point(p2.y,p2.x),color,2);         
        }
    }
}
void Detector::PreprocessImage(){
    img_final_ = cv::Mat::zeros(cv::Size(image_.cols,image_.rows),image_.type());
    cv::warpPerspective(image_, img_warped_, trans_matrix_, image_.size());

    //convert image to YCrCb color space
    cv::Mat channels[3];
    //Separate the cr channel: which contains the yellow line information
   
    // subtract_average_intensity2(img_gray);
    
    cv::filter2D(img_cb,img_cb,CV_8U,gabor_kernel_);        

    // //Combine the Cr channel with the grayscale image      
    
}

void Detector::GetCorrectedLine(cv::Mat &img_filt,std::vector<cv::Point2i> &ref_line,std::vector<cv::Point2i> line){
   /////////////////////
}
bool Detector::CompareCorrectLineError(std::vector<cv::Point2i> &prev_l,
                                std::vector<cv::Point2i> &cur_l,
                                int &error_limit){
        ///////////

        return true;       
    }                               
}
void Detector::FitPolynomial(std::vector<cv::Point2i> &line_pixels, std::vector<cv::Point2i> &line){
    //////////////
}

void Detector::GetLinePixels(int argmax_left,int argmax_right,
                           std::vector<cv::Point>& l_line_px,
                           std::vector<cv::Point>& r_line_px,
                           std::vector<BoxData>& l_boxes,
                           std::vector<BoxData>& r_boxes){
    ///////////////////////////

}

void Detector::FilterLinePixels(int l_start,
                      std::vector<cv::Point2i>& line_pixels, std::vector<BoxData>& line_boxes){
    /////////
}

void Detector::AdjustStartPoints(int& argmax_left,int& argmax_right){
    // if only one of the starting point is found
    
        // what if the values are the same? shift one of them by the average lane width
        
        // if the distance between starting points are not in range (outside the average lane width),  
        // calculate the weighted average start position of previous and current starting
       
    //checking with previous start positions
   
}

void Detector::GetLineStartPoints(int &argmax_left,int& argmax_right){
    
    
    //count pixels for each column...histogram
    std::vector<int> hist;
    ///////

    if(hist.size()>0){
        auto max_val_ptr = std::max_element(hist.begin(),hist.begin()+int(hist.size()/2));
        int max_val_left = *max_val_ptr;
        argmax_left=  x_corn + std::distance(hist.begin(),max_val_ptr);
        if(argmax_left == x_corn){
            argmax_left = 0;
        }

        max_val_ptr = std::max_element(hist.begin()+int(hist.size()/2),hist.end());
        int max_val_right = *max_val_ptr;
        argmax_right= x_corn + img.cols*0.5 + std::distance(hist.begin()+int(hist.size()/2),max_val_ptr);
        if(argmax_right == (x_corn+img.cols*0.5)){
            argmax_right = int(img_filtered_.cols*0.5);
        }
    }

    
        //no starting points detected in the current frame
        
        //no detection of the starting points, the image must be blank
        std::cout<<"    No detection -> Couldn't find the starting points of the lines \n";
            

}

void Detector::ApplyImageFiltering(){
    /////////////////
}

void Detector::GetZeroIndices(int thresh,std::vector<cv::Point2i> &zero_indices){

    ///////////////

}

void Detector::GetSelectIndices(std::vector<cv::Point2i> &indices,
                                          std::vector<cv::Point2i> &selected_indices){
    ////////////////
}

void Detector::GetFinalIndices(std::vector<cv::Point2i> &indices,int thresh, 
                     std::vector<cv::Point2i> &final_indices){
    //////////////////////////////
}

void Detector::FilterImage(std::vector<cv::Point2i> &indices){
 
    /////////////////////////////
}

void Detector::SubtractAverageIntensity(cv::Mat &img){
    /////////////////////////
}

void Detector::InitializeParams(){

    const double dim = 7.0;
    const int ker_dim = int(dim);
    const double sigma_ = dim;
    const double theta_0 = 0.0;
    const double theta_pi = CV_PI;
    const double lambd_ = dim;
    const double gama_  = 0.138;
    const double psi_   = 1.43; 
    const cv::Size k_size = cv::Size(ker_dim,ker_dim);
    
    cv::Point2f inPt[4];
    cv::Point2f outPt[4];                      
    
    //////////

    trans_matrix_ = cv::getPerspectiveTransform(inPt,outPt);
    inv_trans_matrix_ = cv::getPerspectiveTransform(outPt,inPt);

    //calculate the gabor filter kernel

    //erosion kernel for morphology
    erosion_kernel_ = cv::Mat::ones(3, 3, CV_8U);

}

void Detector::AnalyzeDetection(std::vector<BoxData>& l_boxes,
                                std::vector<BoxData>& r_boxes,
                                bool& l_detected, bool& r_detected,
                                int& l_empty, int& r_empty,
                                cv::Scalar& l_color,cv::Scalar& r_color){
    l_empty = 0;
    r_empty = 0;
    //////////////////                 
}

void Detector::AdjustLine(std::vector<cv::Point>& l_pixs,
                          std::vector<cv::Point>& r_pixs,
                          bool left,bool right,
                          std::vector<cv::Point>& l_line,
                          std::vector<cv::Point>& r_line){
    ///////////////////////
                          
}

void Detector::CorrectParallel(std::vector<cv::Point>& l_line,
                               std::vector<cv::Point>& r_line,
                               std::vector<cv::Point>& l_pix,
                               std::vector<cv::Point>& r_pix,
                               int l_empty,int r_empty){
    ///////////////

}
