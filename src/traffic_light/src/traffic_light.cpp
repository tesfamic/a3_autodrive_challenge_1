#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>
#include "aggies_msgs/PathData.h"

class TL_Classifier{

        ros::NodeHandle nh_;
        ros::Subscriber image_sub_;
        ros::Subscriber image_sub2_;
        ros::Subscriber traject_sub_;
        ros::Publisher tl_state_pub_;
        ros::Publisher ts_pub_;

        cv::dnn::Net net;
        cv::dnn::Net tl_net;

        std::vector<std::string> outNames;
        std::vector<std::string> classes;
        
        const float confThreshold = 0.50; 
        const float nmsThreshold = 0.4; 
        const float scale = 0.00392; 
        const cv::Scalar mean_v = cv::Scalar(0,0,0);
        const bool swapRB = true;
        const int inpWidth =384;// 512;//384;//192;//416; //192;//
        const int inpHeight =288;// 384;//288;//192;//
        const std::vector<int> class_ids = {0,1,2,3,5,6,7,9,11};
        const std::vector<std::string> class_names = {"Ped","Bike","Car","MotBike","Bus","Train","Truck","TL","Stop"};
        const std::map<int,std::string> labels ={{0,"Ped"},{1,"Bike"},{2,"Car"},{3,"MotBike"},
                                                 {5,"Bus"},{6,"Train"},{7,"Truck"},{9,"TL"},{11,"Stop"}};

        int red_counter_;
        ros::Time first_red_time_;
        cv::Mat image1_, image2_;

        int camera_choice_;// = 1- camera1 - with zoom, 2: without zoom

    public:
        TL_Classifier(){
            camera_choice_ = 1;
            
            std::string w_path = "yolov3/yolov3.weights"; 
            std::string cf_path = "yolov3/yolov3.cfg";
            std::string cl_path = "yolov3/coconames.txt";

            std::ifstream class_file;
            class_file.open(cl_path,std::ios::in);
            if (!class_file.is_open())
                std::cout<<"\n Unable to read the classes file.\n";

            std::string line;
            while (std::getline(class_file, line)) {
                classes.push_back(line);
            }

            net = cv::dnn::readNet(cf_path,w_path);// readNetFromDarknet(cf_path,w_path);    
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);// 
            net.setPreferableTarget(cv::dnn::DNN_TARGET_OPENCL);//DNN_TARGET_CPU);
            outNames = net.getUnconnectedOutLayersNames();

            std::string tl_pb = "tl.bin";
            std::string tl_xml = "tl.xml";
            tl_net = cv::dnn::readNetFromModelOptimizer(tl_xml,tl_pb);
            tl_net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
            tl_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            
            cv::setNumThreads(-1);

        }
        ~TL_Classifier(){}
        void ImageCb(const sensor_msgs::CompressedImageConstPtr& img){//ImageConstPtr& msg){                                      
            cv::Mat image;
            try{
                //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                image = cv::imdecode(cv::Mat(img->data),1);
            }catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }   
            if (image.empty()){
                std::cout<<"Empty image."<<std::endl;               
                return;
            }
            image1_ = image;
            ProcessImage();
        }
        
        void GetBboxes(cv::Mat& image,std::vector<Box>& bboxes){
    
            cv::Mat blob, frame=image;
            if (image.empty()) {
                std::cout<<"\n Frame is empty. \n";        
                return;
            }
            cv::Size inpSize(inpWidth > 0? inpWidth : image.cols,
                             inpHeight > 0 ? inpHeight : image.rows);
            cv::dnn::blobFromImage(image, blob, scale, inpSize, mean_v, swapRB, false);
            // Run the model.
            net.setInput(blob);
            //// continues ...           
        }

        std::string ClassLabel(int class_id){
            return labels.at(class_id);
        }

        void ProcessImage(){                                     
            cv::Mat image;
            cv::Mat img_org;
            if(camera_choice_==1){
                image = image1_;
            }

            ts_pub_.publish(ts_msg);

            cv::addWeighted(img_org,0.6,image,0.4,0,image);
            cv::imshow("img",image);
            cv::waitKey(1);
        }
        
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tl_classifier");

	TL_Classifier tl_classifier;

	ros::spin();
    
    
    return 0;
}
