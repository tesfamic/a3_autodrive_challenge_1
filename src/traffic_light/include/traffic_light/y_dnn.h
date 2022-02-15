
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/dnn.hpp>

std::vector<cv::Vec4i> get_bboxes(cv::Mat& image, std::vector<std::string> &classes, cv::dnn::Net &net, std::vector<cv::String>& outNames);
void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame);
