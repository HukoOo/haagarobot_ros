#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define ABS(a)      (((a)<(0))?-(a):(a))
//#define MAX(a,b)    (((a)>(b))?(a):(b)) // already defined by openCV
#define Deg2Rad 0.01745329251 // pi/180
#define Rad2Deg 57.295779546 // 180/pi

using namespace cv;
using namespace std;

int imageWidth, imageHeight;
int cam_R_port, cam_L_port;

Mat extrinsicParam;

void readData_P(const FileStorage& node, Mat& cameraMat, Mat& distCoeffs);
void readData_H(const FileStorage& node, Mat& H_L, Mat& H_R);
void readData_A(const FileStorage& node, Mat& alpha_L, Mat& alpha_R);

