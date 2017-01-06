#include "main.h"


int main(int argc, char **argv)
{
    clock_t t_start, t_end;

    ros::init(argc, argv, "haaga_image_publisher");
    ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
 	image_transport::Publisher left_pub = it.advertise("haagarobot/camera/right", 1);
	image_transport::Publisher right_pub = it.advertise("haagarobot/camera/left", 1);
	image_transport::Publisher topview_pub = it.advertise("haagarobot/camera/topview", 1);
	
    // Read Data from XML
    string name_space = nh.getNamespace();
    string xml_path;
    string camera_file  = "calibration_data.xml";
    string topview_file = "H_matrix.xml";
    string alpha_file   = "Alpha_matrix.xml";

    nh.getParam(name_space+"xml_path", xml_path);
 	nh.getParam(name_space+"cam_L_port", cam_L_port);
  	nh.getParam(name_space+"cam_R_port", cam_R_port);
  
    camera_file= xml_path +"/" +camera_file;
    topview_file= xml_path +"/" +topview_file;
    alpha_file= xml_path +"/" +alpha_file;
    
    FileStorage fs_P(camera_file,  FileStorage::READ);
    FileStorage fs_H(topview_file, FileStorage::READ);
    FileStorage fs_A(alpha_file,   FileStorage::READ);


    Mat cameraMat_L, distCoeffs_L;
    readData_P(fs_P, cameraMat_L, distCoeffs_L);
    Mat cameraMat_R, distCoeffs_R;
    readData_P(fs_P, cameraMat_R, distCoeffs_R);

    Mat left_H, right_H;
    readData_H(fs_H, left_H, right_H);

    Mat alpha_L, alpha_R;
    readData_A(fs_A, alpha_L, alpha_R);

    // Camera Input 
    cv::VideoCapture cap_L(cam_L_port); // left
    cv::VideoCapture cap_R(cam_R_port); // right
    if (!cap_R.isOpened())
    {
        printf("cannot connect to cam_R\n");
        return -1;
    }
    if (!cap_L.isOpened())
    {
        printf("cannot connect to cam_L\n");
        return -1;
    }


    //cap_R.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    cap_R.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth);
    cap_R.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight);
    //cap_L.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
    cap_L.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth);
    cap_L.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight);

    // reference from "find_H"
    float dummy_query_data_L[9] = { 115, 0, 320,
                                    0, 115, 197.7,
                                    0, 0, 1};
    float dummy_query_data_R[9] = { 115, 0, 320,
                                    0, 115, 197.7,
                                    0, 0, 1};
    Mat newCamMat_L = Mat(3, 3, CV_32F, dummy_query_data_L);
    Mat newCamMat_R = Mat(3, 3, CV_32F, dummy_query_data_R);

    //Declaration 
    Mat frame_L, undframe_L;
    Mat frame_R, undframe_R;

    Mat topview_L;
    Mat topview_R;
    Mat roi_L;
    Mat roi_R;
    Mat topview = Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));


    Mat imgGray, imgBlur, imgCanny;


    // Undistort (after split) = PART 1/2
    cap_L >> frame_L;
    cap_R >> frame_R;

    const Size& new_size_L = cv::Size(frame_L.cols, frame_L.rows);
    Size size_L = new_size_L.area() != 0 ? new_size_L : frame_L.size();
    Mat map1_L, map2_L;
   fisheye::initUndistortRectifyMap(cameraMat_L, distCoeffs_L, Matx33d::eye(), newCamMat_L, size_L, CV_16SC2, map1_L, map2_L);

    const Size& new_size_R = cv::Size(frame_R.cols, frame_R.rows);
    Size size_R = new_size_R.area() != 0 ? new_size_R : frame_R.size();
    Mat map1_R, map2_R;
    fisheye::initUndistortRectifyMap(cameraMat_R, distCoeffs_R, Matx33d::eye(), newCamMat_R, size_R, CV_16SC2, map1_R, map2_R);

    while(nh.ok())
    {
        t_start = clock();

        cap_L >> frame_L;
        cap_R >> frame_R;
        //cv::imshow("frame_L", frame_L);
        //cv::imshow("frame_R", frame_R);

        // Undistort (after split) = PART 2/2
        cv::remap(frame_L, undframe_L, map1_L, map2_L, INTER_LINEAR, BORDER_CONSTANT);
        cv::remap(frame_R, undframe_R, map1_R, map2_R, INTER_LINEAR, BORDER_CONSTANT);
        // Undistort (before split)
        //        fisheye::undistortImage(frame_L, undframe_L, cameraMat_L, distCoeffs_L, newCamMat_L,
        //                                cv::Size(frame_L.cols, frame_L.rows));
        //        fisheye::undistortImage(frame_R, undframe_R, cameraMat_R, distCoeffs_R, newCamMat_R,
        //                                cv::Size(frame_R.cols, frame_R.rows));

        // Warping
        cv::warpPerspective(undframe_L, topview_L, left_H, cv::Size(frame_L.cols, frame_L.rows));
        cv::warpPerspective(undframe_R, topview_R, right_H, cv::Size(frame_R.cols, frame_R.rows));

        // Combine
        roi_L = topview_L(Rect(0, 0, topview_L.cols/2, topview_L.rows));
        roi_L.copyTo(topview(Rect(0, 0, topview_L.cols/2, topview_L.rows)));
        roi_R = topview_R(Rect(topview_R.cols/2, 0, topview_R.cols/2, topview_R.rows));
        roi_R.copyTo(topview(Rect(topview_R.cols/2, 0, topview_R.cols/2, topview_R.rows)));

        float aL,aR;
                for (int y=0; y<topview.rows; y++)
                {
                    Vec3b* vec_L = alpha_L.ptr<Vec3b>(y);
                    Vec3b* vec_R = alpha_R.ptr<Vec3b>(y);
                    Vec3b* top_A = topview.ptr<Vec3b>(y);
                    Vec3b* top_L = topview_L.ptr<Vec3b>(y);
                    Vec3b* top_R = topview_R.ptr<Vec3b>(y);
                    for (int x=0; x<topview.cols; x++)
                    {
                        aL = (vec_L[x][0])/255.0;
                        aR = (vec_R[x][0])/255.0;
                        top_A[x][0] = aL * top_L[x][0] + aR * top_R[x][0];
                        top_A[x][1] = aL * top_L[x][1] + aR * top_R[x][1];
                        top_A[x][2] = aL * top_L[x][2] + aR * top_R[x][2];
                    }
                }


        //imshow("source", topview);
        
        //Publish images
        sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_L).toImageMsg();
        sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_R).toImageMsg();
        sensor_msgs::ImagePtr topview_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", topview).toImageMsg();
        
        left_pub.publish(left_msg);
        right_pub.publish(right_msg);
        topview_pub.publish(topview_msg);
        
        ros::spinOnce();
        
        char c = (char)cv::waitKey(1);
        if (c == 'q')
            break;

    }
    return 0;
  
   
}

void readData_P(const FileStorage& node, Mat& cameraMat, Mat& distCoeffs)
{
    node["image_width"] >> imageWidth;
    node["image_height"] >> imageHeight;
    node["camera_matrix"] >> cameraMat;
    node["distortion_coefficients"] >> distCoeffs;
    node["extrinsic_parameters"] >> extrinsicParam;
}
void readData_H(const FileStorage& node, Mat& H_L, Mat& H_R)
{
    node["H_LtoC"] >> H_L;
    node["H_RtoC"] >> H_R;
}
void readData_A(const FileStorage& node, Mat& alpha_L, Mat& alpha_R)
{
    node["alpha_left"] >> alpha_L;
    node["alpha_right"] >> alpha_R;
}
