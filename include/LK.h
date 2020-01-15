//
// Created by lgj on 12/31/19.
//

#ifndef ORB_SLAM2_LK_H
#define ORB_SLAM2_LK_H

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <Frame.h>
using namespace std;
using namespace ORB_SLAM2;


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;

vector< cv::Point2f > keypoints;
vector<cv::Point3f> mappointInCurrentFrame;
vector<cv::Point2f> prev_keypoints;
cv::Mat last_color;
/**非关键帧的情况下，使用光流法跟踪关键帧的特征点,使用 PNP_RANSAC 来计算相机位姿
 * 暂未解决 无法把所有帧的位姿加入CameraTrajectory.txt的问题
 * **/

Mat computeMtcwUseLK(KeyFrame *lastKeyFrame, Mat color, bool lastColorIsKeyFrame, Mat K, Mat mDistCoef, Mat &mTcw, int &mnMatchesInliers)
{
    int obsPlus = 0;
    if(lastKeyFrame->mnId>3)
        obsPlus = 3;

    if(last_color.empty())
    {
//        cout<<"fill last color fist time"<<endl;
        last_color = color;
        return cv::Mat();
    }
    /**上一帧是关键帧的情况**/
    if(lastColorIsKeyFrame || keypoints.empty())
    {
//        cout<<lastKeyFrame->mvKeysUn.size()<<endl;
        keypoints.clear();
        mappointInCurrentFrame.clear();

        for(int i=0;i<lastKeyFrame->mvpMapPoints.size();i++)//copy point from keyframe
        {
            if(lastKeyFrame->mvpMapPoints[i]&&lastKeyFrame->mvpMapPoints[i]->Observations()>obsPlus)///if the program died here, try to change 1 to 0
            {
                keypoints.push_back(lastKeyFrame->mvKeysUn[i].pt);
                cv::Point3f pt3f;
                cv::Mat temp;
                temp = lastKeyFrame->mvpMapPoints[i]->GetWorldPos();
                pt3f.x = temp.at<float>(0);
                pt3f.y = temp.at<float>(1);
                pt3f.z = temp.at<float>(2);
                mappointInCurrentFrame.push_back(pt3f);
            }
        }
    }
    /**需要用到的数据*/
    vector<cv::Point2f> next_keypoints;
    prev_keypoints.clear();
    for ( auto kp:keypoints )
        prev_keypoints.push_back(kp);
    vector<unsigned char> status;//判断该点是否跟踪失败
    vector<float> error;
    Mat last_gray,gray;//LK光流法用于跟踪特征点的两帧
    cvtColor(last_color,last_gray,CV_BGR2GRAY);
    cvtColor(color,gray,CV_BGR2GRAY);
//    cout<<"preKeyPointNum"<<prev_keypoints.size()<<endl;
    cv::calcOpticalFlowPyrLK( last_gray, gray, prev_keypoints, next_keypoints, status, error );//计算光流

    /** 把跟丢的点删掉**/
    int i=0;
    for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
    {
        if ( status[i] == 0 )
        {
            iter = keypoints.erase(iter);
            continue;
        }
        *iter = next_keypoints[i];//edit keypoints' coordinate
        iter++;
    }
//    cout<<"tracked keypoints: "<<keypoints.size()<<endl;
    i = 0;
    for ( auto iter=mappointInCurrentFrame.begin(); iter!=mappointInCurrentFrame.end(); i++)//erase the match mappoint while the keypoint is erased
    {
        if ( status[i] == 0 )
        {
            iter = mappointInCurrentFrame.erase(iter);
            continue;
        }
        iter++;
    }
    /**使用PnPRansac计算位姿*/
    vector<cv::Mat> point3D;
    cv:Mat R_vector,T,R;
    vector<int> ransacInlier;
    if(!(mappointInCurrentFrame.empty()||keypoints.empty()||mDistCoef.empty()))
    {
        cout<<"pointNUM"<<mappointInCurrentFrame.size()<<endl;
        if(keypoints.size()<20)
        {
            cout<<"Optical flow need more points"<<endl;
            return cv::Mat();
        }
        cv::solvePnPRansac(mappointInCurrentFrame, keypoints, K, mDistCoef , R_vector, T, false, 50,3, 0.98, ransacInlier, SOLVEPNP_ITERATIVE);

        cv::Rodrigues(R_vector, R);
        Mat_<double> Rt = (Mat_<double>(4, 4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),T.at<double>(0),
                R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),T.at<double>(1),
                R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2),T.at<double>(2),
                0, 0, 0, 1);
        cv::Mat Rt_float;
        Rt.convertTo(Rt_float,CV_32FC1);
        mTcw = Rt_float;//位姿
    }

    if (keypoints.size() == 0)
    {
        cout<<"LK -- all keypoints are lost."<<endl;
        return cv::Mat();
    }
    /** 画出 keypoints*/
    cv::Mat img_show = color.clone();
    int point = 0;
    int iterOfInlier = 0;
    for ( auto kp:keypoints )
    {
        for(iterOfInlier =0;iterOfInlier<ransacInlier.size();iterOfInlier++)
        {
            if(point == ransacInlier[iterOfInlier])
            {
                cv::circle(img_show, kp, 5, cv::Scalar(0, 255, 0), 1);
                cv::circle(img_show, kp, 1, cv::Scalar(0, 255, 0), -1);
            }
        }
        point++;
    }
    mnMatchesInliers = ransacInlier.size();
//    cout<<"PNP inlier: "<<ransacInlier.size()<<endl;
    last_color = color;

    return img_show;//返回值是rgb图片，用于显示光流跟踪到的特征点
}


#endif //ORB_SLAM2_LK_H
