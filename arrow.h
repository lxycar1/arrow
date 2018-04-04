#ifndef ARROW_H_H
#define ARROW_H_H
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include<string>  
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;

Mat contrastStretch(Mat srcImage);
void colorGet(Mat& inputImage, Mat& outputImage);
void fillHole(const Mat srcBw, Mat &dstBw);
void FitEllipse(const Mat inputImage, Mat outputImage);
void RemoveSmallRegion(Mat &Src, Mat &Dst,unsigned int AreaLimit, int CheckMode, int NeihborMode);
void SameRegion(Mat &Src1, Mat &Src2,Mat &Dst);
bool Oneline(Vec4i &a, Vec4i &b);
void Fourline(vector<Vec4i> &lines);
Point2f computeIntersect(Vec4i a, Vec4i b);
vector<Point2f> FouP(vector<Vec4i> lines,Mat image);
void sortCorners(vector<Point2f>& corners) ;
IplImage * jiaozheng( IplImage* image,CvMat *intrinsic,CvMat *distortion );
Mat uplook(IplImage* image);

void imrotate(Mat& img, Mat& newIm, double angle);
std::vector<cv::Point> getPoints(cv::Mat &image, int value);
cv::Mat getblack(Mat &inpimage,int value);
void drawLine(cv::Mat &image,cv::Vec4f &line,int thickness);
int Area(cv::Mat &image);
Point2f getCrossPoint(Vec4f LineA, Vec4f LineB);
cv::Vec4f test(cv::Mat &dst);
Mat equalizeChannelHist(const Mat & inputImage);
double getDistance (CvPoint pointO,CvPoint pointA )    ;


#endif
