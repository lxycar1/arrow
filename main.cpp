//箭靶识别程序
#include"..\\arrow.h"

///全局变量

Mat D0,D1,R0,R1;  
Mat addimg;
int s;
double D_rate,R_rate;


int main()
{
//读取图片
	//【1】视频加载位置和名称
	//CvCapture* capture = cvCreateFileCapture("arrow/Transmtx/VideoTest2.avi");//cvcapture 和 vediocapture 区别：一个是c一个是c++ 
	VideoCapture cap0("arrow/Transmtx/VideoTest0.avi"); 
	VideoCapture cap2("arrow/Transmtx/VideoTest2.avi"); 
	if(!cap0.isOpened()||!cap2.isOpened()) 
	{ 
		return -1; 
	} 
	
	//导入相机内参和畸变系数矩阵
	//【下相机】
	//Mat Intrinsics0,Distortion0;
	//FileStorage in0("arrow/Transmtx/Intrinsics0.xml", FileStorage::READ);
	//in0["Intrinsics"] >> Intrinsics0;
	//in0.release();
	//FileStorage di0("arrow/Transmtx/Distortion0.xml", FileStorage::READ);
	//di0["Distortion"] >> Distortion0;
	//di0.release();
	
	//加载透视变换矩阵
	Mat transmtx0,transmtx2;
	FileStorage tr0("arrow/getpzx/transmtx0.xml", FileStorage::READ);
	tr0["transmtx0"] >> transmtx0;
	tr0.release();
	FileStorage tr2("arrow/getpzx/transmtx2.xml", FileStorage::READ);
	tr2["transmtx2"] >> transmtx2;
	tr2.release();

	//获取初始图像
	
	Mat bw0,bw2;
	Mat down=Mat::zeros(700,500,CV_8UC1);
	Mat right=Mat::zeros(500,700,CV_8UC1);
	
	cap0>>D0; cap2>>R0; 

	warpPerspective(D0, down, transmtx0, down.size()); 
	bw0=getblack(down,140);
	warpPerspective(R0, right, transmtx2, right.size()); 
	bw2=getblack(right,140);


	int i=0,s0,s2;
	while(1) 
	{ 		 
		D0=bw0;
		morphologyEx(D0,D0,MORPH_DILATE,Mat(6,6,CV_8U),Point(-1,-1),1);//下相机前图膨胀
		R0=bw2;
		morphologyEx(R0,R0,MORPH_DILATE,Mat(2,2,CV_8U),Point(-1,-1),1);//右相机前图膨胀
		//消除抖动
		for(i=0;i<12;i++)
		{			
			cap0>>D1; 
			cap2>>R1;			
		}
		if(D1.empty()||R1.empty()) break;

		//矫正相机畸变

		///透视变换【下相机】
		warpPerspective(D1, down, transmtx0, down.size()); 
		imshow("下视频",down); 
		///提取黑色区域
		bw0=getblack(down,140);
		//imshow("二值下视频",bw0); 
		
		D0=bw0-D0;
		morphologyEx(D0,D0,MORPH_OPEN,Mat(4,4,CV_8U),Point(-1,-1),1);//差帧图开操作
		//imshow("差帧0",D0); 
		s0=Area(D0);
		
		//透视变换【右相机】
		warpPerspective(R1, right, transmtx2, right.size()); 
		imshow("右视频",right); 
		bw2=getblack(right,120);
		//imshow("二值右视频",bw2); 		
		R0=bw2-R0;
		morphologyEx(R0,R0,MORPH_OPEN,Mat(4,4,CV_8U),Point(-1,-1),1);//前图闭操作
		//imshow("差帧2",R0); 
		s2=Area(R0);

		//判断是否有箭上靶
		if(s2>300&&s0>300)
		{
			//cout<<s0<<"			"<<s2<<endl;

			//将图片扩充为900X900
			Mat rectD99=Mat::zeros(900,900,CV_8UC1);
			cv::Mat tempD(rectD99,cv::Rect(200,0,D0.cols,D0.rows));
			D0.copyTo(tempD);

			Mat rectR99=Mat::zeros(900,900,CV_8UC1);
			cv::Mat tempR(rectR99,cv::Rect(0,200,R0.cols,R0.rows));
			R0.copyTo(tempR);			

			//获取箭体轴线
			Vec4f D_line=test(rectD99);
			D_rate=(double)s/(double)s0;
			Vec4f R_line=test(rectR99);
			R_rate=(double)s/(double)s2;

			//获取角点
			Point2f point=getCrossPoint(R_line, D_line);//求交点
			point.x=point.x-200;
			point.y=point.y-200;
			//【2】做图像混合加权操作  
			Mat rectD55 = down(Rect(0, 200, 500, 500));
			Mat rectR55 = right(Rect(200, 0, 500, 500));
			addWeighted(rectD55, 0.5, rectR55, 0.5, 0.0, addimg);  
			
			//描圆环
			int r=34;//最小环半径
			Point2f center=Point2f(250,250);

			//显示环数
			int N_cir=(getDistance(point,center)-3)/r;
			N_cir=cvCeil(10-N_cir);													//向上取整
			(N_cir>4)?cout<<"射中： "<<N_cir<<"环！"<<endl:cout<<"未射中有效区域~"<<endl;

			for(int i=1;i<=6;i++)
				circle(addimg, center, i*r, Scalar(120, 120, 120), 3);

			//描箭交点
			circle(addimg, point, 3, Scalar(255, 255, 255), - 1);

			//Show the teminal pic
			imshow("最终融合",addimg); 
		}

		if(waitKey(20) >=0) 
			waitKey(-1);
	} 
return 0;
}

//图片旋转操作
void imrotate(Mat& img, Mat& newIm, double angle){
	int len = max(img.cols, img.rows);
	Point2f pt(len/2.,len/2.);
	Mat r = getRotationMatrix2D(pt,angle,1.0);
	warpAffine(img,newIm,r,Size(len,len));
	//better performance : 
	//Point2f pt(img.cols/2.,img.rows/2.);
	//Mat r = getRotationMatrix2D(pt,angle,1.0);
	//warpAffine(img,newIm,r,img.size());
}

//从图像中提取出需要的坐标点
std::vector<cv::Point> getPoints(cv::Mat &image, int value)
{
	int nl = image.rows; // number of lines
	int nc = image.cols * image.channels();
	std::vector<cv::Point> points;
	for (int j = 0; j < nl; j++)
	{
		uchar* data = image.ptr<uchar>(j);
		for (int i = 0; i < nc; i++)
		{
			if(data[i] == value)
			{
				points.push_back(cv::Point(i, j));
			}
		}
	}
	return points;
}

//提取黑色区域，三通道值和小于value,返回图像矩阵
cv::Mat getblack(Mat &inpimage,int value)
{
	Mat outimage=Mat::zeros(inpimage.rows,inpimage.cols,CV_8UC1);
	Mat channel[3],R,G,B;  
	split(inpimage,channel);  
	B=channel[0];
	G=channel[1];
	R=channel[2];
	for(int i = 0;i < outimage.rows;i++)  
	{  
		uchar* datab = B.ptr<uchar>(i);  
		uchar* datag = G.ptr<uchar>(i);  
		uchar* datar = R.ptr<uchar>(i);  
		uchar* dataout = outimage.ptr<uchar>(i);  
		for(int j = 0;j < outimage.cols;j++)  
		{  
			(datab[j]+datag[j]+datar[j])<value?dataout[j] = 255: dataout[j] = 0;
		}  
	}  
	return outimage;
}

//在图中划线
void drawLine(cv::Mat &image,cv::Vec4f &line,int thickness)
{
	double cos_theta = line[0];
	double sin_theta = line[1];
	double x0 = line[2], y0 = line[3];
	double k = sin_theta / cos_theta;
	double b = y0 - k * x0;
	double x = 0;
	double y = k * x + b;
	int high=image.rows;//y
	int length=image.cols;//x
	double y1=y,y2=k*length+b;//左，右
	float x1=-b/(k*1.0),x2=(high-b)/(k*1.0);//上，下
	std::vector<cv::Point2f> pt(2);
	int i=0;

	if(0<=y1&&y1<=high)
	{
		pt[i].x=0;
		pt[i].y=y1;
		i=1;
	}
	if(0<=x1&&x1<=length)
	{
		pt[i].x=x1;
		pt[i].y=0;
		i=1;
	}
	if(0<=y2&&y2<=high)
	{
		pt[i]=Point2f(length,y2);
		i=1;
	}
	if(0<=x2&&x2<=length)
	{
		pt[i]=Point2f(x2,high);
	}
	cv::line(image, pt[0], pt[1],  cv::Scalar(255), thickness);
}

//求图像面积
int Area(cv::Mat &image)
{
	int area=0;
	for(int i = 0;i < image.rows;i++)  
	{  
		uchar* data = image.ptr<uchar>(i);  
		for(int j = 0;j < image.cols;j++)  
		{  
			(data[j]>0)?area++: area;
		}
	}
	return area;
}

//直线求交
Point2f getCrossPoint(Vec4f LineA, Vec4f LineB)
{
	//LineA:cos_theta = line[0];sin_theta = line[1]; x0 = line[2], y0 = line[3];
	double ka, kb;
	ka = (double)LineA[1]/LineA[0]; //求出LineA斜率
	kb = (double)(LineB[1]/LineB[0]); //求出LineB斜率

	Point2f crossPoint;
	crossPoint.x = (ka*LineA[2] - LineA[3] - kb*LineB[2] + LineB[3]) / (ka - kb);
	crossPoint.y = (ka*kb*(LineA[2] - LineB[2]) + ka*LineB[3] - kb*LineA[3]) / (ka - kb);
	return crossPoint;
}

//通过前后帧提取箭体轴线
cv::Vec4f test(cv::Mat &dst)
{
	//获取坐标点
	std::vector<cv::Point> points = getPoints(dst, 255);

	//直线拟合
	cv::Vec4f line;
	cv::fitLine(points,	line,CV_DIST_L1 ,5,0.01,0.01);//http://blog.csdn.net/liyuanbhu/article/details/50193947

	//直线的图片
	Mat Lin= Mat::zeros(dst.size(),CV_8UC1);

	drawLine(Lin,line, 40);										//描粗拟合线条，放宽范围提取箭体

	Mat yy=Lin&dst;												//拟合区域提取
	s=Area(yy);                                                 //箭体面积
	//再次拟合
	std::vector<cv::Point> pointss = getPoints(yy, 255);
	cv::Vec4f lines;
	cv::fitLine(pointss,lines,CV_DIST_L1 ,5,0.01,0.01);

	//imshow("yyyy", yy);
	//cout<<"差帧总面积: "<<Area(dst)<<endl<<"箭体面积："<<Area(yy)<<endl;

	//最终结果显示
	//drawLine(dst,lines, 1);
	//imshow("end", dst);
	return lines;
}

//直方图均衡化
Mat equalizeChannelHist(const Mat & inputImage)  
{  
	if( inputImage.channels() >= 3 )  
	{  
		vector<Mat> channels;  
		split(inputImage, channels);  

		Mat B,G,R;  

		equalizeHist( channels[0], B );  
		equalizeHist( channels[1], G );  
		equalizeHist( channels[2], R );  

		vector<Mat> combined;  
		combined.push_back(B);  
		combined.push_back(G);  
		combined.push_back(R);  

		Mat result;  
		merge(combined, result);  

		return result;  
	}  
	else
	{
		equalizeHist(inputImage,inputImage);
		return inputImage;  
	}
	
}  

/************************************************************************  
*函数名：        getDistance  
*  
*函数作用：      获取两点之间的距离  
*  
*函数参数：  
*CvPoint2D32f pointO  - 起点  
*CvPoint2D32f pointA  - 终点  
*  
*函数返回值：  
*double           两点之间的距离  
**************************************************************************/    
double getDistance (CvPoint pointO,CvPoint pointA )    
{    
	double distance;    
	distance = powf((pointO.x - pointA.x),2) + powf((pointO.y - pointA.y),2);    
	distance = sqrtf(distance);    

	return distance;    
}    
