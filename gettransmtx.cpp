/*
	该程序用于获取已经定位的相机拍摄箭靶的透视变换矩阵transmtx
	调参点：
			【1】视频加载位置和名称
			【2】相机畸变参数
			【3】保存透视变换矩阵
			【4】uplook函数的对应点
			【5】调整到合适的图像的循环次数i
*/


#include"..\\arrow.h"
Mat transmtx;//要输出的透视变换矩阵

int main() {
	//【1】视频加载位置和名称
	CvCapture* capture = cvCreateFileCapture("arrow/Transmtx/VideoTest2.avi");//cvcapture 和 vediocapture 区别：一个是c一个是c++ 
	if (!capture)//判断是否打开视频文件  
	{  
		return -1;  
	}  

	IplImage* image;
	Mat resultimg;
	CvMat *intrinsic;
	CvMat *distortion ;

	//【4】调整到合适的图像
	for(int i=0;i<10;i++)
	{
	image= cvQueryFrame(capture);
	cvShowImage( "Raw Video", image ); // Show 原图
	waitKey(20);//每帧延时20毫秒
	}
	//【2】相机畸变参数
	intrinsic = (CvMat*)cvLoad("arrow/Transmtx/Intrinsics2.xml");
	distortion = (CvMat*)cvLoad("arrow/Transmtx/Distortion2.xml");

	image=jiaozheng(image,intrinsic,distortion);	
	cvShowImage("Undistort", image);     // Show 矫正后图
	
	resultimg=uplook(image);

	if(Area(resultimg)<5000)
		cout<<"不是目标图像！"<<endl;

	//【3】保存透视变换矩阵
	FileStorage fs("arrow/Transmtx/transmtx2.xml", FileStorage::WRITE);
	fs<<"transmtx0"<<transmtx;
	fs.release();

	waitKey(0);
	return 0;
}

//对比度拉伸
Mat contrastStretch(Mat srcImage)  
{  
	Mat resultImage = srcImage.clone();//"=";"clone()";"copyTo"三种拷贝方式，前者是浅拷贝，后两者是深拷贝。  
	int nRows = resultImage.rows;  
	int nCols = resultImage.cols;  
	//判断图像的连续性  
	if (resultImage.isContinuous())  
	{  
		nCols = nCols*nRows;  
		nRows = 1;  
	}  
	//图像指针操作  
	uchar *pDataMat;  
	int pixMax = 0, pixMin = 255;  
	//计算图像的最大最小值  
	for (int j = 0; j < nRows; j++)  
	{  
		pDataMat = resultImage.ptr<uchar>(j);//ptr<>()得到的是一行指针  
		for (int i = 0; i < nCols; i++)  
		{  
			if (pDataMat[i] > pixMax)  
				pixMax = pDataMat[i];  
			if (pDataMat[i] < pixMin)  
				pixMin = pDataMat[i];  
		}  
	}  
	//对比度拉伸映射  
	for (int j = 0; j < nRows; j++)  
	{  
		pDataMat = resultImage.ptr<uchar>(j);  
		for (int i = 0; i < nCols; i++)  
		{  
			pDataMat[i] = (pDataMat[i] - pixMin) * 255 / (pixMax - pixMin);  
		}  
	}  
	return resultImage;  
}  
//提取颜色环
void colorGet(Mat& inputImage, Mat& outputImage)  
{  
	
	Mat channel[3],R,G,B;  
	split(inputImage,channel);  
	B=channel[0];
	G=channel[1];
	R=channel[2];
	outputImage= Mat::zeros(B.rows,B.cols,CV_8UC1);  

	int rows = outputImage.rows;  
	int cols = outputImage.cols;  
	for(int i = 0;i < rows;i++)  
	{  
		uchar* datab = B.ptr<uchar>(i);  
		uchar* datag = G.ptr<uchar>(i);  
		uchar* datar = R.ptr<uchar>(i);  
		uchar* dataout = outputImage.ptr<uchar>(i);  
		for(int j = 0;j < cols;j++)  
		{  
			if(datab[j]>100&&datab[j]>datag[j]+datar[j])
			dataout[j] = 255;  
		}  
	}  

} 
//填充
void fillHole(const Mat srcBw, Mat &dstBw)
{
	Size m_Size = srcBw.size();
	Mat Temp=Mat::zeros(m_Size.height+2,m_Size.width+2,srcBw.type());//延展图像
	srcBw.copyTo(Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));

	cv::floodFill(Temp, Point(0, 0), Scalar(255));

	Mat cutImg;//裁剪延展的图像
	Temp(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);

	dstBw = srcBw | (~cutImg);
}
//拟合椭圆
void FitEllipse(const Mat inputImage, Mat outputImage)
{
vector<vector<Point> > contours; 
findContours(inputImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  
//我们将在cimage上面绘图  
outputImage = Mat::zeros(inputImage.size(), CV_8UC3);  
//outputImage=inputImage;
for(size_t i = 0; i < contours.size(); i++)  
{  
	//轮廓的边缘点个数  
	size_t count = contours[i].size();  
	//Fitzgibbon的椭圆拟合方法，要求至少6个点，文献：Direct Least Squares Fitting of Ellipses[1999]  
	if( count < 6 )  
		continue;  

	Mat pointsf;  
	//将轮廓中的点转换为以Mat形式存储的2维点集(x,y)  
	Mat(contours[i]).convertTo(pointsf, CV_32F);  

	//最小二次拟合（Fitzgibbon的方法）  
	//box包含了椭圆的5个参数：(x,y,w,h,theta)  
	RotatedRect box = fitEllipse(pointsf);  

	//把那些长轴与短轴之比很多的那些椭圆剔除。  
	if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*3 )  
		continue;  
	//绘制轮廓  
	drawContours(outputImage, contours, (int)i, Scalar::all(255), 1, 8);  

	//绘制椭圆  
	ellipse(outputImage, box, Scalar(0,0,255), 1, CV_AA);  
	//绘制椭圆  
	// ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);  

	//绘制矩形框  
	Point2f vtx[4];  
	//成员函数points 返回 4个矩形的顶点(x,y)  
	box.points(vtx);  
	for( int j = 0; j < 4; j++ )  
		line(outputImage, vtx[j], vtx[(j+1)%4], Scalar(0,255,0), 1, CV_AA);  
}  
imshow("【效果图】", outputImage); 
}
//去除小面积区域
void RemoveSmallRegion(Mat &Src, Mat &Dst,unsigned int AreaLimit, int CheckMode, int NeihborMode)  
{  
//CheckMode: 0代表去除黑区域，1代表去除白区域; NeihborMode：0代表4邻域，1代表8邻域;  
//
//	如果去除小连通区域CheckMode=1,NeihborMode=1去除孔洞CheckMode=0，NeihborMode=0
//
//		记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查 

    int RemoveCount = 0;  
    //新建一幅标签图像初始化为0像素点，为了记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查   
    //初始化的图像全部为0，未检查  
    Mat PointLabel = Mat::zeros(Src.size(), CV_8UC1);  
    if (CheckMode == 1)//去除小连通区域的白色点  
    {   
        for (int i = 0; i < Src.rows; i++)  
        {  
            for (int j = 0; j < Src.cols; j++)  
            {  
                if (Src.at<uchar>(i, j) < 10)  
                {  
                    PointLabel.at<uchar>(i, j) = 3;//将背景黑色点标记为合格，像素为3  
                }  
            }  
        }  
    }  
    else//去除孔洞，黑色点像素  
    {  
        for (int i = 0; i < Src.rows; i++)  
        {  
            for (int j = 0; j < Src.cols; j++)  
            {  
                if (Src.at<uchar>(i, j) > 10)  
                {  
                    PointLabel.at<uchar>(i, j) = 3;//如果原图是白色区域，标记为合格，像素为3  
                }  
            }  
        }  
    }  
  
  
    vector<Point2i>NeihborPos;//将邻域压进容器  
    NeihborPos.push_back(Point2i(-1, 0));  
    NeihborPos.push_back(Point2i(1, 0));  
    NeihborPos.push_back(Point2i(0, -1));  
    NeihborPos.push_back(Point2i(0, 1));  
    if (NeihborMode == 1)  
    {  
        NeihborPos.push_back(Point2i(-1, -1));  
        NeihborPos.push_back(Point2i(-1, 1));  
        NeihborPos.push_back(Point2i(1, -1));  
        NeihborPos.push_back(Point2i(1, 1));  
    }  
    int NeihborCount = 4 + 4 * NeihborMode;  
    int CurrX = 0, CurrY = 0;  
    //开始检测  
    for (int i = 0; i < Src.rows; i++)  
    {  
        for (int j = 0; j < Src.cols; j++)  
        {  
            if (PointLabel.at<uchar>(i, j) == 0)//标签图像像素点为0，表示还未检查的不合格点  
            {   //开始检查  
                vector<Point2i>GrowBuffer;//记录检查像素点的个数  
                GrowBuffer.push_back(Point2i(j, i));  
                PointLabel.at<uchar>(i, j) = 1;//标记为正在检查  
                int CheckResult = 0;  
  
  
                for (unsigned int z = 0; z < GrowBuffer.size(); z++)  
                {  
                    for (int q = 0; q < NeihborCount; q++)  
                    {  
                        CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;  
                        CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;  
                        if (CurrX >= 0 && CurrX<Src.cols&&CurrY >= 0 && CurrY<Src.rows)  //防止越界    
                        {  
                            if (PointLabel.at<uchar>(CurrY, CurrX) == 0)  
                            {  
                                GrowBuffer.push_back(Point2i(CurrX, CurrY));  //邻域点加入buffer    
                                PointLabel.at<uchar>(CurrY, CurrX) = 1;           //更新邻域点的检查标签，避免重复检查    
                            }  
                        }  
                    }  
                }  
                if (GrowBuffer.size()>AreaLimit) //判断结果（是否超出限定的大小），1为未超出，2为超出    
                    CheckResult = 2;  
                else  
                {  
                    CheckResult = 1;  
                    RemoveCount++;//记录有多少区域被去除  
                }  
  
  
                for (unsigned int z = 0; z < GrowBuffer.size(); z++)  
                {  
                    CurrX = GrowBuffer.at(z).x;  
                    CurrY = GrowBuffer.at(z).y;  
                    PointLabel.at<uchar>(CurrY,CurrX)+=CheckResult;//标记不合格的像素点，像素值为2  
                }  
                //********结束该点处的检查**********    
  
  
            }  
        }  
  
  
    }  
  
  
    CheckMode = 255 * (1 - CheckMode);  
    //开始反转面积过小的区域    
    for (int i = 0; i < Src.rows; ++i)  
    {  
        for (int j = 0; j < Src.cols; ++j)  
        {  
            if (PointLabel.at<uchar>(i,j)==2)  
            {  
                Dst.at<uchar>(i, j) = CheckMode;  
            }  
            else if (PointLabel.at<uchar>(i, j) == 3)  
            {  
                Dst.at<uchar>(i, j) = Src.at<uchar>(i, j);  
                  
            }  
        }  
    }  
}  
//相交区域提取
void SameRegion(Mat &Src1, Mat &Src2,Mat &Dst)
{
	//Src1和Src2有顺序，后者只有单个区域
	// 找箭靶连通域  
	vector<Mat> contours(8),contour(2); 
	findContours(Src1,contours, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  
	for(size_t i = 0; i < contours.size(); i++)  
	{  
		drawContours(Dst,contours,i,Scalar(255),1);
		fillHole(Dst,Dst);
		Dst = (Dst & Src2);  
		findContours(Dst,contour, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  
		if (contour.size()>0)  
		{  
			fillHole(Dst,Dst);
			break;
		}  
	} 

} 
// 判断两条线段是否为一条直线，若是，则合并到a中去
bool Oneline(Vec4i &a, Vec4i &b)  
{  
	int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];  
	int deta_x1=x1-x2,deta_y1=y1-y2,deta_x2=x3-x4,deta_y2=y3-y4;
	float l1=sqrt(deta_x1*deta_x1+deta_y1*deta_y1);//a的长度
	float l2=sqrt(deta_x2*deta_x2+deta_y2*deta_y2);//b的长度
	float max;
	vector<float> tem;
	Vec4i new1,endVec;
	for(int i=0;i<2;i++)
	{
		new1[0]=a[i*2];
		new1[1]=a[i*2+1];
		for(int j=0;j<2;j++)
		{			
			new1[2]=b[j*2];
			new1[3]=b[j*2+1];
			tem.push_back(sqrt((new1[0]-new1[2])*(new1[0]-new1[2])+(new1[1]-new1[3])*(new1[1]-new1[3])));
			//tem[2*i+j]=sqrt((new1[0]-new1[2])*(new1[0]-new1[2])+(new1[1]-new1[3])*(new1[1]-new1[3]));//长度
			if(i==0&&j==0)
			{
				max=tem[0];
				continue;
			}
			if(tem[2*i+j]>max)
			{
				max=tem[2*i+j];
			    endVec=new1;//存储最大的线段
			}
		}
	}
	//float addlen1=l1+l2+tem[2*(1-m)+(1-n)];//拼接长度,无重合段
	//float addlen2=l1+l2-tem[2*(1-m)+(1-n)];//拼接长度，有部分重合
	//海伦——秦九昭面积公式
	float p1=(l1+tem[0]+tem[2])/2;//以l1为底边的三角形一
	float p2=(l1+tem[1]+tem[3])/2;//以l1为底边的三角形二
	float S1=sqrt(p1*(p1-l1)*(p1-tem[0])*(p1-tem[2]));
	float S2=sqrt(p2*(p2-l1)*(p2-tem[1])*(p2-tem[3]));
	float h1=S1/l1;
	float h2=S2/l1;
	if(h1<4&&h2<4)//两线段平行且距离相差近
	{
		if(max>l1&&max>l2)//非完全覆盖
		{
		/*	if((addlen1-max)/max<0.05||abs((addlen2-max))/max<0.05)*/
				a=endVec;//修改最后的线段
				return true;
		}
		else
		{
			if(l1<l2)
				a=b;
			return true;
		}
	}
	else{return false;}

}  
//找到四条线
void Fourline(vector<Vec4i> &lines)
{
	bool c=false;
	vector<Vec4i>::iterator itor;  //引入叠加器
	vector<Vec4i> tem;
	while (1)
	{
		if(lines.size()==0)		
			break;
		tem.push_back(lines.back());
		lines.pop_back();//删除最后一个元素
		/*itor=lines.erase(itor);  */
		if(lines.size()==0)
			break;
		for(itor=lines.begin();itor!=lines.end();)//合并线段为一条
		{
			//Vec4i td=*itor,pd=tem.back();
			//c=Oneline(pd, td);
			//tem.back()=pd;
			/*line(Image,Point(pd[0],pd[1]),Point(pd[2],pd[3]),CV_RGB(255,0,0),3);*/
			(Oneline(tem.back(), *itor)==true)?itor=lines.erase(itor):itor++;
		}

	}

	lines=tem;
	int num=tem.size();	
	if(num>4)
	{
		double AB,ab;
		c=true;
		while (c)
		{
			c=false;
			for(int i=1;i<num;i++)
			{
				AB=sqrt((tem[i-1][0]-tem[i-1][2])*(tem[i-1][0]-tem[i-1][2])+(tem[i-1][1]-tem[i-1][3])*(tem[i-1][1]-tem[i-1][3]));
				ab=sqrt((tem[i][0]-tem[i][2])*(tem[i][0]-tem[i][2])+(tem[i][1]-tem[i][3])*(tem[i][1]-tem[i][3]));
				if(AB<ab)
				{
					swap(tem[i],tem[i-1]);
					c=true;
				}
			}
		}
		lines.clear();
		for(int j=0;j<4;j++)
		{
			lines.push_back(tem[j]);
		}
	}

}
//直线求交
Point2f computeIntersect(Vec4i a, Vec4i b)  
{  
	int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3], x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];  
//	float denom;  

	if (float d = ((float)(x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4)))  
	{  
		Point2f pt;  
		pt.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;  
		pt.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;  
		return pt;  
	}  
	else  
		return Point2f(-1, -1);  
}  
//找到四个交点
vector<Point2f> FouP(vector<Vec4i> lines,Mat image)
{
	vector<Point2f> Pointall;
	//计算直线的交点，保存在图像范围内的部分
	for(size_t k=1;k<lines.size();k++)
	{
		int num=0;//对第一根直线交点计数
		int Num=0;
		for (size_t j = 1; j < lines.size(); j++)  
		{  
			Point2f pt = computeIntersect(lines[0], lines[j]);  
			if (pt.x >= 0 && pt.y >= 0 && pt.x <= image.cols && pt.y <= image.rows)             //保证交点在图像的范围之内
			{
				Pointall.push_back(pt);  
				num++;
			}else
			{
				Num=j;//记录不相交的直线
			}
		}  
		if(num==2)
		{
			for(size_t i=1;i<4;i++)
			{
				if(i==Num)
				{
					continue;
				}else
				{
					Point2f pt = computeIntersect(lines[Num], lines[i]);  
					Pointall.push_back(pt); 
				}
			}
			break;//找到点，跳出循环
		}else
		{
			Vec4i temp=lines[0];
			lines[0]=lines[k];
			lines[k]=temp;
			Pointall.clear();//不符合要求的点全部清除
		}
	}
	return Pointall;
}
//确定四个点的顺序
void sortCorners(vector<Point2f>& corners)  
{  
	vector<Point2f> top, bot;  
	Point2f center(0,0);  
	for(size_t i = 0; i < corners.size(); i++)
	{
		center.x+=corners[i].x;
		center.y+=corners[i].y;
	}
	center.x=center.x/corners.size();
	center.y=center.y/corners.size();
	for (size_t i = 0; i < corners.size(); i++)  
	{  
		if (corners[i].y < center.y)  
			top.push_back(corners[i]);  
		else  
			bot.push_back(corners[i]);  
	}  
	corners.clear();  

	if (top.size() == 2 && bot.size() == 2){  
		Point2f tl = top[0].x > top[1].x ? top[1] : top[0];  
		Point2f tr = top[0].x > top[1].x ? top[0] : top[1];  
		Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];  
		Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];  
		
		corners.push_back(tl);  //末端插入
		corners.push_back(tr);  
		corners.push_back(br);  
		corners.push_back(bl);  
	}  
}

//矫正图片畸变
IplImage * jiaozheng( IplImage* image,CvMat *intrinsic,CvMat *distortion )
{
	IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );

	//保存映射矩阵
	cvSave("arrow/Transmtx/mapx.xml",mapx);
	cvSave("arrow/Transmtx/mapy.xml",mapy);
	
	cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

	IplImage *t = cvCloneImage(image);
	cvRemap( t, image, mapx, mapy );     // Undistort image
	return t;
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

//返回靶纸图片的目标图像
Mat uplook(IplImage* image)
{
	//提取蓝色环
	Mat src_B,Image,channel[3],src_b;
	Image=cvarrToMat(image);  
	split(Image,channel);  
	src_B=channel[0];//蓝色通道
	colorGet(Image,src_B);

	

	//填充
	  //输入图像//输出图像//定义操作：MORPH_OPEN为开操作，MORPH_CLOSE为闭操作//单元大小，这里是3*3的8位单元	//开闭操作位置	//开闭操作次数
	morphologyEx(src_B,src_B,MORPH_CLOSE,Mat(15,15,CV_8U),Point(-1,-1),1);
	fillHole(src_B,src_B);

	//找到面积最大的最大连通域  
	vector<Mat> contours(100); 
	findContours(src_B,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  
	
	double maxArea = 0;  
	int m;
	for(size_t i = 0; i < contours.size(); i++)  
	{  
		double area = contourArea(contours[i]);  
		if (area > maxArea)  
		{  
			maxArea = area;  
			m=i;
		}  
	} 
	// draw
	Mat cir_B=Mat::zeros(src_B.size(),CV_8UC1);
	drawContours(cir_B,contours,m,Scalar(255),1);
	fillHole(cir_B,cir_B);
	/*imshow("蓝色圆环",cir_B);*/

	//与原二值图做与，交集为箭靶
	cvtColor(Image,src_b,CV_RGB2GRAY) ;//蓝色通道
	//拉伸
	src_b = contrastStretch(src_b); 
	//二值化，阈值100
	threshold( src_b,  src_b, 100, 255, 0);
	Mat bw= src_b.clone();//储存靶纸二值图
	//取反
	src_b=~src_b;
	//填充
	fillHole(src_b,src_b);
	//取箭靶区域(开操作）
	morphologyEx(src_b,src_b,MORPH_OPEN,Mat(15,15,CV_8U),Point(-1,-1),1);
	////箭靶jb
	Mat jb= Mat::zeros(src_b.rows,src_b.cols,CV_8UC1);
	Mat jB= Mat::zeros(src_b.rows,src_b.cols,CV_8UC1);
	Mat temIm= Mat::zeros(src_b.rows,src_b.cols,CV_8UC1);
	SameRegion(src_b,cir_B,jb);//jb为箭靶与蓝色闭合椭圆相交区域
	vector<Mat> Contours(20),contour(2); 
	findContours(src_b,Contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  
	for(size_t i = 0; i < Contours.size(); i++)  
	{  
		drawContours(temIm,Contours,i,Scalar(255),1);
		fillHole(temIm,temIm);
		temIm = (temIm & jb);  
		findContours(temIm,contour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);  
		if (contour.size()>0)  
		{  
			drawContours(jB,Contours,i,Scalar(255),1);
			fillHole(jB,jB);
			break;
		}  
	} 
	imshow("箭靶", jB); 


	//定位靶纸bz
	Mat bz=(jB&bw);
	bz=(bz|cir_B);//并运算，填充靶环内部
	fillHole(bz,bz);
	imshow("靶纸", bz); 
	/*imwrite("3.bmp",bz);*/
	
	//取箭靶区域(开操作）
	morphologyEx(bz,bz,MORPH_CLOSE,Mat(15,15,CV_8U),Point(-1,-1),1);
	//填充
	fillHole(bz,bz);
	Canny(bz,bz,100,100,3);  //提取轮廓
	int Minlength=bz.rows/15;
	int maxLineGap=30;
	vector<Vec4i> lines(100);  //预先分配内存，避免出错
	vector<Point2f> Pointall;
	HoughLinesP(bz,lines,1,CV_PI/180,Minlength,Minlength,maxLineGap);  
	//1像素分辨能力  1度的角度分辨能力        >70可以检测成连线       30是最小线长  
	//在直线L上的点（且点与点之间距离小于maxLineGap=10的）连成线段，然后这些点全部删除，并且记录该线段的参数，就是起始点和终止点

	//找到四条线
	Fourline(lines);
	/*for(size_t i=0;i<lines.size();i++)
	{
		Vec4i pd=lines[i];
		line(Image,Point(pd[0],pd[1]),Point(pd[2],pd[3]),CV_RGB(255,0,0),3);
	}*/
	imshow("临时直线", Image); 
	/*imwrite("4.bmp",Image);*/
	if(lines.size()!=4)
	{
		printf("找到线段条数：%d\n",lines.size());
		//return -1;
	}
	
	 Pointall=FouP(lines,bz);//计算直线的四个交点
	 sortCorners(Pointall);//按照顺序排列四个点

	 //透视变换
	 std::vector<cv::Point2f> quad_pts;  
 
	 ////【下相机】
	 //quad_pts.push_back(cv::Point2f(0, 200));  
	 //quad_pts.push_back(cv::Point2f(500, 200));  
	 //quad_pts.push_back(cv::Point2f(500, 700));  
	 //quad_pts.push_back(cv::Point2f(0, 700));  
	 //Mat quad=Mat::zeros(700,500,CV_8UC1);
	 //【右相机】
	 quad_pts.push_back(cv::Point2f(200, 0));  
	 quad_pts.push_back(cv::Point2f(700, 0));  
	 quad_pts.push_back(cv::Point2f(700, 500));  
	 quad_pts.push_back(cv::Point2f(200, 500));  
	 Mat quad=Mat::zeros(500,700,CV_8UC1);

	 transmtx = getPerspectiveTransform(Pointall, quad_pts);  
	 warpPerspective(Image, quad, transmtx, quad.size());  
	 imshow("透视变换", quad); 
	 //imwrite("arrow/d1.bmp",quad);
	 return quad;
}
