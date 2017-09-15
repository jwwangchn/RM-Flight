#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <unistd.h>
#include <algorithm> 
#include <opencv2/opencv.hpp>
#include <sys/time.h>


using namespace std;
using namespace cv;

//#define _DEBUG

#define H 480
#define W 640

#define Gray_th 210
#define Rgb_th 110
#define contour_size_min 20  //灯条最小
#define contour_size_max 55 //灯条最大
#define contour_size_ratio 3 //长宽比
#define light_d_angle 10
#define light_d_height 15
#define light_ratio_max 7
#define light_ratio_min 4
#define light_rect_dangle 12

bool compare_angle(const RotatedRect rect1,const RotatedRect rect2)
{
    return (rect1.angle<rect2.angle);
}

bool compare_size(const RotatedRect rect1,const RotatedRect rect2)
{
    return ((rect1.size.height*rect1.size.width)<(rect2.size.height*rect2.size.width));
}


class BaseArmorDetection
{
public:
    Mat img_gray,img_rgb;
    vector<Mat> img_bgr;  
    Rect roi;
    Point center,precenter;
    bool RedorBlue;
    
    struct timeval tv;
    vector<Point2f> pt;
    int nCount;
    
    Mat preimg;
    
    BaseArmorDetection(const Size &size,bool Red)
    {
	img_gray.create(size,CV_8UC1);
	roi=Rect(0,0,size.width,size.height);
	RedorBlue=Red;
	nCount=0;
	precenter=Point(0,0);
    }
    
    void setImg(Mat &gray,Mat &rgb);
    bool colorDetection();
    bool opticalflowDection();
    Point detect();
    void adjectRoi();
    float ptDistance(Point2f&,Point2f&);
};


void BaseArmorDetection::setImg(Mat &gray,Mat &rgb)
{
    preimg=img_rgb.clone();
    img_rgb=rgb.clone();
    
    img_gray=gray.clone();
    split(rgb,img_bgr);
    
}

bool BaseArmorDetection::colorDetection()
{
    Mat roi_grayt,roi_gray,roi_rb;
    Mat rb;
    pt.clear();
    center=Point(0,0);
    
    if (RedorBlue)
	rb=img_bgr[2]-img_bgr[0];
    else
	rb=img_bgr[0]-img_bgr[2];
    
    threshold(img_gray(roi), roi_grayt, Gray_th, 255, THRESH_BINARY);	
//     threshold(rb(roi), roi_rb, Rgb_th, 255, THRESH_BINARY);
//     //  dilate(roi_gray,roi_gray,getStructuringElement( 0,Size(2,2), Point(1, 1)),Point(-1, -1),1);
//     dilate(roi_rb,roi_rb,getStructuringElement( 0,Size(5,5), Point(1, 1)),Point(-1, -1),2);
//     imshow("gray",roi_gray);
//     imshow("rbb",rb);
//     
//     bitwise_and(roi_rb,roi_gray,add);//灯柱
//     
    
    vector<vector<Point> > contours;  
    vector<vector<Point> > contours_ok; 
    vector<Vec4i> hierarchy;  
    roi_gray=roi_grayt.clone();
    findContours(roi_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
 
    
    //lightbar
    if (contours.size()<4)
	return false;
    vector<RotatedRect> contours_rect;		//Find fit courtous
    for(vector<vector<Point> >::iterator iter=contours.begin();iter!=contours.end();)
    {
	//cout<<iter->size()<<"   ";
	if (iter->size()<contour_size_min || iter->size()>contour_size_max)
	{
	    iter=contours.erase(iter);
	}
	else
	{
	    
	    RotatedRect fitrect=fitEllipse(*iter);
	   	
	    if((fitrect.size.width/fitrect.size.height >contour_size_ratio ||fitrect.size.height/fitrect.size.width>contour_size_ratio)&& min(fitrect.size.width,fitrect.size.height)>2.5)
	    {
		contours_rect.push_back(fitrect);
		#ifdef _DEBUG
 		string str=format("(%d:%f Y:%f)",iter->size(),fitrect.size.width/fitrect.size.height,min(fitrect.size.width,fitrect.size.height));
		putText(roi_grayt,str,fitrect.center,CV_FONT_HERSHEY_COMPLEX, 0.4, CV_RGB(255, 255, 255), 1);
		cout<<"rect  "<<fitrect.angle<<"  "<<fitrect.size.width<<"   "<<fitrect.size.height<<endl;
		#endif
	    }
	    ++iter;
	}
    }
    
    cout<<"ss"<<contours_rect.size()<<endl;
    #ifdef _DEBUG
    imshow("gray",roi_grayt);
    
    drawContours(roi_gray,contours,-1, Scalar(120), CV_FILLED);
    Scalar color(rand() & 255, rand() & 255, rand() & 255);
    for(int i=0;i<contours_rect.size();i++)
    {
	Point2f vertices[4];
	contours_rect[i].points(vertices);
	for (int i = 0; i < 4; i++)
	{
	    // line(add, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 255, 255), 1);
	}
	string str=format("(%d:%f Y:%f)",i,contours_rect[i].angle,contours_rect[i].size.width);
	putText(roi_gray,str,contours_rect[i].center,CV_FONT_HERSHEY_COMPLEX, 0.4, CV_RGB(255, 255, 255), 1);
	
    }
    
    #endif
    //armor
    if (contours_rect.size()<4)
	return false;
    
    vector<RotatedRect> armor_rect;
    for(int i=0;i<contours_rect.size()-1;i++)
    {
	for(int j=i+1;j<contours_rect.size();j++)
	{
	    if (abs(contours_rect[i].angle-contours_rect[j].angle)>light_d_angle && abs(abs(contours_rect[i].angle-contours_rect[j].angle)-180)>light_d_angle)
	    {
		#ifdef _DEBUG
		cout<<"refuse1  :"<<i<<":"<<j<<contours_rect[i].angle<<contours_rect[i].size.width<<"  "<<contours_rect[j].angle<<endl;
		#endif
		continue;
	    }
	    
	    float dheight=max(contours_rect[i].size.height,contours_rect[i].size.width)-max(contours_rect[j].size.height,contours_rect[j].size.width);
	    
	    if (dheight>light_d_height)
	    {
		#ifdef _DEBUG
		cout<<"refuse2  :"<<i<<":"<<j<<dheight<<endl;
		#endif
		continue;
	    }
	    
	    Point2f verticesi[4];
	    contours_rect[i].points(verticesi);
	    Point2f verticesj[4];
	    contours_rect[j].points(verticesj);
	    vector<Point2f> rect_min;
	    for (int k=0;k<4;k++)
	    {
		rect_min.push_back(verticesi[k]);
		rect_min.push_back(verticesj[k]);
	    }
	    RotatedRect rect_armor=minAreaRect(rect_min);
	    
	    if (max(rect_armor.size.width,rect_armor.size.height)/min(rect_armor.size.width,rect_armor.size.height)>light_ratio_max||max(rect_armor.size.width,rect_armor.size.height)/min(rect_armor.size.width,rect_armor.size.height)<light_ratio_min)
	    {
		#ifdef _DEBUG
		cout<<"refuse3:  "<<i<<":"<<j<<max(rect_armor.size.width,rect_armor.size.height)/min(rect_armor.size.width,rect_armor.size.height)<<endl;
		#endif
		continue;
	    }
	    
	    if (abs(rect_armor.angle-contours_rect[i].angle+90)>light_rect_dangle && abs(rect_armor.angle-contours_rect[i].angle)>light_rect_dangle && abs(rect_armor.angle-contours_rect[i].angle+180)>light_rect_dangle)
	    {
		#ifdef _DEBUG
		cout<<"refuse4:  "<<i<<":"<<j<<rect_armor.angle-contours_rect[i].angle<<endl;
		#endif
		continue;
	    }
	    #ifdef _DEBUG
	    Point2f vertices[4];
	    rect_armor.points(vertices);
	    for (int i = 0; i < 4; i++)
	    {
		line(roi_gray, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 255, 255), 1);
	    }
	    #endif
	    armor_rect.push_back(rect_armor);
	    
	}
	
	
    }
    
    //double armor
    if (armor_rect.size()>1){
	for (int i=0;i<armor_rect.size()-1;i++)
	{
	    for (int j=i+1;j<armor_rect.size();j++)
	    {
		if(abs(armor_rect[i].size.width-armor_rect[j].size.width)<5 &&abs(armor_rect[i].size.height-armor_rect[j].size.height)<5 )
		{
		    center.x=(armor_rect[i].center.x+armor_rect[j].center.x)/2+roi.x;
		    center.y=(armor_rect[i].center.y+armor_rect[j].center.y)/2+roi.y;
		    circle(roi_gray,Point((armor_rect[i].center.x+armor_rect[j].center.x)/2,(armor_rect[i].center.y+armor_rect[j].center.y)/2),5,Scalar(255),3);
		}
		
	    }
	}
    }
    else if (armor_rect.size()==1)
    {
	center.x=armor_rect[0].center.x+roi.x;
	center.y=armor_rect[0].center.y+roi.y;
    }
    #ifdef _DEBUG
    imshow("3",roi_gray);
    waitKey(1);
    #endif
    if (center.x!=0)
    {
	return true;
    }
    else 
	return false;
    
    
    
    
    
    
    
}

bool BaseArmorDetection::opticalflowDection()
{
    vector<Point2f> prePt,curPt;
    vector<uchar> status;
    vector<float> err;
    prePt.push_back(Point2f(precenter.x-roi.x,precenter.y-roi.y));
    // cout<<prePt.size()<<endl;
    calcOpticalFlowPyrLK(preimg(roi),img_rgb(roi),prePt,curPt,status,err);
    center.x=curPt[0].x+roi.x;
    center.y=curPt[0].y+roi.y;
    // cout<<"OP:"<<(short)status[0]<<endl;
    return (short)status[0];
}
Point BaseArmorDetection::detect()
{
    //cout<<"PPPPT:"<<center<<precenter<<endl;
    
    if (precenter.x==0)
    {
	colorDetection();
	precenter=center;
	roi=Rect(0,0,W,H);
	center=Point(0,0);
	nCount=0;
    }
    else
    {
	if(colorDetection())
	{
	    precenter=center;
	    nCount=0;
	}
	else
	{
	    nCount++;
	    if (nCount>8)
	    {
		precenter=Point(0,0);
		roi=Rect(0,0,W,H);
		center=Point(0,0);
	    }
	    else
	    {
		if(!opticalflowDection()||center.x<20||center.x>620||center.y<20||center.y>460)
		{  
		    
		    precenter=Point(0,0);
		    roi=Rect(0,0,W,H);
		    center=Point(0,0);
		}
		else
		{
		    precenter=center;
		}
	    }
	    
	    
	    
	}
    }
    adjectRoi();
    return center;
    
}
void BaseArmorDetection::adjectRoi()
{
    
    int rectwidth=200;
    if (center.x!=0)
    {
	roi.width=rectwidth;
	roi.height=rectwidth;
	
	if ((center.x-rectwidth/2)<0)
	    roi.x=0;
	else if ((center.x+rectwidth/2)>W)
	    roi.x=(W-rectwidth);
	else
	    roi.x=center.x-rectwidth/2;
	
	if ((center.y-rectwidth/2)<0)
	    roi.y=0;
	else if ((center.y+rectwidth/2)>H)
	    roi.y=(H-rectwidth);
	else
	    roi.y=center.y-rectwidth/2;
	
	
    }
    else
	roi=Rect(0,0,W,H);
}

float BaseArmorDetection::ptDistance(Point2f& p1,Point2f& p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)* (p1.y-p2.y));
    
}
