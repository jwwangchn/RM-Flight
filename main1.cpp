//System Header
#include <iostream>
#include <string>
#include <cstring>
#include <pthread.h>
#include <unistd.h>
#include <fstream>
#include <semaphore.h>
#include <stdio.h>
#include <signal.h>
#include <opencv2/opencv.hpp>

//DJI Linux Application Headers
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxFlight.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

//DJI OSDK Library Headers
#include "DJI_API.h"

//User Library Headers
#include "CircleDetect.h"
#include "uart.h"
#include "PID.h"
#include "BaseArmorDetect.h"
#include "LedController.h"
#include "para.h"
#include "aruco/aruco.h"
#include "RMVideoCapture.hpp"
#include "Predictor.hpp"

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;
 

//DEBUG parameter
#define SHOWIMG 0


pthread_rwlock_t img_spinlock;


int r=0;

bool RUNNING_MODE=false;
bool SHUT_MODE=false;
bool LAND_MODE=false;
int BASEDETECT_MODE=0;

void sig_callback(int signum)
{
    printf("exit read image\n");
    RUNNING_MODE=false;
    return;
}

struct ImgData
{
    Mat img;
    unsigned int framenum;
};
ImgData image;

static void *readLoop(void *data)
{
#ifndef USE_VIDEO
    RMVideoCapture cap("/dev/video0", 3);
    cap.setVideoFormat(640, 480, 1);
    cap.setExposureTime(0, 70);//越高越亮
    cap.startStream();
    cap.info();
    //cap.setpara(0,15,100,120);
    cap.getCurrentSetting();
    ImgData img;
    image.img.create(480,640,CV_8UC3);
    cap>>image.img;
    
    double start,end;
    RUNNING_MODE=true;
    while(RUNNING_MODE)
    {
	cap>>img.img;
	img.framenum=cap.getFrameCount();
	pthread_rwlock_wrlock(&img_spinlock);
	image.img=img.img.clone();
	image.framenum=img.framenum;
	pthread_rwlock_unlock(&img_spinlock);
    }
    printf("get_images_loop thread exit! \n");
    cap.closeStream();
#else
    ImgData img;
    image.img.create(480,640,CV_8UC3);
    RUNNING_MODE=true;
    string str;
    int n=312;
    while(RUNNING_MODE)
    {
	
	str=format("/home/ubuntu/Documents/Flight2d/build/%d.jpg",n);
	img.img=imread(str);
	img.framenum=n;
	n++;
	if(img.img.empty())
	    RUNNING_MODE=false;
	pthread_rwlock_wrlock(&img_spinlock);
	image.img=img.img.clone();
	image.framenum=img.framenum;
	pthread_rwlock_unlock(&img_spinlock);
	usleep(15000);
    }
    
    
#endif
}





static void *detectLoop(void *data)
{
    //飞机初始化
    LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
    CoreAPI* api = new CoreAPI(serialDevice);
    Flight* flight = new Flight(api);
    LinuxThread read(api, 2);
    Camera* camera=new Camera(api);
    VirtualRC* vrc=new VirtualRC(api);
    BroadcastData bcd;
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
    int setupStatus = setup(serialDevice, api, &read);
    
    //图像初始化
    ImgData image_get;
    Mat image_gray;
    CircleDetect circledect(Size(640,480),1);
    LedController led("/sys/class/gpio/gpio158/value");
    LedController ledr("/sys/class/gpio/gpio157/value");
    
    for(int i=0;i<4;i++)
    {
	led.ledON();
	usleep(100000);
	led.ledOFF();
	usleep(100000);
    }
    
    
    //PID
    double dx=0,dy=0;
    PIDctrl pidX(aPx,aIx,aDx,0.5);
    PIDctrl pidY(aPy,aIy,aDy,0.5);
    PIDctrl pidX2(bPx,bIx,bDx,0.5);
    PIDctrl pidY2(bPy,bIy,bDy,0.5);
    
    //串口
    char device[]="/dev/ttyTHS0";
    int fd=uartOpen(device);    
    if (fd==-1)
	printf("device error");
    uartSet(fd);
    static unsigned char motor_flag;

    //二维码
    aruco::MarkerDetector MDetector;
    vector< aruco::Marker > markers;
    aruco::Dictionary::DICT_TYPES  dict=aruco::Dictionary::DICT_TYPES::TAG16h5;
    MDetector.setDictionary(dict);//sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    MDetector.setThresholdParams(10, 9);
    MDetector.setThresholdParamRange(2, 0);
    
    double start=0,end=0;
    
    api->setControl(0,1);
    int landCount=0;
    
    while (!RUNNING_MODE)
	sleep(2);
    
    
    while(RUNNING_MODE)
    {
	end=getTickCount();
	//cout<<(end-start)/getTickFrequency()<<endl;
	start=getTickCount();
	
	bcd=api->getBroadcastData(); 
	pthread_rwlock_rdlock (&img_spinlock);
	image_get.img=image.img.clone();
	image_get.framenum=image.framenum;
	pthread_rwlock_unlock(&img_spinlock);	
	cvtColor(image_get.img,image_gray,CV_RGB2GRAY);
	
	
	if (bcd.rc.gear == -4545)//Circle Detection
	{
	    circledect.setImg(image_gray);
	 
	    if (circledect.circleDetection()==true)
	    {
		dy=circledect.center.x-cam_x;//60
		dx=cam_y-circledect.center.y+circledect.radius/1.54;// 25
		if (dx<0)
		{
		    led.ledON();
		    ledr.ledOFF();
		}
		else
		{
		    ledr.ledON();
		    led.ledOFF();
		}
		r=circledect.radius;
		
		if (LAND_MODE && bcd.rc.mode == 8000)
		{
		    api->setControl(1,1);  
		    dx=cam_y-circledect.center.y+88;
		    cout<<"Frame:"<<image_get.framenum<<" *Prepare land*  R:"<<r<<"  C:"<<circledect.center<<"  PID: "<<pidX2.output<<"  "<<pidY2.output<<"  dx: "<<dx<<"  dy:"<<dy<<"  V: "<<bcd.v.x<<" "<<bcd.v.y<<" "<<bcd.v.z<<"  H:"<<bcd.pos.height<<endl;
		    pidX2.calc(dx); 
		    pidY2.calc(dy);
		    
		    if (r<140)
			flight->setMovementControl(0x4B,pidX2.output,pidY2.output,-0.1,0);
		    
		    if (abs(dx) < 25 && abs(dy) < 20)//15
		    {
			landCount++;
			if (landCount>6)//4
			{
			    ackReturnData landingack=landing(api,flight,1);
			    LAND_MODE=false;
			}
			
			    flight->setMovementControl(0x4B,0,0,0,0);
		    }
		    else
		    {
			landCount=0;
			flight->setMovementControl(0x4B,pidX2.output,pidY2.output,0,0);
		    }
		}
		else if (!LAND_MODE && bcd.status == 3 && bcd.rc.mode == 8000)
		{
		    api->setControl(1,1);                             
		    
		    pidX.calc(dx);
		    pidY.calc(dy);
		    
		    if (abs(dx)<30 && abs(dy)<30 && abs(bcd.v.x)<0.08 && abs(bcd.v.y)<0.08)
		    {
			flight->setMovementControl(0x4B,pidX.output,pidY.output,-0.15,0);
			cout<<"Frame:"<<image_get.framenum<<" *Circle down*  R:"<<r<<"  PID: "<<pidX.output<<"  "<<pidY.output<<"  dx:"<<dx<<"  dy:"<<dy<<"  V:"<<bcd.v.x<<" "<<bcd.v.y<<endl;
			if (r>135)
			    LAND_MODE=true;	
		    }
		    else
		    {
			flight->setMovementControl(0x4B,pidX.output,pidY.output,0,0);
			cout<<"Frame:"<<image_get.framenum<<" *Circle*  R:"<<r<<"  PID: "<<pidX.output<<"  "<<pidY.output<<"  dx:"<<dx<<"  dy:"<<dy<<"  V:"<<bcd.v.x<<" "<<bcd.v.y<<endl;
			
		    }
		}
		
	    }
	    else
	    {
		led.ledOFF();
		ledr.ledOFF();
		cout<<"lost circle"<<endl;
		dx=0;dy=0;
		//	pidX.reset();
		//	pidY.reset();
		pidX2.reset();
		pidY2.reset();
		if (bcd.rc.mode == 8000 && bcd.status == 1)//Catch ball
		{		
		    if (bcd.rc.pitch>8000)//电机正转
		    {
			motor_flag=0xCC;
			uartSend(fd,&motor_flag,1);
		    }
		    else if (bcd.rc.roll<-8000)//电机停
		    {
			motor_flag=0xDD;
			uartSend(fd,&motor_flag,1);
		    }
		    else if (bcd.rc.pitch<-8000)//电机反
		    {
			motor_flag=0xEE;
			uartSend(fd,&motor_flag,1);
		    }
		    usleep(20000);
		}
	    }
	    
	    
	    #if SHOWIMG
	    circledect.drawCircle(image_get.img);
	    imshow("img",image_get.img);
	    waitKey(1);
	    #endif
	}
	else //TagsDetection
	{
	    pidX.reset();
	    pidY.reset();
	    
	    markers= MDetector.detect(image_get.img);
	    cout<<"Marker Find:"<<markers.size()<<endl;
	    
	    if (markers.size()>1)
	    {
		for (unsigned int i = 0; i < markers.size(); i++) 
		{
		    markers[i].draw(image_get.img, Scalar(0, 0, 255));
		    if (markers[i].id==0)
		    {  
			ackReturnData landingack=landing(api,flight,1);
		    }
		}
	    }
	    
	    #if SHOWIMG
	    imshow("img",image_get.img);
	    waitKey(1);
	    #endif
	}

	
    }
    
    
    
    int cleanupStatus = cleanup(serialDevice, api, flight, &read);
    if (cleanupStatus == -1)
    {
	cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
	return 0;
    }
    cout << "Program exited successfully." << endl;
}



int main(int argc, char **argv)
{
        
    signal(SIGINT,sig_callback);
    
    cout<<"For x: P-"<<aPx<<"  I-"<<aIx<<"  D-"<<aDx<<"\nFor y: P-"<<aPy<<"  I-"<<aIy<<"  D-"<<aDy<<endl;
    
    pthread_attr_t attr;
    struct sched_param schedparam;
    pthread_t read_thread;
    pthread_t detect_thread;    
    pthread_t tags_thread;
    
    pthread_rwlock_init(&img_spinlock, 0);
    
    
    if(0 != geteuid())
    {
	printf("Please run ./test as root!\n");
	
	return -1;
    }
    
    
    if (pthread_create(&read_thread, NULL, readLoop, NULL) != 0)
    {
	printf("Read_thread create");
	return -1;
    }
    

    
    if (pthread_create(&detect_thread, NULL, detectLoop, NULL) != 0)
    {
	printf("detect_thread create");
	return -1;
    }
    
    LedController led("/sys/class/gpio/gpio158/value");
    for (int i=0;i<4;i++){
	led.ledON();
	usleep(200000);
	led.ledOFF();
	usleep(200000);
    }
    
    
    pthread_join(detect_thread, NULL);/*wait for read_thread exit*/
    pthread_join(read_thread, NULL);/*wait for read_thread exit*/
    
    
    sleep(3);
    
    return 0;
}
