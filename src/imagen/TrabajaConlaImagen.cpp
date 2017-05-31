/*
Here is a node that listens to a ROS image message topic,
converts the image into a cv::Mat, draws a circle on it and 
displays the image using OpenCV. The image is then republished over ROS. 
*/

   #include <ros/ros.h>
   #include <image_transport/image_transport.h>
   #include <cv_bridge/cv_bridge.h>
   #include <sensor_msgs/image_encodings.h>
   #include <opencv2/imgproc/imgproc.hpp>
   #include <opencv2/highgui/highgui.hpp>
   #include "opencv2/highgui/highgui.hpp"
   #include "opencv2/imgproc/imgproc.hpp"
   #include <tf2_ros/transform_listener.h>
   #include <geometry_msgs/TransformStamped.h>
   #include <geometry_msgs/Twist.h>
   #include <pcl_ros/point_cloud.h>
   #include <pcl/point_types.h>
   #include <boost/foreach.hpp>
   #include <geometry_msgs/Pose.h>
   #include <iostream>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;
   using namespace cv_bridge;//LA
   
  static const char WINDOW[] = "Image window";
   
   geometry_msgs::Twist goal;
   double rate_hz = 10;
   class ImageConverter
   {
   	ros::NodeHandle nh_;
   	image_transport::ImageTransport it_;
   	image_transport::Subscriber image_sub_;
   	image_transport::Publisher image_pub_;
   	ros::NodeHandle nh;
   	ros::Publisher pub_destino;


   public:
   	ImageConverter()
   	: it_(nh_)
   	{
//   		image_pub_ = it_.advertise("out", 1);
  image_sub_ = it_.subscribe("/app/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);//estaba suscrito a "in"
  
  pub_destino = nh.advertise<geometry_msgs::Twist>("/target_position_topic", rate_hz); 

//  cv::namedWindow(WINDOW);
}

~ImageConverter()
{
//	cv::destroyWindow(WINDOW);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat imgrec1 = cv::Mat(cv_ptr->image, cv::Rect(0,320,295,35)).clone();
	cv::Mat imgrec2 = cv::Mat(cv_ptr->image, cv::Rect(345,320,295,35)).clone();
       //Código original del ejemplo

     /* if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
         cv::circle(cv_ptr->image, cv::Point(50, 50), 1, CV_RGB(255,0,0));
   */

//Inicia lo copiado de Houges


/*       const char* filename = argc >= 2 ? argv[1] : "pic1.jpg";
 Mat src = imread(filename, 0);
 if(src.empty())
 {
     help();
     cout << "can not open " << filename << endl;
     return -1;
 }
*/
//Cambiando todos los src por cv_ptr - ahora no
//PROBLEMA CON SRC, NO SE PUEDE SUSTITUIR CON cv_ptr, ENTONCES CON QUÉ?



	Mat dst1, cdst1, dst2, cdst2;
 Canny(imgrec1, dst1, 50, 200, 3); //a escala de grises     C
 cvtColor(dst1, cdst1, CV_GRAY2BGR); //contornos

Canny(imgrec2, dst2, 50, 200, 3); //a escala de grises     C
 cvtColor(dst2, cdst2, CV_GRAY2BGR); //contornos

 #if 0
  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
 #else
 vector<Vec4i> lines1;
  HoughLinesP(dst1, lines1, 1, CV_PI/180, 16, 10, 10 );//calcula las lineas  (x,x,x,x, min de puntos para linea, ni idea, longitud min lineas?)
  double xmax = 0;
  for( size_t i = 0; i < lines1.size(); i++ ) //llena el vector lines con lineas
  {
  	Vec4i l1 = lines1[i];
  	line( cdst1, Point(l1[0], l1[1]), Point(l1[2], l1[3]), Scalar(233,241,0), 2, CV_AA);
    //Points de line son el primer y el ultimo punto de cada linea. l viene en formato [xinicio, yinicio, xfin,yfin]
    //std::cout << l; // Hay que publicar el promedio de los primeros dos puntos
  	if(l1[0]>xmax)
  		xmax = l1[0];
  }//line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), COLOR DE LA LINEA Scalar(azul,verde,rojo), grosor de linea, CV_AA);


  vector<Vec4i> lines2;
  HoughLinesP(dst2, lines2, 1, CV_PI/180, 16, 10, 10 );//calcula las lineas  (x,x,x,x, min de puntos para linea, ni idea, longitud min lineas?)
  double xmin = 640;
  for( size_t i = 0; i < lines2.size(); i++ ) //llena el vector lines con lineas
  {
  	Vec4i l2 = lines2[i];
  	line( cdst2, Point(l2[0], l2[1]), Point(l2[2], l2[3]), Scalar(233,241,0), 2, CV_AA);
    //Points de line son el primer y el ultimo punto de cada linea. l viene en formato [xinicio, yinicio, xfin,yfin]
    //std::cout << l; // Hay que publicar el promedio de los primeros dos puntos
  	if((l2[0]+345)<xmin)
  		xmin = l2[0]+345;
  }//line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), COLOR DE LA LINEA Scalar(azul,verde,rojo), grosor de linea, CV_AA);
 #endif
//imshow("source", imgrec);
  double target = (xmin+xmax)/2;
  
  goal.linear.x = target;
  goal.linear.y = 100;
  goal.linear.z = 0;

  ROS_INFO_STREAM(goal);
 //waitKey();

 //Hasta aqui lo copiado de Houges

  /*
  cv::circle(cv_ptr->image, cv::Point(target, 250), 1, CV_RGB(255,0,0),5);
    cv::circle(cv_ptr->image, cv::Point(xmin, 250), 1, CV_RGB(255,0,0));
    cv::circle(cv_ptr->image, cv::Point(320, 250), 1, CV_RGB(0,0,255),2);
    cv::circle(cv_ptr->image, cv::Point(xmax, 250), 1, CV_RGB(255,0,0));
    
    imshow("detected lines 1", cdst1);
    imshow("detected lines 2", cdst2);
   cv::imshow(WINDOW, cv_ptr->image);
  */
  
  //cv::waitKey(3);

//  image_pub_.publish(cv_ptr->toImageMsg());
  pub_destino.publish(goal);
}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
