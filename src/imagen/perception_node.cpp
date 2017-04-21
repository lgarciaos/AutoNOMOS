#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <math.h>
#include <iostream>

#define PI 3.14159265

geometry_msgs::Twist destiny_position;
double rate_hz = 5;
ros::Publisher pub_pose;
ros::Publisher pub_lidar;
std::string nombre;
double distanciaEscalares = 0.0;

struct coords
{
      int object[100];
      double x[100];
      double y[100];
}; 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// PERCEPCION DE LIDAR
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
     //scan->ranges[] are laser readings
     coords perspectiva;
     // printf("\nLIDAR");
     int max =0;
     bool objetoiniciado = false;
     
     
     int obj = -1;
     int v = 0;
     for(int i=0;i<360;i++)
     {
	// evitar la carcaza del carro, carcaza: 52-62, 141-154, 205-218, 295-308
	// || (i>62 && i<141) || (i>154 && i<205) || (i>218 && i<295) ||
	// solo considera el frente
	// SE QUITO LA PERCEPCION TRASERA (i>154 && i<205) ||
	// Y LATERAL (i>62 && i<141) ||  (i>218 && i<295) ||
//	if(i<52 || i>308 )
     	
    // revisar cual es cual
    if ((i>62 && i<141) || (i>154 && i<205) || (i>218 && i<295) || (i>0 && i<52) || (i>308&&i<360))
	{
		// buscar puntos menor a 3m
		if(scan->ranges[i] > 0.1 && scan->ranges[i] < distanciaEscalares) { // antes intensity > 0
			
			//if(!objetoiniciado) {
			//	max = i;
				objetoiniciado = true;
				obj=1;
			//}
			// printf("\n%d, %f", i, scan->ranges[i]);
			double x1 = scan->ranges[i] * cos(i*PI/180);
			double y1 = scan->ranges[i] * sin(i*PI/180);
			
			/*if(max != i)
			{
				double x2 = scan->ranges[max] * cos(max*PI/180);
				double y2 = scan->ranges[max] * sin(max*PI/180);
				double dist = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
				if(abs(max-i)>1 || abs(dist)>0.05)
				{
					// cambio de objeto
					obj++;
					// printf("\ni: %d max: %d, distancia: %f obj: %d", i, max, dist, obj);
				}
			}
			*/
			perspectiva.object[v] = obj;
			perspectiva.x[v] = x1;
			perspectiva.y[v] = y1;
			// printf("\nPuntos: %d (%f,%f)",obj,x1,y1);
			v++;
			
			max = i;
		}
	}
     }
	
	// itera sobre objetos identificados, agrupa vertices por promedio
	double vertobjs [obj][2];
	int nobj = -1;
	int vxobj =0;
	for(int j=0;j<v;j++)
     	{
		if(nobj != perspectiva.object[j]) {
			if(nobj > 0)
			{
				vertobjs[nobj-1][0] /= vxobj;
				vertobjs[nobj-1][1] /= vxobj;
			}
			nobj = perspectiva.object[j];
			vxobj=0;
		}
		vxobj++;
		vertobjs[nobj][0] += perspectiva.x[j];
		vertobjs[nobj][1] += perspectiva.y[j];	

		if(j == v-1){
			vertobjs[nobj][0] /= vxobj;
			vertobjs[nobj][1] /= vxobj;
		}
	}
	
	// vertices de objetos observados
	geometry_msgs::Twist car_follow;
	// printf("\nVertices");
	double x=0.0;
	double y=0.0;
	for(int j=0;j<obj+1;j++)
     	{
		printf("\n Objeto vertice: %d: (%f, %f)",j,vertobjs[j][0], vertobjs[j][1]);
		x += vertobjs[j][0];
		y += vertobjs[j][1];
	}
	printf("\nObjetivo (%f,%f), objs:%d",x,y,obj+1);	
	
	if(x > 0 && y > 0) {
		x=x/(obj+1);
		y=y/(obj+1);
	}

	printf("\nObjetivo (%f,%f)",x,y);

	if(x!=x || y!=y) // parece que es para diferente de undefined
	{}
	else {
		car_follow.linear.x = x;
		car_follow.linear.y = y;
	        double grados = atan2(x,y)*180/PI;
	     
		printf("\n LIDAR PUNTO OBJETIVO: (%f, %f), grados: %f",x, y,grados);
		car_follow.angular.z = grados;
		pub_lidar.publish(car_follow);
	}
}



int main(int argc, char** argv){
	ros::init(argc, argv, "perception_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh_("~");

	ros::Rate loop_rate(rate_hz);


	std::string node_name = ros::this_node::getName();
	// ROS_INFO_STREAM("Obteniendo p");

	priv_nh_.param<double>(node_name+"/distancia_lidar", distanciaEscalares, 1.6);

	/*
	const std::string PARAM1 = "~lidar";
	bool okx = ros::param::get(PARAM1, nombre);
	if(!okx) {
		ROS_FATAL_STREAM("No se pudo obtener el parametro " << PARAM1);
		exit(1);
	}
	else
		std::cout<<"\nSolo concentrarse en lidar:"<<nombre;
	*/

	pub_lidar = nh.advertise<geometry_msgs::Twist>("/target_pose", rate_hz);
	ros::Subscriber scanSub=nh.subscribe<sensor_msgs::LaserScan>("/scan",1, processLaserScan);
	while(nh.ok())
	{
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	return 0;
};

