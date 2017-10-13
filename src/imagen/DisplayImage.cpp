#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>
#include <stdio.h>
#include <math.h>

#define PI 3.14159265

std::vector<cv::Point> old;
ros::Publisher pub_pose;
double rate_hz = 5;
sensor_msgs::ImagePtr imgmsg;
ros::Publisher pub_image;

struct datos{
	double p;
	int e;
	double x;
	double y;
};

void dibujarPoligonoCarro( cv::Mat img )
{
    int lineType = 8;
    /* Create some points */
    cv::Point rook_points[1][28];
    rook_points[0][0] = cv::Point( 0, 480 );
    rook_points[0][1] = cv::Point( 0, 468 );
    rook_points[0][2] = cv::Point( 13, 455 );
    rook_points[0][3] = cv::Point( 14, 450 );
    rook_points[0][4] = cv::Point( 43, 420 );
    rook_points[0][5] = cv::Point( 109, 390 );
    rook_points[0][6] = cv::Point( 141, 381 );
    rook_points[0][7] = cv::Point( 186, 375 );
    rook_points[0][8] = cv::Point( 184, 360 );
    rook_points[0][9] = cv::Point( 216, 360 );
    rook_points[0][10] = cv::Point( 216, 343 );
    rook_points[0][11] = cv::Point( 216, 343 );
    rook_points[0][12] = cv::Point( 233, 343 );
    rook_points[0][13] = cv::Point( 234, 357 );
    rook_points[0][14] = cv::Point( 253, 357 );
    rook_points[0][15] = cv::Point( 253, 357 );
    rook_points[0][16] = cv::Point( 254, 369 );
    rook_points[0][17] = cv::Point( 254, 369 );
    rook_points[0][18] = cv::Point( 410, 370 );
    rook_points[0][19] = cv::Point( 410, 370 );
    rook_points[0][20] = cv::Point( 423, 350 );
    rook_points[0][21] = cv::Point( 439, 359 );
    rook_points[0][22] = cv::Point( 480, 357 );
    rook_points[0][23 ] = cv::Point( 486, 376 );
    rook_points[0][24 ] = cv::Point( 546, 383 );
    rook_points[0][25 ] = cv::Point( 640, 423 );
    rook_points[0][26 ] = cv::Point( 640, 480 );
    const cv::Point* ppt[1] = { rook_points[0] };
    int npt[] = { 27 };
    cv::fillPoly( img,
              ppt,
              npt,
                  1,
              cv::Scalar( 0, 0, 0 ),
              lineType );
}



// lado=0 izquierda lado=1 derecha de referencia
int masCercano(std::vector<cv::Point> puntos, cv::Point referencia, int restriccionEnX, int cercanos[], int lenArray, int lado){	
	int idxmasCercano = -1;
	int distanciaMin = 600;

	int mayorY = 0;
	if(referencia.x==320 && referencia.y==480)
	{
		mayorY = 220;
		if(lado == 1)
			restriccionEnX = 400;
	}

	for(size_t i = 0; i < puntos.size(); i++ )
	{
		cv::Point p = puntos[i];
		if(lado == 0){
			
			if(p.x < restriccionEnX && p.y > mayorY && p.y < referencia.y)
			{			
				double distancia = sqrt(pow(p.x-referencia.x,2)+pow(p.y-referencia.y,2));
				bool yaExiste = false;	
				for(int j=0;j<lenArray;j++) {
					if(cercanos[j] == i)
						yaExiste = true;
				}
				if((distancia > 30) && (distancia < distanciaMin) && !yaExiste)
				{
					distanciaMin = distancia;
					idxmasCercano = i;
				}
			}
		}
		else {
			if(p.x > restriccionEnX && p.y > mayorY && p.y < referencia.y)
			{			
				double distancia = sqrt(pow(p.x-referencia.x,2)+pow(p.y-referencia.y,2));
				bool yaExiste = false;	
				for(int j=0;j<lenArray;j++) {
					if(cercanos[j] == i)
						yaExiste = true;
				}
				if((distancia > 80) && (distancia < distanciaMin) && !yaExiste)
				{
					distanciaMin = distancia;
					idxmasCercano = i;
				}
			}
		}
	}
	printf("\nmas cercano de (%d,%d) es %d: (%d,%d)", referencia.x, referencia.y, idxmasCercano, puntos[idxmasCercano].x, puntos[idxmasCercano].y);
	return idxmasCercano;
}


datos pendienteLado(cv::Mat imagen, std::vector<cv::Point> objetos, int lado, int centroX, int centroY, int noPuntos, int limite){

	// encontrar punto mas cercano a la izquierda
	int color = 100;
	if(lado==1)
		color = 255;
	int idxCercanos[noPuntos];
	int encontrados = 1;
	idxCercanos[0]=masCercano(objetos,cv::Point(centroX,centroY),320, idxCercanos, noPuntos, lado);
	for(int i=1;i<noPuntos;i++){
		if(lado==0)
			idxCercanos[i]=masCercano(objetos,objetos[idxCercanos[i-1]],640, idxCercanos, noPuntos, lado);
		else
			idxCercanos[i]=masCercano(objetos,objetos[idxCercanos[i-1]],0, idxCercanos, noPuntos, lado);
		if(idxCercanos[i]>-1)
			encontrados++;
	}
	// printf("\npunto cercanos OK");
	
	// obtener la pendiente de los puntos mas cercanos entre si
	/*
	double noventa = 1.57; // 90 grados
	double pendientes[noPuntos];
	for(int i=1;i<noPuntos;i++){
		if(idxCercanos[i] != -1 && idxCercanos[i-1] != -1)
		{
			double yreal1 = 480 - objetos[idxCercanos[i]].y;
			double yreal2 = 480 - objetos[idxCercanos[i-1]].y;
			
			double y = (yreal1-yreal2);
			double x = (objetos[idxCercanos[i]].x-objetos[idxCercanos[i-1]].x);	
			printf("\n y,x: (%f,%f)", y,x);
			// pendientes[i-1]=(yreal2-yreal1)/(objetos[idxCercanos[i-1]].x-objetos[idxCercanos[i]].x);
			
			if(x!=0) {
				double rad = atan2(y,x) * 180 / PI; // grados, al parecer ya resta 90
				pendientes[i-1] = rad;
				// printf("\nRadianes: %f", rad);
			}
			else
				pendientes[i-1] = noventa;
			
			// mostrar punto del que se obtuvo la pendiente		
			cv::line(imagen, objetos[idxCercanos[i-1]], objetos[idxCercanos[i-1]], cv::Scalar(100,color,255), 8, CV_AA);
		}
	}
	// mostrar punto del que se obtuvo la pendiente
	if(idxCercanos[noPuntos-1] != -1 && idxCercanos[noPuntos] != -1)
	{
		cv::line(imagen, objetos[idxCercanos[noPuntos-1]], objetos[idxCercanos[noPuntos-1]], cv::Scalar(100,color,255), 8, CV_AA);
	}

	
	// printf("\npendientes OK");

	// promediar pendientes
	double pendiente=0.0;
	for(int i=0;i<noPuntos-1;i++){
		pendiente += pendientes[i];
		printf("\n Grados: %f", pendientes[i]);
	}
	pendiente/=noPuntos-1;
	
	/*
	if(pendiente > 0)
		pendiente = limite - pendiente;
	else if(pendiente < 0)
		pendiente = -limite - pendiente;
	*/

	// printf("encontrados: %d", encontrados);


	datos res;
	int n=0;
	for(int i=1;i<noPuntos;i++){
		if(idxCercanos[i] != -1) {
			res.x+=objetos[idxCercanos[i]].x;
			res.y+=objetos[idxCercanos[i]].y;
			n++;
		}
	}
	res.x/=n;
	res.y/=n;
	
	//res.p=pendiente;
	res.e=n;
	return res;
}


cv::Mat puntoAMoverse(cv::Mat& entrada){
	// threshold para quedarse solo con lo blanco
	 cv::Mat dstthreshold, dilate_dst, erosion_dst;
  	 cv::inRange(entrada,cv::Scalar(160,160,160), cv::Scalar(255,255,255),dstthreshold);

	 // erosion
	 int erosion_size = 1; //20;
	 // tipos de erosion: MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
	 int erosion_type = cv::MORPH_RECT;
	 cv::Mat element = cv::getStructuringElement( erosion_type,
                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                       cv::Point( erosion_size, erosion_size ) );
  	  
	 // eliminar ruido (revisar)
	 cv::erode( dstthreshold, erosion_dst, element );
	 cv::dilate( erosion_dst, dilate_dst, element );
	 
	 // mostrar histograma
	 // cv::Mat histImage = histograma(Result); // crear histograma de la imagen
	 
	 cv::Mat contours, cdst;
	 cv::Canny(dilate_dst,contours,50,350);
	 cv::Mat contoursInv;
	 int houghVote;
	 cv::vector<cv::Vec4i> lines;


	 cv::HoughLinesP(contours, lines, 1, CV_PI/180, 10, 5, 3 );
	 cvtColor(contours, cdst, CV_GRAY2BGR);
	 for( size_t i = 0; i < lines.size(); i++ )
	 {
	    cv::Vec4i l = lines[i];
	    double m = ((double)l[3]-(double)l[1])/((double)l[2]-(double)l[0]); //  y2-y1/x2-x1
	    // if(abs(m)>0.2)
	    cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,50), 2, CV_AA);	     
	 }
	 
	 std::vector<std::vector<cv::Point> > c_contours;
	 std::vector<cv::Vec4i> hierarchy;
	 cv::findContours(contours, c_contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	double centroCarroX = 320;
	double centroCarroY = 480;
	
	/*
	double maxDX = 0.0;
	double maxDY = 0.0;
	double maxIX = 0.0;
	double maxIY = 0.0;
	*/

	// obtener puntos centrales de cada objeto observado
	int sized=0, sizei=0;
	std::vector<cv::Point> objetos;
	for(size_t i = 0; i < c_contours.size(); i++ )
	{
 		std::vector<cv::Point> o = c_contours[i];
		double x =0.0;
		double y =0.0;
		for( size_t j = 0; j < o.size(); j++ )
	  	{
			x+=o[j].x;
			y+=o[j].y;
			// agrega todos los puntos
			objetos.push_back(o[j]);
		}
		x=x/o.size();
		y=y/o.size();
		// dibujar el punto
		cv::line(cdst, cv::Point(x, y), cv::Point(x, y), cv::Scalar(0,i*30,255), 3, CV_AA);
		// anterior, agrega objetos
		// objetos.push_back(cv::Point(x, y));
		
	}
	printf("\nobj centrales OK");
	
	int limite = 7;
	datos datosIzquierda;
	datosIzquierda = pendienteLado(cdst, objetos, 0, centroCarroX, centroCarroY, 3,limite);
	//printf("\nIzquierda, pendiente: %f, encontrados: %d", datosIzquierda.p, datosIzquierda.e);
	printf("\nIzquierda (%f,%f)", datosIzquierda.x, datosIzquierda.y);
	datos datosDerecha;
	datosDerecha = pendienteLado(cdst, objetos, 1, centroCarroX, centroCarroY, 4, limite);
	//printf("\nDerecha, pendiente: %f, encontrados: %d", datosDerecha.p, datosDerecha.e);
	printf("\nDerecha (%f,%f)", datosDerecha.x, datosDerecha.y);

	double gprom;
	
	if(datosIzquierda.e>1 && datosDerecha.e>1)
		gprom = (datosIzquierda.x+datosDerecha.x)/2;
	else if(datosIzquierda.e>1)
		gprom = datosIzquierda.x + 30;
	else if(datosDerecha.e>1)
		gprom = datosDerecha.x - 30;
	printf("\n Grados promedio: %f",  gprom);
		
	//double pendiente = gprom;
	//pendiente = pendiente*PI/180;

	//printf("\nPromedio pendientes OK: %f", pendiente);

	// punto a moverse
	double p2Y = 100;
	double p2X = gprom;
	//if(pendiente> || pendiente<-10.0 || pendiente == 0.0)
	//p2X = p2Y/tan(pendiente);
	
	
	// p2X -= 640;
	// punto con respecto al carro
	//double xreal = p2X;
	//double yreal = p2Y;
	// double angulo = atan(yreal/xreal) * 180/PI;
	
	// coordenadas de la imagen
	//p2Y = 400 - p2Y;
	//p2X += 320;

	// mostrar el punto en la imagen
	cv::line(cdst, cv::Point(centroCarroX, centroCarroY), cv::Point(p2X, p2Y), cv::Scalar(100,30,255), 10, CV_AA);

	printf("\nSegundo punto: (%f, %f)", p2X, p2Y);
	//printf("\nSegundo punto (procesado): (%f, %f)", xreal, yreal);

	// aqui se tiene que enviar la posicion con respecto al robot del punto	
	
	geometry_msgs::Twist desired_pose;	
	desired_pose.linear.x = p2X;
	desired_pose.linear.y = p2Y;
	desired_pose.angular.z = gprom;
	pub_pose.publish(desired_pose);
	
	
	return cdst;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	try
	{
	 // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  
	 // histograma de la imagen
	 cv::Mat src = cv_bridge::toCvShare(msg , "bgr8")->image;
	 cv::Mat Result = cv_bridge::toCvShare(msg , "bgr8")->image;
	 
	 // crea un rectangulo en la parte superior
	 // cv::rectangle( Result, cv::Point( 0, 0 ),cv::Point( 640, 200),cv::Scalar(0, 0, 0 ),CV_FILLED,
	 // CV_AA );
	 
	 dibujarPoligonoCarro(Result);
	 
	 
    cv::Point2f inputQuad[4]; 
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
         
    // Lambda Matrix
    cv::Mat lambda( 2, 4, CV_32FC1 );
    //Input and Output Image;
    cv::Mat input, output;
     
    //Load the image
    input = Result;//imread( argv[1], 1 );
    // Set the lambda matrix the same type and size as input
    lambda = cv::Mat::zeros( input.rows, input.cols, input.type() );
 
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input 
    inputQuad[0] = cv::Point2f( 0,200 );
    inputQuad[1] = cv::Point2f( 640 ,200);
    inputQuad[2] = cv::Point2f( 640,400);
    inputQuad[3] = cv::Point2f( 0,400  );  
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = cv::Point2f( 0,0 );
    outputQuad[1] = cv::Point2f( input.cols-1,0);
    outputQuad[2] = cv::Point2f( input.cols-1,input.rows-1);
    outputQuad[3] = cv::Point2f( 0,input.rows-1  );
 
    // Get the Perspective Transform Matrix i.e. lambda 
    lambda = cv::getPerspectiveTransform( inputQuad, outputQuad );
    // Apply the Perspective Transform just found to the src image
    cv::warpPerspective(input,output,lambda,output.size() );
	
	
	printf("\nPunto output: ");
	cv::Mat cdst2 = puntoAMoverse(output);
	imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cdst2).toImageMsg();
	pub_image.publish(imgmsg);

	/*
	   http://docs.opencv.org/trunk/d9/db0/tutorial_hough_lines.html
	*/


        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
	
	// para guardar la imagen
	/* int key =cv::waitKey(0);	
	if ( key == 32 ) // espacio
	{
	    static int count = 0;
	    ROS_ASSERT(cv::imwrite( std::string( "file" )  + toString(count++) + std::string( ".png" ), cv_ptr->image ));
	}
	*/
}




int main(int argc, char** argv )
{
	ros::init(argc, argv, "image_node");
  	ros::NodeHandle nh;
  	ros::Rate loop_rate(1);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/app/camera/rgb/image_raw", 10, imageCallback);
	pub_pose = nh.advertise<geometry_msgs::Twist>("/target_pose", rate_hz);
	pub_image = nh.advertise<sensor_msgs::Image>("/target_image", rate_hz);
	ros::spin();

}
