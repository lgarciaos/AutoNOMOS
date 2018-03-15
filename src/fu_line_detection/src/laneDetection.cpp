#include "laneDetection.h"

using namespace std;

// to show output on windows
// #define PAINT_OUTPUT 
#define PUBLISH_DEBUG_OUTPUT 

static const uint32_t MY_ROS_QUEUE_SIZE = 1;

#define PI 3.14159265

#define RATE_HZ 30

#define NUM_STATES 9*3

image_transport::CameraPublisher image_publisher;
image_transport::CameraPublisher image_publisher_ransac;
image_transport::CameraPublisher image_publisher_lane_markings;


cv::Mat scanLinesMat;


double f_u;
double f_v;
double c_u;
double c_v;
double cam_deg;
double cam_height;



//msgs head
unsigned int head_sequence_id = 0;
ros::Time head_time_stamp;
std::string rgb_frame_id = "_rgb_optical_frame";
sensor_msgs::CameraInfoPtr rgb_camera_info;

geometry_msgs::Point punto_des;

// try kernel width 5 for now
const static int g_kernel1DWidth = 5;

std::string nombre_estado [9] = {"DNL",   "OL",   "LL",   "LC",   "CC",   "RC",   "RR",   "OR", "DNR"};

cLaneDetectionFu::cLaneDetectionFu(ros::NodeHandle nh)
    : nh_(nh), priv_nh_("~")
{
    std::string node_name = ros::this_node::getName();

    ROS_INFO("Node name: %s",node_name.c_str());

    priv_nh_.param<std::string>(node_name+"/camera_name", camera_name, "/usb_cam/image_raw"); 


    priv_nh_.param<int>(node_name+"/cam_w", cam_w, 640);
    priv_nh_.param<int>(node_name+"/cam_h", cam_h, 480);
    priv_nh_.param<int>(node_name+"/proj_y_start", proj_y_start, 415);
    priv_nh_.param<int>(node_name+"/proj_image_h", proj_image_h, 40);
    priv_nh_.param<int>(node_name+"/proj_image_w", proj_image_w, 80);
    priv_nh_.param<int>(node_name+"/proj_image_horizontal_offset", proj_image_horizontal_offset, 0);
    priv_nh_.param<int>(node_name+"/roi_top_w", roi_top_w, 62);
    priv_nh_.param<int>(node_name+"/roi_bottom_w", roi_bottom_w, 30);
    
    priv_nh_.param<int>(node_name+"/maxYRoi", maxYRoi, 5);
    priv_nh_.param<int>(node_name+"/minYDefaultRoi", minYDefaultRoi, 39);
    priv_nh_.param<int>(node_name+"/minYPolyRoi", minYPolyRoi, 39);

    priv_nh_.param<int>(node_name+"/defaultXLeft", defaultXLeft, 10);
    priv_nh_.param<int>(node_name+"/defaultXCenter", defaultXCenter, 30);
    priv_nh_.param<int>(node_name+"/defaultXRight", defaultXRight, 50);

    priv_nh_.param<int>(node_name+"/defaultYHorizontal", defaultYHorizontal, 100);

    priv_nh_.param<int>(node_name+"/interestDistancePoly", interestDistancePoly, 10);
    priv_nh_.param<int>(node_name+"/interestDistanceDefault", interestDistanceDefault, 10);
    
    priv_nh_.param<int>(node_name+"/iterationsRansac", iterationsRansac, 10);
    priv_nh_.param<double>(node_name+"/proportionThreshould", proportionThreshould, 0.5);
    
    priv_nh_.param<int>(node_name+"/m_gradientThreshold", m_gradientThreshold, 10);
    priv_nh_.param<int>(node_name+"/m_nonMaxWidth", m_nonMaxWidth, 10);
    priv_nh_.param<int>(node_name+"/laneMarkingSquaredThreshold", laneMarkingSquaredThreshold, 25);

    priv_nh_.param<int>(node_name+"/angleAdjacentLeg", angleAdjacentLeg, 25);
    
    priv_nh_.param<int>(node_name+"/scanlinesVerticalDistance", scanlinesVerticalDistance, 1);
    priv_nh_.param<int>(node_name+"/scanlinesMaxCount", scanlinesMaxCount, 100);

    priv_nh_.param<int>(node_name+"/detectLaneStartX", detectLaneStartX, 38);

    priv_nh_.param<int>(node_name+"/maxAngleDiff", maxAngleDiff, 10);

    priv_nh_.param<int>(node_name+"/polyY1", polyY1, 35);
    priv_nh_.param<int>(node_name+"/polyY2", polyY2, 30);
    priv_nh_.param<int>(node_name+"/polyY3", polyY3, 15);


    int cam_h_half = cam_h/2;

    priv_nh_.param<double>(node_name+"/f_u", f_u, 624.650635); 
    priv_nh_.param<double>(node_name+"/f_v", f_v, 626.987244); 
    priv_nh_.param<double>(node_name+"/c_u", c_u, 309.703230); 
    priv_nh_.param<double>(node_name+"/c_v", c_v, 231.473613); 
    priv_nh_.param<double>(node_name+"/cam_deg", cam_deg, 27); 
    priv_nh_.param<double>(node_name+"/cam_height", cam_height, 18);

    ipMapper = IPMapper(cam_w, cam_h_half, f_u, f_v, c_u, c_v, cam_deg, cam_height);
    

    proj_image_w_half = proj_image_w/2;

    polyDetectedLeft     = false;
    polyDetectedCenter   = false;
    polyDetectedRight    = false;
    polyDetectedHorizontal = false;

    bestPolyLeft         = std::make_pair(NewtonPolynomial(), 0);
    bestPolyCenter       = std::make_pair(NewtonPolynomial(), 0);
    bestPolyRight        = std::make_pair(NewtonPolynomial(), 0);

    bestPolyHorizontal        = std::make_pair(NewtonPolynomial(), 0);

    laneMarkingsLeft     = std::vector<FuPoint<int>>();
    laneMarkingsCenter   = std::vector<FuPoint<int>>();
    laneMarkingsRight    = std::vector<FuPoint<int>>();

    polyLeft             = NewtonPolynomial();
    polyCenter           = NewtonPolynomial();
    polyRight            = NewtonPolynomial();
    polyHorizontal       = NewtonPolynomial();

    supportersLeft       = std::vector<FuPoint<int>>();
    supportersCenter     = std::vector<FuPoint<int>>();
    supportersRight      = std::vector<FuPoint<int>>();

    supportersHorizontal      = std::vector<FuPoint<int>>();

    prevPolyLeft         = NewtonPolynomial();
    prevPolyCenter       = NewtonPolynomial();
    prevPolyRight        = NewtonPolynomial();

    prevPolyHorizontal        = NewtonPolynomial();

    pointsLeft           = std::vector<FuPoint<int>>();
    pointsCenter         = std::vector<FuPoint<int>>();
    pointsRight          = std::vector<FuPoint<int>>();

    pointsHorizontal          = std::vector<FuPoint<int>>();

    lanePoly             = NewtonPolynomial();
    lanePolynomial       = LanePolynomial();

    maxDistance = 1;  

    lastAngle = 0; 

    head_time_stamp = ros::Time::now();
    
    read_images_ = nh.subscribe(nh_.resolveName(camera_name), MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessInput,this);

    sub_planning = nh.subscribe("/planning", MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessPlanning,this);
    sub_localization = nh.subscribe("/localization_array", MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::get_localization, this);

    planningxy = nh.subscribe("/planningxy", MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessPlanningXY,this);

    //publish_curvature = nh.advertise<std_msgs::Float32>("/lane_model/curvature", MY_ROS_QUEUE_SIZE);
    publish_angle = nh.advertise<std_msgs::Float32>("/lane_model/angle", MY_ROS_QUEUE_SIZE);

    pub_right = nh.advertise<nav_msgs::GridCells>("/points/right", MY_ROS_QUEUE_SIZE);
    pub_center = nh.advertise<nav_msgs::GridCells>("/points/center", MY_ROS_QUEUE_SIZE);
    pub_left = nh.advertise<nav_msgs::GridCells>("/points/left", MY_ROS_QUEUE_SIZE);

    pub_horizontal = nh.advertise<nav_msgs::GridCells>("/points/horizontal", MY_ROS_QUEUE_SIZE);

    pub_ransac_right = nh.advertise<nav_msgs::GridCells>("/points/ransac_right", MY_ROS_QUEUE_SIZE);
    pub_ransac_center = nh.advertise<nav_msgs::GridCells>("/points/ransac_center", MY_ROS_QUEUE_SIZE);
    pub_ransac_left = nh.advertise<nav_msgs::GridCells>("/points/ransac_left", MY_ROS_QUEUE_SIZE);

    pub_lane_model = nh.advertise<nav_msgs::GridCells>("/points/lane_model", MY_ROS_QUEUE_SIZE);
    pub_ransac_horizontal = nh.advertise<nav_msgs::GridCells>("/points/ransac_horizontal", MY_ROS_QUEUE_SIZE);

    image_transport::ImageTransport image_transport(nh);
    
    image_publisher = image_transport.advertiseCamera("/lane_model/lane_model_image", MY_ROS_QUEUE_SIZE);

    #ifdef PUBLISH_DEBUG_OUTPUT
        image_publisher_ransac = image_transport.advertiseCamera("/lane_model/ransac", MY_ROS_QUEUE_SIZE);
        image_publisher_lane_markings = image_transport.advertiseCamera("/lane_model/lane_markings", MY_ROS_QUEUE_SIZE);
    #endif



    if (!rgb_camera_info)
    {
        rgb_camera_info.reset(new sensor_msgs::CameraInfo());
        rgb_camera_info->width = proj_image_w;
        rgb_camera_info->height = proj_image_h+50;
    }
    // //ROS_INFO_STREAM("Finishing cLaneDetectionFu");
    //from camera properties and ROI etc we get scanlines (=line segments, úsečky)
    //these line segments are lines in image on whose we look for edges
    //the outer vector represents rows on image, inner vector is vector of line segments of one row, usualy just one line segment
    //we should generate this only once in the beginning! or even just have it pregenerated for our cam
    scanlines = getScanlines();
}

cLaneDetectionFu::~cLaneDetectionFu()
{
}

void cLaneDetectionFu::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
    //ROS_INFO_STREAM("ProcessInput beginning ");

    // clear some stuff from the last cycle
    bestPolyLeft = std::make_pair(NewtonPolynomial(), 0);
    bestPolyCenter = std::make_pair(NewtonPolynomial(), 0);
    bestPolyRight = std::make_pair(NewtonPolynomial(), 0);

    bestPolyHorizontal = std::make_pair(NewtonPolynomial(), 0);

    supportersLeft.clear();
    supportersCenter.clear();
    supportersRight.clear();

    supportersHorizontal.clear();

    //use ROS image_proc or opencv instead of ip mapper?

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    cv::Mat image = cv_ptr->image.clone();

    Mat cut_image = image(cv::Rect(0,cam_h/2,cam_w,cam_h/2));
    // Mat cut_image = image(cv::Rect(0,0,cam_w,cam_h));
    Mat remapped_image = ipMapper.remap(cut_image);

    #ifdef PAINT_OUTPUT
        //cv::imshow("IPmapped image", remapped_image);
        //cv::waitKey(1);
    #endif

	// (640/2)-40+0,415,80,40)
	// 280,415,80,40

    cv::Mat transformedImage = remapped_image(cv::Rect((cam_w/2)-proj_image_w_half+proj_image_horizontal_offset,
            proj_y_start,proj_image_w,proj_image_h)).clone();
    
    // cv::Mat transformedImage = remapped_image.clone();
    cv::flip(transformedImage, transformedImage, 0);


    /*
    // DETECT HORIZONTAL LINES ADDED
    */

    /*
    #ifdef PAINT_OUTPUT
    	cv::imshow("Cut IPmapped image", transformedImage);   
    	cv::waitKey(1);
    #endif
    */

    //scanLinesMat = transformedImage.clone();
    //scanlines = getScanlines();

    //scanlines -> edges (in each scanline we find maximum and minimum of kernel fn ~= where the edge is)
    //this is where we use input image!
    vector<vector<EdgePoint>> edges = cLaneDetectionFu::scanImage(transformedImage, RIGHT); // RIGHT es solo por poner algo diferente de HORIZONTAL
    vector<vector<EdgePoint>> edgesHorizontal = cLaneDetectionFu::scanImage(transformedImage, HORIZONTAL); // HORIZONTAL es para obtener lineas horizontales

    cv::Mat transformedImagePaintable;
    cv::Mat transformedImagePaintableHorizontal;

    //---------------------- DEBUG OUTPUT EDGES ---------------------------------//
    #ifdef PUBLISH_DEBUG_OUTPUT
        transformedImagePaintable = transformedImage.clone();
        cv::cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
        for(int i = 0; i < (int)edges.size(); i++)
        {
            for(int j=0; j < edges[i].size(); j++) {
                FuPoint<int> edge = edges[i][j].getImgPos();
                cv::Point edgeLoc = cv::Point(edge.getX(), edge.getY());
                cv::circle(transformedImagePaintable,edgeLoc,1,cv::Scalar(0,0,edges[i][j].getValue()),-1);    
            }            
        }


        transformedImagePaintableHorizontal = transformedImage.clone();
        cv::cvtColor(transformedImagePaintableHorizontal,transformedImagePaintableHorizontal,CV_GRAY2BGR);
        for(int i = 0; i < (int)edgesHorizontal.size(); i++)
        {
            for(int j=0; j < edgesHorizontal[i].size(); j++) {
                FuPoint<int> edge = edgesHorizontal[i][j].getImgPos();
                cv::Point edgeLoc = cv::Point(edge.getX(), edge.getY());
                cv::circle(transformedImagePaintableHorizontal,edgeLoc,1,cv::Scalar(0,0,edgesHorizontal[i][j].getValue()),-1);    
            }            
        }
    #ifdef PAINT_OUTPUT    
	        
        //cv::imshow("ROI, scanlines and edges", transformedImagePaintable);

		//cv::imshow("ROI, edgesHorizontal", transformedImagePaintableHorizontal);
        //cv::waitKey(1);
    #endif
    #endif
    //---------------------- END DEBUG OUTPUT EDGES ------------------------------//

    //edges -> lane markings
    vector<FuPoint<int>> laneMarkings = cLaneDetectionFu::extractLaneMarkings(edges);
    vector<FuPoint<int>> laneMarkingsH = cLaneDetectionFu::extractLaneMarkingsHorizontal(edgesHorizontal);

    //---------------------- DEBUG OUTPUT LANE MARKINGS ---------------------------------//
    #ifdef PUBLISH_DEBUG_OUTPUT
        transformedImagePaintable = transformedImage.clone();
        cv::cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);
        for(int i = 0; i < (int)laneMarkings.size(); i++)
        {         
            FuPoint<int> marking = laneMarkings[i];
            cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(153, 102, 51),-1);         
        }

        transformedImagePaintableHorizontal = transformedImage.clone();
        cv::cvtColor(transformedImagePaintableHorizontal,transformedImagePaintableHorizontal,CV_GRAY2BGR);
        for(int i = 0; i < (int)laneMarkingsH.size(); i++)
        {         
            FuPoint<int> marking = laneMarkingsH[i];
            cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
            cv::circle(transformedImagePaintableHorizontal,markingLoc,1,cv::Scalar(0,255,0),-1);         
        }
    #ifdef PAINT_OUTPUT
        //cv::imshow("Lane Markings", transformedImagePaintable);

        //cv::imshow("L. Markings Horizontal", transformedImagePaintableHorizontal);
        //cv::waitKey(1);
    #endif
    #endif
    //---------------------- END DEBUG OUTPUT LANE MARKINGS ------------------------------//

    // start actual execution
    buildLaneMarkingsLists(laneMarkings);
    buildLaneMarkingsListsHorizontal(laneMarkingsH);

    //---------------------- DEBUG OUTPUT GROUPED LANE MARKINGS ---------------------------------//
    geometry_msgs::Point pt;
    #ifdef PUBLISH_DEBUG_OUTPUT
        // transformedImagePaintable = transformedImage.clone();
        // cv::cvtColor(transformedImagePaintable,transformedImagePaintable,CV_GRAY2BGR);

    	// RULER
        for (int i = 0; i < 200; i = i + 10)
        {
            // //ROS_INFO_STREAM("Printing: " << i);
            cv::Point markingLoc = cv::Point(i, 0);
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(0,221,237),-1); 
            markingLoc = cv::Point(i, 10);
            cv::putText(transformedImagePaintable,std::to_string(i),markingLoc,6,.15,cv::Scalar(0,221,237));    
            markingLoc = cv::Point(0, i);
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(0,221,237),-1); 
            markingLoc = cv::Point(10, i);
            cv::putText(transformedImagePaintable,std::to_string(i),markingLoc,6,.15,cv::Scalar(0,237,221));    
        }

        array_left.cell_width = (int) laneMarkingsLeft.size() ;
        array_left.cell_height = 1;
        array_center.cell_width = (int) laneMarkingsCenter.size() ;
        array_center.cell_height = 1;
        array_right.cell_width = (int) laneMarkingsRight.size() ;
        array_right.cell_height = 1;

        array_horizontal.cell_width = (int) laneMarkingsHorizontal.size() ;
        array_horizontal.cell_height = 1;

        array_left.cells.clear();
        array_center.cells.clear();
        array_right.cells.clear();
        array_horizontal.cells.clear();

        for(int i = 0; i < (int)laneMarkingsLeft.size(); i++)
        {         
            FuPoint<int> marking = laneMarkingsLeft[i];
            cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(0,0,139),-1);
            
            pt.x = marking.getX();
            pt.y = marking.getY();
            pt.z = 0;
            array_left.cells.push_back(pt);
        }
        for(int i = 0; i < (int)laneMarkingsCenter.size(); i++)
        {         
            FuPoint<int> marking = laneMarkingsCenter[i];
            cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(0,100,0),-1);  
            pt.x = marking.getX();
            pt.y = marking.getY();
            pt.z = 0;
            array_center.cells.push_back(pt);
        }
        for(int i = 0; i < (int)laneMarkingsRight.size(); i++)
        {         
            FuPoint<int> marking = laneMarkingsRight[i];
            cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(139,0,0),-1);  
            pt.x = marking.getX();
            pt.y = marking.getY();
            pt.z = 0;
            array_right.cells.push_back(pt);      
        }

        /*
        for(int i = 0; i < (int)laneMarkingsHorizontal.size(); i++)
        {         
            FuPoint<int> marking = laneMarkingsHorizontal[i];
            cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
            cv::circle(transformedImagePaintable,markingLoc,1,cv::Scalar(255,0,255),-1);  
            pt.x = marking.getX();
            pt.y = marking.getY();
            pt.z = 0;
            array_horizontal.cells.push_back(pt);      
        }
        */

        //LEFT
        cv::Point2d p1l(defaultXLeft,minYPolyRoi);
        cv::Point2d p2l(defaultXLeft,maxYRoi-1);
        cv::line(transformedImagePaintable,p1l,p2l,cv::Scalar(0,0,139));

        //CENTER
        cv::Point2d p1c(defaultXCenter,minYPolyRoi);
        cv::Point2d p2c(defaultXCenter,maxYRoi-1);
        cv::line(transformedImagePaintable,p1c,p2c,cv::Scalar(0,100,0));

        //RIGHT
        cv::Point2d p1r(defaultXRight,minYPolyRoi);
        cv::Point2d p2r(defaultXRight,maxYRoi-1);
        cv::line(transformedImagePaintable,p1r,p2r,cv::Scalar(139,0,0));

        //HORIZONTAL
        /*
        cv::Point2d p1h(0,defaultYHorizontal);
        cv::Point2d p2h(160,defaultYHorizontal);
        cv::line(transformedImagePaintable,p1h,p2h,cv::Scalar(255,0,255));
		*/
        
        //ROI
        cv::Point2d p1(proj_image_w_half-(roi_bottom_w/2),maxYRoi-1);
        cv::Point2d p2(proj_image_w_half+(roi_bottom_w/2),maxYRoi-1);
        cv::Point2d p3(proj_image_w_half+(roi_top_w/2),minYPolyRoi);
        cv::Point2d p4(proj_image_w_half-(roi_top_w/2),minYPolyRoi);
        cv::line(transformedImagePaintable,p1,p2,cv::Scalar(0,200,0));
        cv::line(transformedImagePaintable,p2,p3,cv::Scalar(0,200,0));
        cv::line(transformedImagePaintable,p3,p4,cv::Scalar(0,200,0));
        cv::line(transformedImagePaintable,p4,p1,cv::Scalar(0,200,0));

        pub_left.publish(array_left);
        pub_center.publish(array_center);
        pub_right.publish(array_right);
        pub_horizontal.publish(array_horizontal);

        pubRGBImageMsg(transformedImagePaintable, image_publisher_lane_markings);

    #ifdef PAINT_OUTPUT
        cv::imshow("Grouped Lane Markings", transformedImagePaintable);
        cv::waitKey(1);
    #endif
    #endif
    //---------------------- END DEBUG OUTPUT GROUPED LANE MARKINGS ------------------------------//

    ransac();

    //---------------------- DEBUG OUTPUT RANSAC POLYNOMIALS ---------------------------------//
    
    // cv::Mat transformedImagePaintableLaneModel = transformedImage.clone();
    // cv::cvtColor(transformedImagePaintableLaneModel,transformedImagePaintableLaneModel,CV_GRAY2BGR);
    
    #ifdef PUBLISH_DEBUG_OUTPUT
        //cv::Mat transformedImagePaintableRansac = transformedImage.clone();
        //cv::cvtColor(transformedImagePaintableRansac,transformedImagePaintableRansac,CV_GRAY2BGR);
        
        //cv::Mat transformedImagePaintableLaneModel = transformedImage.clone();
    	//cv::cvtColor(transformedImagePaintableLaneModel,transformedImagePaintableLaneModel,CV_GRAY2BGR);

        // geometry_msgs::Point pt;
        array_ransac_left.cells.clear();
        array_ransac_center.cells.clear();
        array_ransac_right.cells.clear();
        array_ransac_horizontal.cells.clear();

        array_ransac_left.cell_width =  maxYRoi - minYPolyRoi;
        array_ransac_left.cell_height = 1;
        array_ransac_center.cell_width = maxYRoi - minYPolyRoi ;
        array_ransac_center.cell_height = 1;
        array_ransac_right.cell_width = maxYRoi - minYPolyRoi;
        array_ransac_right.cell_height = 1;
        array_ransac_horizontal.cell_width = 160 - 0;
        array_ransac_horizontal.cell_height = 1;

        for(int i = minYPolyRoi; i < maxYRoi; i++)
        {
            cv::Point pointLocLeft = cv::Point(polyLeft.at(i), i);
            cv::circle(transformedImagePaintable,pointLocLeft,0,cv::Scalar(0,0,255),-1);
            cv::Point pointLocCenter = cv::Point(polyCenter.at(i), i);
            cv::circle(transformedImagePaintable,pointLocCenter,0,cv::Scalar(0,255,0),-1);
            cv::Point pointLocRight = cv::Point(polyRight.at(i), i);
            cv::circle(transformedImagePaintable,pointLocRight,0,cv::Scalar(255,0,0),-1);
            pt.x = pointLocLeft.x;
            pt.y = pointLocLeft.y;
            pt.z = 0;
            array_ransac_left.cells.push_back(pt);
            pt.x = pointLocCenter.x;
            pt.y = pointLocCenter.y;
            pt.z = 0;
            array_ransac_center.cells.push_back(pt);
            pt.x = pointLocRight.x;
            pt.y = pointLocRight.y;
            pt.z = 0;
            array_ransac_right.cells.push_back(pt);
        }

        /* mostrar puntos HORIZONTALES
        if(polyDetectedHorizontal){
            for(int i = 0; i < 160; i++) {
                double y = polyHorizontal.at(i);
                cv::Point pointLocHorizontal = cv::Point(i, y);
                cv::circle(transformedImagePaintableLaneModel,pointLocHorizontal,0,cv::Scalar(200,0,200),-1);
                // ROS_INFO_STREAM("POLY HORIZONTAL x:" << i << ", y:" << y);
                pt.x = pointLocHorizontal.x;
                pt.y = pointLocHorizontal.y;
                pt.z = 0;
                array_ransac_horizontal.cells.push_back(pt);
            }
        }
		*/

        // PLANEACION
        /*
        for(int i=0; i<path_planned.cell_width;i++) {
            if(path_planned.cells[i].x < proj_image_w && path_planned.cells[i].y - 10 > 0) {
                cv::Point pointPath = cv::Point(path_planned.cells[i].x, path_planned.cells[i].y);
                cv::Point pointText = cv::Point(path_planned.cells[i].x, path_planned.cells[i].y - 10);
                cv::Point pointTextEstado = cv::Point(80, 155);

                // center, radius, color, thickness
                cv::circle(transformedImagePaintableRansac,pointPath,2,cv::Scalar(200,200,0),-1);
                cv::putText(transformedImagePaintableRansac,std::to_string(i),pointText,FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(0,221,237));
                cv::putText(transformedImagePaintableRansac,std::to_string(estado),pointTextEstado,FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(200,221,0));
                // ROS_INFO_STREAM("X: " << pointPath.x << ", Y: " << pointPath.y);
            }
        }
		
        cv::Point pun_des = cv::Point(punto_des.x, punto_des.y);
        cv::circle(transformedImagePaintableRansac,pun_des,3,cv::Scalar(0,221,237),-1);
		*/

        pub_ransac_left.publish(array_ransac_left);
        pub_ransac_center.publish(array_ransac_center);
        pub_ransac_right.publish(array_ransac_right);
        pub_ransac_horizontal.publish(array_ransac_right);

        pubRGBImageMsg(transformedImagePaintable, image_publisher_ransac);

    #ifdef PAINT_OUTPUT
        cv::imshow("RANSAC results", transformedImagePaintable);
        cv::waitKey(1);
    #endif
    #endif
    //---------------------- END DEBUG OUTPUT RANSAC POLYNOMIALS ------------------------------//

    detectLane();

    pubAngle();

    // Esta seccion del codigo es la que dibuja el modelo del carril
//#ifdef PUBLISH_DEBUG_OUTPUT

    //cv::Mat transformedImagePaintableLaneModel = transformedImage.clone();
    //cv::cvtColor(transformedImagePaintableLaneModel,transformedImagePaintableLaneModel,CV_GRAY2BGR);

    if (lanePolynomial.hasDetected()) {

	array_lane_model.cells.clear();
	array_lane_model.cell_width=maxYRoi-minYPolyRoi;
	array_lane_model.cell_height=1;

        int r = lanePolynomial.getLastUsedPosition() == LEFT ? 255 : 0;
        int g = lanePolynomial.getLastUsedPosition() == CENTER ? 255 : 0;
        int b = lanePolynomial.getLastUsedPosition() == RIGHT ? 255 : 0;

        
        for(int i = minYPolyRoi; i < maxYRoi; i++)
        {
            cv::Point pointLoc = cv::Point(lanePolynomial.getLanePoly().at(i)+proj_image_w_half, i);
            cv::circle(transformedImagePaintable,pointLoc,0,cv::Scalar(b,g,r),-1);

            // aqui pintar lane  
            pt.x = pointLoc.x;
            pt.y = pointLoc.y;
            pt.z = 0;
            array_lane_model.cells.push_back(pt);
        }     

        std::vector<FuPoint<int>> supps = lanePolynomial.getLastUsedPosition() == LEFT 
            ? supportersLeft 
            : lanePolynomial.getLastUsedPosition() == CENTER ? supportersCenter : supportersRight;

        for(int i = 0; i < (int)supps.size(); i++)
        {         
            FuPoint<int> supp = supps[i];
            cv::Point suppLoc = cv::Point(supp.getX(), supp.getY());
            cv::circle(transformedImagePaintable,suppLoc,1,cv::Scalar(b,g,r),-1); 
        }    

        //-------------------------
        // PLANEACION
        /*
        for(int i=0; i<path_planned.cell_width;i++) {
            if(path_planned.cells[i].x < proj_image_w && path_planned.cells[i].y - 10 > 0) {
                cv::Point pointPath = cv::Point(path_planned.cells[i].x, path_planned.cells[i].y);
                cv::Point pointText = cv::Point(path_planned.cells[i].x, path_planned.cells[i].y - 10);
                
                // muestra las coordenadas de todos los estados
                cv::circle(transformedImagePaintable,pointPath,2,cv::Scalar(200,200,0),-1);
                cv::putText(transformedImagePaintable,std::to_string(i),pointText,FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(0,221,237));
                
            }
        }
        cv::Point pun_des = cv::Point(punto_des.x, punto_des.y);
        cv::circle(transformedImagePaintable,pun_des,3,cv::Scalar(0,221,237),-1);
        // muestra en el estado en que me encuentro
        cv::Point pointTextEstado = cv::Point(80, 155);
        if(estado >=0)
        	cv::putText(transformedImagePaintable,nombre_estado[estado],pointTextEstado,FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(200,221,0));
        else
        	cv::putText(transformedImagePaintable,"?",pointTextEstado,FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(200,221,0));
     	*/
        //-------------------------

        cv::Point pointLoc = cv::Point(proj_image_w_half,proj_image_h);
        cv::circle(transformedImagePaintable,pointLoc,2,cv::Scalar(0,0,255),-1); 

        cv:Point anglePointLoc = cv::Point(sin(lastAngle*PI/180)*angleAdjacentLeg+proj_image_w_half,proj_image_h-angleAdjacentLeg);
        cv::line(transformedImagePaintable,pointLoc,anglePointLoc,cv::Scalar(255,255,255));
    
	//pub_lane_model.publish(array_lane_model);
    } else {
        cv::Point pointLoc = cv::Point(5,5);
        cv::circle(transformedImagePaintable,pointLoc,3,cv::Scalar(0,0,255),0);
    }

    pub_lane_model.publish(array_lane_model);
    pubRGBImageMsg(transformedImagePaintable, image_publisher);

    //---------------------- DEBUG OUTPUT LANE POLYNOMIAL ---------------------------------//
    #ifdef PAINT_OUTPUT
        cv::imshow("Lane polynomial", transformedImagePaintable);
        cv::waitKey(1);
    #endif
//#endif
    //---------------------- END DEBUG OUTPUT LANE POLYNOMIAL ------------------------------//

    //ROS_INFO_STREAM("ProcessInput Ending");
}

/* EdgeDetector methods */

/**
 * Compute scanlines. Each may consist of multiple segments, split at regions
 * that should not be inspected by the kernel.
 * @param side
 * @return vector of segments of scanlines, walk these segments with the kernel
 */
vector<vector<LineSegment<int>> > cLaneDetectionFu::getScanlines() {
    //ROS_INFO_STREAM("getScanlines beginning");
    vector<vector<LineSegment<int>> > scanlines;

    vector<cv::Point> checkContour;
    checkContour.push_back(cv::Point(proj_image_w_half-(roi_bottom_w/2),maxYRoi-1));
    checkContour.push_back(cv::Point(proj_image_w_half+(roi_bottom_w/2),maxYRoi-1));
    checkContour.push_back(cv::Point(proj_image_w_half+(roi_top_w/2),minYPolyRoi));
    checkContour.push_back(cv::Point(proj_image_w_half-(roi_top_w/2),minYPolyRoi));
    
    int scanlineStart = 0;
    int scanlineEnd = proj_image_w;

    int segmentStart = -1;
    vector<LineSegment<int>> scanline;
    //i = y; j = x;
    for (int i = 1;
        (i/scanlinesVerticalDistance) < scanlinesMaxCount && i <= proj_image_h;
        i += scanlinesVerticalDistance) {
        scanline = vector<LineSegment<int>>();
        
        // walk along line
        for (int j = scanlineStart; j <= scanlineEnd; j ++) {
            bool isInside = pointPolygonTest(checkContour, cv::Point(j, i),false) >= 0; // esta en ROI
            
            // start new scanline segment
            if (isInside && j < scanlineEnd) {
                if (segmentStart == -1) segmentStart = j;
            // found end of scanline segment, reset start
            } else if (segmentStart != -1) {                
                scanline.push_back(
                        LineSegment<int>(
                                FuPoint<int>(segmentStart, i),
                                FuPoint<int>(j-1, i)
                            )
                        );
                        //ROS_INFO_STREAM("x1:" << segmentStart <<", y1:" << i);
                        //ROS_INFO_STREAM("x2:" << j-1 << ", y2" << i);
                segmentStart = -1;
            }
        }
        // push segments found
        if (scanline.size()) {
            scanlines.push_back(scanline);
        }
    }

    /* BORRAR ESTO ES DE DEBUG, PERO EL METODO SE EJECUTA EN EL CONSTRUCTOR Y TODAVIA NO HAY IMAGEN
    cv::cvtColor(scanLinesMat,scanLinesMat,CV_GRAY2BGR);
    // walk over all scanlines
    int color = 0;
    for (auto scanline : scanlines) {
        // set all brightness values on scanline to 0;

        for (auto vec : scanline){
            FuPoint<int> si = vec.getStart();  
            FuPoint<int> se = vec.getEnd();            
            color = (color + 10) % 255;
            cv::Point p1 = cv::Point(si.getX(), si.getY());
            cv::Point p2 = cv::Point(se.getX(), se.getY());
            cv::line(scanLinesMat,p1,p2,cv::Scalar(0,color,color));
        }
    }
    cv::imshow("scanLines",scanLinesMat);
    */

    //ROS_INFO_STREAM("getScanlines ending");
    return scanlines;
    
}

/**
 * Walk with prewitt/sobel kernel along all scanlines.
 * @param image
 * @return All edgePoints on side, sorted by scanlines.
 */
vector<vector<EdgePoint>> cLaneDetectionFu::scanImage(cv::Mat image, ePosition position) {
    //ROS_INFO_STREAM("scanImage beginning");
    ////ROS_INFO_STREAM("scanImage() - " << scanlines.size() << " scanlines.");
    vector<vector<EdgePoint>> edgePoints;
    
    //const Image &image = getImage();
    //const ImageDimensions &imgDim = getImageDimensions();
    //const OmnidirectionalCameraMatrix &cameraMatrix = getOmnidirectionalCameraMatrix();

    // scanline length can maximal be image height/width
    int scanlineMaxLength = image.cols;
    
    // store kernel results on current scanline in here
    vector<int> scanlineVals(scanlineMaxLength, 0);

    // walk over all scanlines
    for (auto scanline : scanlines) {
        // set all brightness values on scanline to 0;
        std::fill(scanlineVals.begin(), scanlineVals.end(), 0);
        int offset = 0;
        if (scanline.size()) {
            offset = scanline.front().getStart().getY();
        }

        // scanline consisting of multiple segments
        // walk over each but store kernel results for whole scanline
        
        for (auto segment : scanline) {         
            int start = segment.getStart().getX();
            int end = segment.getEnd().getX();
            
            // walk along segment
            for (int i = start; i < end - g_kernel1DWidth; i++) {
                int sum_x = 0;                

                //cv::Mat uses ROW-major system -> .at(y,x)
                // use kernel width 5 and try sobel kernel
                sum_x -= image.at<uint8_t>(offset-1, i+1);
                sum_x -= image.at<uint8_t>(offset-1, i);
                // kernel is 0
                sum_x += image.at<uint8_t>(offset-1, i+2);
                sum_x += image.at<uint8_t>(offset-1, i+4);

                sum_x -= 2*image.at<uint8_t>(offset, i);
                sum_x -= 2*image.at<uint8_t>(offset, i+1);
                // kernel is 0
                sum_x += 2*image.at<uint8_t>(offset, i+2);
                sum_x += 2*image.at<uint8_t>(offset, i+4);

                sum_x -= image.at<uint8_t>(offset+1, i);
                sum_x -= image.at<uint8_t>(offset+1, i+1);
                // kernel is 0
                sum_x += image.at<uint8_t>(offset+1, i+2);
                sum_x += image.at<uint8_t>(offset+1, i+4);

                // intento por mostrar lineas horizontales
                int sum_y = 0;

                if(position == HORIZONTAL){
                    sum_y -= image.at<uint8_t>(offset-1, i);
                    sum_y -= 2*image.at<uint8_t>(offset-1, i+1);
                    sum_y -= 2*image.at<uint8_t>(offset-1, i+2);
                    sum_y -= image.at<uint8_t>(offset-1, i+3);

                    //sum_y -= image.at<uint8_t>(offset, i);
                    //sum_y -= 2*image.at<uint8_t>(offset, i+1);
                    //sum_y -= 2*image.at<uint8_t>(offset, i+2);
                    //sum_y -= image.at<uint8_t>(offset, i+3);

                    sum_y += image.at<uint8_t>(offset+1, i);
                    sum_y += 2*image.at<uint8_t>(offset+1, i+1);
                    sum_y += 2*image.at<uint8_t>(offset+1, i+2);
                    sum_y += image.at<uint8_t>(offset+1, i+3);

                    //sum_y += image.at<uint8_t>(offset+3, i);
                    //sum_y += 2*image.at<uint8_t>(offset+3, i+1);
                    //sum_y += 2*image.at<uint8_t>(offset+3, i+2);
                    //sum_y += image.at<uint8_t>(offset+3, i+3);
                }
                
                int gradient = sqrt(sum_x*sum_x+sum_y*sum_y);
                int angle = atan2(sum_y,sum_x) * 180.0 / CV_PI;

                // +4 because of sobel weighting
                sum_x = sum_x / (3 * g_kernel1DWidth + 4);
                sum_y = sum_y / (3 * g_kernel1DWidth + 4);



                ////ROS_INFO_STREAM(sum << " is kernel sum.");
                // sustitui sum por gradient
                if(position == HORIZONTAL){
                    // los bordes verticales tienen un angulo de 0, por tanto los horizontales 90
                    if (std::abs(sum_y) > m_gradientThreshold && (angle >= 80 && angle <= 130) ) {
                        // set scanlineVals at center of kernel
                        scanlineVals[i + g_kernel1DWidth/2] = sum_y;
                    }
                }
                else {
                    if (std::abs(sum_x) > m_gradientThreshold) {
                        // set scanlineVals at center of kernel
                        scanlineVals[i + g_kernel1DWidth/2] = sum_x;
                    }
                }
            }
            // ROS_INFO_STREAM("max: " << maxG << ", min: "<< minG << ", mean: " << mean/suma);
        }

        // after walking over all segments of one scanline
        // do non-max-suppression
        // for both minima and maxima at same time
        // TODO: Jannis: find dryer way
        int indexOfLastMaximum = 0;
        int valueOfLastMaximum = 0;
        int indexOfLastMinimum = 0;
        int valueOfLastMinimum = 0;
        for (int i = 1; i < scanlineMaxLength -1; i++) {
            // check if maximum
            if (scanlineVals[i] > 0) {
                if (scanlineVals[i] < scanlineVals[i-1] or scanlineVals[i] < scanlineVals[i+1]) {
                    scanlineVals[i] = 0;
                }
                else {
                    // this pixel can just survive if the next maximum is not too close
                    if (i - indexOfLastMaximum > m_nonMaxWidth) {
                        // this is a new maximum
                        indexOfLastMaximum = i;
                        valueOfLastMaximum = scanlineVals[i];
                    }
                    else {
                        if (valueOfLastMaximum < scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMaximum] = 0;
                            indexOfLastMaximum = i;
                            valueOfLastMaximum = scanlineVals[i];
                        }
                        else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
            // check if minimum
            if (scanlineVals[i] < 0) {
                if (scanlineVals[i] > scanlineVals[i-1] or scanlineVals[i] > scanlineVals[i+1]) {
                    scanlineVals[i] = 0;
                }
                else {
                    // this pixel can just survive if the next minimum is not too close
                    if (i - indexOfLastMinimum > m_nonMaxWidth) {
                        // this is a new minimum
                        indexOfLastMinimum = i;
                        valueOfLastMinimum = scanlineVals[i];
                    }
                    else {
                        if (valueOfLastMinimum > scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMinimum] = 0;
                            indexOfLastMinimum = i;
                            valueOfLastMinimum = scanlineVals[i];
                        }
                        else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
        }
        // collect all the edgePoints for scanline
        vector<EdgePoint> scanlineEdgePoints;
        for (int i = 0; i < static_cast<int>(scanlineVals.size()); i++) {
            if (scanlineVals[i] != 0) {
                FuPoint<int> imgPos = FuPoint<int>(i, offset);
                
                FuPoint<Meter> relPos = FuPoint<Meter>();//offset, i);//cameraMatrix.transformToLocalCoordinates(imgPos);
                scanlineEdgePoints.push_back(EdgePoint(imgPos, relPos, scanlineVals[i]));
            }
        }
        edgePoints.push_back(std::move(scanlineEdgePoints));
    }
    // after walking along all scanlines
    // return edgePoints
    //ROS_INFO_STREAM("scanImage ending");
    return edgePoints;
}

/* Planning */

// get planned path to show it on the image
void cLaneDetectionFu::ProcessPlanning(const nav_msgs::GridCells& path){
    path_planned.cell_width = path.cell_width;
    path_planned.cells = path.cells;
}

void cLaneDetectionFu::ProcessPlanningXY(const geometry_msgs::Point& path){
    punto_des = path;
}

void cLaneDetectionFu::get_localization(const std_msgs::Float32MultiArray& locArray) {
    
    // detectar estado de mayor probabilidad para imprimirlo
    float max=0;
	for(int i=0;i<NUM_STATES;i++){
	 	if(locArray.data[i]>max){
	 		max=locArray.data[i];
	 	}
	}
	estado=-1;
	int countEstados=0;
	for(int i=NUM_STATES-1;i>=0;i--){
        if(locArray.data[i]==max){
            // ROS_INFO_STREAM("Estas en:" << nombre_estado[i]);
            estado = i;
            countEstados++;
        }
    	}

    if (countEstados==1)
    	estado=(int)floor(estado/3);
    //else
    //	estado=-1; // no se pudo determinar el estado, ya que hay mas de uno posible

}

/* LaneMarkingDetector methods */


//uses Edges to extract lane markings
std::vector<FuPoint<int>> cLaneDetectionFu::extractLaneMarkings(const std::vector<std::vector<EdgePoint>>& edges) {
    //ROS_INFO_STREAM("extractLaneMarkings beginning");
    vector<FuPoint<int>> result;

    for (const auto& line : edges) {
        if (line.empty()) continue;
    
        for (
            auto edgePosition = line.begin(), nextEdgePosition = edgePosition + 1;
            nextEdgePosition != line.end();
            edgePosition = nextEdgePosition, ++nextEdgePosition
        ) {
            if (edgePosition->isPositive() and not nextEdgePosition->isPositive()) {
                FuPoint<int> candidateStartEdge = edgePosition->getImgPos();
                FuPoint<int> candidateEndEdge = nextEdgePosition->getImgPos();
                if ((candidateStartEdge - candidateEndEdge).squaredMagnitude() < laneMarkingSquaredThreshold) {
                    result.push_back(center(candidateStartEdge, candidateEndEdge));
                }
            }
        }
    }
    //ROS_INFO_STREAM("extractLaneMarkings ending");
    return result;
}

//uses Edges to extract lane markings
std::vector<FuPoint<int>> cLaneDetectionFu::extractLaneMarkingsHorizontal(const std::vector<std::vector<EdgePoint>>& edges) {
    //ROS_INFO_STREAM("extractLaneMarkings beginning");
    vector<FuPoint<int>> result;

    for (const auto& line : edges) {
        if (line.empty()) continue;
    
        for (
            auto edgePosition = line.begin(), nextEdgePosition = edgePosition + 1;
            nextEdgePosition != line.end();
            edgePosition = nextEdgePosition, ++nextEdgePosition
        ) {
            if (edgePosition->isPositive() and nextEdgePosition->isPositive()) {
                FuPoint<int> candidateStartEdge = edgePosition->getImgPos();
                FuPoint<int> candidateEndEdge = nextEdgePosition->getImgPos();
                if ((candidateStartEdge - candidateEndEdge).squaredMagnitude() < laneMarkingSquaredThreshold) {
                    result.push_back(center(candidateStartEdge, candidateEndEdge));
                }
            }
        }
    }
    //ROS_INFO_STREAM("extractLaneMarkings ending");
    return result;
}


/**
 * Creates three vectors of lane marking points out of the given lane marking
 * point vector.
 *
 * A point has to lie within the ROI of the previously detected lane polynomial
 * or within the default ROI, if no polynomial was detected.
 * The lists are the input data for the RANSAC algorithm.
 *
 * @param laneMarkings  a vector containing all detected lane markings
 */
void cLaneDetectionFu::buildLaneMarkingsLists(
        const std::vector<FuPoint<int>> &laneMarkings) {
    //ROS_INFO_STREAM("buildLaneMarkingsLists beginning");
    laneMarkingsLeft.clear();
    laneMarkingsCenter.clear();
    laneMarkingsRight.clear();

    
    //Order from bottom to top on Y axis
    
    if (!(polyDetectedLeft && polyDetectedCenter && polyDetectedRight))
    	MergeSort::mergeSort(laneMarkings,0,laneMarkings.size()-1); 

    //printf("SORTED \n");
    //for (int i=laneMarkings.size()-1; i>0 ; i--){
    //	printf("(%d, %d)\n", laneMarkings[i].getX(), laneMarkings[i].getY());
    //}
    //printf("END SORTED \n");

    // si hay poligonos detectados (esto funciona bien)
    std::vector<int> yaEtiquetados;
    FuPoint<int> laneMarking;
    // LEFT
    if ( polyDetectedLeft || polyDetectedCenter || polyDetectedRight ){
    	for (int i=laneMarkings.size()-1; i>0 ; i--) {
    		laneMarking = laneMarkings[i];
	        if (polyDetectedLeft) {
	            if (isInPolyRoi(polyLeft, laneMarking)) {
	                laneMarkingsLeft.push_back(laneMarking);
	                yaEtiquetados.push_back(i);
	                continue;
	            }
	        }
	        if (polyDetectedCenter) {
	            if (isInPolyRoi(polyCenter, laneMarking)) {
	                laneMarkingsCenter.push_back(laneMarking);
	                yaEtiquetados.push_back(i);
	                continue;
	            }
	        }
	        if (polyDetectedRight) {
	            if (isInPolyRoi(polyRight, laneMarking)) {
	                laneMarkingsRight.push_back(laneMarking);
	                yaEtiquetados.push_back(i);
	                continue;
	            }
	        }
	    }
	}
    

    //marcar puntos
    if ( !(polyDetectedLeft && polyDetectedCenter && polyDetectedRight)){
    	//assign bottom points according to position    	
    	// label first lane marks
    	for (int i=laneMarkings.size()-1; i>0 ; i--) {

    		if(std::find(yaEtiquetados.begin(), yaEtiquetados.end(), i) != yaEtiquetados.end()) {
			    continue;
			}

    		if(laneMarkings[i].getY()>110){
	    		laneMarking = laneMarkings[i];
		        //j=i;
		        if (isInDefaultRoi(LEFT, laneMarking)) {
		            laneMarkingsLeft.push_back(laneMarking);
		            yaEtiquetados.push_back(i);
		            continue;
		        }
		        if (isInDefaultRoi(CENTER, laneMarking)) {
		            laneMarkingsCenter.push_back(laneMarking);
		            yaEtiquetados.push_back(i);
		            continue;
		        }
		        if (isInDefaultRoi(RIGHT, laneMarking)) {
		            laneMarkingsRight.push_back(laneMarking);
		            yaEtiquetados.push_back(i);
		            continue;
		        }
		    }
	    }
	    
	    //printf("%d\n", j);
	    // printf("Sizes: L: %d, C: %d, R: %d\n",laneMarkingsLeft.size(),laneMarkingsCenter.size(),laneMarkingsRight.size());

	    // using the minimum distance of the next point to the first points, arrange the next point
	    FuPoint<int> p1;
	    for (int i=laneMarkings.size()-1; i>0; i--) {
	    	if(std::find(yaEtiquetados.begin(), yaEtiquetados.end(), i) != yaEtiquetados.end()) {
			    continue;
			}


		    	laneMarking = laneMarkings[i];
		    	//LEFT
		    	double distanceL = 1000;
		    	if(laneMarkingsLeft.size()>0) {
		    		p1=laneMarkingsLeft[laneMarkingsLeft.size()-1];
		    		distanceL = distanceBetweenPoints(p1, laneMarking);
		    	}
		    	//CENTER
		    	double distanceC = 1000;
		    	if(laneMarkingsCenter.size()>0) {
		    		p1=laneMarkingsCenter[laneMarkingsCenter.size()-1];
		    		distanceC = distanceBetweenPoints(p1, laneMarking);
		    	}
		    	//RIGHT
		    	double distanceR = 1000;
		    	if(laneMarkingsRight.size()>0) {
		    		p1=laneMarkingsRight[laneMarkingsRight.size()-1];
		    		distanceR = distanceBetweenPoints(p1, laneMarking);
		    	}

		    	if(distanceL <= distanceC && distanceL <= distanceR){
					laneMarkingsLeft.push_back(laneMarking);
		    		continue;
		    	}
		    	if(distanceC <= distanceL && distanceC <= distanceR){
		    		laneMarkingsCenter.push_back(laneMarking);
		    		continue;
		    	}
		    	if(distanceR <= distanceC && distanceR <= distanceL){
		    		laneMarkingsRight.push_back(laneMarking);
		    		continue;
		    	}
		    
	    }
	    // printf("Added: L: %d, C: %d, R: %d\n",laneMarkingsLeft.size(),laneMarkingsCenter.size(),laneMarkingsRight.size());
    }



    //ROS_INFO_STREAM("buildLaneMarkingsLists ending");
}


/**
 * Creates three vectors of lane marking points out of the given lane marking
 * point vector.
 *
 * A point has to lie within the ROI of the previously detected lane polynomial
 * or within the default ROI, if no polynomial was detected.
 * The lists are the input data for the RANSAC algorithm.
 *
 * @param laneMarkings  a vector containing all detected lane markings
 */
void cLaneDetectionFu::buildLaneMarkingsListsHorizontal(
        const std::vector<FuPoint<int>> &laneMarkings) {
    //ROS_INFO_STREAM("buildLaneMarkingsLists beginning");
    laneMarkingsHorizontal.clear();

    for (FuPoint<int> laneMarking : laneMarkings) {
        
        if (polyDetectedHorizontal) {
            if (isInPolyRoi(polyHorizontal, laneMarking)) {
                laneMarkingsHorizontal.push_back(laneMarking);
                continue;
            }
        }


        if (isInDefaultRoi(HORIZONTAL, laneMarking)) {
            laneMarkingsHorizontal.push_back(laneMarking);
            continue;
        }
    }

    //ROS_INFO_STREAM("buildLaneMarkingsLists ending");
}



/**
 * Calculates the horizontal distance between a point and the default line given
 * by its position.
 *
 * @param line  The position of the default line (LEFT, CENTER or RIGHT)
 * @param p     The given point
 * @return      The horizontal distance between default line and point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistanceToDefaultLine(ePosition &line, FuPoint<int> &p) {
    //ROS_INFO_STREAM("horizDistanceToDefaultLine beginning");
    double pX = p.getX();
    double distance = 0;

    switch (line) {
    case LEFT:
        distance = std::abs(pX - defaultXLeft);
        break;
    case CENTER:
        distance = std::abs(pX - defaultXCenter);
        break;
    case RIGHT:
        distance = std::abs(pX - defaultXRight);
        break;
    }
    //ROS_INFO_STREAM("horizDistanceToDefaultLine ending");
    return distance;
}

/**
 * Calculates the horizontal distance between a point and the default line given
 * by its position.
 *
 * @param line  The position of the default line (LEFT, CENTER or RIGHT)
 * @param p     The given point
 * @return      The horizontal distance between default line and point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::vertDistanceToDefaultLine(ePosition &line, FuPoint<int> &p) {
    //ROS_INFO_STREAM("horizDistanceToDefaultLine beginning");
    double pY = p.getY();
    double distance = 0;

    distance = std::abs(pY - defaultYHorizontal);
    //ROS_INFO_STREAM("horizDistanceToDefaultLine ending");
    return distance;
}

/**
 * Calculates the horizontal distance between a point and a polynomial.
 *
 * @param poly  The given polynomial
 * @param p     The given point
 * @return      The horizontal distance between the polynomial and the point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistanceToPolynomial(NewtonPolynomial& poly, FuPoint<int> &p) {
    //ROS_INFO_STREAM("horizDistanceToPolynomial beginning");
    double pY = p.getY();
    double pX = p.getX();

    double polyX = poly.at(pY);
    double distance = std::abs(pX - polyX);
    //ROS_INFO_STREAM("horizDistanceToPolynomial ending");
    return distance;
}

/**
 * Calculates the horizontal distance between a point and a polynomial.
 *
 * @param poly  The given polynomial
 * @param p     The given point
 * @return      The horizontal distance between the polynomial and the point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::vertDistanceToPolynomial(NewtonPolynomial& poly, FuPoint<int> &p) {
    //ROS_INFO_STREAM("horizDistanceToPolynomial beginning");
    double pY = p.getY();
    double pX = p.getX();

    double polyY = poly.at(pX);
    double distance = std::abs(pY - polyY);
    //ROS_INFO_STREAM("horizDistanceToPolynomial ending");
    return distance;
}

/**
 * Method, that checks if a point lies within the default ROI of a position.
 *
 * @param position  The position of the default ROI
 * @param p         The given point, which is checked
 * @return          True, if the point lies within the default ROI
 */
bool cLaneDetectionFu::isInDefaultRoi(ePosition position, FuPoint<int> &p) {
    //ROS_INFO_STREAM("isInDefaultRoi beginning");
    if (p.getY() < minYDefaultRoi || p.getY() > maxYRoi) {
        return false;
    }
    else if (position == HORIZONTAL && vertDistanceToDefaultLine(position, p)
            <= interestDistanceDefault){
        return true;
    }
    else if (horizDistanceToDefaultLine(position, p)
            <= interestDistanceDefault) {
        return true;
    }
    else {
        return false;
    }
    //ROS_INFO_STREAM("isInDefaultRoi ending");
}

/**
 * Method, that checks if a point lies within the the ROI of a polynomial.
 *
 * @param poly      The polynomial, whose ROI is used
 * @param p         The point, which is checked
 * @return          True, if the point lies within the polynomial's ROI
 */
bool cLaneDetectionFu::isInPolyRoi(NewtonPolynomial &poly, FuPoint<int> &p) {
    //ROS_INFO_STREAM("isInPolyRoi beginning");
    if (p.getY() < minYPolyRoi || p.getY() > maxYRoi) {
        return false;
    }
    else if (horizDistanceToPolynomial(poly, p) <= interestDistancePoly) {
        return true;
    }
    else {
        return false;
    }
    //ROS_INFO_STREAM("isInPolyRoi ending");
}

/**
 * Calculates the horizontal distance between two points.
 *
 * @param p1    The first point
 * @param p2    The second point
 * @return      The horizontal distance between the two points, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistance(FuPoint<int> &p1, FuPoint<int> &p2) {
    //ROS_INFO_STREAM("horizDistance beginning");
    double x1 = p1.getX();
    double x2 = p2.getX();
    //ROS_INFO_STREAM("horizDistance ending");
    return std::abs(x1 - x2);
}

/**
 * Calculates the distance between two points.
 *
 * @param p1    The first point
 * @param p2    The second point
 * @return      The distance between the two points
 */
double cLaneDetectionFu::distanceBetweenPoints(FuPoint<int> &p1, FuPoint<int> &p2) {
    //ROS_INFO_STREAM("horizDistance beginning");
    double x1 = p1.getX();
    double x2 = p2.getX();
    double y1 = p1.getY();
    double y2 = p2.getY();
    //ROS_INFO_STREAM("horizDistance ending");
    return std::sqrt((x2 - x1)*(x2 - x1)+(y2 - y1)*(y2 - y1));
}

/**
 * Calculates the gradient of a polynomial at a given x value. The used formula
 * was obtained by the following steps:
 * - start with the polynomial of 2nd degree in newton basis form:
 *   p(x) = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 * - expand the equation and sort it by descending powers of x
 * - form the first derivative
 *
 * Applying the given x value then results in the wanted gradient.
 *
 * @param x         The given x value
 * @param points    The data points used for interpolating the polynomial
 * @param coeffs    The coefficients under usage of the newton basis
 * @return          The gradient of the polynomial at x
 */
double cLaneDetectionFu::gradient(double x, std::vector<FuPoint<int>> &points,
        std::vector<double> coeffs) {
    return 2 * coeffs[2] * x + coeffs[1] - coeffs[2] * points[1].getY()
            - coeffs[2] * points[0].getY();
}

/**
 * Calculates the x value of the point where the normal of the tangent of a
 * polynomial at a given point p intersects with a second polynomial.
 *
 * The formula for the intersection point is obtained by setting equal the
 * following two formula:
 *
 * 1. the formula of the normal in point-slope-form:
 *     y - p_y = -(1 / m) * (x - p_x) which is the same as
 *           y = -(x / m) + (p_x / m) + p_y
 *
 * 2. the formula of the second polynomial of 2nd degree in newton basis form:
 *           y = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 *
 * Expanding everything and moving it to the right side gives a quadratic
 * equation in the general form of 0 = ax^2 + bx + c, which can be solved using
 * the general quadratic formula x = (-b +- sqrt(b^2 - 4ac)) / 2a
 *
 * The three cases for the discriminant are taken into account.
 *
 * @param p         The point of the first poly at which its tangent is used
 * @param m         The gradient of the tangent
 * @param points    The data points used for interpolating the second polynomial
 * @param coeffs    The coeffs of the second polynomial with newton basis
 * @return          The x value of the intersection point of normal and 2nd poly
 */
double cLaneDetectionFu::intersection(FuPoint<double> &p, double &m,
        std::vector<FuPoint<int>> &points, std::vector<double> &coeffs) {
    double a = coeffs[2];
    double b = coeffs[1] - (coeffs[2] * points[1].getY())
            - (coeffs[2] * points[0].getY()) + (1.0 / m);
    double c = coeffs[0] - (coeffs[1] * points[0].getY())
            + (coeffs[2] * points[0].getY() * points[1].getY())
            - p.getY() - (p.getX() / m);

    double dis = std::pow(b, 2) - (4 * a * c);
    double x1 = 0;
    double x2 = 0;

    if (dis < 0) {
        return -1;
    }
    else if (dis == 0) {
        return -b / (2 * a);
    }
    else {
        x1 = (-b + std::sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
        x2 = (-b - std::sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    return fmax(x1, x2);
}

/**
 * Calculates the gradient of a second polynomial at the point, at which the
 * normal of the tangent of the first polynomial at the given point
 * intersects with the second polynomial.
 *
 * @param x         The given x value of the point on the first polynomial
 * @param poly1     The first polynomial
 * @param points1   The data points used for interpolating the first poly
 * @param points2   The data points used for interpolating the second poly
 * @param coeffs1   The coeffs of the first poly using newton basis
 * @param coeffs2   The coeffs of the second poly using newton basis
 * @param m1        The gradient of the first poly at x
 * @return          The gradient of the second poly at the intersection point
 */
double cLaneDetectionFu::nextGradient(double x, NewtonPolynomial &poly1,
        std::vector<FuPoint<int>> &points1, std::vector<FuPoint<int>> &points2,
        std::vector<double> coeffs1, std::vector<double> coeffs2, double m1) {

    FuPoint<double> p = FuPoint<double>(x, poly1.at(x));
    double x2 = intersection(p, m1, points2, coeffs2);

    return gradient(x2, points2, coeffs2);
}

/**
 * Check two gradients for similarity. Return true if the difference in degree
 * is less than 10.
 *
 * @param m1    The first gradient
 * @param m2    The second gradient
 * @return      True, if the diffenence between the gradients is less than 10°
 */
bool cLaneDetectionFu::gradientsSimilar(double &m1, double &m2) {
    double a1 = atan(m1) * 180 / PI;
    double a2 = atan(m2) * 180 / PI;

    if (abs(a1 - a2) < 10) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * Finds the position of the polynomial with the highest proportion.
 * @return The position of the best polynomial
 */
ePosition cLaneDetectionFu::maxProportion() {
    ePosition maxPos = LEFT;
    double maxVal = bestPolyLeft.second;

    if (bestPolyCenter.second > maxVal) {
        maxPos = CENTER;
        maxVal = bestPolyCenter.second;
    }

    if (bestPolyRight.second > maxVal) {
        maxPos = RIGHT;
    }

    return maxPos;
}

/**
 * Create the lane polynomial starting from the detected polynomial of the
 * given position. A lane polynomial is formed by shifting points with
 * different x-values of the used polynomial along the normals of the polynomial
 * at this points to the distance, where the respective lane polynomial is
 * expected to lie.
 *
 * @param position  The position of the detected polynomial used as reference
 */
void cLaneDetectionFu::createLanePoly(ePosition position) {
    //ROS_INFO_STREAM("createLanePoly beginning");
    lanePoly.clear();

    double coef = 1.2;

    double x1 = minYPolyRoi+5;
    double x2 = minYPolyRoi + ((proj_image_h-minYPolyRoi)/2);
    double x3 = proj_image_h-5;

    FuPoint<double> pointRight1;
    FuPoint<double> pointRight2;
    FuPoint<double> pointRight3;

    double dRight = 0;

    NewtonPolynomial usedPoly;

    double y1;
    double y2;
    double y3;
 
    if (position == LEFT) {
        usedPoly = polyLeft;
        dRight = defaultXLeft-5;
    }
    else if (position == CENTER) {
        usedPoly = polyCenter;
        //dRight = defaultXCenter-5;
        dRight = defaultXCenter+5;
    }
    else if (position == RIGHT) {
        usedPoly = polyRight;
        dRight = defaultXRight+5;
    }


    pointRight1 = FuPoint<double>(x1, usedPoly.at(x1) - dRight);
    pointRight2 = FuPoint<double>(x2, usedPoly.at(x2) - dRight);
    pointRight3 = FuPoint<double>(x3, usedPoly.at(x3) - dRight);

    // create the lane polynomial out of the shifted points
    lanePoly.addDataXY(pointRight1);
    lanePoly.addDataXY(pointRight2);
    lanePoly.addDataXY(pointRight3);

    lanePolynomial.setLanePoly(lanePoly);
    lanePolynomial.setDetected();
    lanePolynomial.setLastUsedPosition(position);

    //ROS_INFO_STREAM("createLanePoly ending");
}

/**
 * Create the lane polynomial starting from the detected polynomial of the
 * given position. A lane polynomial is formed by shifting points with
 * different x-values of the used polynomial along the normals of the polynomial
 * at this points to the distance, where the respective lane polynomial is
 * expected to lie.
 *
 * @param position  The position of the detected polynomial used as reference
 */
void cLaneDetectionFu::createLanePolyHorizontal(ePosition position) {
    //ROS_INFO_STREAM("createLanePoly beginning");
    lanePoly.clear();

    double coef = 1.2;

    double x1 = minYPolyRoi+5;
    double x2 = minYPolyRoi + ((proj_image_h-minYPolyRoi)/2);
    double x3 = proj_image_h-5;

    FuPoint<double> pointRight1;
    FuPoint<double> pointRight2;
    FuPoint<double> pointRight3;

    double dRight = 0;

    NewtonPolynomial usedPoly;

    double y1;
    double y2;
    double y3;
 
    if (position == LEFT) {
        usedPoly = polyLeft;
        dRight = defaultXLeft-5;
    }
    else if (position == CENTER) {
        usedPoly = polyCenter;
        dRight = defaultXCenter-5;
    }
    else if (position == RIGHT) {
        usedPoly = polyRight;
        dRight = defaultXRight+5;
    }
    else if (position == HORIZONTAL) {

        x1 = 0;
        x2 = (proj_image_w)/2;
        x3 = proj_image_w-5;

        usedPoly = polyHorizontal;
        dRight = defaultYHorizontal+5;
    }


    pointRight1 = FuPoint<double>(x1, usedPoly.at(x1) - dRight);
    pointRight2 = FuPoint<double>(x2, usedPoly.at(x2) - dRight);
    pointRight3 = FuPoint<double>(x3, usedPoly.at(x3) - dRight);

    ROS_INFO_STREAM("Crear poly horizontal: (" << x1 << "," << usedPoly.at(x1) - dRight << "), " << "(" << x2 << "," << usedPoly.at(x2) - dRight << "), " << "(" << x3 << "," << usedPoly.at(x3) - dRight << ")");

    /*
    if (position == HORIZONTAL) {
        pointRight1 = FuPoint<double>(x1, usedPoly.at(x1) - dRight);
        pointRight2 = FuPoint<double>(x2, usedPoly.at(x2) - dRight);
        pointRight3 = FuPoint<double>(x3, usedPoly.at(x3) - dRight);
    }
    */

    // create the lane polynomial out of the shifted points
    
    
    lanePoly.addDataXY(pointRight1);
    lanePoly.addDataXY(pointRight2);
    lanePoly.addDataXY(pointRight3);

    lanePolynomial.setLanePoly(lanePoly);
    lanePolynomial.setDetected();
    lanePolynomial.setLastUsedPosition(position);
    
    
    //ROS_INFO_STREAM("createLanePoly ending");
}

//original method, should be better, but doesn't work correctly in our case when RIGHT polynomial is used
/*void LaneDetector::createLanePoly(ePosition position) {
    lanePoly.clear();

    double x1 = 0.05;
    double x2 = 0.4;
    double x3 = 1.0;

    FuPoint<double> pointRight1;
    FuPoint<double> pointRight2;
    FuPoint<double> pointRight3;

    double m1 = 0;
    double m2 = 0;
    double m3 = 0;

    double dRight = 0;

    NewtonPolynomial usedPoly;

    /*
     * Depending on the sign of the gradient of the poly at the different
     * x-values and depending on which position we are, we have to add or
     * subtract the expected distance to the respective lane polynomial, to get
     * the wanted points.
     *
     * The calculation is done for the x- and y-components of the points
     * separately using the trigonometric ratios of right triangles and the fact
     * that arctan of some gradient equals its angle to the x-axis in degree.
     */
    /*if (position == LEFT) {
        usedPoly = polyLeft;
        m1 = gradient(x1, pointsLeft, usedPoly.getCoefficients());
        m2 = gradient(x2, pointsLeft, usedPoly.getCoefficients());
        m3 = gradient(x3, pointsLeft, usedPoly.getCoefficients());

        dRight = defaultXLeft;

        if (m1 > 0) {
            pointRight1 = FuPoint<double>(x1 + dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) + dRight * sin(atan(-1 / m1)));
        }
        else {
            pointRight1 = FuPoint<double>(x1 - dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) - dRight * sin(atan(-1 / m1)));
        }

        if (m2 > 0) {
            pointRight2 = FuPoint<double>(x2 + dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) + dRight * sin(atan(-1 / m2)));
        }
        else {
            pointRight2 = FuPoint<double>(x2 - dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) - dRight * sin(atan(-1 / m2)));
        }

        if (m3 > 0) {
            pointRight3 = FuPoint<double>(x3 + dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) + dRight * sin(atan(-1 / m3)));
        }
        else {
            pointRight3 = FuPoint<double>(x3 - dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) - dRight * sin(atan(-1 / m3)));
        }
    }
    else if (position == CENTER) {
        usedPoly = polyCenter;
        m1 = gradient(x1, pointsCenter, usedPoly.getCoefficients());
        m2 = gradient(x2, pointsCenter, usedPoly.getCoefficients());
        m3 = gradient(x3, pointsCenter, usedPoly.getCoefficients());

        dRight = defaultXCenter;

        if (m1 > 0) {
            pointRight1 = FuPoint<double>(x1 + dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) + dRight * sin(atan(-1 / m1)));
        }
        else {
            pointRight1 = FuPoint<double>(x1 - dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) - dRight * sin(atan(-1 / m1)));
        }

        if (m2 > 0) {
            pointRight2 = FuPoint<double>(x2 + dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) + dRight * sin(atan(-1 / m2)));
        }
        else {
            pointRight2 = FuPoint<double>(x2 - dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) - dRight * sin(atan(-1 / m2)));
        }

        if (m3 > 0) {
            pointRight3 = FuPoint<double>(x3 + dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) + dRight * sin(atan(-1 / m3)));
        }
        else {
            pointRight3 = FuPoint<double>(x3 - dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) - dRight * sin(atan(-1 / m3)));
        }
    }
    else if (position == RIGHT) {
        usedPoly = polyRight;
        m1 = gradient(x1, pointsRight, usedPoly.getCoefficients());
        m2 = gradient(x2, pointsRight, usedPoly.getCoefficients());
        m3 = gradient(x3, pointsRight, usedPoly.getCoefficients());

        dRight = defaultXCenter;

        if (m1 > 0) {
            pointRight1 = FuPoint<double>(x1 - dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) - dRight * sin(atan(-1 / m1)));
        }
        else {
            pointRight1 = FuPoint<double>(x1 + dRight * cos(atan(-1 / m1)),
                    usedPoly.at(x1) + dRight * sin(atan(-1 / m1)));
        }

        if (m2 > 0) {
            pointRight2 = FuPoint<double>(x2 - dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) - dRight * sin(atan(-1 / m2)));
        }
        else {
            pointRight2 = FuPoint<double>(x2 + dRight * cos(atan(-1 / m2)),
                    usedPoly.at(x2) + dRight * sin(atan(-1 / m2)));
        }

        if (m3 > 0) {
            pointRight3 = FuPoint<double>(x3 - dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) - dRight * sin(atan(-1 / m3)));
        }
        else {
            pointRight3 = FuPoint<double>(x3 + dRight * cos(atan(-1 / m3)),
                    usedPoly.at(x3) + dRight * sin(atan(-1 / m3)));
        }
    }

    // create the lane polynomial out of the shifted points
    lanePoly.addDataXY(pointRight1);
    lanePoly.addDataXY(pointRight2);
    lanePoly.addDataXY(pointRight3);

    lanePolynomial.setLanePoly(lanePoly);
    lanePolynomial.setDetected();
    lanePolynomial.setLastUsedPosition(position);
}*/

/**
 * Decide, which of the detected polynomials (if there are any) should be used
 * as reference for creating the lane polynomials.
 *
 * @param startX    The x-value, starting from which we compare the detected polys
 */
void cLaneDetectionFu::detectLane() {
    //ROS_INFO_STREAM("detectLane beginning");
    int startX = detectLaneStartX;

    if (polyDetectedLeft && !polyDetectedCenter && !polyDetectedRight) {
        createLanePoly(LEFT);
    }
    else if (!polyDetectedLeft && polyDetectedCenter && !polyDetectedRight) {
        createLanePoly(CENTER);
    }
    else if (!polyDetectedLeft && !polyDetectedCenter && polyDetectedRight) {
        createLanePoly(RIGHT);
    }
    else if (polyDetectedLeft && polyDetectedCenter && !polyDetectedRight) {
        double gradLeft = gradient(startX, pointsLeft,
                polyLeft.getCoefficients());

        double gradCenter = nextGradient(startX, polyLeft, pointsLeft,
                pointsCenter, polyLeft.getCoefficients(),
                polyCenter.getCoefficients(), gradLeft);

        if (gradientsSimilar(gradLeft, gradCenter)) {
            createLanePoly(CENTER);
        }
        else {
            if (bestPolyLeft.second >= bestPolyCenter.second) {
                createLanePoly(LEFT);
            }
            else {
                createLanePoly(CENTER);
            }
        }
    }
    else if (!polyDetectedLeft && polyDetectedCenter && polyDetectedRight) {
        double gradCenter = gradient(startX, pointsCenter,
                polyCenter.getCoefficients());

        double gradRight = nextGradient(startX, polyCenter, pointsCenter,
                pointsRight, polyCenter.getCoefficients(),
                polyRight.getCoefficients(), gradCenter);

        if (gradientsSimilar(gradCenter, gradRight)) {
            createLanePoly(RIGHT);
            printf("Similar \n");
        }
        else {
            if (bestPolyCenter.second >= bestPolyRight.second) {
                createLanePoly(CENTER);
                printf("Best poly center\n");
            }
            else {
                createLanePoly(RIGHT);
                printf("else best poly\n");
            }
        }
    }
    else if (polyDetectedLeft && !polyDetectedCenter && polyDetectedRight) {
        double gradLeft = gradient(startX, pointsLeft,
                polyLeft.getCoefficients());

        double gradRight = nextGradient(startX, polyLeft, pointsLeft,
                pointsRight, polyLeft.getCoefficients(),
                polyRight.getCoefficients(), gradLeft);

        if (gradientsSimilar(gradLeft, gradRight)) {
            createLanePoly(RIGHT);
        }
        else {
            if (bestPolyLeft.second >= bestPolyRight.second) {
                createLanePoly(LEFT);
            }
            else {
                createLanePoly(RIGHT);
            }
        }
    }
    else if (polyDetectedLeft && polyDetectedCenter && polyDetectedRight) {
        double gradRight = gradient(startX, pointsRight,
                polyRight.getCoefficients());

        double gradCenter2 = gradient(startX, pointsCenter,
                polyCenter.getCoefficients());

        double gradCenter1 = nextGradient(startX, polyRight, pointsRight,
                pointsCenter, polyRight.getCoefficients(),
                polyCenter.getCoefficients(), gradRight);

        //double gradLeft1 = nextGradient(startX, polyRight, pointsRight,
        //        pointsLeft, polyRight.getCoefficients(),
        //        polyLeft.getCoefficients(), gradRight);

        double gradLeft2 = nextGradient(startX, polyCenter, pointsCenter,
                pointsLeft, polyCenter.getCoefficients(),
                polyLeft.getCoefficients(), gradCenter2);

        if (gradientsSimilar(gradRight, gradCenter1)) {
            // ?!
            //if (gradientsSimilar(gradCenter1, gradLeft1)) {
                createLanePoly(RIGHT);
            //}
            //else {
            //    createLanePoly(RIGHT);
            //}
        }
        else {
            if (gradientsSimilar(gradCenter2, gradLeft2)) {
                createLanePoly(CENTER);
            }
            else {
                //ROS_ERROR("Creating lane according to max proportion.");
                ePosition maxPos = maxProportion();
                
                createLanePoly(maxPos);         
            }
        }
    }
    else if (!polyDetectedLeft && !polyDetectedCenter && !polyDetectedRight) {
        lanePoly.clear();
        lanePolynomial.setNotDetected();
    }
    //ROS_INFO_STREAM("detectLane ending");

    // if (polyDetectedHorizontal) {
    //     createLanePolyHorizontal(HORIZONTAL);
    // }
}

/**
 * Starts the RANSAC algorithm for detecting each of the three lane marking
 * polynomials.
 */
void cLaneDetectionFu::ransac() {
    //ROS_INFO_STREAM("ransac beginning");
    polyDetectedLeft = ransacInternal(LEFT, laneMarkingsLeft, bestPolyLeft,
            polyLeft, supportersLeft, prevPolyLeft, pointsLeft);

    polyDetectedCenter = ransacInternal(CENTER, laneMarkingsCenter,
            bestPolyCenter, polyCenter, supportersCenter, prevPolyCenter,
            pointsCenter);

    polyDetectedRight = ransacInternal(RIGHT, laneMarkingsRight, bestPolyRight,
            polyRight, supportersRight, prevPolyRight, pointsRight);

    polyDetectedHorizontal = ransacInternal(HORIZONTAL, laneMarkingsHorizontal, bestPolyHorizontal,
            polyHorizontal, supportersHorizontal, prevPolyHorizontal, pointsHorizontal);
    // ROS_INFO_STREAM("ransac ending");
}

/**
 * Detects a polynomial with RANSAC in a given list of lane marking edge points.
 *
 * @param position      The position of the wanted polynomial
 * @param laneMarkings  A reference to the list of lane marking edge points
 * @param bestPoly      A reference to a pair containing the present best
 *                      detected polynomial and a value representing the fitting
 *                      quality called proportion
 * @param poly          A reference to the polynomial that gets detected
 * @param supporters    A reference to the supporter points of the present best
 *                      polynomial
 * @param prevPoly      A reference to the previous polynomial detected at this
 *                      position
 * @param points        A reference to the points selected for interpolating the
 *                      present best polynomial
 * @return              true if a polynomial could be detected and false when not
 */
bool cLaneDetectionFu::ransacInternal(ePosition position,
        std::vector<FuPoint<int>>& laneMarkings,
        std::pair<NewtonPolynomial, double>& bestPoly, NewtonPolynomial& poly,
        std::vector<FuPoint<int>>& supporters, NewtonPolynomial& prevPoly,
        std::vector<FuPoint<int>>& points) {
    //ROS_INFO_STREAM("ransacInternal beginning");

    if (laneMarkings.size() < 7) {
        return false;
    }

    int iterations = 0;

    // sort the lane marking edge points
    std::vector<FuPoint<int>> sortedMarkings = laneMarkings;

    std::sort(sortedMarkings.begin(), sortedMarkings.end(),
            [](FuPoint<int> a, FuPoint<int> b) {
                return a.getY() < b.getY();
            });

    std::vector<FuPoint<int>> tmpSupporters = std::vector<FuPoint<int>>();

    // vectors for points selected from the bottom, mid and top of the sorted
    // point vector
    std::vector<FuPoint<int>> markings1 = std::vector<FuPoint<int>>();
    std::vector<FuPoint<int>> markings2 = std::vector<FuPoint<int>>();
    std::vector<FuPoint<int>> markings3 = std::vector<FuPoint<int>>();

    bool highEnoughY = false;

    // Points are selected from the bottom, mid and top. The selection regions
    // are spread apart for better results during RANSAC
    for (std::vector<FuPoint<int>>::size_type i = 0; i != sortedMarkings.size();
            i++) {
        if(position == HORIZONTAL) {
            if (i < double(sortedMarkings.size())*1/3) {
                markings1.push_back(sortedMarkings[i]);
            }
            else if (i >= (double(sortedMarkings.size())*1/3)
                    && i < (double(sortedMarkings.size())*2/3)) {
                markings2.push_back(sortedMarkings[i]);
            }
            else if (i >= (double(sortedMarkings.size())*2/3)) {
                markings3.push_back(sortedMarkings[i]);
            }
        }
        else {
            if (i < double(sortedMarkings.size()) / 7) {
                markings1.push_back(sortedMarkings[i]);
            }
            else if (i >= (double(sortedMarkings.size()) / 7) * 3
                    && i < (double(sortedMarkings.size()) / 7) * 4) {
                markings2.push_back(sortedMarkings[i]);
            }
            else if (i >= (double(sortedMarkings.size()) / 7) * 6) {
                markings3.push_back(sortedMarkings[i]);
            }

            if (sortedMarkings[i].getY() > 5) {
                highEnoughY = true;
            }
        }
    }

    //what is this for?
    if (position == CENTER) {
        if (!highEnoughY) {
            prevPoly = poly;
            poly.clear();
            return false;
        }
    }

    // save the polynomial from the previous picture
    prevPoly = poly;

    while (iterations < iterationsRansac) {
        iterations++;

        // randomly select 3 different lane marking points from bottom, mid and
        // top
        int pos1 = rand() % markings1.size();
        int pos2 = rand() % markings2.size();
        int pos3 = rand() % markings3.size();

        FuPoint<int> p1 = markings1[pos1];
        FuPoint<int> p2 = markings2[pos2];
        FuPoint<int> p3 = markings3[pos3];

        double p1X = p1.getX();
        double p1Y = p1.getY();
        double p2X = p2.getX();
        double p2Y = p2.getY();
        double p3X = p3.getX();
        double p3Y = p3.getY();

        // clear poly for reuse
        poly.clear();

        // create a polynomial with the selected points
        if(position==HORIZONTAL){
            poly.addData(p1Y, p1X);
            poly.addData(p2Y, p2X);
            poly.addData(p3Y, p3X);
        }
        else
        {
            poly.addData(p1X, p1Y);
            poly.addData(p2X, p2Y);
            poly.addData(p3X, p3Y);
        }

        // check if this polynomial is not useful
        if (!polyValid(position, poly, prevPoly)) {
            poly.clear();
            continue;
        }

        // count the supporters and save them for debugging
        int count1 = 0;
        int count2 = 0;
        int count3 = 0;

        // find the supporters
        tmpSupporters.clear();

        for (FuPoint<int> p : markings1) {
            if (position == HORIZONTAL)
            {
                if (vertDistanceToPolynomial(poly, p) <= maxDistance) {
                    count1++;
                    tmpSupporters.push_back(p);
                }
            }
            else {
                if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                    count1++;
                    tmpSupporters.push_back(p);
                }
            }
        }

        for (FuPoint<int> p : markings2) {
            if (position == HORIZONTAL)
            {
                if (vertDistanceToPolynomial(poly, p) <= maxDistance) {
                    count2++;
                    tmpSupporters.push_back(p);
                }
            }
            else {
                if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                    count2++;
                    tmpSupporters.push_back(p);
                }
            }
        }

        for (FuPoint<int> p : markings3) {
            if (position == HORIZONTAL)
            {
                if (vertDistanceToPolynomial(poly, p) <= maxDistance) {
                    count3++;
                    tmpSupporters.push_back(p);
                }
            }
            else {
                if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                    count3++;
                    tmpSupporters.push_back(p);
                }
            }
        }

        if (count1 == 0 || count2 == 0 || count3 == 0) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly had no supporters in one of the regions");
            continue;
        }

        // calculate the proportion of supporters of all lane markings
        double proportion = (double(count1) / markings1.size()
                + double(count2) / markings2.size()
                + 3 * (double(count3) / markings3.size())) / 5;

        

        if (proportion < proportionThreshould) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly proportion was smaller than threshold");
            continue;
        }

        // check if poly is better than bestPoly
        if (proportion > bestPoly.second) {
            bestPoly = std::make_pair(poly, proportion);
            supporters = tmpSupporters;

            points.clear();
            points.push_back(p1);
            points.push_back(p2);
            points.push_back(p3);

            /*
            if(position == HORIZONTAL){
                ROS_INFO_STREAM("Buscando HORIZONTAL: " << proportion << "," << count1 << "," << count2 << "," << count3);
            
                ROS_INFO_STREAM("P1 x: " << p1.getX() << ", y: " << p1.getY());
                ROS_INFO_STREAM("P2 x: " << p2.getX() << ", y: " << p2.getY());
                ROS_INFO_STREAM("P3 x: " << p3.getX() << ", y: " << p3.getY());

                ROS_INFO_STREAM("P1C x: " << p1.getX() << ", y: " << poly.at(p1.getX()));
            }
            */
        }
    }

    poly = bestPoly.first;

    /*
    if(position == HORIZONTAL){
            ROS_INFO_STREAM("Poly degree: " << poly.getDegree());
        }
    */

    if (poly.getDegree() == -1) {
        return false;
    }
    //ROS_INFO_STREAM("ransacInternal ending");
    return true;
}

/**
 * Method, that checks, if a polynomial produced during RANSAC counts as usable.
 *
 * @param position  The position of the polynomial, that is checked
 * @param poly      The polynomial, that is checked
 * @param prevPoly  The previous polynomial detected at this position
 * @return          True, if the polynomial counts as valid
 */
bool cLaneDetectionFu::polyValid(ePosition position, NewtonPolynomial poly,
        NewtonPolynomial prevPoly) {
    //ROS_INFO_STREAM("polyValid");
    FuPoint<int> p1 = FuPoint<int>(poly.at(polyY1), polyY1);    //y = 75

    if (horizDistanceToDefaultLine(position, p1) > 10) {
        //ROS_INFO("Poly was to far away from default line at y = 25");
        return false;
    }

    FuPoint<int> p2 = FuPoint<int>(poly.at(polyY2), polyY2);    //y = 60

    if (horizDistanceToDefaultLine(position, p2) > 20) {
        //ROS_INFO("Poly was to far away from default line at y = 25");
        return false;
    }

    FuPoint<int> p3 = FuPoint<int>(poly.at(polyY3), polyY3);    //y = 40

    if (horizDistanceToDefaultLine(position, p3) > 40) {
        //ROS_INFO("Poly was to far away from default line at y = 30");
        return false;
    }

    if (prevPoly.getDegree() != -1) {
        FuPoint<int> p4 = FuPoint<int>(poly.at(polyY1), polyY1);
        FuPoint<int> p5 = FuPoint<int>(prevPoly.at(polyY1), polyY1);

        if (horizDistance(p4, p5) > 5) {//0.05 * meters) {
            //ROS_INFO("Poly was to far away from previous poly at y = 25");
            return false;
        }

        FuPoint<int> p6 = FuPoint<int>(poly.at(polyY2), polyY2);
        FuPoint<int> p7 = FuPoint<int>(prevPoly.at(polyY2), polyY2);

        if (horizDistance(p6, p7) > 5) {//0.05 * meters) {
            //ROS_INFO("Poly was to far away from previous poly at y = 30");
            return false;
        }
    }

    //ROS_INFO_STREAM("polyValid ending");

    return true;
}



void cLaneDetectionFu::pubRGBImageMsg(cv::Mat& rgb_mat, image_transport::CameraPublisher publisher)
{
    //ROS_INFO_STREAM("pubRGBImageMsg beginning");
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    rgb_img->header.seq = head_sequence_id;
    rgb_img->header.stamp = head_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;

    publisher.publish(rgb_img, rgb_camera_info);
    //ROS_INFO_STREAM("pubRGBImageMsg ending");
}

void cLaneDetectionFu::pubAngle()
{
    //ROS_INFO_STREAM("pubAngle beginning");
    if (!lanePolynomial.hasDetected()) {
	return;
    }

    double oppositeLeg = lanePolynomial.getLanePoly().at(proj_image_h-angleAdjacentLeg);    
    double result = atan (oppositeLeg/angleAdjacentLeg) * 180 / PI;

    if (std::abs(result - lastAngle) > maxAngleDiff) {
        if (result - lastAngle > 0) {
            result = lastAngle + maxAngleDiff;
        } else {
            result = lastAngle - maxAngleDiff;
        }
    }

    lastAngle = result;

    std_msgs::Float32 angleMsg;

    angleMsg.data = result;

    publish_angle.publish(angleMsg);
    //ROS_INFO_STREAM("pubAngle ending");
}

void cLaneDetectionFu::pubGradientAngle()
{
    int position = proj_image_h - angleAdjacentLeg;

    double val1 = lanePolynomial.getLanePoly().at(position);
    double val2 = lanePolynomial.getLanePoly().at(position+1);

    double result = atan (val1-val2) * 180 / PI;

    lastAngle = result;

    std_msgs::Float32 angleMsg;

    angleMsg.data = result;

    publish_angle.publish(angleMsg);
}

void cLaneDetectionFu::config_callback(line_detection_fu::LaneDetectionConfig &config, uint32_t level) {
    ROS_INFO_STREAM("Reconfigure Request");

    defaultXLeft = config.defaultXLeft;
    defaultXCenter = config.defaultXCenter;
    defaultXRight = config.defaultXRight;
    interestDistancePoly = config.interestDistancePoly;
    interestDistanceDefault= config.interestDistanceDefault;
    iterationsRansac = config.iterationsRansac;
    maxYRoi = config.maxYRoi;
    minYDefaultRoi = config.minYDefaultRoi;
    minYPolyRoi = config.minYPolyRoi;
    polyY1 = config.polyY1;
    polyY2 = config.polyY2;
    polyY3 = config.polyY3;
    detectLaneStartX = config.detectLaneStartX;
    maxAngleDiff = config.maxAngleDiff;
    proj_y_start = config.proj_y_start;
    roi_top_w = config.roi_top_w;
    roi_bottom_w = config.roi_bottom_w;
    proportionThreshould = config.proportionThreshould;
    m_gradientThreshold = config.m_gradientThreshold;
    m_nonMaxWidth = config.m_nonMaxWidth;
    laneMarkingSquaredThreshold = config.laneMarkingSquaredThreshold;
    angleAdjacentLeg = config.angleAdjacentLeg;
    scanlinesVerticalDistance = config.scanlinesVerticalDistance;
    scanlinesMaxCount = config.scanlinesMaxCount;

    scanlines = getScanlines();
    //ROS_INFO_STREAM("after getScanlines");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cLaneDetectionFu");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE_HZ);

    cLaneDetectionFu node = cLaneDetectionFu(nh);

    dynamic_reconfigure::Server<line_detection_fu::LaneDetectionConfig> server;
    dynamic_reconfigure::Server<line_detection_fu::LaneDetectionConfig>::CallbackType f;
    f = boost::bind(&cLaneDetectionFu::config_callback, &node, _1, _2);
    server.setCallback(f);

    // //ROS_INFO_STREAM("Before while");
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        // //ROS_INFO_STREAM("At while");
    }
    return 0;
}
