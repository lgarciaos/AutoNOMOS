/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**/
// Christoph Brickl, AEV:

/*! \brief cSpurerkennung
          
	Zur Erkennung einer Spur wird das RGB Bild der Asus Xtion eingelesen und verarbeitet
	Weiteres Vorgehen:
	Zuschneiden des Orginal Bildes auf die eingestellte Gr๖฿e
	Erzeugen eines Graustufen Bildes
	Anwenden eines Schwellwertfilters
	Kantendedektion
	Suchen nach Kanten auf den eingestellten cm-Marken
	Auswerten der gefundenen Punkte ob sinnvolle Linie erstellt werden kann
	Anzeigen der Linie mit Hilfe der GLC

 */

#ifndef _LaneDetection_FILTER_HEADER_
#define _LaneDetection_FILTER_HEADER_
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>

#include <stdlib.h>

#include "tools/LineSegment.h"
#include "tools/Edges.h"
#include "tools/NewtonPolynomial.h"
#include "tools/LanePolynomial.h"
#include "tools/enums.h"
#include "tools/IPMapper.h"

#include "tools/dbscan.h"

#include <dynamic_reconfigure/server.h>
#include <line_detection_fu_mod/LaneDetectionConfig.h>


#include "nav_msgs/GridCells.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
// #include "msg/pts_array.msg"

using namespace std;


class cLaneDetectionFu
{
    private:

        // the node handle
        ros::NodeHandle nh_;

        // Node handle in the private namespace
        ros::NodeHandle priv_nh_;

        // subscribers
        ros::Subscriber read_images_;
        ros::Subscriber sub_planning;
        ros::Subscriber planningxy;
        ros::Subscriber sub_localization;
        ros::Subscriber sub_vel;
        ros::Subscriber sub_des_state;

        // publishers
        ros::Publisher publish_angle;

        ros::Publisher pub_left;
        ros::Publisher pub_center;
        ros::Publisher pub_right;
        ros::Publisher pub_horizontal;

        ros::Publisher pub_ransac_left;
        ros::Publisher pub_ransac_center;
        ros::Publisher pub_ransac_right;

        ros::Publisher pub_lane_model;
        ros::Publisher pub_ransac_horizontal;

        ros::Publisher pub_speed;
        ros::Publisher pub_steering;
        ros::Publisher pub_orientation;

        IPMapper ipMapper;

        std::string camera_name;
        
        int cam_w;
        int cam_h;
        int proj_y_start;
        int proj_image_h;
        int proj_image_w;
        int proj_image_w_half;
        int proj_image_horizontal_offset;
        int roi_top_w;
        int roi_bottom_w;

        int scanlinesVerticalDistance;
        int scanlinesMaxCount;

        vector<vector<LineSegment<int>> > scanlines;

        int m_gradientThreshold;
        int m_nonMaxWidth;

        int polyY1;
        int polyY2;
        int polyY3;

        int estado;
        int dbscan_epsilon;
        

        /**
         * The lane marking polynomials detected in the current picture.
         */
        NewtonPolynomial polyLeft;
        NewtonPolynomial polyCenter;
        NewtonPolynomial polyRight;
        NewtonPolynomial polyHorizontal;

        /**
         *  default lane marking lines as polynomials
         */
        NewtonPolynomial polyDefaultLeft;
        NewtonPolynomial polyDefaultCenter;
        NewtonPolynomial polyDefaultRight;

        /**
         * Horizontal relative positions of the default lane marking lines.
         *
         * These lines are situated in a position, where the lane markings of a
         * straight lane would show up in the relative coordinate system with the
         * car standing in the center of the right lane.
         */
        int defaultXLeft;
        int defaultXCenter;
        int defaultXRight;
        int defaultYHorizontal;

        /**
         * The maximum distance of a point to a polynomial so that it counts as a
         * supporter.
         */
        int maxDistance;

        /**
         * The horizontal distance to the last detected polynomial, within lane
         * markings have to lie to be considered for the detection of the next
         * polynomial. The width of the polynomial region of interest is two times
         * this distance.
         */
        int interestDistancePoly;

        /**
         * The horizontal distance to the default line, within lane markings have to
         * lie to be considered for the detection of a polynomial. The width of the
         * default region of interest is two times this distance.
         */
        int interestDistanceDefault;

        /**
         * The minimal y of the ROIs. Points with smaller y-Values are not
         * used in RANSAC.
         */
        int maxYRoi;

        /**
         * The maximal y of default ROIs. Points with bigger y-Values are not used.
         */
        int minYDefaultRoi;

        /**
         * The maximal y of the polynomial ROIs. Points with bigger y-Values are not
         * used.
         */
        int minYPolyRoi;

        /**
         * The minimal proportion of supporters of all points within a ROI.
         * Polynomials with lower proportions are discarded.
         */
        double proportionThreshould;

        /**
         * Number of RANSAC iterations
         */
        int iterationsRansac;

        /**
         * flags to determine if a valid polynomial was detected in the last frame
         * and therefore the polynomial ROI should be used or if no polynomial could
         * be detected and the default ROI is used.
         */
        bool polyDetectedLeft;
        bool polyDetectedCenter;
        bool polyDetectedRight;

        bool polyDetectedHorizontal;

        /**
         * pairs for saving the best lane polynomials produced during RANSAC
         *
         * first : current best NewtonPolynomial
         * second: proportion of supporters of used lane marking points (quality)
         */
        std::pair<NewtonPolynomial, double> bestPolyLeft;
        std::pair<NewtonPolynomial, double> bestPolyCenter;
        std::pair<NewtonPolynomial, double> bestPolyRight;

        std::pair<NewtonPolynomial, double> bestPolyHorizontal;


        /**
         * Lists containing the lane marking points selected for detecting the lane
         * polynomials during RANSAC
         */
        std::vector<FuPoint<int>> laneMarkingsLeft;
        std::vector<FuPoint<int>> laneMarkingsCenter;
        std::vector<FuPoint<int>> laneMarkingsRight;

        std::vector<FuPoint<int>> laneMarkingsHorizontal;

        /**
         * Newton interpolation data points selected for the best polynomial
         */
        std::vector<FuPoint<int>> pointsLeft;
        std::vector<FuPoint<int>> pointsCenter;
        std::vector<FuPoint<int>> pointsRight;
        std::vector<FuPoint<int>> pointsHorizontal;

        /**
         * Vectors containing the supporters of the best polynomial
         */
        std::vector<FuPoint<int>> supportersLeft;
        std::vector<FuPoint<int>> supportersCenter;
        std::vector<FuPoint<int>> supportersRight;
        std::vector<FuPoint<int>> supportersHorizontal;

        /**
         * The polynomials detected on the previous picture
         */
        NewtonPolynomial prevPolyLeft;
        NewtonPolynomial prevPolyCenter;
        NewtonPolynomial prevPolyRight;
        NewtonPolynomial prevPolyHorizontal;

        /**
         * The polynomial representing the center of the right lane
         */
        NewtonPolynomial lanePoly;

        LanePolynomial lanePolynomial;

        int laneMarkingSquaredThreshold;

        int angleAdjacentLeg;

        int detectLaneStartX;

        double lastAngle;

        int maxAngleDiff;

        nav_msgs::GridCells array_left;
        nav_msgs::GridCells array_center;
        nav_msgs::GridCells array_right;

        nav_msgs::GridCells array_horizontal;

        nav_msgs::GridCells path_planned;

        nav_msgs::GridCells array_ransac_left;
        nav_msgs::GridCells array_ransac_center;
        nav_msgs::GridCells array_ransac_right;

        nav_msgs::GridCells array_lane_model;
        nav_msgs::GridCells array_ransac_horizontal;

        double rate_hz = 30;

    public:
                
    	cLaneDetectionFu(ros::NodeHandle nh);

    	virtual ~cLaneDetectionFu();
        
        void ProcessInput(const sensor_msgs::Image::ConstPtr& msg);
        
        void pubRGBImageMsg(cv::Mat& rgb_mat, image_transport::CameraPublisher publisher);

        void pubAngle();

        void pubGradientAngle();

        std::vector<std::vector<LineSegment<int>> > getScanlines();

        std::vector<std::vector<EdgePoint> > scanImage(cv::Mat image, ePosition position);

        std::vector<FuPoint<int>> extractLaneMarkings(const std::vector<std::vector<EdgePoint>>& edges);

        std::vector<FuPoint<int>> extractLaneMarkingsHorizontal(const std::vector<std::vector<EdgePoint>>& edges);

        int buildLaneMarkingsLists(const dbscan::point_t *points, const int num_points, const unsigned int num_clusters );

        void buildLaneMarkingsListsHorizontal(const std::vector<FuPoint<int>> &laneMarkings);

        int horizDistanceToDefaultLine(ePosition &line, FuPoint<int> &p);

        int vertDistanceToDefaultLine(ePosition &line, FuPoint<int> &p);

        int horizDistanceToPolynomial(NewtonPolynomial& poly, FuPoint<int> &p);

        int vertDistanceToPolynomial(NewtonPolynomial& poly, FuPoint<int> &p);

        bool isInDefaultRoi(ePosition position, FuPoint<int> &p);

        bool isInPolyRoi(NewtonPolynomial &poly, FuPoint<int> &p);

        void ransac();

        bool ransacInternal(ePosition position,
                std::vector<FuPoint<int>>& laneMarkings,
                std::pair<NewtonPolynomial, double>& bestPoly, NewtonPolynomial& poly,
                std::vector<FuPoint<int>>& supporters, NewtonPolynomial& prevPoly,
                std::vector<FuPoint<int>>& points);

        bool polyValid(ePosition, NewtonPolynomial, NewtonPolynomial);

        int horizDistance(FuPoint<int> &p1, FuPoint<int> &p2);

        double distanceBetweenPoints(FuPoint<int> &p1, FuPoint<int> &p2);

        void createLanePoly(ePosition position);

        void createLanePolyHorizontal(ePosition position);

        void detectLane();

        double gradient(double, std::vector<FuPoint<int>>&,
                std::vector<double>);

        double intersection(FuPoint<double>&, double&, std::vector<FuPoint<int>>&,
                std::vector<double>&);

        double nextGradient(double, NewtonPolynomial&,
                std::vector<FuPoint<int>>&, std::vector<FuPoint<int>>&,
                std::vector<double>, std::vector<double>, double);

        bool gradientsSimilar(double&, double&);

        ePosition maxProportion();

        void config_callback(line_detection_fu::LaneDetectionConfig &config, uint32_t level);

        void get_localization(const std_msgs::Float32MultiArray& locArray);

        void get_ctrl_action(const geometry_msgs::Twist& val);

        void get_ctrl_desired_state(const std_msgs::Float64& val);

        bool ackerman_control_next_points(double y_next_dist, cv::Point& pt_car, cv::Point& y_next_pt, 
            cv::Point& y_next_pt2);

        void ackerman_control(cv::Mat& imagePaint, NewtonPolynomial& polyLeft, NewtonPolynomial& polyCenter,
    NewtonPolynomial& polyRight);
};

#endif 
