/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for simulation of robotic exploration of unknown
 *	terrain. On each update, the robot is provided with traversability 
 * 	data of the area in its trapezoidal field of view. The ground truth
 * 	to be given to the robot are loaded from a .txt. file given by 
 *	the filename in config. 
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Oct 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include "Task.hpp"

#include <stdlib.h>
#include <time.h>

#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
#include <orocos/envire/Orocos.hpp>


using namespace traversability_explorer;

Task::Task(std::string const& name)
    :   TaskBase(name),
        mpEnv(NULL), 
        mpFrameNode(NULL), 
        mpTravGrid(NULL), 
        mCurrentPose()
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    :   TaskBase(name, engine), 
        mpFrameNode(NULL), 
        mpTravGrid(NULL), 
        mCurrentPose()
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    a = _robot_fov_a.get()/2/_traversability_map_scalex.get();
    b = _robot_fov_b.get()/2/_traversability_map_scalex.get();
    l = _robot_fov_l.get()/_traversability_map_scalex.get();
    loadTraversabilityMap(_filename.get());
    /* TEST 
    mCurrentPose.cov_position = Eigen::Matrix3d::Identity() * 0.001;
    mCurrentPose.orientation.setIdentity();
    mCurrentPose.orientation
      = Eigen::Quaterniond( Eigen::AngleAxisd(0.0/180.0*M_PI, Eigen::Vector3d::UnitZ()));
    mCurrentPose.position = Eigen::Vector3d(3,0.1,0);   
    /* END TEST */
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook(); // /*
    base::Time t0 = base::Time::now();
    int rttPortState;
    base::samples::RigidBodyState newPose;
    rttPortState = _robot_pose.readNewest(newPose);
    if( rttPortState == RTT::NewData &&
        rttPortState != RTT::NoData ) 
    {
        mCurrentPose = newPose;
        updateTraversabilityMap();
        std::cout << "TraversabilityExplorer::updateHook(): new pose, updated trav. map" << std::endl;
    }
    base::Time t1 = base::Time::now();
    // */
    envire::OrocosEmitter emitter_tmp(mpEnv, _traversability_map);
    emitter_tmp.setTime(base::Time::now());
    emitter_tmp.flush();    
    base::Time t2 = base::Time::now();
    std::cout << "Time: Trav. map updated in "        << (t1-t0).toMilliseconds() << "ms, "
              << "trav. map written out in "    << (t2-t1).toMilliseconds() << "ms." << std::endl;
    //    */
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

/* --------------------------------------------------------------
    PRIVATE METHODS
---------------------------------------------------------------*/

void Task::loadTraversabilityMap(std::string filename){
    std::cout<< "TraversabilityExplorer: map being loaded from: "
        << filename << ", correct?" << std::endl;
   
    /* OPEN THE FILE AND LOAD THE TRAVERSABILITY NUMERIC DATA INTO STD VECTOR */
    std::string line;
    std::ifstream datafile(filename.c_str(), std::ios::in);
    Nrow = 0; Ncol = 0;    
    std::vector <int> row;

    int MIN_COST, MAX_COST;
    MIN_COST = std::numeric_limits<int>::max();
    MAX_COST = 0;

    if( datafile.is_open() ){
        while ( std::getline(datafile, line) ){
            std::stringstream ss(line);
            std::string cell;
           while( std::getline(ss, cell, ' ') ){ 
                int val;
                std::stringstream numericValue(cell);
                numericValue >> val;
                if(val>MAX_COST){
                    MAX_COST = val;
                }
                if(val<MIN_COST){
                    MIN_COST = val;
                }
                row.push_back(val);
                Ncol++;
            }
            groundTruth.push_back(row);
            row.clear();
            Nrow++;
        }
        datafile.close(); 
        Ncol /= Nrow;
        std::cout << "Cost map of " << Nrow 
                  << " x "          << Ncol << " loaded." << std::endl;
        std::cout << "Cost values are in integer values of {" 
                  << MIN_COST << ", ..., " << MAX_COST << "}."<< std::endl;
        bool validCostRange = MIN_COST>0 && MAX_COST <= SBPL_MAX_COST+1;
        if (!validCostRange){
            std::cout << "Invalid cost range!!!" << std::endl;
            assert(validCostRange);    
        }
        
    } else {
        std::cout << "Problem opening the file" << std::endl;
        return;
    }
    /* INITIALIZE UNEXPLORED TRAVERSABILITY MAP */
    if(mpEnv != NULL) {
        delete mpEnv;
    }
    mpEnv = new envire::Environment(); 
    envire::TraversabilityGrid* trav = new envire::TraversabilityGrid(
            (size_t) Nrow, (size_t) Ncol,
            _traversability_map_scalex.get(),
            _traversability_map_scaley.get());
    mpEnv->attachItem(trav);
    mpTravGrid = trav;
    // Creates a shared-pointer from the passed reference.
    envire::TraversabilityGrid::ArrayType& travData = mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
    boost::shared_ptr<envire::TraversabilityGrid::ArrayType> mpTravData;
    mpTravData = boost::shared_ptr<envire::TraversabilityGrid::ArrayType>(&travData, NullDeleter());

    // Defines driveability values.

    // Defines driveability values.
    // trav->setTraversabilityClass(0, envire::TraversabilityClass(0.5)); // unknown
    

    
    trav->setTraversabilityClass(0, envire::TraversabilityClass(0.5));
    std::cout << "Setting class #0 to 0.5 traversability."<< std::endl;
    
    
    
    for (int i = 0; i < SBPL_MAX_COST; ++i)
    {
        sbplCostToClassID[i] = i+2;
        //std::cout << "Mapping " << i+1 << " to " << i+2 << std::endl;   
    }
    sbplCostToClassID[SBPL_MAX_COST] = 1;
    //std::cout << "Mapping " << SBPL_MAX_COST+1 << " to " << 1 << std::endl;   

    // ASSOCIATING CLASS ID WITH TRAVERSABILITY
    

    double travVal;
    // class 1 reserved for obstacle (will be used instead of 21)
    trav->setTraversabilityClass(1, envire::TraversabilityClass(0.0));
    std::cout << "Setting class #" << 1
            <<  " to " << 0.0 << " traversability." << std::endl;

    int cost;
    // TODO: MAY SKIP CLASSES BELOW MIN_COST IN ORDER NOT TO BIAS UNKNOWN AREAS
    // those are set to 2nd highest traversability
    // but 2nd highest registered class, or 2nd highest value in the grid??
    for(cost = 1; cost < SBPL_MAX_COST+1; ++cost) {
        travVal = ((double)(SBPL_MAX_COST+1 - cost));
        travVal /= SBPL_MAX_COST;
        trav->setTraversabilityClass(sbplCostToClassID[cost-1], envire::TraversabilityClass(travVal));
        std::cout << "Setting class #" << sbplCostToClassID[cost-1]
            <<  " to " << travVal << " traversability." << std::endl;
    }
 
    for(int x=0; x < Nrow;  x++)
    {
        //row = groundTruth.at(x);
        for(int y=0; y < Ncol; y++)
        {
            // Set all to unknown
            trav->setTraversability(0, x, y);   
            trav->setProbability(1.0, x, y);
        }
    }
    std::cout << "Traversability map initialization completed." << std::endl;
}


void Task::updateTraversabilityMap()
{
    envire::TraversabilityGrid::ArrayType& travData = 
        mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);

    /* Create the Field of view trapezoid */
    int side, numSides; 
    numSides = 4;

    // Robot position in the 2D grid-scaled coordinates
    Eigen::Vector2f xr;
    xr = Eigen::Vector2f( mCurrentPose.position(0), mCurrentPose.position(1));
    xr /= _traversability_map_scalex.get();
    Eigen::Matrix2f Rot2D;
    Rot2D << cos(mCurrentPose.getYaw()), -sin(mCurrentPose.getYaw()), 
             sin(mCurrentPose.getYaw()),  cos(mCurrentPose.getYaw());

    /* Define the sides of the trapezoid in the robot coordinate frame
       and transform them into the world reference frame grid           */
	std::vector<Eigen::Vector2f> sides(numSides);
    sides.at(0) = Rot2D*Eigen::Vector2f(0, a) +xr;
    sides.at(1) = Rot2D*Eigen::Vector2f(l, b) +xr;
    sides.at(2) = Rot2D*Eigen::Vector2f(l,-b) +xr;
    sides.at(3) = Rot2D*Eigen::Vector2f(0,-a) +xr;

    /* Get the bounding box of the trapezoid */
    int xm, xM, ym, yM;    
    xm = Nrow; xM = 0; ym = Ncol; yM = 0;
    for(side = 0; side < numSides; side++){
        xm = xm > sides.at(side).x() ? sides.at(side).x() : xm;
        xM = xM < sides.at(side).x() ? sides.at(side).x() : xM;
        ym = ym > sides.at(side).y() ? sides.at(side).y() : ym;
        yM = yM < sides.at(side).y() ? sides.at(side).y() : yM;
        // std::cout << "(" << sides.at(side).x() <<", "<< sides.at(side).y() << ")";
    }
    
    /* Crop the bounding box to the overlap with the map */
    xm = xm < 0 ? 0 : xm;
    ym = ym < 0 ? 0 : ym;
    xM = xM > Nrow-1 ? Nrow-1 : xM;
    yM = yM > Ncol-1 ? Ncol-1 : yM;

    /* DEBUG OUTPUT /
    std::cout << "Bounding box: x in " <<
                 xm << " to " <<
                 xM << ", y in " <<
                 ym << " to " <<
                 yM << ". " << std::endl;
    //    */
    /* -----------------------------------------------------------------*/

    /* Ray Casting Algorithm to determine the trapezoid inliers*/
    float epsilon = (xM-xm)/100.0; // Padding
    std::vector<Eigen::Vector3f> lineCoeff(4);

    /* For each side ot the trapezoid determine the equation
       of line in the form of Ax + By + C = 0 */
    for (side = 0; side < numSides; side++){
       getLineCoeff(sides.at(side), sides.at( (side+1)%numSides), lineCoeff.at(side));
       /* Print Line Coefficents                      /
       std::cout << "a = " << lineCoeff.at(side).x() <<
                  ", b = " << lineCoeff.at(side).y() <<
                  ", c = " << lineCoeff.at(side).z() << std::endl;
       // */
    } 
    
    /* For points in the bounding box, test whether are they in the trapezoid */   
    Eigen::Vector2f origin, testedPoint;
    int y1, y2, intersections;
    for(int itX = xm; itX <= xM; itX++){
        y1 = yM; y2 = ym;

        /* Find lower end of trapezoid */
        for(int itY = ym; itY <= yM; itY++){
            intersections = 0;
            testedPoint << itX,   itY;
            origin << ((float)xm)-epsilon, ((float)itY)-epsilon;
            for (side = 0; side < numSides; side++){
                if ( areIntersecting(origin, testedPoint,
                     lineCoeff.at(side), sides.at(side), sides.at( (side+1) % numSides )))
                	intersections++;
            }
            // If the number of intersection with the ray is odd, its inside
            if( (intersections & 1) == 1){
                y1 = itY;
                break;
            }
        }

        /* Find upper end of the trapezoid */
        for(int itY = yM; itY > y1; itY--){
            intersections = 0;
            testedPoint << itX,   itY;
            origin << ((float)xm)-epsilon, ((float)itY)-epsilon;
            for (side = 0; side < numSides; side++){
                if ( areIntersecting(origin, testedPoint,
                     lineCoeff.at(side), sides.at(side), sides.at( (side+1) % numSides )))
                {
                    intersections++;
                }
            }
            if( (intersections & 1) == 1){
                y2 = itY;
                break;
            }
        }

        /* Fill the cells in FOV from the current data colum */
        int cost;
        for(int itY = ym; itY <= yM; itY++){
        	if ( itY >= y1 && itY <= y2) {
        		cost = groundTruth[itX][itY];
                mpTravGrid->setTraversability(sbplCostToClassID[cost-1], itX, itY);   
        	}
        }
    }
   	return;
}

void Task::getLineCoeff(Eigen::Vector2f& v1, Eigen::Vector2f& v2, Eigen::Vector3f& lineCoeff){
        lineCoeff(0) = v2(1) - v1(1);                  // y2 - y1
        lineCoeff(1) = v1(0) - v2(0);                  // x1 - x2
        lineCoeff(2) = v2(0) * v1(1) - v1(0) * v2(1);  // x2*y1 - x1*y2
        return;
}

bool Task::areIntersecting(Eigen::Vector2f& v1, Eigen::Vector2f& v2,
                     Eigen::Vector3f& lineCoeff, 
                     Eigen::Vector2f& vertex1, Eigen::Vector2f& vertex2)
{
    float d1, d2;
    Eigen::Vector3f point; // 2D helper point in homogenous coordinates

    /* Test the position of the Ray Origin and Tested point 
        wrt the side of the Polygon  */ 
    point << v1(0), v1(1), 1;
    d1 = lineCoeff.dot(point);
    point << v2(0), v2(1), 1;
    d2 = lineCoeff.dot(point);
    /*/ std::cerr << "Case 1: d1 = " << d1 << ", d2 = " << d2 << std::endl;   //*/

    /* Both are on the same side = no intersection */
    if (d1 > 0 && d2 > 0) return false;
    if (d1 < 0 && d2 < 0) return false;

    /* Coefficients of the ray line given by v1 --> v2 */
    Eigen::Vector3f rayLine;
    getLineCoeff(v1, v2, rayLine);

    /* Test the position of the Polygon Vertices wrt the ray */ 
    point << vertex1(0), vertex1(1), 1;
    d1 = rayLine.dot(point);
    point << vertex2(0), vertex2(1), 1;
    d2 = rayLine.dot(point);
    /*/    std::cerr << "Case 2: d1 = " << d1 << ", d2 = " << d2 << std::endl;   //*/

    /* Both are on the same side = no intersection */
    if (d1 > 0 && d2 > 0) return false;
    if (d1 < 0 && d2 < 0) return false;

    /* Collinear case */
    if (lineCoeff(0)*rayLine(1)-lineCoeff(1)*rayLine(0) == 0.0f ){
        return true; // OK in this application
    } else {
    /* Intersecting at one point */
        return true;
    }
}









