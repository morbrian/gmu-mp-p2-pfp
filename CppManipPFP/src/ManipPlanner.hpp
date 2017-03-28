#ifndef MANIP_PLANNER_HPP_
#define MANIP_PLANNER_HPP_

#include "ManipSimulator.hpp"

const double SCALE_ATT = 1.0;           // scale factor for attraction
const double SCALE_REP = 10.0;          // scale factor for repulsion
const double SCALE_THETA = 0.01;        // scaling gradient to small increment
const double SENSOR_RANGE = 10.0;       // distance when obstacles matter
const double TARGET_RADIUS = 2.0;       // pseudo goal radius
const double STUCK_BOUNDS = 1.0;        // bounding size to decide stuck
const int STUCK_THRESHHOLD = 400;       // how many moves the robot can make in
                                        // the same bounding box before "stuck"

// record of local progress in any direction
struct MotionTrack
{
    int stuck_meter;
    Point marker;
};

class ManipPlanner
{
public:
    ManipPlanner(ManipSimulator * const manipSimulator);
            
    ~ManipPlanner(void);

/*
 * This is the function that you should implement.
 * This function needs to compute by how much the link angles should change
 * so that the robot makes a small move toward the goal while avoiding
 * obstacles, as guided by the potential field.
 *
 * allLinksDeltaTheta(j) should contain the small delta change for the angle
 * associated with the j-th link.
 *
 * Note that the attractive potential should be defined only between the end
 * effector point on the manipulator and the goal center. 
 *
 * The repulsive potential on the other hand should be defined between each
 * obstacle and each link end.
 *
 * This will ensure that, when possible, the end effector moves toward the
 * goal while every link avoids collisions with obstacles.
 *
 * You have access to the simulator.
 * You can use the methods available in simulator to get all the information
 * you need to correctly implement this function
 *
 */
    void ConfigurationMove(double allLinksDeltaTheta[]);
    
protected:    
    ManipSimulator  *m_manipSimulator;

    // location of last real goal, so we know if we should reset counters
    Point true_goal;
    
    // local progress tracker, to detect when we are stuck in one area
    MotionTrack track;

    // the link id that wants to point toward goal
    int att_link;
};

#endif
