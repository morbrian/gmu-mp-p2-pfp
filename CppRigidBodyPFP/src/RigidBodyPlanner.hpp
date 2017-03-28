#ifndef RIGID_BODY_PLANNER_HPP_
#define RIGID_BODY_PLANNER_HPP_

#include "RigidBodySimulator.hpp"

#include <vector>
using std::vector;

#include <set>
using std::set;

const double SCALE_ATT = .01;          // attraction scaling
const double SCALE_REP = 100.0;        // repulsion scaling
const double THETA_TIC = 0.02;         // angle response to to potential
const double SCALE_STEP = 0.025;       // distance to move per opportunity
const double SENSOR_RANGE = 1.0;       // threshhold dist of obstacle repulsion
const double WAY_POINT_RADIUS = 4.0;   // radius of way point
const double STUCK_BOUNDS = 1.0;       // bounding box size to decide stuck
const int STUCK_THRESHHOLD = 400;      // how many moves the robot can make in
                                       // the same bounding box before "stuck"

// record of local progress in any direction
struct MotionTrack
{
    int stuck_meter;
    Point marker;
};

// description of a robot configuration
struct RobotConfig
{
    double              m_x;
	double              m_y;
	double              m_theta;
};

struct RigidBodyMove
{
    double m_dx;
    double m_dy;
    double m_dtheta;
};

class RigidBodyPlanner
{
public:
    RigidBodyPlanner(RigidBodySimulator * const simulator);
            
    ~RigidBodyPlanner(void);

    /*
     * This is the function that you should implement.
     * This function needs to compute by how much the position (dx, dy) 
     * and orientation (dtheta) should change so that the robot makes a small 
     * move toward the goal while avoiding obstacles, 
     * as guided by the potential field.
     *
     * You have access to the simulator.
     * You can use the methods available in simulator to get all the information
     * you need to correctly implement this function
     *
     */
    RigidBodyMove ConfigurationMove(void);
    
    // TODO:
    // REMOVE BEFORE TURN IN -- DEBUGGING ONLY
    vector<Point>& GetWayPoints()
    {
        return way_points;
    }
    
protected:
    RigidBodySimulator *m_simulator;
    
    // local progress tracker, to detect when we are stuck in one area
    MotionTrack track;
    
    // location of actual goal, independent of way point stack
    Point true_goal;
    
    // way points are used as attraction potentials when robot seems stuck
    vector<Point> way_points;
};

#endif
