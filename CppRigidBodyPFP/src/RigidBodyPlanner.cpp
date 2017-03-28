#include "RigidBodyPlanner.hpp"

#include <vector>
using std::vector;

#include <iostream>
using std::cout;

//
// point_distance
// Calculate distance between 2 points (x1, y1) and (x2, y2)
// @return distance between points
//
double point_distance(const double x1, const double y1, const double x2, const double y2);

//
// near_target
// Calculate whether point p1 is with in a radius r of p2
// @return true if p1 is near p2
//
bool near_target(const Point& p1, const Point& p2, double r);

//
// Choose available way points, or choose the actual goal if no way points.
// @return point representing target area
//
Point find_goal(const RigidBodySimulator& m_simulator, vector<Point>& way_points);

//
// Modify the track to reflect modified robot position.
//
void update_tracker(const RobotConfig& q, MotionTrack& track);

//
// Add one or more way_points, depending on the detected problem.
//
void redirect(RigidBodySimulator& m_simulator, 
              MotionTrack& track, vector<Point>& way_points);

//
// Calculate the transpose of the matrix and return it.
// @param matrix matrix of arbitrary size
// @return transpose of matrix
//
vector< vector<double> > transpose(const vector< vector<double> >& matrix);

//
// Calculate the Forward Kinematics for control point r_j at configuration q.
// @param q robot configuration (x,y,theta)
// @param r_j the jth control point on the robot
// @return (x,y) positions of the control point after FK
//
Point calc_fk(const RobotConfig& q, const Point& r_j);

//
// Calculate the atractive potential of the point fk_j
// @param fk_j forward kinimatics of control point j
// @param goal workspace coordinates of the goal
// @return (x,y) calculated attractive potential
//
Point calc_att(const Point& fk_j, const Point& goal);

//
// Calculate the repulsive potential of the point fk_j
// @param fk_j forward kinimatics of control point j
// @param m_simulator provides access to obstacle points closest to fk_j
// @return (x,y) calculated attractive potential
//
Point calc_rep(const Point& fk_j, RigidBodySimulator& m_simulator);

//
// Calculate the Jacobian for the control point j
// @param q robot configuration
// @param r_j localized point j on robot
// @return 2x3 matrix
//
vector< vector<double> > calc_jacob(const RobotConfig& q, const Point& r_j);
void print_jacob(const vector< vector< vector<double> > >& jacob_q);
void print_matrix(const vector< vector<double> >& m);

//
// Compute the sumation of Jacobian * Point potentials
// (can be used for att or rep, assumes the rep points were summed elsewhere)
// @param jacob_q the Jacobian over all control points
// @param u_pot the potentials
//
RobotConfig sum_pot(const vector< vector< vector<double> > >& jacob_q, const vector<Point>& u_pot);

RigidBodyPlanner::RigidBodyPlanner(RigidBodySimulator * const simulator)
{
    m_simulator = simulator;
}

RigidBodyPlanner::~RigidBodyPlanner(void)
{
    //do not delete m_simulator  
}

RigidBodyMove RigidBodyPlanner::ConfigurationMove(void)
{
    RigidBodyMove move;
    
    // Current Robot Configuration
    RobotConfig q =
    {
        m_simulator->GetRobotX(),
        m_simulator->GetRobotY(),
        m_simulator->GetRobotTheta()
    };
    
    // if somebody moved the true goal, then empty way points
    if (true_goal.m_x != m_simulator->GetGoalCenterX()
        || true_goal.m_y != m_simulator->GetGoalCenterY())
    {
        true_goal.m_x = m_simulator->GetGoalCenterX();
        true_goal.m_y = m_simulator->GetGoalCenterY();
        way_points.clear();
        track.stuck_meter = 0;
        track.marker.m_x = q.m_x;
        track.marker.m_y = q.m_y;
    }

    update_tracker(q, track);
    if (track.stuck_meter > STUCK_THRESHHOLD)
    {
        redirect(*m_simulator, track, way_points);
        cout << "Way Points: " << way_points.size() << "\n";
    }
    
    Point goal = find_goal(*m_simulator, way_points);
        
    // list of control points on robot
    const double* v_list = m_simulator->GetRobotVertices();
    int n_vert = m_simulator->GetNrRobotVertices();

    //
    // Calculate global coordinates of each vertex
    //
    vector<Point> g_cp_list;
    for (int j = 0; j < n_vert; ++j)
    {
        Point cp_j = 
        {
            v_list[2 * j],
            v_list[2 * j + 1]
        };
        g_cp_list.push_back(cp_j);
    }
    
    //
    // Calculate local coordinates of each vertex
    //
    vector<Point> l_cp_list;
    for (int j = 0; j < n_vert; ++j)
    {
        Point cp_j = 
        {
            v_list[2 * j] - q.m_x,
            v_list[2 * j + 1] - q.m_y
        };
        l_cp_list.push_back(cp_j);
    }
    
    //
    // Calculate/Populate Forward Kinematics
    //
    /*
    vector<Point> fk_q;
    vector<Point>::iterator l_cp_j = l_cp_list.begin();
    vector<Point>::iterator l_cp_end = l_cp_list.end();
    for ( ; l_cp_j != l_cp_end; ++l_cp_j)        
        fk_q.push_back(calc_fk(q, *l_cp_j));
     */

    // NOTE: THIS IS NOT APPLIED FK.. FK IS NOW SAME AS GLOBAL COORDS
    vector<Point> fk_q;
    vector<Point>::iterator g_cp_j = g_cp_list.begin();
    vector<Point>::iterator g_cp_end = g_cp_list.end();
    for ( ; g_cp_j != g_cp_end; ++g_cp_j)        
        fk_q.push_back(*g_cp_j);

    //
    // Calculate/Populate Attractive Potential
    //
    vector<Point> u_att;
    vector<Point>::iterator fk_j = fk_q.begin();
    vector<Point>::iterator fk_end = fk_q.end();
    for ( ; fk_j != fk_end; ++fk_j)
        u_att.push_back(calc_att(*fk_j, goal));
    
    //
    // Calculate/Populate Repulsive Potential
    //
    vector<Point> u_rep;
    fk_j = fk_q.begin();
    fk_end = fk_q.end();
    for ( ; fk_j != fk_end; ++fk_j)
        u_rep.push_back(calc_rep(*fk_j, *m_simulator));

    //
    // Compute Jacobian
    //
    vector< vector< vector<double> > > jacob;
    g_cp_j = g_cp_list.begin();
    g_cp_end = g_cp_list.end();
    for ( ; g_cp_j != g_cp_end; ++g_cp_j)
        jacob.push_back(calc_jacob(q, *g_cp_j));
    
    //print_jacob(jacob);
    
    //
    // Compute Overall Gradient in Configuration Space
    // Sum att first, then rep separately
    //
    RobotConfig ucs_att = sum_pot(jacob, u_att);
    RobotConfig ucs_rep = sum_pot(jacob, u_rep);
    
    //
    // Use the attractive / repulsive forces to get delta
    //
    RobotConfig ucs_delta;
    ucs_delta.m_x = ucs_att.m_x + ucs_rep.m_x;
    ucs_delta.m_y = ucs_att.m_y + ucs_rep.m_y;
    ucs_delta.m_theta = ucs_att.m_theta + ucs_rep.m_theta;

    // next, the step should be an approx constant distance each time,
    // so scale (x,y) such that the distance moved is similar each time.
    // try for step = 0.025
    double unscaled = sqrt(ucs_delta.m_x * ucs_delta.m_x + ucs_delta.m_y * ucs_delta.m_y);
    double ratio = SCALE_STEP / unscaled;
    move.m_dx = - ucs_delta.m_x * ratio;
    move.m_dy = - ucs_delta.m_y * ratio;
    move.m_dtheta = - ucs_delta.m_theta / (0.0000000001 +fabs(ucs_delta.m_theta)) * THETA_TIC;

    return move;
}
                              
// distance from starting point (x1,y1) to end point (x2, y2)
double point_distance(const double x1, const double y1, const double x2, const double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// return true if p1 is with in r distance of p2
bool near_target(const Point& p1, const Point& p2, double r)
{	
	return point_distance(p1.m_x, p1.m_y, p2.m_x, p2.m_y) <= r;
}

// Choose available way points, or choose the actual goal if no way points.
Point find_goal(const RigidBodySimulator& m_simulator, vector<Point>& way_points)
{
    Point goal;
    
    // goal is a way point if present, otherwise it is the real goal
    if (way_points.empty())
    {
        goal.m_x = m_simulator.GetGoalCenterX();
        goal.m_y = m_simulator.GetGoalCenterY();
    }
    else
    {
        Point r_center = 
        { 
            m_simulator.GetRobotX(), 
            m_simulator.GetRobotY() 
        };
        
        goal = way_points.back();
        
        if (near_target(r_center, goal, WAY_POINT_RADIUS))
        {
            way_points.pop_back();
            if (way_points.empty())
            {
                goal.m_x = m_simulator.GetGoalCenterX();
                goal.m_y = m_simulator.GetGoalCenterY();
            }
            else
                goal = way_points.back();
        }
    }
    
    return goal;
}

// Modify the track to reflect modified robot position.
void update_tracker(const RobotConfig& q, MotionTrack& track)
{
    double d = point_distance(q.m_x, q.m_y, track.marker.m_x, track.marker.m_y);
    
    if (d <= STUCK_BOUNDS)
        track.stuck_meter++;
    else 
    {
        track.stuck_meter = 0;
        track.marker.m_x = q.m_x;
        track.marker.m_y = q.m_y;
    }
}

//
// Add one or more way_points, depending on the detected problem.
//
void redirect(RigidBodySimulator& m_simulator, 
              MotionTrack& track, vector<Point>& way_points)
{
    // we are going to add a new way point, so reset the stuck meter
    track.stuck_meter = 0;
    
    // try to find another goal near the goal.
    // just keep placing way_points on a curvy line from the previous
    // way point until we finally can reach it?
    // robot is "center point", and goal is on edge
    
    Point r_c =
    {
        m_simulator.GetRobotX(),
        m_simulator.GetRobotY()
    };
    
    Point g_c = find_goal(m_simulator, way_points);
    
    double dist = point_distance(r_c.m_x, r_c.m_y, g_c.m_x, g_c.m_y);
    double theta = atan2(g_c.m_y - r_c.m_y, g_c.m_x - r_c.m_x);
    if (theta < 0.0)
        theta += 2.0 * M_PI;
    
    cout << "theta: " << theta - 0.2 << "\n";
    // I am always going to choose new goals in a clock wise order
    // otherwise way points could just bunch up in the same location
    Point wp =
    {
        dist * cos(theta - 0.2) + r_c.m_x,
        dist * sin(theta - 0.2) + r_c.m_y
    };
    
    way_points.push_back(wp);
}

// Calculate the transpose of the matrix and return it.
vector< vector<double> > transpose(const vector< vector<double> >& m1)
{
    vector< vector<double> > m2;
    if (m1.size() == 0)
        return m2;
    
    vector<double> top_row = m1.at(0);
    int j = 0;
    do 
    {        
        vector<double> m2_row;
        for (int k = 0; k < m1.size(); ++k)
        {
            vector<double> m1_row = m1.at(k);
            m2_row.push_back(m1_row.at(j));
        }
        m2.push_back(m2_row);
        ++j;
    } while(j < top_row.size());
    
    return m2;
}

// forward kinematics for control point r_j
Point calc_fk(const RobotConfig& q, const Point& r_j)
{
    Point fk_j = 
    {
        r_j.m_x * cos(q.m_theta) - r_j.m_y * sin(q.m_theta) + q.m_x,
        r_j.m_x * sin(q.m_theta) + r_j.m_y * cos(q.m_theta) + q.m_y
    };
    
    return fk_j;
}


// Calculate the atractive potential of the point fk_j
Point calc_att(const Point& fk_j, const Point& goal)
{
    Point att_j =
    {
        SCALE_ATT * (fk_j.m_x - goal.m_x),
        SCALE_ATT * (fk_j.m_y - goal.m_y)
    };
    
    return att_j;
}

// Calculate the repulsive potential of the point fk_j
Point calc_rep(const Point& fk_j, RigidBodySimulator& m_simulator)
{
    Point rep_j =
    {
        0.0, 0.0
    };
    
    Point obs_i;
    double obs_dist;
    int n_obs = m_simulator.GetNrObstacles();

    for (int i = 0; i < n_obs; ++i)
    {
        obs_i = m_simulator.ClosestPointOnObstacle(i, fk_j.m_x, fk_j.m_y);
        obs_dist = point_distance(fk_j.m_x, fk_j.m_y, obs_i.m_x, obs_i.m_y);
        if (obs_dist <= SENSOR_RANGE)
        {            
            rep_j.m_x += SCALE_REP * (obs_i.m_x - fk_j.m_x);
            rep_j.m_y += SCALE_REP * (obs_i.m_y - fk_j.m_y); 
        }
    }
    return rep_j;
}

// Calculate the Jacobian for the control point j
vector< vector<double> > calc_jacob(const RobotConfig& q, const Point& r_j)
{
    vector< vector<double> > jacob;
    vector<double> row1;
    // build row 1
    row1.push_back( 1.0 ); 
    row1.push_back( 0.0 ); 
    row1.push_back( - r_j.m_x * sin(q.m_theta) - r_j.m_y * cos(q.m_theta) );
    jacob.push_back(row1);
    
    vector<double> row2;
    // build row 2
    row2.push_back( 0.0 ); 
    row2.push_back( 1.0 ); 
    row2.push_back( r_j.m_x * cos(q.m_theta) - r_j.m_y * sin(q.m_theta) );
    jacob.push_back(row2);

    return jacob;
}

// Compute the sumation of Jacobian * Potentials
RobotConfig sum_pot(const vector< vector< vector<double> > >& jacob_q, const vector<Point>& u_pot)
{
    RobotConfig rc = { 0.0, 0.0, 0.0 };
    
    Point pot_j;
    vector< vector<double> > j_t;
    for (int j = 0; j < jacob_q.size(); ++j)
    {
        j_t = transpose(jacob_q.at(j));
        
        pot_j = u_pot.at(j);
        // j_t is 3x2 matrix, multiply by 2x1 "Point"
        rc.m_x += j_t[0][0] * pot_j.m_x + j_t[0][1] * pot_j.m_y;
        rc.m_y += j_t[1][0] * pot_j.m_x + j_t[1][1] * pot_j.m_y;
        rc.m_theta += j_t[2][0] * pot_j.m_x + j_t[2][1] * pot_j.m_y;
    }
    
    return rc;
}

// for debugging
void print_jacob(const vector< vector< vector<double> > >& jacob_q)
{
    cout << "==== Jacobian Set Follows ====\n";
    for (int i = 0; i < jacob_q.size(); ++i)
    {
        cout << "=== " << i << " ===\n";
        print_matrix(jacob_q.at(i));
        print_matrix(transpose(jacob_q.at(i)));
    }
}

// for debugging
void print_matrix(const vector< vector<double> >& m)
{
    cout << "[\n";
    for (int j = 0; j < m.size(); ++j)
    {
        vector<double> row = m.at(j);
        cout << "\t[";
        for (int k = 0; k < row.size(); ++k)
        {
            cout << " " << row.at(k) << " ";
        }
        cout << "]\n";
    }
    cout << "]\n";
}

