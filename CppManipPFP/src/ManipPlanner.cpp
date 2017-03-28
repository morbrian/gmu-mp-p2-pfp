#include "ManipPlanner.hpp"

#include <iostream>
using std::cout;

#include <vector>
using std::vector;

#include <cmath>

//
// point_distance
// Calculate distance between 2 points (x1, y1) and (x2, y2)
// @return distance between points
//
double point_distance(const double x1, const double y1, const double x2, const double y2);

// for debugging
void print_matrix(const vector< vector<Point> >& m);

//
// Modify the track to reflect modified robot position.
//
void update_tracker(ManipSimulator& m_manipSimulator, int link_id, MotionTrack& track);

//
// near_target
// Calculate whether point p1 is with in a radius r of p2
// @return true if p1 is near p2
//
bool near_target(double x1, double y1, double x2, double y2, double r);

//
// Create and return the jacobian over all links of the simulator.
// @return jacobian mantrix
//
vector< vector<Point> > calc_jacob(ManipSimulator& m_manipSimulator);

//
// Calculate the attractive force for a link given the provided jacobian.
// @param m_manipSimulator simulator provides goal and link FK values
// @param goal attraction point
// @param jacob_j jacobian for link j
// @param j link number in the chain
//
vector<double> calc_att(ManipSimulator& m_manipSimulator, const Point& goal, const vector<Point>& jacob_j, int j);

//
// Calculate the repulsive force for a link given the provided jacobian.
// @param m_manipSimulator simulator provides goal and link FK values
// @param obs repulsion point
// @param jacob_j jacobian for link j
// @param j link number in the chain
//
vector<double> calc_rep(ManipSimulator& m_manipSimulator, const Point& obs, const vector<Point>& jacob_j, int j);

ManipPlanner::ManipPlanner(ManipSimulator * const manipSimulator)
{
    m_manipSimulator = manipSimulator;
    
    att_link = -1;
}

ManipPlanner::~ManipPlanner(void)
{
    //do not delete m_simulator  
}

void ManipPlanner::ConfigurationMove(double allLinksDeltaTheta[])
{
	if (m_manipSimulator->HasRobotReachedGoal())
		return;
    
    int n_links = m_manipSimulator->GetNrLinks();
    
    // if somebody moved the true goal, then reset counters
    if (true_goal.m_x != m_manipSimulator->GetGoalCenterX()
        || true_goal.m_y != m_manipSimulator->GetGoalCenterY())
    {
        true_goal.m_x = m_manipSimulator->GetGoalCenterX();
        true_goal.m_y = m_manipSimulator->GetGoalCenterY();
        track.stuck_meter = 0;
        track.marker.m_x = m_manipSimulator->GetLinkEndX(n_links - 1);
        track.marker.m_y = m_manipSimulator->GetLinkEndY(n_links - 1);
        att_link = -1;
        cout << "Control reset to end link\n";
    }
    
    if (att_link == -1)
        att_link = n_links - 1;
    
    update_tracker(*m_manipSimulator, att_link, track);
    
    // if we are stuck now, or if we were stuck and the alternate link
    // has reached the goal, then iterate control to the next link
    if (track.stuck_meter > STUCK_THRESHHOLD
        || (att_link != (n_links - 1) && 
                near_target(m_manipSimulator->GetLinkEndX(att_link),
                       m_manipSimulator->GetLinkEndY(att_link),
                       m_manipSimulator->GetGoalCenterX(),
                       m_manipSimulator->GetGoalCenterY(),
                       TARGET_RADIUS)))
    {
        if (att_link > 0)
            att_link--;
        else 
            att_link = n_links -1;
        
        cout << "Stuck, give control to link: " << att_link << "\n";
    }
 
    for (int i = 0; i < n_links; ++i)
        allLinksDeltaTheta[i] = 0.0;
    
    Point goal = 
    {
        m_manipSimulator->GetGoalCenterX(),
        m_manipSimulator->GetGoalCenterY()
    };
    
    vector< vector<Point> > jacob = calc_jacob(*m_manipSimulator);
    
    // put attractive forces in allLinksDeltaTheta,
    // using jacobian for every link
    for (int j = att_link; j < att_link + 1; ++j)
    {
        vector<double> u_att_j = calc_att(*m_manipSimulator, goal, jacob.at(j), j);
        vector<double>::iterator att_k = u_att_j.begin();
        vector<double>::iterator att_end = u_att_j.end();
        for (int k = 0; 
             k < n_links && att_k != att_end; 
             ++k, ++att_k)
        {
            allLinksDeltaTheta[k] += *att_k;
        }
    }

    // put repulsive forces in allLinksDeltaTheta,
    // using jacobian for every link
    int n_obs = m_manipSimulator->GetNrObstacles();
	int check = 0;
    for (int j = 0; j < n_links; ++j)
    {

        Point link_j =
        {
            m_manipSimulator->GetLinkEndX(j),
            m_manipSimulator->GetLinkEndY(j)
        };
        for (int i = 0; i < n_obs; ++i)
        {
            Point obs_i = m_manipSimulator->ClosestPointOnObstacle(i, link_j.m_x, link_j.m_y);
            if (point_distance(link_j.m_x, link_j.m_y, obs_i.m_x, obs_i.m_y) < SENSOR_RANGE)
            {
				if(point_distance(link_j.m_x, link_j.m_y, obs_i.m_x, obs_i.m_y) < 3)
					check = -1;

                vector<double> u_rep_j = calc_rep(*m_manipSimulator, obs_i, jacob.at(j), j);
                vector<double>::iterator rep_k = u_rep_j.begin();
                vector<double>::iterator rep_end = u_rep_j.end();
                for (int k = 0; 
                     k < n_links && rep_k != rep_end; 
                     ++k, ++rep_k)
                {
                    allLinksDeltaTheta[k] += *rep_k;
                }
            }
        }
    }
    
	if(check == 0)
		att_link = n_links-1;

    // populate allLinksDetlaTheta with u_att + u_rep and scale appropriately
    for (int i = 0; i < n_links; ++i)
        allLinksDeltaTheta[i] *= - SCALE_THETA;
    
}

// distance from starting point (x1,y1) to end point (x2, y2)
double point_distance(const double x1, const double y1, const double x2, const double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// for debugging
void print_matrix(const vector< vector<Point> >& m)
{
    cout << "[\n";
    for (int j = 0; j < m.size(); ++j)
    {
        vector<Point> row = m.at(j);
        cout << "\t[";
        for (int k = 0; k < row.size(); ++k)
        {
            cout << " (" << row.at(k).m_x << ", " << row.at(k).m_y << ") ";
        }
        cout << "]\n";
    }
    cout << "]\n";
}

// Modify the track to reflect modified robot position.
void update_tracker(ManipSimulator& m_manipSimulator, int link_id, MotionTrack& track)
{
    double link_x = m_manipSimulator.GetLinkEndX(link_id);
    double link_y = m_manipSimulator.GetLinkEndY(link_id);
    
    double d = point_distance(link_x, link_y, track.marker.m_x, track.marker.m_y);
    
    if (d <= STUCK_BOUNDS)
        track.stuck_meter++;
    else 
    {
        track.stuck_meter = 0;
        track.marker.m_x = link_x;
        track.marker.m_y = link_y;
    }
}

// return true if p1 is with in r distance of p2
bool near_target(double x1, double y1, double x2, double y2, double r)
{	
	return point_distance(x1, y1, x2, y2) <= r;
}

// Create and return the jacobian over all links of the simulator.
vector< vector<Point> > calc_jacob(ManipSimulator& m_manipSimulator)
{
    int n_links = m_manipSimulator.GetNrLinks();
    vector< vector<Point> > jacob;
    Point p;
    for (int j = 0; j < n_links; ++j)
    {
        vector<Point> row;
        for (int i = 0; i < n_links; ++i)
        {
            if (i <= j)
            {
                p.m_x = - m_manipSimulator.GetLinkEndY(j) + m_manipSimulator.GetLinkStartY(i);
                p.m_y = m_manipSimulator.GetLinkEndX(j) - m_manipSimulator.GetLinkStartX(i);
            }
            else 
            {
                // fill out the rest of the row with (0, 0) after i > j
                p.m_y = 0.0;
                p.m_x = 0.0;
            }
            row.push_back(p);
        }
        jacob.push_back(row);
    }
    
    return jacob;
}

// Calculate the attractive force for a link given the provided jacobian.
vector<double> calc_att(ManipSimulator& m_manipSimulator, const Point& goal, const vector<Point>& jacob_j, int j)
{
    Point u_att =
    {
        m_manipSimulator.GetLinkEndX(j) - goal.m_x,
        m_manipSimulator.GetLinkEndY(j) - goal.m_y
    };
    // normalize the attractive
    double u_att_mag = point_distance(0.0, 0.0, u_att.m_x, u_att.m_y);
    u_att.m_x /= u_att_mag;
    u_att.m_y /= u_att_mag;

    vector<double> dtheta;
    for (int i = 0; i <= j; ++i)
    {
        dtheta.push_back(SCALE_ATT * (jacob_j[i].m_x * u_att.m_x + jacob_j[i].m_y * u_att.m_y));
    }
    return dtheta;
}

// Calculate the repulsive force for a link given the provided jacobian.
vector<double> calc_rep(ManipSimulator& m_manipSimulator, const Point& obs, const vector<Point>& jacob_j, int j)
{
    Point u_rep =
    {
        obs.m_x - m_manipSimulator.GetLinkEndX(j),
        obs.m_y - m_manipSimulator.GetLinkEndY(j)
    };
    // normalize the attractive
    double u_rep_mag = point_distance(0.0, 0.0, u_rep.m_x, u_rep.m_y);
    u_rep.m_x /= u_rep_mag;
    u_rep.m_y /= u_rep_mag;
    
    // put attractive forces in allLinksDeltaTheta,
    // using last link's jacobian only
    vector<double> dtheta;
    for (int i = 0; i <= j; ++i)
    {
        dtheta.push_back(SCALE_REP/ (u_rep_mag * u_rep_mag * u_rep_mag * u_rep_mag) * (jacob_j[i].m_x * u_rep.m_x + jacob_j[i].m_y * u_rep.m_y));
    }
    return dtheta;
}






