/**
 *@file Graphics.hpp
 *@author Erion Plaku 
 *@brief Graphics for running simulation and setting up problem
 */

#ifndef  GRAPHICS_HPP_
#define  GRAPHICS_HPP_

#include "ManipPlanner.hpp"
#include "ManipSimulator.hpp"
#include <vector>

class Graphics
{   
public:
    Graphics(const int nrLinks, const double linkLength);
    
    ~Graphics(void);

    void MainLoop(void);

protected:
    void HandleEventOnTimer(void);
    void HandleEventOnDisplay(void);
    void HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY);
    void HandleEventOnMouseMotion(const double mousePosX, const double mousePosY);
    void HandleEventOnKeyPress(const int key);

    void DrawCircle2D(const double cx, const double cy, const double r);

    static void CallbackEventOnDisplay(void);
    static void CallbackEventOnMouse(int button, int state, int x, int y);
    static void CallbackEventOnMouseMotion(int x, int y);
    static void CallbackEventOnTimer(int id);
    static void CallbackEventOnKeyPress(unsigned char key, int x, int y);
    static void MousePosition(const int x, const int y, double *posX, double *posY);

    ManipSimulator  m_simulator;
    ManipPlanner   *m_planner;

    int  m_selectedCircle;
    bool m_editRadius;
    bool m_run;
    
    std::vector<double> m_dthetas;
    
};

#endif
