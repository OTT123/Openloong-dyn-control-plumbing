/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024 Humanoid Robot (Shanghai) Co., Ltd, under Apache 2.0.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "useful_math.h"
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include "PVT_ctrl.h"
#include "pino_kin_dyn.h"
#include "data_logger.h"
#include "wbc_priority.h"
#include "gait_scheduler.h"
#include "foot_placement.h"
#include "joystick_interpreter.h"

// MuJoCo load and compile model
char error[1000] = "Could not load binary model";
mjModel* mj_model = mj_loadXML("../models/go2/scene.xml", 0, error, 1000);
mjData* mj_data = mj_makeData(mj_model);

//************************
// main function
int main(int argc, const char** argv)
{
    // ini classes
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    Pin_KinDyn kinDynSolver("../models/go2/go2_description.urdf"); // kinematics and dynamics solver
    DataBus RobotState(18); // data bus
    
    /// ----------------- sim Loop ---------------
    double simEndTime=30;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    double startSteppingTime=3;
    double startWalkingTime=5;

    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);

    while( !glfwWindowShouldClose(uiController.window))
    {
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/60.0 && uiController.runSim) // press "1" to pause and resume, "2" to step the simulation
        {
            mj_step(mj_model, mj_data);

            simTime=mj_data->time;
            printf("-------------%.3f s------------\n",simTime);
            mj_interface.updateSensorValues();
            mj_interface.dataBusWrite(RobotState);

            // // update kinematics and dynamics info
            // kinDynSolver.dataBusRead(RobotState);
            // kinDynSolver.computeJ_dJ();
            // kinDynSolver.computeDyn();
            // kinDynSolver.dataBusWrite(RobotState);
        }
        if (mj_data->time>=simEndTime)
        {
            break;
        }

        uiController.updateScene();
    }

//    // free visualization storage
    uiController.Close();

    return 0;
}