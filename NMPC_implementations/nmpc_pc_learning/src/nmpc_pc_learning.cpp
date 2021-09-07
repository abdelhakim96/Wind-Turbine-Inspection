/**
 * @file   nmpc_pc_learning.cpp
 * @author Mohit Mehndiratta
 * @date   April 2019
 *
 * @copyright
 * Copyright (C) 2019.
 */

#include <nmpc_pc_learning.h>

using namespace Eigen;

NMPCworkspace nmpcWorkspace;
NMPCvariables nmpcVariables;

NMPC_PC::NMPC_PC(struct nmpc_struct_ &_nmpc_inp_struct)
{
  is_control_init = false;

  nmpc_inp_struct = _nmpc_inp_struct;
  nmpc_inp_struct_0 = _nmpc_inp_struct;

  WN.resize(NMPC_NYN);
  WN << nmpc_inp_struct.W_Wn_factor*nmpc_inp_struct.W[0], nmpc_inp_struct.W_Wn_factor*nmpc_inp_struct.W[1],
        nmpc_inp_struct.W_Wn_factor*nmpc_inp_struct.W[2], nmpc_inp_struct.W_Wn_factor*nmpc_inp_struct.W[3],
        nmpc_inp_struct.W_Wn_factor*nmpc_inp_struct.W[4], nmpc_inp_struct.W_Wn_factor*nmpc_inp_struct.W[5];

  // --------------------
  // ACADO NMPC CONTROLLER
  // --------------------

  nmpc_struct.initializeSolver = boost::bind(nmpc_initializeSolver);
  nmpc_struct.preparationStep = boost::bind(nmpc_preparationStep);
  nmpc_struct.feedbackStep = boost::bind(nmpc_feedbackStep);
  nmpc_struct.getKKT = boost::bind(nmpc_getKKT);
  nmpc_struct.getObjective = boost::bind(nmpc_getObjective);
  nmpc_struct.printDifferentialVariables = boost::bind(nmpc_printDifferentialVariables);
  nmpc_struct.printControlVariables = boost::bind(nmpc_printControlVariables);

  nmpc_struct.acado_N = NMPC_N;
  nmpc_struct.acado_NX = NMPC_NX;
  nmpc_struct.acado_NY = NMPC_NY;
  nmpc_struct.acado_NYN = NMPC_NYN;
  nmpc_struct.acado_NU = NMPC_NU;
  nmpc_struct.acado_NOD = NMPC_NOD;

  nmpc_struct.x0 = &nmpcVariables.x0[0];
  nmpc_struct.x = &nmpcVariables.x[0];
  nmpc_struct.od = &nmpcVariables.od[0];
  nmpc_struct.y = &nmpcVariables.y[0];
  nmpc_struct.yN = &nmpcVariables.yN[0];
  nmpc_struct.u = &nmpcVariables.u[0];
  nmpc_struct.W = &nmpcVariables.W[0];
  nmpc_struct.WN = &nmpcVariables.WN[0];

  nmpc_cmd_struct.roll_ang = 0.0;
  nmpc_cmd_struct.pitch_ang = 0.0;
  nmpc_cmd_struct.yaw_ang = nmpc_inp_struct.U_ref(2);
  nmpc_cmd_struct.Fz = nmpc_inp_struct.U_ref(3);
  nmpc_cmd_struct.Fz_scaled = ( (1 - 0)/(nmpc_inp_struct.max_Fz_scale - nmpc_inp_struct.min_Fz_scale) ) * (nmpc_cmd_struct.Fz - nmpc_inp_struct.min_Fz_scale);
  nmpc_cmd_struct.exe_time = 0.0;
  nmpc_cmd_struct.kkt_tol = 0.0;
  nmpc_cmd_struct.obj_val = 0.0;

  if (nmpc_inp_struct.verbose)
  {
    std::cout<<"***********************************\n";
    std::cout<<"Constructor of the class NMPC_PC is created\n";
    std::cout<<"***********************************\n";
  }

}

NMPC_PC::~NMPC_PC()
{
  if (nmpc_inp_struct.verbose)
    std::cout<<"Destructor of the class NMPC_PC\n";
}

bool NMPC_PC::return_control_init_value()
{
  return NMPC_PC::is_control_init;
}

void NMPC_PC::nmpc_init(std::vector<double> posref, struct acado_struct& acadostruct)
{

  if (nmpc_inp_struct.verbose)
  {
    std::cout<<"***********************************\n";
    std::cout<<"outer_nmpc_initController - start\n";
  }

  // Initialize the solver
  // ---------------------
  acadostruct.initializeSolver();

  // NMPC: initialize/set the states
  // ---------------------
  for (int i = 0; i < acadostruct.acado_NX * (acadostruct.acado_N + 1); ++i)
  {
    acadostruct.x[i] = 0.0;
  }

  // NMPC: initialize/set the controls
  // ---------------------
  for (int i = 0; i < acadostruct.acado_N; ++i)
  {
    for (int j = 0; j < acadostruct.acado_NU; ++j)
      acadostruct.u[(i * acadostruct.acado_NU) + j] = nmpc_inp_struct.U_ref(j);
  }

  // NMPC: initialize/set the online data
  // ---------------------
  for (int i = 0; i < acadostruct.acado_NOD * (acadostruct.acado_N + 1); ++i)
  {
    acadostruct.od[i] = 0.0;
  }

  // NMPC: initialize the measurements/reference
  // ---------------------
  for (int i = 0; i < acadostruct.acado_NY * acadostruct.acado_N; ++i)
  {
    acadostruct.y[i] = 0.0;
  }
  for (int i = 0; i < acadostruct.acado_NYN; ++i)
  {
    acadostruct.yN[i] = 0.0;
  }

  // NMPC: initialize the current state feedback
  // ---------------------
#if ACADO_INITIAL_STATE_FIXED
  for (int i = 0; i < acadostruct.acado_NX; ++i)
  {
    if (i < 3)
    {
      acadostruct.x0[i] = posref[i];
    }
    else
      acadostruct.x0[i] = 0;
  }
#endif

  // NMPC: initialize the weight matrices
  // ---------------------
  for(int i = 0; i < acadostruct.acado_NY; ++i )
  {
    for(int j = 0; j < acadostruct.acado_NY; ++j )
    {
      if(i==j)
        acadostruct.W[(i * acadostruct.acado_NY) + j] = nmpc_inp_struct.W[i];
      else
        acadostruct.W[(i * acadostruct.acado_NY) + j] = 0.0;
    }
  }
//  std::cout<<"W_0 = "<<nmpc_inp_struct.W<<"\n";
  for(int i = 0; i < acadostruct.acado_NYN; ++i )
  {
    for(int j = 0; j < acadostruct.acado_NYN; ++j )
    {
      if(i==j)
        acadostruct.WN[(i * acadostruct.acado_NYN) + j] = WN[i];
      else
        acadostruct.WN[(i * acadostruct.acado_NYN) + j] = 0.0;
    }
  }
//  std::cout<<"WN_0 = "<<WN<<"\n";

  // Prepare first step
  // ------------------
  acadostruct.preparationStep();

  if (nmpc_inp_struct.verbose)
  {
    std::cout<<"Outer NMPC: initialized correctly\n";
    std::cout<<"***********************************\n";
  }
  is_control_init = true;
}

void NMPC_PC::nmpc_core(struct nmpc_struct_ &_nmpc_inp_struct, struct acado_struct &acadostruct, struct command_struct &commandstruct,
                        Eigen::Vector3d &reftrajectory, Eigen::Vector3d &refvelocity,
                        std::vector<double> &distFx, std::vector<double> &distFy, std::vector<double> &distFz,
                        std::vector<double> &currentposatt, std::vector<double> &currentvelrate)
{
  nmpc_inp_struct = _nmpc_inp_struct;

  // To avoid sending nan values to optimization problem
  nan_check_for_dist_estimates(distFx, distFy, distFz);

  // set the current state feedback
  set_measurements(acadostruct, distFx, distFy, distFz, currentposatt, currentvelrate);

  // set the reference path
  set_reftrajectory(acadostruct, reftrajectory, refvelocity);

  // NMPC: calc and apply control and prepare optimization for the next step
  // ----------------------------------------------------------------------

  // Execute Calculation (Optimization)
  clock_t stopwatch;
  stopwatch = clock();
  acado_feedbackStep_fb = acadostruct.feedbackStep();

  if (nmpc_inp_struct.verbose && acado_feedbackStep_fb != 0)
  {
    std::cout<<"ACADO ERROR: " << acado_feedbackStep_fb<<"\n";
    std::cout<<
        "acado outer nmpc controller states: x, y, z, u, v, w = " << acadostruct.x0[0] << ", "
        << acadostruct.x0[1] << ", " << acadostruct.x0[2] << ", " << acadostruct.x0[3] << ", "
        << acadostruct.x0[4] << ", " << acadostruct.x0[5] << "\n";
  }

  // Apply the new control immediately to the process, first NU components.
  commandstruct.roll_ang = acadostruct.u[0];
  commandstruct.pitch_ang = acadostruct.u[1];
  if (nmpc_inp_struct.yaw_control)
    commandstruct.yaw_ang = acadostruct.u[2];
  else
    commandstruct.yaw_ang = nmpc_inp_struct.U_ref(2);
  commandstruct.Fz = acadostruct.u[3];
  commandstruct.Fz_scaled = ( (1 - 0)/(nmpc_inp_struct.max_Fz_scale - nmpc_inp_struct.min_Fz_scale) ) * (commandstruct.Fz - nmpc_inp_struct.min_Fz_scale);
  commandstruct.kkt_tol = acadostruct.getKKT();
  commandstruct.obj_val = acadostruct.getObjective();

  // Settings for the next iteration
  acadostruct.preparationStep();

  // Calculate the entire execution time!
  commandstruct.exe_time = ((double)(clock() - stopwatch))/CLOCKS_PER_SEC;

//    ROS_INFO_STREAM("Stoptime outer NMPC: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");

/* ------ NMPC_DEBUG ------*/
//  acadostruct.printDifferentialVariables();
//  acadostruct.printControlVariables();
}

void NMPC_PC::set_measurements(struct acado_struct &acadostruct, std::vector<double> &distFx,
                               std::vector<double> &distFy, std::vector<double> &distFz,
                               std::vector<double> &currentposatt, std::vector<double> &currentvelrate)
{
  for (int i=0; i<3; ++i)
  {
    acadostruct.x0[i] = currentposatt[i];
    acadostruct.x0[i+3] = currentvelrate[i];
  }
  for (int i = 0; i < acadostruct.acado_N + 1; ++i)
  {
    acadostruct.od[(i * acadostruct.acado_NOD)]     = distFx[i];
    acadostruct.od[(i * acadostruct.acado_NOD) + 1] = distFy[i];
    acadostruct.od[(i * acadostruct.acado_NOD) + 2] = distFz[i];
    acadostruct.od[(i * acadostruct.acado_NOD) + 3] = currentvelrate[3];
    acadostruct.od[(i * acadostruct.acado_NOD) + 4] = currentvelrate[4];
    acadostruct.od[(i * acadostruct.acado_NOD) + 5] = currentvelrate[5];
  }

  // Recompute U_ref based on new disturbance estimates
  // phi_ref = asin(Fy_dist/Fz)
  nmpc_inp_struct.U_ref(0) = asin((distFy[0]*1.7)/acadostruct.u[3]);
  // theta_ref = asin(-Fx_dist/Fz)
  nmpc_inp_struct.U_ref(1) = asin(-(distFx[0]*1.7)/acadostruct.u[3]);
  nmpc_inp_struct.U_ref(3) = (nmpc_inp_struct_0.U_ref(3) - distFz[0]*3.65); // works for GP
//  nmpc_inp_struct.U_ref(3) = (nmpc_inp_struct_0.U_ref(3) - distFz[0]*2.0);  // works for NMHE
}

void NMPC_PC::set_reftrajectory(struct acado_struct &acadostruct, Vector3d &reftrajectory,
                                                                  Vector3d &refvelocity)
{
  acadostruct.yN[0] = reftrajectory(0);
  acadostruct.yN[1] = reftrajectory(1);
  acadostruct.yN[2] = reftrajectory(2);
  acadostruct.yN[3] = refvelocity(0);
  acadostruct.yN[4] = refvelocity(1);
  acadostruct.yN[5] = refvelocity(2);

  for(int i = 0; i < acadostruct.acado_N; ++i )
  {
    for(int j = 0; j < acadostruct.acado_NY; ++j )
    {
      if (j < acadostruct.acado_NX)
        acadostruct.y[(i * acadostruct.acado_NY) + j] = acadostruct.yN[j];
      else
        acadostruct.y[(i * acadostruct.acado_NY) + j] = nmpc_inp_struct.U_ref(j - acadostruct.acado_NX);
    }
  }
}

void NMPC_PC::nan_check_for_dist_estimates(std::vector<double> &distFx, std::vector<double> &distFy,
                                  std::vector<double> &distFz)
{
  if (std::isnan(distFx[0]) == true || std::isnan(distFx[NMPC_N-1]) == true)
  {
    std::cout<<"NAN received for Fx estimates! \n";
    std::cout<<"Zero values are enforced! \n";
    std::fill(distFx.begin(), distFx.end(), 0);
  }
  if (std::isnan(distFy[0]) == true || std::isnan(distFy[NMPC_N-1]) == true)
  {
    std::cout<<"NAN received for Fy estimates! \n";
    std::cout<<"Zero values are enforced! \n";
    std::fill(distFy.begin(), distFy.end(), 0);
  }
  if (std::isnan(distFz[0]) == true || std::isnan(distFz[NMPC_N-1]) == true)
  {
    std::cout<<"NAN received for Fz estimates! \n";
    std::cout<<"Zero values are enforced! \n";
    std::fill(distFz.begin(), distFz.end(), 0);
  }
}
