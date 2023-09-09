#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

V3F ClipV3F(V3F vector, float limit);

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);

#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // 
  // Define inverce matix for 
  //        Ftot     = +F1 +F2 +F3 +F4  
  //        Mx/l     = +F1 -F2 +F3 -F4  
  //        My/l     = +F1 +F2 -F3 -F4  
  //        Mz/kappa = -F1 +F2 +F3 -F4  
  //      
  //       [Ftot , Mx/l , My/l , Mz/kappa]T  = R x [F1 , F2 , F3 , F4]T
  // 
  //       [F1 , F2 , F3 , F4]T = inv(R) x [Ftot , Mx/l , My/l , Mz/kappa]T

    float l = L / sqrt(2); // L- length of arm from centre of quadrocopter to motor

    float M[4][4] = {{ +1.0f , +1.0f , +1.0f , -1.0f }, \
                     { +1.0f , -1.0f , +1.0f , +1.0f }, \
                     { +1.0f , +1.0f , -1.0f , +1.0f }, \
                     { +1.0f , -1.0f , -1.0f , -1.0f }};

    float c     = collThrustCmd;
    float p_bar = momentCmd.x / l;
    float q_bar = momentCmd.y / l;
    float r_bar = momentCmd.z / kappa;

    float F1 = (M[0][0] * c + M[0][1] * p_bar + M[0][2] * q_bar + M[0][3] * r_bar)/4; // front left
    float F2 = (M[1][0] * c + M[1][1] * p_bar + M[1][2] * q_bar + M[1][3] * r_bar)/4; // front right
    float F3 = (M[2][0] * c + M[2][1] * p_bar + M[2][2] * q_bar + M[2][3] * r_bar)/4; // rear right
    float F4 = (M[3][0] * c + M[3][1] * p_bar + M[3][2] * q_bar + M[3][3] * r_bar)/4; // rear left

    cmd.desiredThrustsN[0] = F1;
    cmd.desiredThrustsN[1] = F2;
    cmd.desiredThrustsN[2] = F3;
    cmd.desiredThrustsN[3] = F4;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F pqr_err = (pqrCmd - pqr);
  V3F I(Ixx,Iyy,Izz);
  momentCmd = I * kpPQR * pqr_err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // accelCmd -> [x_dot_dot, y_dot_dot, z_dot_dot]
  // attitude -> [a, b, c, d] => R
  // collThrustCmd -> c
  float b_x_a = R(0, 2);
  float b_y_a = R(1, 2);

  if (sqrt(accelCmd.x * accelCmd.x + accelCmd.y * accelCmd.x)> maxAccelXY) {
      V3F vector(accelCmd.x, accelCmd.y, 0);
      vector = ClipV3F(vector, maxAccelXY);
      accelCmd.x = vector.x;
      accelCmd.y = vector.y;
  }

  float b_x_c_target = -accelCmd.x / (collThrustCmd / mass);
  float b_y_c_target = -accelCmd.y / (collThrustCmd / mass);

  b_x_c_target = CONSTRAIN(b_x_c_target, -maxTiltAngle, maxTiltAngle);
  b_y_c_target = CONSTRAIN(b_y_c_target, -maxTiltAngle, maxTiltAngle);

  float b_x_c_dot = kpBank * (b_x_c_target - b_x_a);
  float b_y_c_dot = kpBank * (b_y_c_target - b_y_a);

  float R11 = R(0, 0);
  float R21 = R(1, 0);
  float R12 = R(0, 1);
  float R22 = R(1, 1);
  float R33 = R(2, 2);

  float p_c = (R21 * b_x_c_dot - R11 * b_y_c_dot) / R33;
  float q_c = (R22 * b_x_c_dot - R12 * b_y_c_dot) / R33;

  pqrCmd.x = p_c;
  pqrCmd.y = q_c;
  pqrCmd.z = 0;
      /////////////////////////////// END STUDENT CODE ////////////////////////////

   return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // I'll use PIDff controller
  // thrust = c = (u1_bar - g) / b_z
  // b_z = R33 = R(2,2)
  // u1_bar = kpPosZ * posZ_err + kpVelZ * velZ_err + KiPosZ * integratedAltitudeError + accelZCmd

  velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  float posZ_err = posZCmd - posZ;
  float velZ_err = velZCmd - velZ;
  integratedAltitudeError += posZ_err * dt * dt ;
  float u1_bar = kpVelZ * kpPosZ * posZ_err + kpVelZ * velZ_err + KiPosZ * integratedAltitudeError + accelZCmd;
  thrust = -mass * (u1_bar - 9.81f) / R(2, 2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F posError = (posCmd - pos);
  posError.z = 0;

  V3F velCmd_tot = velCmd + kpPosXY* posError;
  velCmd = ClipV3F(velCmd_tot, maxSpeedXY);

  V3F velError = (velCmd_tot - vel);
  velError.z = 0;
  
  accelCmd += kpVelXY * velError;
  accelCmd = ClipV3F(accelCmd, maxAccelXY);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float yawErr = fmodf(yawCmd - yaw, 2.0f * (float)F_PI) ;
  if (yawErr > (float)F_PI) {
      yawErr -= (2.0f * (float)F_PI);
  }
  else if (yawErr < -(float)F_PI) {
      yawErr += (2.0f * (float)F_PI);
  }
  yawRateCmd = kpYaw * yawErr;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}

// Limits V3F vector magnitude to certain value
V3F ClipV3F(V3F vector, float limit)
{
    if (limit > 0) {
        if (vector.mag() > limit) {
            vector = vector.norm() * limit;
        }
    }
    return vector;
}
