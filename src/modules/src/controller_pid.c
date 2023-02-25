#define DEBUG_MODULE "CONTROLLER_PID"
#include "debug.h"

#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"
// #include "controller_single_ppid.h"
// #include "single_qc_ppid.h"
#include "motors.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

static bool mode=true;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{ 
  /* Use setpoint->attitudeQuaternion as switch.
    For quadrotor mode, all kinds of methods to set setpoint 
    will keep setpoint->attitudeQuaternion = 0.
    In the contrast, for gimbal mode, we will use w, x, y, z, 
    so that they won't be 0. 
  */ 
  if ((setpoint->attitudeQuaternion.w+setpoint->attitudeQuaternion.x+setpoint->attitudeQuaternion.y+setpoint->attitudeQuaternion.z) == 0){
    mode = true;
    // if (setpoint->thrust != 0){
    //   DEBUG_PRINT("True\n");
    // }
  }
  else{
    mode = false;
    DEBUG_PRINT("False\n");
  }
  if (mode){
    // DEBUG_PRINT("Current mode is conventional quadrotor!\n");
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      // Rate-controled YAW is moving YAW angle setpoint
      if (setpoint->mode.yaw == modeVelocity) {
        attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);
        
        #ifdef YAW_MAX_DELTA
        float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
        // keep the yaw setpoint within +/- YAW_MAX_DELTA from the current yaw
          if (delta > YAW_MAX_DELTA)
          {
            attitudeDesired.yaw = state->attitude.yaw + YAW_MAX_DELTA;
          }
          else if (delta < -YAW_MAX_DELTA)
          {
            attitudeDesired.yaw = state->attitude.yaw - YAW_MAX_DELTA;
          }
        #endif
      } else {
        attitudeDesired.yaw = setpoint->attitude.yaw;
      }

      attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
    }

    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
      positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
    }

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      // Switch between manual and automatic position control
      if (setpoint->mode.z == modeDisable) {
        actuatorThrust = setpoint->thrust;
      }
      if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
        attitudeDesired.roll = setpoint->attitude.roll;
        attitudeDesired.pitch = setpoint->attitude.pitch;
      }

      attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                  attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                  &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

      // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
      // value. Also reset the PID to avoid error buildup, which can lead to unstable
      // behavior if level mode is engaged later
      if (setpoint->mode.roll == modeVelocity) {
        rateDesired.roll = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID();
      }
      if (setpoint->mode.pitch == modeVelocity) {
        rateDesired.pitch = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID();
      }

      // TODO: Investigate possibility to subtract gyro drift.
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                              rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

      attitudeControllerGetActuatorOutput(&control->roll,
                                          &control->pitch,
                                          &control->yaw);

      control->yaw = -control->yaw;

      cmd_thrust = control->thrust;
      cmd_roll = control->roll;
      cmd_pitch = control->pitch;
      cmd_yaw = control->yaw;
      r_roll = radians(sensors->gyro.x);
      r_pitch = -radians(sensors->gyro.y);
      r_yaw = radians(sensors->gyro.z);
      accelz = sensors->acc.z;
    }

    control->thrust = actuatorThrust;

    if (control->thrust == 0)
    {
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;

      cmd_thrust = control->thrust;
      cmd_roll = control->roll;
      cmd_pitch = control->pitch;
      cmd_yaw = control->yaw;

      attitudeControllerResetAllPID();
      positionControllerResetAllPID();

      // Reset the calculated YAW angle for rate control
      attitudeDesired.yaw = state->attitude.yaw;
    }    
  }
  else{
    // add gimbal controller here.
    // DEBUG_PRINT("Current mode is gimbal thrust generator!\n");
    
    control->thrust = 30000;  // use to test switching mode
  }
}

// void controllerSinglePPIDInit(void)
// {
//   single_qc_ppid_initialize();
// }

// bool controllerSinglePPIDTest(void)
// {
//   return true;
// }

// void controllerSinglePPID(control_t *control, setpoint_t *setpoint,
//                                          const sensorData_t *sensors,
//                                          const state_t *state,
//                                          const uint32_t tick)
// {
//   single_qc_ppid_U.index = setpoint->attitude.roll;

//   single_qc_ppid_U.qw_op = setpoint->attitudeQuaternion.w;
//   single_qc_ppid_U.qx_op = setpoint->attitudeQuaternion.x;
//   single_qc_ppid_U.qy_op = setpoint->attitudeQuaternion.y;
//   single_qc_ppid_U.qz_op = setpoint->attitudeQuaternion.z;

//   single_qc_ppid_U.qw_IMU = state->attitudeQuaternion.w;
//   single_qc_ppid_U.qx_IMU = state->attitudeQuaternion.x;
//   single_qc_ppid_U.qy_IMU = state->attitudeQuaternion.y;
//   single_qc_ppid_U.qz_IMU = state->attitudeQuaternion.z;

//   single_qc_ppid_U.alpha_desired = setpoint->attitude.pitch;
//   single_qc_ppid_U.beta_desired = setpoint->attitude.yaw;

//   single_qc_ppid_U.omega_x = -sensors->gyro.y;
//   single_qc_ppid_U.beta_speed = sensors->gyro.x;
//   single_qc_ppid_U.omega_z = sensors->gyro.z;

//   single_qc_ppid_U.thrust = setpoint->thrust;

//   single_qc_ppid_step();

//   if (setpoint->thrust < 0.000898f)
//   {
//     motorsSetRatio(0, 0);
//     motorsSetRatio(1, 0);
//     motorsSetRatio(2, 0);
//     motorsSetRatio(3, 0);    
//   }
//   else
//   {
//     motorsSetRatio(0, single_qc_ppid_Y.m1);
//     motorsSetRatio(1, single_qc_ppid_Y.m2);
//     motorsSetRatio(2, single_qc_ppid_Y.m3);
//     motorsSetRatio(3, single_qc_ppid_Y.m4);  
//   }
// }

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

