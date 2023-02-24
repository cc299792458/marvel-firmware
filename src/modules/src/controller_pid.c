#define DEBUG_MODULE "CONTROLLER_PID"
#include "debug.h"

#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "math.h"

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

const float PI = 3.14159f;
static bool mode=true;

static float deriv_alpha;
static float integ_alpha;
static float pre_error_alpha;
static float deriv_beta;
static float integ_beta;
static float pre_error_beta;

const float arm_length = 0.031f;
const float ctau = 0.00596f;

const float kp_alpha = 3.0f;
const float ki_alpha = 0.0f;
const float kd_alpha = 0.0f;
const float kp_beta = 3.0f;
const float ki_beta = 0.0f;
const float kd_beta = 0.0f;

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
  // if ((setpoint->attitudeQuaternion.w+setpoint->attitudeQuaternion.x+setpoint->attitudeQuaternion.y+setpoint->attitudeQuaternion.z) == 0){
  //   mode = true;
  //   int num = 1;
  //   if (setpoint->thrust != 0){
  //     DEBUG_PRINT("%d\n", num);
  //   }
  // }
  // else{
  //   mode = false;
  //   DEBUG_PRINT("False\n");
  // }
  mode = false;
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

    if (RATE_DO_EXECUTE(POSITION_RATE, tick)) { // 100Hz
      positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
    }

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) { // 500Hz
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
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)){  // 500Hz
      float alpha = 0.0f;
      float beta = 0.0f;
      float alphaAcc = 0.0f;
      float betaAcc = 0.0f;
      gimbalJointEstimator(setpoint, state, &alpha, &beta);
      // DEBUG_PRINT("alpha%f\n:", (double)alpha);

      gimbalControllerPID(setpoint, alpha, beta, &alphaAcc, &betaAcc);
      // DEBUG_PRINT("alphaAcc:%f\n", (double)alphaAcc);
      gimbalMotorCommandMapping(setpoint, alphaAcc, betaAcc, alpha, beta, control);
      // control->thrust = 30000;  // use to test switching mode
    }
    
  }
}

void quat2Rotmat(float rotmat[3][3], float quat[4]){
  // quat: w, x, y, z
  float w = quat[0];
  float x = quat[1];
  float y = quat[2];
  float z = quat[3];
  rotmat[0][0] = 1-2*y*y-2*z*z;
  rotmat[0][1] = 2*x*y-2*w*z;
  rotmat[0][2] = 2*x*z+2*w*y;
  rotmat[1][0] = 2*x*y+2*w*z;
  rotmat[1][1] = 1-2*x*x-2*z*z;
  rotmat[1][2] = 2*y*z-2*w*x;
  rotmat[2][0] = 2*x*z-2*w*y;
  rotmat[2][1] = 2*y*z+2*w*x;
  rotmat[2][2] = 1-2*y*y-2*y*y;
}

void matMulti(float mat1[3][3], float mat2[3][3], float mat_result[3][3]){
  mat_result[0][0] = mat1[0][0]*mat2[0][0]+mat1[0][1]*mat2[1][0]+mat1[0][2]*mat2[2][0];
  mat_result[0][1] = mat1[0][0]*mat2[0][1]+mat1[0][1]*mat2[1][1]+mat1[0][2]*mat2[2][1];
  mat_result[0][2] = mat1[0][0]*mat2[0][2]+mat1[0][1]*mat2[1][2]+mat1[0][2]*mat2[2][2];
  mat_result[1][0] = mat1[1][0]*mat2[0][0]+mat1[1][1]*mat2[1][0]+mat1[1][2]*mat2[2][0];
  mat_result[1][1] = mat1[1][0]*mat2[0][1]+mat1[1][1]*mat2[1][1]+mat1[1][2]*mat2[2][1];
  mat_result[1][2] = mat1[1][0]*mat2[0][2]+mat1[1][1]*mat2[1][2]+mat1[1][2]*mat2[2][2];
  mat_result[2][0] = mat1[2][0]*mat2[0][0]+mat1[2][1]*mat2[1][0]+mat1[2][2]*mat2[2][0];
  mat_result[2][1] = mat1[2][0]*mat2[0][1]+mat1[2][1]*mat2[1][1]+mat1[2][2]*mat2[2][1];
  mat_result[2][2] = mat1[2][0]*mat2[0][2]+mat1[2][1]*mat2[1][2]+mat1[2][2]*mat2[2][2];  
}

void matTrans(float mat[3][3], float mat_result[3][3]){
  mat_result[0][0] = mat[0][0];
  mat_result[0][1] = mat[1][0];
  mat_result[0][2] = mat[2][0];
  mat_result[1][0] = mat[0][1];
  mat_result[1][1] = mat[1][1];
  mat_result[1][2] = mat[2][1];
  mat_result[2][0] = mat[0][2];
  mat_result[2][1] = mat[1][2];
  mat_result[2][2] = mat[2][2];
}

void rotMatZ(float mat[3][3], float index){
  float theta = index * PI / 4;
  // float mat[3][3];
  mat[0][0] = cos(theta);
  mat[1][1] = cos(theta);
  mat[0][1] = sin(theta);
  mat[1][0] = -sin(theta);
}

void gimbalJointEstimator(setpoint_t *setpoint, const state_t *state, float *alpha, float *beta){
  float R_Bi_i[3][3];
  float R_temp_1[3][3];
  float R_temp_2[3][3];

  float quat_W_B[4];
  float quat_W_i[4];

  float R_W_B[3][3];
  quat2Rotmat(R_W_B, quat_W_B);
  float R_B_Bi[3][3];
  rotMatZ(R_B_Bi, setpoint->attitude.yaw);
  float R_W_i[3][3];
  quat2Rotmat(R_W_i, quat_W_i);
  
  matMulti(R_W_B, R_B_Bi, R_temp_1);
  matTrans(R_temp_1, R_temp_2);
  matMulti(R_temp_2, R_W_i, R_Bi_i);

  *alpha = atan2(R_Bi_i[2][1], R_Bi_i[1][1]);
  *beta = atan2(R_Bi_i[0][2], R_Bi_i[0][0]);
}

void gimbalControllerPID(setpoint_t *setpoint, float alpha, float beta, float *alphaAcc, float *betaAcc){
  float desired_alpha = setpoint->attitude.roll;
  float desired_beta = setpoint->attitude.pitch;
  float error_alpha = desired_alpha - alpha;
  float error_beta = desired_beta - beta;
  float dt = (float)(1.0f/ATTITUDE_RATE);
  // DEBUG_PRINT("da:%f\n", (double)desired_alpha);
  DEBUG_PRINT("ea:%f\n", (double)error_alpha);
  deriv_alpha = (error_alpha - pre_error_alpha) / dt;
  deriv_beta = (error_beta - pre_error_beta) / dt;
  
  pre_error_alpha = error_alpha;
  pre_error_beta = error_beta;

  integ_alpha += error_alpha * dt;
  integ_beta += error_beta * dt;

  *alphaAcc = kp_alpha * error_alpha + ki_alpha * integ_alpha + kd_alpha * deriv_alpha;
  *betaAcc = kp_beta * error_beta + ki_beta * integ_beta + kd_beta * deriv_beta;
}

void gimbalMotorCommandMapping(setpoint_t *setpoint, float alphaAcc, float betaAcc, float alpha, float beta, control_t *control){
  
  float thrust = setpoint->thrust;
  float tauix = alphaAcc*((float)cos(beta));
  float tauiy = betaAcc;
  float tauiz = alphaAcc*((float)sin(beta));
  float c1 = 0.25f;    //const
  float c2 = (float)1/(4*arm_length);
  float c3 = (float)1/(4*ctau);
  control->roll=c1*thrust+c2*tauix-c2*tauiy-c3*tauiz;   //t1
  control->pitch=c1*thrust-c2*tauix-c2*tauiy+c3*tauiz;   //t2
  control->yaw=c1*thrust-c2*tauix+c2*tauiy-c3*tauiz;   //t3
  control->thrust=c1*thrust+c2*tauix+c2*tauiy+c3*tauiz;   //t4
  // DEBUG_PRINT("t1:%f\n", (double)control->roll);
  // DEBUG_PRINT("t2:%f\n", (double)control->pitch);
  // DEBUG_PRINT("t3:%f\n", (double)control->yaw);
  // DEBUG_PRINT("t4:%f\n", (double)control->thrust);
}
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

