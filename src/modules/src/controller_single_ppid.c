#define DEBUG_MODULE "SinglePPID"

#include "stabilizer.h"
#include "stabilizer_types.h"

// #include "attitude_controller.h"
#include "sensfusion6.h"
// #include "position_controller.h"
#include "controller_single_ppid.h"
#include "single_qc_ppid.h"
#include "debug.h"
#include "motors.h"

#include "log.h"
#include "param.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)


void controllerSinglePPIDInit(void)
{ 
  // DEBUG_PRINT("I have init!\n");
  single_qc_ppid_initialize();
}

bool controllerSinglePPIDTest(void)
{
  return true;
}

void controllerSinglePPID(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  // DEBUG_PRINT("SinglePPID Controller Is Running\n");
  // setpoint->attitude.yaw = 0.0f;           //test
  
  // setpoint->attitudeQuaternion.w = 1.0f;   //test
  // setpoint->attitudeQuaternion.x = 0.0f;   //test
  // setpoint->attitudeQuaternion.y = 0.0f;   //test
  // setpoint->attitudeQuaternion.z = 0.0f;   //test

  // setpoint->attitude.roll = 0.0f;          //test
  // setpoint->attitude.pitch = 0.0f;         //test

  // setpoint->thrust = 0.2f;                 //test

  ex_input.index = setpoint->attitude.yaw;

  if(setpoint->attitudeQuaternion.w+setpoint->attitudeQuaternion.x+setpoint->attitudeQuaternion.y+setpoint->attitudeQuaternion.z==0.0f){
    setpoint->attitudeQuaternion.w = 1.0f;   //test
    setpoint->attitudeQuaternion.x = 0.0f;   //test
    setpoint->attitudeQuaternion.y = 0.0f;   //test
    setpoint->attitudeQuaternion.z = 0.0f;   //test
    // DEBUG_PRINT("F\n");
  }
  ex_input.qw_op = setpoint->attitudeQuaternion.w;
  ex_input.qx_op = setpoint->attitudeQuaternion.x;
  ex_input.qy_op = setpoint->attitudeQuaternion.y;
  ex_input.qz_op = setpoint->attitudeQuaternion.z;

  ex_input.qw_IMU = state->attitudeQuaternion.w;
  ex_input.qx_IMU = state->attitudeQuaternion.x;
  ex_input.qy_IMU = state->attitudeQuaternion.y;
  ex_input.qz_IMU = state->attitudeQuaternion.z;

  // ex_input.qw_IMU = 1.0f;   //test
  // ex_input.qx_IMU = 0.0f;   //test
  // ex_input.qy_IMU = 0.0f;   //test
  // ex_input.qz_IMU = 0.0f;   //test

  ex_input.alpha_desired = setpoint->attitude.roll;
  ex_input.beta_desired = setpoint->attitude.pitch;

  ex_input.omega_x = -sensors->gyro.y;
  ex_input.beta_speed = sensors->gyro.x;
  ex_input.omega_z = sensors->gyro.z;

  // ex_input.omega_x = 0.0f;     //test
  // ex_input.beta_speed = 0.0f;  //test
  // ex_input.omega_z = 0.0f;     //test

  //0~65532 --> 0~24
  ex_input.thrust = 4 * (2.508e-9F * setpoint->thrust * setpoint->thrust - 3.754e-6F * setpoint->thrust);
  // ex_input.thrust = setpoint->thrust;

  // Add a condition judgement here
  // if (setpoint->thrust > 0){    
  //   single_qc_ppid_step();
  // }
  single_qc_ppid_step();

  if (setpoint->thrust < 0.000898f)
  {
    motorsSetRatio(0, 0);
    motorsSetRatio(1, 0);
    motorsSetRatio(2, 0);
    motorsSetRatio(3, 0);    
  }
  else
  {
    // DEBUG_PRINT("%f\n", (double)ex_output.m1);
    // DEBUG_PRINT("%f\n", (double)ex_output.m2);
    // DEBUG_PRINT("%f\n", (double)ex_output.m3);
    // DEBUG_PRINT("%f\n", (double)ex_output.m4);
    motorsSetRatio(0, ex_output.m1);
    motorsSetRatio(1, ex_output.m2);
    motorsSetRatio(2, ex_output.m3);
    motorsSetRatio(3, ex_output.m4);
  }
}


// log/param name can't be too long, otherwise error
// only one log group is allowed

LOG_GROUP_START(sctrl_ppid)

LOG_ADD(LOG_FLOAT, e_alpha, &ex_output.error_alpha)
LOG_ADD(LOG_FLOAT, e_beta, &ex_output.error_beta)
LOG_ADD(LOG_FLOAT, e_alphas, &ex_output.error_alphas)
LOG_ADD(LOG_FLOAT, e_betas, &ex_output.error_betas)
LOG_ADD(LOG_FLOAT, u_alpha, &ex_output.u_alpha)
LOG_ADD(LOG_FLOAT, u_beta, &ex_output.u_beta)

LOG_ADD(LOG_FLOAT, t_be, &ex_output.t_betae)
LOG_ADD(LOG_FLOAT, t_bin, &ex_output.t_betain)
LOG_ADD(LOG_FLOAT, t_ae, &ex_output.t_alphae)
LOG_ADD(LOG_FLOAT, t_ain, &ex_output.t_alphain)

LOG_ADD(LOG_FLOAT, x_gyro, &ex_input.omega_x)
LOG_ADD(LOG_FLOAT, b_gyro, &ex_input.beta_speed)
LOG_ADD(LOG_FLOAT, z_gyro, &ex_input.omega_z)

LOG_ADD(LOG_FLOAT, t_m1, &ex_output.t_m1)
LOG_ADD(LOG_FLOAT, t_m2, &ex_output.t_m2)
LOG_ADD(LOG_FLOAT, t_m3, &ex_output.t_m3)
LOG_ADD(LOG_FLOAT, t_m4, &ex_output.t_m4)

LOG_GROUP_STOP(sctrl_ppid)




PARAM_GROUP_START(sparam_ppid)
PARAM_ADD(PARAM_FLOAT, pgaina, &single_qc_ppid_P.pgaina)
PARAM_ADD(PARAM_FLOAT, igaina, &single_qc_ppid_P.igaina)
PARAM_ADD(PARAM_FLOAT, dgaina, &single_qc_ppid_P.dgaina)

PARAM_ADD(PARAM_FLOAT, pgainb, &single_qc_ppid_P.pgainb)
PARAM_ADD(PARAM_FLOAT, igainb, &single_qc_ppid_P.igainb)
PARAM_ADD(PARAM_FLOAT, dgainb, &single_qc_ppid_P.dgainb)

PARAM_ADD(PARAM_FLOAT, pgainas, &single_qc_ppid_P.pgainas)
PARAM_ADD(PARAM_FLOAT, igainas, &single_qc_ppid_P.igainas)
PARAM_ADD(PARAM_FLOAT, dgainas, &single_qc_ppid_P.dgainas)

PARAM_ADD(PARAM_FLOAT, pgainbs, &single_qc_ppid_P.pgainbs)
PARAM_ADD(PARAM_FLOAT, igainbs, &single_qc_ppid_P.igainbs)
PARAM_ADD(PARAM_FLOAT, dgainbs, &single_qc_ppid_P.dgainbs)

PARAM_ADD(PARAM_FLOAT, t_mod, &single_qc_ppid_P.torque_modifier)

PARAM_ADD(PARAM_FLOAT, s_tx, &single_qc_ppid_P.sat_tx)
PARAM_ADD(PARAM_FLOAT, s_ty, &single_qc_ppid_P.sat_ty)
PARAM_ADD(PARAM_FLOAT, s_tz, &single_qc_ppid_P.sat_tz)
PARAM_GROUP_STOP(sparam_ppid)

