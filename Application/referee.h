#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
    RED_HEAVYENGINEER     = 1,
    RED_RESERVED          = 2,
    RED_INFANTRY_1        = 3,
    RED_INFANTRY_2        = 4,
    RED_INFANTRY_3        = 5,
    RED_AERIAL            = 6,
    RED_SENTRY            = 7,
    RED_DART              = 8,
    RED_RADAR             = 9,
    RED_BASE              = 10,
    RED_OUTPOST           = 11,

    BLUE_HEAVYENGINEER    = 101,
    BLUE_RESERVED         = 102,
    BLUE_INFANTRY_1       = 103,
    BLUE_INFANTRY_2       = 104,
    BLUE_INFANTRY_3       = 105,
    BLUE_AERIAL           = 106,
    BLUE_SENTRY           = 107,
    BLUE_DART             = 108,
    BLUE_RADAR            = 109,
    BLUE_BASE             = 110,
    BLUE_OUTPOST          = 111,
} robot_id_t;
//0x0001
typedef enum
{
    GAME_TYPE_RMUC       = 1,  // RoboMaster机甲大师超级对抗赛
    GAME_TYPE_SINGLE     = 2,  // RoboMaster机甲大师高校单项赛
    GAME_TYPE_ICRA_AI    = 3,  // ICRA RoboMaster高校人工智能挑战赛
    GAME_TYPE_RMUL_3V3   = 4,  // RoboMaster机甲大师高校联盟赛3V3对抗
    GAME_TYPE_RMUL_INF   = 5   // RoboMaster机甲大师高校联盟赛步兵对抗
} GameType_e;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_e;
//0x0002
typedef enum
{
    GAME_RESULT_DRAW    = 0,  // 平局
    GAME_RESULT_RED_WIN = 1,  // 红方胜利
    GAME_RESULT_BLUE_WIN= 2   // 蓝方胜利
} GameResult_e;

//0x0003

typedef struct __attribute__((__packed__)) //0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_state_t;

typedef struct __attribute__((__packed__)) //0x0002
{
    uint8_t winner;
}game_result_t;
/*
typedef struct __attribute__((__packed__)) /0x0003
{
    uint16_t ally_1_robot_HP; 
	uint16_t ally_2_robot_HP; 
	uint16_t ally_3_robot_HP; 
	uint16_t ally_4_robot_HP; 
	uint16_t reserved; 
	uint16_t ally_7_robot_HP; 
	uint16_t ally_outpost_HP; 
	uint16_t ally_base_HP;
}game_robot_HP_t;
*/
typedef struct __attribute__((__packed__)) //0x0003
{ 
  uint16_t ally_1_robot_HP;  
  uint16_t reserved; 
  uint16_t ally_3_robot_HP;  
  uint16_t ally_4_robot_HP;  
  int16_t damage_difference;  
  uint16_t ally_7_robot_HP;  
  uint16_t outpost_R_HP;  
  uint16_t ally_base_HP; 
  uint16_t outpost_B_HP; 
  uint16_t enemy_base_HP; 
  uint16_t ally_control_degree; 
  uint16_t enemy_control_degree; 
}game_robot_HP_t; 
/*
typedef struct __attribute__((__packed__)) //0x0101
{
    uint32_t event_data;
}event_data_t;
*/
typedef struct __attribute__((__packed__)) //0x0101
{ 
  uint32_t event_data; 
  uint32_t event_data_2; 
}event_data_t; 

typedef struct __attribute__((__packed__)) //0x0104
{
    uint8_t level;
    uint8_t offending_robot_id; 
	uint8_t count;
}referee_warning_t;

typedef struct __attribute__((__packed__)) //0x0105
{ 
  uint8_t dart_remaining_time; 
  uint16_t dart_info; 
}dart_info_t; 
/*
typedef struct __attribute__((__packed__)) //0x0201
{
    uint8_t robot_id; 
	uint8_t robot_level; 
	uint16_t current_HP; 
	uint16_t maximum_HP; 
	uint16_t shooter_barrel_cooling_value; 
	uint16_t shooter_barrel_heat_limit; 
	uint16_t chassis_power_limit; 
	uint8_t power_management_gimbal_output : 1; 
	uint8_t power_management_chassis_output : 1; 
	uint8_t power_management_shooter_output : 1;
}robot_status_t;
*/
typedef struct __attribute__((__packed__)) //0x0201 
{ 
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  float bullet_speed_limit; 
  uint8_t power_management_gimbal_output : 1; 
  uint8_t power_management_chassis_output : 1;  
  uint8_t power_management_shooter_output : 1; 
}robot_status_t; 

typedef struct __attribute__((__packed__)) //0x0202
{
    uint16_t reserved0; 
	uint16_t reserved1; 
	float reserved2; 
	uint16_t buffer_energy; 
	uint16_t shooter_17mm_barrel_heat; 
	uint16_t shooter_42mm_barrel_heat; 
}power_heat_data_t;

typedef struct __attribute__((__packed__)) //0x0203
{
    float x; 
	float y; 
	float angle; 
}robot_pos_t;
/*
typedef struct __attribute__((__packed__)) //0x0204
{
    uint8_t recovery_buff; 
	uint16_t cooling_buff; 
	uint8_t defence_buff; 
	uint8_t vulnerability_buff; 
	uint16_t attack_buff; 
	uint8_t remaining_energy; 
}buff_t;
*/
typedef struct __attribute__((__packed__)) //0x0204
{ 
  uint8_t recovery_buff;  
  uint16_t cooling_buff;  
  uint8_t defence_buff;  
  uint8_t vulnerability_buff; 
  uint16_t attack_buff; 
 
}buff_t; 
/*
typedef struct __attribute__((__packed__)) //0x0206
{
	uint8_t armor_id : 4; 
	uint8_t HP_deduction_reason : 4; 
}hurt_data_t;
*/
typedef struct __attribute__((__packed__)) //0x0206 
{ 
  uint8_t armor_id : 4; 
  uint8_t HP_deduction_reason : 4; 
  uint16_t hit_sequence; 
  int16_t hit_position_x; 
  int16_t hit_position_y; 
}hurt_data_t; 

typedef struct __attribute__((__packed__)) //0x0207
{
    uint8_t bullet_type; 
	uint8_t shooter_number; 
	uint8_t launching_frequency; 
	float initial_speed; 
}shoot_data_t;

typedef struct __attribute__((__packed__)) //0x0208
{
    uint16_t projectile_allowance_17mm; 
	uint16_t projectile_allowance_42mm; 
	uint16_t remaining_gold_coin; 
	uint16_t projectile_allowance_fortress;
}projectile_allowance_t;

typedef struct __attribute__((__packed__)) //0x0209 
{ 
  uint32_t rfid_status;  
  uint16_t rfid_status_2; 
}rfid_status_t; 

typedef struct __attribute__((__packed__)) //0x020B
{ 
  float heavyengineer_x;  
  float heavyengineer_y;  
  float reserved0; 
  float reserved1; 
  float standard_3_x; 
  float standard_3_y; 
  float standard_4_x; 
  float standard_4_y;
  float reserved2; 
  float reserved3; 
}ground_robot_position_t;

typedef struct __attribute__((__packed__)) //0x020D 
{ 
  uint32_t sentry_info; 
  uint16_t sentry_info_2; 
uint64_t sentry_info_3; 
}sentry_info_t; 

typedef struct __attribute__((__packed__)) //0x0301
{
    uint16_t data_cmd_id; 
	uint16_t sender_id; 
	uint16_t receiver_id; 
	uint8_t user_data[112]; 
}robot_interaction_data_t;
/*
typedef struct __attribute__((__packed__)) //0x0302
{ 
	uint8_t data[30];
}custom_robot_data_t;
*/
typedef struct __attribute__((__packed__)) //0x0303
{ 
  float target_position_x; 
  float target_position_y; 
  uint8_t cmd_keyboard; 
  uint8_t target_robot_id; 
  uint16_t cmd_source; 
}map_command_t; 
/*
typedef struct __attribute__((__packed__)) //0x0308
{ 
	uint8_t data[30];
}robot_custom_data_t;
*/
typedef struct __attribute__((__packed__)) //0x0308
{ 
  uint16_t sender_id; 
  uint16_t receiver_id; 
  uint8_t user_data[30]; 
}custom_info_t; 

typedef struct __attribute__((__packed__)) //0x0310
{ 
	uint8_t data[300];
}robot_custom_data_2_t;
/*
typedef struct __attribute__((__packed__)) //0x0311
{ 
	uint8_t data[30];
}robot_custom_data_3_t;
*/
typedef struct __attribute__((__packed__)) //0x0311
{ 
	uint8_t data[60];
}robot_custom_data_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

void get_buffer_energy_and_shooter_barrel_heat(fp32 *buffer_energy, fp32 *shooter_barrel_heat);

extern uint8_t get_robot_id(void);

#endif
