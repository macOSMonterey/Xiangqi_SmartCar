#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_state_t game_state;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t event_data;
referee_warning_t referee_warning;
dart_info_t dart_info;

robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_pos;
buff_t buff;
hurt_data_t hurt_data;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;
ground_robot_position_t ground_robot_position;
sentry_info_t sentry_info;

robot_interaction_data_t robot_interaction_data;
map_command_t map_command;
custom_info_t custom_info;
robot_custom_data_2_t robot_custom_data_2;
robot_custom_data_t robot_custom_data;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(game_state_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
	
    memset(&event_data, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));
	
    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));

    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&custom_info, 0, sizeof(custom_info_t));
    memset(&robot_custom_data_2, 0, sizeof(robot_custom_data_2_t));
    memset(&robot_custom_data, 0, sizeof(robot_custom_data_t));
}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
        }
        break;
		
        case EVENT_DATA_CMD_ID:
        {
            memcpy(&event_data, frame + index, sizeof(event_data_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
        }
        break;
        case DART_INFO_CMD_ID:
        {
            memcpy(&dart_info, frame + index, sizeof(dart_info_t));
        }
        break;
		
        case ROBOT_STATUS_CMD_ID:
        {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
        }
        break;
        case BUFF_CMD_ID:
        {
            memcpy(&buff, frame + index, sizeof(buff_t));
        }
        break;
        case HURT_DATA_CMD_ID:
        {
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
        }
        break;
        case PROJECTILE_ALLOWANCE_CMD_ID:
        {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
        }
        break;
        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
        }
        break;
        case SENTRY_INFO_CMD_ID:
        {
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
        }
        break;
        
        case ROBOT_INTERACTION_CMD_ID:
        {
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
        }
        break;
        case MAP_COMMAND_CMD_ID:
        {
            memcpy(&map_command, frame + index, sizeof(map_command_t));
        }
        break;
        case CUSTOM_INFO_CMD_ID:
        {
            memcpy(&custom_info, frame + index, sizeof(custom_info_t));
        }
        break;
        case ROBOT_CUSTOM_DATA_2_CMD_ID:
        {
            memcpy(&robot_custom_data_2, frame + index, sizeof(robot_custom_data_2_t));
        }
        break;
        case ROBOT_CUSTOM_DATA_CMD_ID:
        {
            memcpy(&robot_custom_data, frame + index, sizeof(robot_custom_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}

//获取缓冲能量和发射机构的射击热量
void get_buffer_energy_and_shooter_barrel_heat(fp32 *buffer_energy, fp32 *shooter_barrel_heat)
{
    *buffer_energy = power_heat_data.buffer_energy;;
    *shooter_barrel_heat = power_heat_data.shooter_17mm_barrel_heat;
}

//获取id
uint8_t get_robot_id(void)
{
    return robot_status.robot_id;
}

//0x0001
uint8_t get_game_type(void)
{
    return game_state.game_type;
}

uint8_t get_game_progress(void)
{
    return game_state.game_progress;
}

uint16_t get_stage_remain_time(void)
{
    return game_state.stage_remain_time;
}

// 己方1号重装机器人血量
uint16_t get_our_1_heavy_hp(void)
{
    return game_robot_HP.ally_1_robot_HP;
}

// 己方3号步兵机器人血量
uint16_t get_our_3_infantry_hp(void)
{
    return game_robot_HP.ally_3_robot_HP;
}

// 己方4号步兵机器人血量
uint16_t get_our_4_infantry_hp(void)
{
    return game_robot_HP.ally_4_robot_HP;
}

// 己方7号哨兵机器人血量
uint16_t get_our_7_sentry_hp(void)
{
    return game_robot_HP.ally_7_robot_HP;
}

// 己方全队总伤害与对方全队总伤害之差
int16_t get_damage_diff(void)
{
    return game_robot_HP.damage_difference;
}

// R前哨血量
uint16_t get_R_outpost_hp(void)
{
    return game_robot_HP.outpost_R_HP;
}
/*
// B前哨血量
uint16_t get_B_outpost_hp(void)
{
    return battlefield.B_outpost_hp;
}

// 己方基地血量
uint16_t get_our_base_hp(void)
{
    return battlefield.our_base_hp;
}

// 对方基地血量
uint16_t get_enemy_base_hp(void)
{
    return battlefield.enemy_base_hp;
}

// 己方累计控制度
uint16_t get_our_accum_control(void)
{
    return battlefield.our_accum_control;
}

// 对方累计控制度
uint16_t get_enemy_accum_control(void)
{
    return battlefield.enemy_accum_control;
}
*/
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    //*power = power_heat_data.chassis_power;
    //*buffer = power_heat_data.chassis_power_buffer;

}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    //*heat0_limit = robot_state.shooter_heat0_cooling_limit;
    //*heat0 = power_heat_data.shooter_heat0;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    //*heat1_limit = robot_state.shooter_heat1_cooling_limit;
    //*heat1 = power_heat_data.shooter_heat1;
}