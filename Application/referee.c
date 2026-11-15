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

        default:
        {
            break;
        }
    }
}

//获取缓冲能量和发射机构的射击热量
void get_buffer_energy_and_shooter_barrel_heat(float *buffer_energy, float *shooter_barrel_heat)
{
    *buffer_energy = power_heat_data.buffer_energy;;
    *shooter_barrel_heat = power_heat_data.shooter_17mm_barrel_heat;
}
