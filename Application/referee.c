#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

REMOTE_t remote;
FEEDBACK_t feedback;

void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    memset(&remote, 0, sizeof(REMOTE_t));
    memset(&feedback, 0, sizeof(FEEDBACK_t));
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
        case REMOTE_CMD_ID:
        {
            memcpy(&remote, frame + index, sizeof(REMOTE_t));
        }
        break;
        case FEEDBACK_CMD_ID:
        {
            memcpy(&feedback, frame + index, sizeof(FEEDBACK_t));
        }
        break;

        default:
        {
            break;
        }
    }
}

void Get_REMOTE(float *vx, float *vy, float *yaw)
{
    *vx = remote.vx;
    *vy = remote.vy;
    *yaw = remote.yaw;
}

void Get_FEEDBACK(float *vx, float *vy, float *yaw)
{
    *vx = feedback.vx;
    *vy = feedback.vy;
    *yaw = feedback.yaw;
}
