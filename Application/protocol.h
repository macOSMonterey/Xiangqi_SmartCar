#ifndef PROTOCOL_H
#define PROTOCOL_H

/*
**串口协议格式
**通信方式为串口
**配置为：常规链路的波特率为 115200，图传链路的波特率为 921600
**8数据位，1位停止位，无硬件流控，无校验位。
*/
#define HEADER_SOF 0xA5		//帧头起始字节
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

//（SOF+len+seq+CRC8）
#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2

#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,		//比赛状态数据，固定以 1Hz 频率发送
    GAME_RESULT_CMD_ID                = 0x0002,		//比赛结果数据，比赛结束触发发送
    GAME_ROBOT_HP_CMD_ID              = 0x0003,		//机器人血量数据，固定以 3Hz 频率发送
    EVENT_DATA_CMD_ID                 = 0x0101,		//场地事件数据，固定以 1Hz 频率发送
    REFEREE_WARNING_CMD_ID            = 0x0104,		//裁判警告数据，己方判罚/判负时触发发送，其余时间以 1Hz 频率发送
    DART_INFO_CMD_ID                  = 0x0105,   //飞镖发射相关数据，固定以1Hz频率发送 
    ROBOT_STATUS_CMD_ID               = 0x0201,		//机器人性能体系数据，固定以10Hz 频率发送
    POWER_HEAT_DATA_CMD_ID            = 0x0202,		//实时底盘缓冲能量和射击热量数据，固定以 10Hz 频率发送
    ROBOT_POS_CMD_ID                  = 0x0203,		//机器人位置数据，固定以1Hz频率发送 
    BUFF_CMD_ID                       = 0x0204, 	//机器人增益和底盘能量数据，固定以 3Hz 频率发送
    HURT_DATA_CMD_ID                  = 0x0206,		//伤害状态数据，伤害发生后发送
    SHOOT_DATA_CMD_ID                 = 0x0207,		//实时射击数据，弹丸发射后发送
    PROJECTILE_ALLOWANCE_CMD_ID       = 0x0208,		//允许发弹量，固定以 10Hz 频率发送
    RFID_STATUS_CMD_ID                = 0x0209,   //机器人RFID模块状态，固定以3Hz频率发送
    GROUND_ROBOT_POSITION_CMD_ID      = 0x020B,   //地面机器人位置数据，固定以1Hz频率发送 
    SENTRY_INFO_CMD_ID                = 0x020D,   //哨兵自主决策信息同步，固定以1Hz频率发送
    ROBOT_INTERACTION_CMD_ID          = 0x0301,		//机器人交互数据，发送方触发发送，频率上限为 30Hz
    MAP_COMMAND_CMD_ID                = 0x0303,   //选手端小地图交互数据，选手端触发发送
    CUSTOM_INFO_CMD_ID                = 0x0308,   //选手端小地图接收机器人数据，频率上限为3Hz
    ROBOT_CUSTOM_DATA_2_CMD_ID        = 0x0310,   //机器人发送给自定义终端的数据，频率上限为50Hz
    ROBOT_CUSTOM_DATA_CMD_ID          = 0x0311,   //自定义终端发送给机器人的自定义指令，频率上限为75Hz
}referee_cmd_id_t;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

//解包步骤
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

//解包状态机
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
