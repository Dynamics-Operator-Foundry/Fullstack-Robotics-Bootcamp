#define POT_RAW_TOPIC 60000
#define PULSE_RAW_TOPIC 60001
#define JOINT_CMD_TOPIC 60002
#define JOINT_DEG_TOPIC 60003

#define CTRL_MODE 0
#define TWIN_MODE 1
#define CALI_MODE 2

struct fredo_msg{
    double time = 0;
    double joint1 = 0;
    double joint2 = 0;
    double joint3 = 0;
};
