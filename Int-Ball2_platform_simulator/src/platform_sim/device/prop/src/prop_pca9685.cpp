
#include "prop/prop_pca9685.h"


//------------------------------------------------------------------------------
// コンストラクタ
PropPCA9685::PropPCA9685():
is_file_open_(false)
{}

//------------------------------------------------------------------------------
// デストラクタ
PropPCA9685::~PropPCA9685()
{
    shutdown();
}

//------------------------------------------------------------------------------
// I2C通信初期化
int PropPCA9685::initialize(const char*& file_name, const int& addr, const unsigned short& freq)
{
#ifdef WITH_PCA9685

    // デバイスファイルオープン
    if((file_descriptor_ = open(file_name, O_RDWR)) < 0)
    {
        return 0x01;
    }
    is_file_open_ = true;

    // I2C SLAVEのアドレス設定
    if(ioctl(file_descriptor_, I2C_SLAVE, addr)  < 0)
    {
        close(file_descriptor_);
        return 0x02;
    }

    // 初期化
    i2cWrite(REG_MODE1, CMD_RESTART);
    i2cWrite(REG_MODE2, CMD_TOTEM_POLE);

    // 周波数設定
    setFrequency(freq);

#endif

    return 0;
}

//------------------------------------------------------------------------------
// I2C 1Byteデータ送信
bool PropPCA9685::i2cWrite(const unsigned char& reg, const unsigned char& data)
{
#ifdef WITH_PCA9685

    unsigned char buff[2];
    buff[0] = reg;
    buff[1] = data;

    if(write(file_descriptor_, buff, 2) != 2)
    {
        return false;
    }

#endif

    return true;
}

//------------------------------------------------------------------------------
// PWM周波数設定
void PropPCA9685::setFrequency(const unsigned short& frequency)
{
    // PWM frequency PRE_SCALE
    unsigned char  pre_scale = DEFAULT_PRE_SCALE;
    unsigned short freq      = limitter(frequency, FREQ_MIN, FREQ_MAX);

    int val   = static_cast<int>(OSCILLATOR_CLOCK / static_cast<double>(RESOLUTION * freq) + 0.5 + EPS) - 1;
    pre_scale = static_cast<unsigned char>(val);

    // RESTART MODE
    i2cWrite(REG_MODE1,     CMD_SLEEP);
    i2cWrite(REG_PRE_SCALE, pre_scale);
    i2cWrite(REG_MODE1,     CMD_RESTART);
}

//------------------------------------------------------------------------------
// デューティ比からPWM制御信号へ変換する
void PropPCA9685::convertDutyCommand(const float& duty, unsigned char *command)
{
    float          ld  = limitter(duty, DUTY_MIN, DUTY_MAX);
    int            val = static_cast<int>(ld * (RESOLUTION - 1) + 0.5 + EPS);
    unsigned short pwm = static_cast<unsigned short>(val);
    unsigned short dly = CMD_DELAY;

    command[LED_ON_LOW]   = 0x00FF & dly;
    command[LED_ON_HIGH]  = dly >> 8;
    command[LED_OFF_LOW]  = 0x00FF & pwm;
    command[LED_OFF_HIGH] = pwm >> 8;
}

//------------------------------------------------------------------------------
// PWM制御ボードにPWM制御信号を送信する
void PropPCA9685::setPWM(const std::vector<float>& duty)
{
    int size = (duty.size() <= CH_MAX)? duty.size() : CH_MAX;

    for(int fan_id = 0; fan_id < size; fan_id++)
    {
        unsigned char reg1  = fan_id * REG_OFFSET + LED_ON_L;
        unsigned char reg2  = fan_id * REG_OFFSET + LED_ON_H;
        unsigned char reg3  = fan_id * REG_OFFSET + LED_OFF_L;
        unsigned char reg4  = fan_id * REG_OFFSET + LED_OFF_H;

        unsigned char command[4];
        convertDutyCommand(duty[fan_id], command);

        i2cWrite(reg1,command[0]);
        i2cWrite(reg2,command[1]);
        i2cWrite(reg3,command[2]);
        i2cWrite(reg4,command[3]);
    }
}

//------------------------------------------------------------------------------
// I2C通信停止
void PropPCA9685::shutdown()
{
    if(is_file_open_)
    {
        std::cout << " -> Close PCA9685 Device File" << std::endl;
        is_file_open_ = (close(file_descriptor_) == 0);
    }
}

// End Of File -----------------------------------------------------------------
