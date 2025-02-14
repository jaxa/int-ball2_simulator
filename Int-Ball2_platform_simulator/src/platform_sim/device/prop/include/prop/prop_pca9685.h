
#pragma once
#include <iostream>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include "prop/prop_common.h"

#define EPS                1.0E-10
#define REG_MODE1          0x00			// Resigter Number of Mode register 1
#define REG_MODE2          0x01			// Resigter Number of Mode register 2
#define REG_PRE_SCALE      0xFE			// Register Number of prescaler for PWM Output frequency
#define REG_OFFSET         0x04
#define LED_ON_L           0x06
#define LED_ON_H           0x07
#define LED_OFF_L          0x08
#define LED_OFF_H          0x09
#define CMD_SLEEP          0x10
#define CMD_RESTART        0x80
#define CMD_TOTEM_POLE     0x04
#define RESOLUTION         4096
#define FREQ_MIN           24			// [Hz]
#define FREQ_MAX           1526			// [Hz]
#define DUTY_MIN           0.F			// [-]
#define DUTY_MAX           1.F			// [-]
#define CH_MAX             16           // PCA9685のチャネル数
#define DEFAULT_PRE_SCALE  0x001E	 	// 200[Hz]
#define OSCILLATOR_CLOCK   25000000		//  25[MHz]
#define CMD_DELAY          0			// Delay(ON Timing(0-4095))
#define LED_ON_LOW         0			// ON  Timing(Byte 0)
#define LED_ON_HIGH        1			// ON  Timing(Byte 1)
#define LED_OFF_LOW        2			// OFF Timing(Byte 2)
#define LED_OFF_HIGH       3			// OFF Timing(Byte 3)

/**
* @brief 推進機能ノード　PWM制御信号送信クラス
*/
class PropPCA9685
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** コンストラクタ */
	explicit PropPCA9685();

	/** デストラクタ */
	~PropPCA9685();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	PropPCA9685(const PropPCA9685&)            = delete;

	/** コピー代入演算子. */
	PropPCA9685& operator=(const PropPCA9685&) = delete;

	/** ムーブコンストラクタ. */
	PropPCA9685(PropPCA9685&&)                 = delete;

	/** ムーブ代入演算子. */
	PropPCA9685& operator=(PropPCA9685&&)      = delete;

	//----------------------------------------------------------------------
	// 実装
public:
	/** I2C通信初期化
	 * @param [in]      fileName   デバイスファイル名(PCA9685)  　
	 * @param [in]      addr       I2C SLAVEのアドレス
	 * @param [in]      freq       PWM周波数[Hz]
	 * @return                     エラー識別子(0: 成功  1: ファイルオープン失敗  2: I2C SLAVEアドレス設定失敗)
	 */
	int  initialize(const char*& file_name, const int& addr, const unsigned short& freq);

	/** I2c通信停止 */
	void shutdown();

	/** PWM制御信号送信
	 * @param [in]      duty       ファン駆動デューティ比
	 */
	void setPWM(const std::vector<float>& duty);

	//--------------------------------------------------------------------------
	// 実装
private:
	/** PWM周波数設定
	 * @param [in]      frequency   PWM周波数[Hz]　
	 */
	void setFrequency(const unsigned short& frequency);

	/** I2C 1Byteデータ送信
	 * @param [in]      reg        レジスタアドレス
	 * @param [in]      data       データ
	 * @return                     エラーフラグ(true: 成功  false: 送信失敗)
	 */
	bool i2cWrite(const unsigned char& reg, const unsigned char& data);

	/** デューティ比からPWM制御信号へ変換
	 * @param [in]      duty       ファン駆動デューティ比
	 * @param [in]      command    PWM制御信号
	 */
	void convertDutyCommand(const float& duty, unsigned char *command);
	
	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ファイルディスクリプタ */
	int                              file_descriptor_;

	/** ファイルオープン判定フラグ */
	bool                             is_file_open_;
	
};

// End Of File -----------------------------------------------------------------
