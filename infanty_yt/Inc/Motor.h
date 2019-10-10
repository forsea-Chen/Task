/**
 * @file motor.h
 * @author BigeYoung, M3chD09
 * @brief  电机库
 * @version 1.0
 * @date Jun 26, 2019
 */

#pragma once

#ifdef __cplusplus

#include "stm32f4xx_hal.h"
#include "PID.h"
#include "drv_can.h"
#include "main.h"
class MotorBase
{
public:
    MotorBase(uint8_t id) : ID(id) {}
    virtual ~MotorBase() {}

    virtual void update(uint8_t can_rx_data[]) = 0;
    virtual bool CheckID(uint32_t StdID) const { return StdID == this->REC_ID_INIT() + (uint32_t)ID; }
    float getAngle() const { return angle; }

    const uint8_t ID = 0;

    float Out = 0; //输出到电调的电流值

protected:
    float angle = 0;

    virtual uint32_t REC_ID_INIT() const { return 0; };

    void update_angle(uint8_t can_rx_data[])
    {
        last_encoder = encoder;
        encoder = (uint16_t)(can_rx_data[0] << 8 | can_rx_data[1]);
        msg_cnt++;
        if (msg_cnt < 50)
        {
            encoder_offset = encoder;
            return;
        }
        if (this->encoder - this->last_encoder > 4096)
            this->round_cnt--;
        else if (this->encoder - this->last_encoder < -4096)
            this->round_cnt++;
        int32_t total_encoder = round_cnt * 8192 + encoder - encoder_offset;
        angle = total_encoder / ENCODER_ANGLE_RATIO();
    }

private:
    uint16_t encoder = 0, last_encoder = 0, encoder_offset = 0, msg_cnt = 0;
    int32_t round_cnt = 0;
    virtual int16_t ENCODER_MAX() const { return 8192; }
    virtual float ENCODER_ANGLE_RATIO() const { return 8192.0f / 360.0f; }
    virtual float MAX_CURRENT() const { return 65535; }
};

class MotorSpeed : public MotorBase
{
public:
    MotorSpeed(uint8_t id) : MotorBase(id) {}
    virtual ~MotorSpeed(){};
    virtual float getSpeed() const { return this->speed; }
    virtual void update(uint8_t can_rx_data[]) = 0;

protected:
    int16_t speed = 0;
    void update_speed(uint8_t can_rx_data[])
    {
      this->speed = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }
};

class Motor_820R : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x200; }
    virtual uint32_t SEND_ID_LOW() const { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x1ff; }
    virtual float MAX_CURRENT() const { return 16384; }
    Motor_820R(uint8_t id) : MotorSpeed(id) {}

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
    }
};

class Motor_GM3510 : public MotorBase
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x204; }
    virtual uint32_t SEND_ID_LOW() const { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() const { return 0; }
    virtual float MAX_CURRENT() const { return 29000; }
    Motor_GM3510(uint8_t id) : MotorBase(id) {}

    int16_t getTorque() const { return torque; }

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        torque = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }

protected:
    int16_t torque = 0;
};

class Motor_6623 : public MotorBase
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x204; }
    virtual uint32_t SEND_ID_LOW() const { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x2ff; }
    virtual float MAX_CURRENT() const { return 5000; }
    enum
    {
        Yaw = 1,
        Pitch,
        Roll,
        Resv,
        Ex1,
        Ex2,
        Ex3,
        Ex4
    };
    Motor_6623(uint8_t id) : MotorBase(id) {}

    int16_t getTorque() const { return torque; }

    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        torque = (int16_t)(can_rx_data[2] << 8 | can_rx_data[3]);
    }

protected:
    int16_t torque = 0;
};

class Motor_C610 : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x200; }
    virtual uint32_t SEND_ID_LOW() const { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x1ff; }
    virtual float MAX_CURRENT() const { return 10000; }
    Motor_C610(uint8_t id) : MotorSpeed(id) {}
    virtual void update(uint8_t can_rx_data[]) override
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->torque = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
    }
    int16_t getTorque() const { return torque; }

protected:
    int16_t torque = 0;
};

class Motor_C620 : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x200; }
    virtual uint32_t SEND_ID_LOW() const { return 0x200; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x1ff; }
    virtual float MAX_CURRENT() const { return 16384; }
    Motor_C620(uint8_t id) : MotorSpeed(id) {}
    uint8_t getTempature() const { return temperature; }
    int16_t getGivenCurrent() const { return givenCurrent; }

    virtual void update(uint8_t can_rx_data[])
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        givenCurrent = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        temperature = can_rx_data[6];
    }

protected:
    uint8_t temperature = 0;
    int16_t givenCurrent = 0;
};

class Motor_GM6020 : public MotorSpeed
{
public:
    virtual uint32_t REC_ID_INIT() const { return 0x204; }
    virtual uint32_t SEND_ID_LOW() const { return 0x1ff; }
    virtual uint32_t SEND_ID_HIGH() const { return 0x2ff; }
    virtual float MAX_CURRENT() const { return 30000; }
    Motor_GM6020(uint8_t id) : MotorSpeed(id) {}
    virtual void update(uint8_t can_rx_data[])
    {
        this->update_angle(can_rx_data);
        this->update_speed(can_rx_data);
        this->givenCurrent = (int16_t)(can_rx_data[4] << 8 | can_rx_data[5]);
        this->temperature = can_rx_data[6];
    }

protected:
    uint8_t temperature = 0;
    int16_t givenCurrent = 0;
};

template <class MotorType, int N>
void MotorMsgSend(CAN_HandleTypeDef *hcan, MotorType (&motors)[N])
{
    uint8_t TxDataLow[8] = {0};
    uint8_t TxDataHigh[8] = {0};
    bool low = false;
    bool high = false;
    for (int i = 0; i < N; i++)
    {
        int16_t out = Constrain(motors[i].Out, -motors[0].MAX_CURRENT(), motors[0].MAX_CURRENT());
        if (motors[i].ID <= 4 && motors[i].ID > 0)
        {
            low = true;
            TxDataLow[motors[i].ID * 2 - 2] = (out >> 8) & 0xff;
            TxDataLow[motors[i].ID * 2 - 1] = out & 0xff;
        }
        else if (motors[i].ID < 8 && motors[i].ID > 4)
        {
            high = true;
            TxDataLow[motors[i].ID * 2 - 10] = (out >> 8) & 0xff;
            TxDataLow[motors[i].ID * 2 - 9] = out & 0xff;
        }
    }

    if (low)
        can_msg_bytes_send(hcan, TxDataLow, 8, motors[0].SEND_ID_LOW());

    if (high)
        can_msg_bytes_send(hcan, TxDataHigh, 8, motors[0].SEND_ID_HIGH());
}

template <class MotorType>
void MotorMsgSend(CAN_HandleTypeDef *hcan, MotorType &motor)
{
    MotorType motor_arr[1] = {motor};
    MotorMsgSend(hcan, motor_arr);
}

#endif
