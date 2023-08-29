#pragma once
#include "../motor.h"
#include <iostream>
#include <memory>

namespace serial {
    class Serial;
};

namespace Motor
{
    namespace CMD
    {
        namespace RMDS
        {
            static const size_t IDX_FRAME_HEAD = 0;
            static const size_t IDX_CMD = 1;
            static const size_t IDX_ID = 2;
            static const size_t IDX_DATA_LEN = 3;
            static const size_t IDX_HEAD_CHK = 4;
            static const size_t IDX_DATA_START = 5;
            static const size_t FRAME_LEN = 5;
            static const uint8_t FRAME_HEAD = 0x3E;

            static const double MAX_SPEED = 20000.0;

            // Control
            static const uint8_t MOTOR_START = 0x88;
            static const uint8_t MOTOR_PAUSE = 0x81;
            static const uint8_t MOTOR_SHUTDOWN = 0x80;

            // R: Read, W: Write
            // [R/W]_[Target]_[Dst]
            // PID
            static const uint8_t R_PID = 0x30;
            static const uint8_t W_PID_RAM = 0x31;
            static const uint8_t W_PID_ROM = 0x32;

            // ACC
            static const uint8_t R_ACC = 0x33;
            static const uint8_t W_ACC = 0x34;

            // Encoder
            static const uint8_t R_ENCODER = 0x90;
            static const uint8_t W_ENCODER_ROM = 0x91;
            static const uint8_t W_CURPOS_ROM = 0x19;

            // Angle
            static const uint8_t R_MULTILOOPANG = 0x92;  // 当前电机的多圈绝对值角度
            static const uint8_t R_SINGALLOOPANG = 0x94; // 当前电机的单圈绝对值角度
            static const uint8_t W_DEFANG = 0x95;

            // Error & State
            static const uint8_t R_ERROR = 0x9A;
            static const uint8_t W_ERROR = 0x9B;
            static const uint8_t R_STATE1 = 0x9A;
            static const uint8_t R_STATE2 = 0x9C;
            static const uint8_t R_STATE3 = 0x9D;

            // Close Loop Control
            // 输出指定功率
            static const uint8_t W_POWER = 0xA0;
            // 输出指定速度
            static const uint8_t W_SPEED = 0xA2;

            // Angle Control (1：默认速度，2：指定速度)
            // 旋转到特定角度，这里有考虑从上电开始电机全部旋转角度
            // 比如电机上电后旋转四圈后使用下面指令旋转到 0°，会让电机往回转四圈
            // -360 ~ 360
            static const uint8_t W_MLANG1 = 0xA3;
            static const uint8_t W_MLANG2 = 0xA4;
            // 旋转到特定角度，不考虑电机已经旋转了多少圈
            // 比如电机上电后旋转了四圈多 10°
            // 在不指定倒转的情况下会旋转 350° 变成第五圈的 0 °
            // 0 ~ 360 (允许倒转)
            static const uint8_t W_SLANG1 = 0xA5;
            static const uint8_t W_SLANG2 = 0xA6;
            // 从当前位置开始再选转指定角度
            // -360 ~ 360
            static const uint8_t W_ADDANG1 = 0xA7; // 默认速度旋转指定角度 0 ~ 360
            static const uint8_t W_ADDANG2 = 0xA8; // 指定速度旋转指定角度 0 ~ 360

            static const uint8_t R_INFO = 0x12;
        }
    }

    class RMDS : public Motor
    {
    public:
        RMDS() = delete;
        RMDS(std::string dev, uint8_t id);
        virtual ~RMDS();

        // speed (degree / s)
        // 如果 speed = -1, 电机会按照上一次速度 m_speed 运行
        // m_speed 不会是 0
        virtual bool rotate(double speed) override;
        virtual bool rotate() override
        {
            return rotate(m_speed);
        }

        virtual bool pause() override;

        // degree: 0 ~ 360
        // speed: -2000 ~ 2000 (degree / s)
        virtual bool rotateTo(double degree, double speed) override;
        // degree: 0 ~ 360
        virtual bool rotateTo(double degree) override
        {
            return rotateTo(degree, m_speed);
        };

        // degree: -360 ~ 360
        // speed: -2000 ~ 2000 (degree / s)
        virtual bool rotateMore(double degree, double speed) override;
        // degree: -360 ~ 360
        virtual bool rotateMore(double degree) override
        {
            return rotateMore(degree, m_speed);
        };

        double getCurrentPose();

        // 这个系列电机独有的多圈工作模式, 不过其记忆的位置都是上电之后的, 这个需要注意
        bool rotateMTo(double degree, double speed);
        bool rotateMTo(double degree)
        {
            return rotateMTo(degree, m_speed);
        }

        bool rotateMMore(double degree, double speed);
        bool rotateMMore(double degree)
        {
            return rotateMMore(degree, m_speed);
        }

        double getCurrentMPose();

    private:
        bool send();

        inline void updateSpeed(double &speed)
        {
            if (speed != 0)
                m_speed = speed;
        }

    private:
        uint8_t m_id;
        std::unique_ptr<serial::Serial> m_sender;
    };

}