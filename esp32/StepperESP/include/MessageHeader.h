// -*- lsst-c++ -*-
#ifndef _MESSAGE_HEADER_H
#define _MESSAGE_HEADER_H

class ESP32MessageHeader
{   
    public:
        ~ESP32MessageHeader() {}
        ESP32MessageHeader() {}

        static const int16_t WRITE_HEADER          = 0x7800;

        static const int16_t SEND_ENCODER          = 0x7857;
        static const int16_t SEND_FULL_SPEED_PROFILE = 0x7869;
        static const int16_t SEND_DEBUG_MSG        = 0x7870;
        static const int16_t SEND_ERROR            = 0x7890;
        static const int16_t SEND_HUMAN_MESSAGE    = 0x7877;
        static const int16_t SEND_INIT_READY       = 0x7855;
        static const int16_t SEND_PARAMS           = 0x7856;
        static const int16_t SEND_RX_MSG_CNT       = 0x7860;
        

        static const int16_t READ_HEADER  = 0x7900;

        static const int16_t SOFT_RESET   = 0x7954;
        static const int16_t HARD_RESET   = 0x7955;
        static const int16_t SHUTDOWN     = 0x7960;

        static const int16_t READ_SPEED_CMD         = 0x7901;
        static const int16_t READ_ACCEL_PROFILE_CFG = 0x7956;
        static const int16_t READ_DYNAMIC_ACCEL_CFG = 0x7957;
        static const int16_t READ_UPDATE_PERIOD_CFG = 0x7958;
        static const int16_t READ_ENCODER_CFG       = 0x7959;
};

#endif