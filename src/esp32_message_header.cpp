#include <boost/python.hpp>
#include <MessageHeader.h>

class ROSInterfaceMessageHeader
{
    public:
        const int16_t READ_HEADER          = ESP32MessageHeader::WRITE_HEADER;

        const int16_t READ_ENCODER          = ESP32MessageHeader::SEND_ENCODER;
        const int16_t READ_ERROR            = ESP32MessageHeader::SEND_ERROR;
        const int16_t READ_HUMAN_MESSAGE    = ESP32MessageHeader::SEND_HUMAN_MESSAGE;
        const int16_t READ_INIT_READY       = ESP32MessageHeader::SEND_INIT_READY;
        const int16_t READ_FULL_SPEED_PROFILE = ESP32MessageHeader::SEND_FULL_SPEED_PROFILE;
        const int16_t READ_DEBUG_MSG        = ESP32MessageHeader::SEND_DEBUG_MSG;
        const int16_t READ_PARAMS           = ESP32MessageHeader::SEND_PARAMS;
        const int16_t READ_RX_MSG_CNT       = ESP32MessageHeader::SEND_RX_MSG_CNT;
        

        const int16_t WRITE_HEADER            = ESP32MessageHeader::READ_HEADER;

        const int16_t SOFT_RESET              = ESP32MessageHeader::SOFT_RESET;
        const int16_t HARD_RESET              = ESP32MessageHeader::HARD_RESET;
        const int16_t SHUTDOWN                = ESP32MessageHeader::SHUTDOWN;

        const int16_t WRITE_SPEED_CMD         = ESP32MessageHeader::READ_SPEED_CMD;
        const int16_t WRITE_ACCEL_PROFILE_CFG = ESP32MessageHeader::READ_ACCEL_PROFILE_CFG;
        const int16_t WRITE_DYNAMIC_ACCEL_CFG = ESP32MessageHeader::READ_DYNAMIC_ACCEL_CFG;
        const int16_t WRITE_UPDATE_PERIOD_CFG = ESP32MessageHeader::READ_UPDATE_PERIOD_CFG;
        const int16_t WRITE_ENCODER_CFG       = ESP32MessageHeader::READ_ENCODER_CFG;
        
};


BOOST_PYTHON_MODULE(esp32_message_header)
{
    using namespace boost::python;
    class_<ROSInterfaceMessageHeader>
    ("ROSInterfaceMessageHeader")
        .def_readonly("READ_HEADER", &ROSInterfaceMessageHeader::READ_HEADER)
        .def_readonly("READ_ENCODER", &ROSInterfaceMessageHeader::READ_ENCODER)
        .def_readonly("READ_FULL_SPEED_PROFILE", &ROSInterfaceMessageHeader::READ_FULL_SPEED_PROFILE)
        .def_readonly("READ_DEBUG_MSG", &ROSInterfaceMessageHeader::READ_DEBUG_MSG)        
        .def_readonly("READ_PARAMS", &ROSInterfaceMessageHeader::READ_PARAMS)
        .def_readonly("READ_ERROR", &ROSInterfaceMessageHeader::READ_ERROR)
        .def_readonly("READ_HUMAN_MESSAGE", &ROSInterfaceMessageHeader::READ_HUMAN_MESSAGE)
        .def_readonly("READ_INIT_READY", &ROSInterfaceMessageHeader::READ_INIT_READY)
        .def_readonly("READ_RX_MSG_CNT", &ROSInterfaceMessageHeader::READ_RX_MSG_CNT)
        .def_readonly("WRITE_HEADER", &ROSInterfaceMessageHeader::WRITE_HEADER)
        .def_readonly("SOFT_RESET", &ROSInterfaceMessageHeader::SOFT_RESET)
        .def_readonly("HARD_RESET", &ROSInterfaceMessageHeader::HARD_RESET)
        .def_readonly("SHUTDOWN", &ROSInterfaceMessageHeader::SHUTDOWN)
        .def_readonly("WRITE_SPEED_CMD", &ROSInterfaceMessageHeader::WRITE_SPEED_CMD)
        .def_readonly("WRITE_ACCEL_PROFILE_CFG", &ROSInterfaceMessageHeader::WRITE_ACCEL_PROFILE_CFG)
        .def_readonly("WRITE_DYNAMIC_ACCEL_CFG", &ROSInterfaceMessageHeader::WRITE_DYNAMIC_ACCEL_CFG)
        .def_readonly("WRITE_UPDATE_PERIOD_CFG", &ROSInterfaceMessageHeader::WRITE_UPDATE_PERIOD_CFG)
        .def_readonly("WRITE_ENCODER_CFG", &ROSInterfaceMessageHeader::WRITE_ENCODER_CFG)
    ;
}