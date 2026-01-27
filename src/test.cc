#include <serial/serial.h>

#include "SerialTask/SerialSend.hpp"
#include "SerialTask/SerialRead.hpp"





int main(){
    serial::Serial serial_port;
    SerialTask::DefaultConfig(serial_port);
    serial_port.open();

    current_angles = SerialTask::GetAnglesNow(serial_port);
    
    offset_angles = offset_calculator.CalculateOffsetAngles(last_predict_center, last_w, last_h, has_detection);

    SerialTask::SerialSend(*serial_port, current_angles, pitch_delta, yaw_delta);
}