#pragma once

#include <cstdint>
#include "sensor.hpp"

enum class SensorFSM {INIT, READ, PROCESS, IDLE, ERROR};

template<typename LedT, typename UartT>
void fsm_step(SensorFSM& st, Isensor& sensor, LedT& Led, UartT& Uart) {

    uint32_t measurement = 0;
    uint32_t sensor_offset = (1 << 11); //Sensor 0 value (rescales to signed range)
    SensorError err = SensorError::None;

    switch (st) {
        case SensorFSM::INIT :
            {
                err = sensor.init();
                if (err == SensorError::None) {
                    Uart.send("Sensor initialization successful\r\n\0");
                    st = SensorFSM::READ;
                } else {
                    Uart.send("Sensor initialization error\r\n\0");
                    st = SensorFSM::ERROR;
                }
                break;
            }

            
        case SensorFSM::READ :
            {
                err = sensor.read(measurement);
                if(err == SensorError::None) {
                    st = SensorFSM::PROCESS;
                } else {
                    blink_led(Led,50,450); //Slow blink
                    st = SensorFSM::ERROR;
                }


                break;
            }
        case SensorFSM::PROCESS :
            {
            
                int16_t val = static_cast<int32_t>(measurement - sensor_offset);
                Uart.send("Sensor value [deg C]: \0");
                Uart.sendInt(val);
                st = SensorFSM::IDLE;
                break;
            }
        case SensorFSM::IDLE :
            {
                blink_led(Led, 20, 980);
                st = SensorFSM::READ;
                break;
            }
        case SensorFSM::ERROR :
            {
                blink_led(Led, 200, 200);
                switch(err) {
                    case SensorError::Communication :
                        {
                        Uart.send("Sensor communication error!\r\n\0");
                        break;
                        }
                    case SensorError::InvalidData :
                        {
                        Uart.send("Sensor data are not valid!\r\n\0");
                        break;
                        }
                    case SensorError::SensorID :
                        {
                        Uart.send("Sensor ID doesn't match!\r\n\0");
                        break;
                        }
                    case SensorError::Timeout :
                        {
                        Uart.send("Sensor communication timeout!\r\n\0");
                        break;
                        }
                    default :
                        {
                        Uart.send("Invalid sensor operation!\r\n\0");
                        break;
                        }
                    }
                    break;
                }

            
        default :
            {
            st = SensorFSM::ERROR; //Invalid transition
            break;
            }
    }
}

//Helper function to blink LED on a given rate
static void blink_led(auto& Led, uint32_t on_ms, uint32_t off_ms) {
    static uint32_t last = 0;
    static bool on = false;

    if (on) {
        if (due(last,on_ms)) {
            Led.clear();
            on = false;
        }
    }else {
        if (due(last,off_ms)) {
            Led.set();
            on = true;
        }
    }
}