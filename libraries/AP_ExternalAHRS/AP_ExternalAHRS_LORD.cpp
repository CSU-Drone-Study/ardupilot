/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  suppport for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_LORD.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LORD::AP_ExternalAHRS_LORD(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    // uart = hal.serial(1);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        hal.console->printf("LORD IS NOT CONNECTED ANYMORE\n");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD ExternalAHRS initialised");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }

    baudrate = 115200;
}

void AP_ExternalAHRS_LORD::update_thread()
{
    if(!portOpened) {
        portOpened = true;
        uart->begin(baudrate);
        hal.scheduler->delay(1000);
    }

    while(true) {
        build_packet();
        hal.scheduler->delay(1);
    }
}


//use all available bytes to continue building packets where we left off last loop
void AP_ExternalAHRS_LORD::build_packet() {
    uint64_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t b = uart->read();
        switch (message_in.state) {
            default:
            case ParseState::WaitingFor_SyncOne:
                if (b == SYNC_ONE) {
                    message_in.packet.header[0] = 0x75;
                    message_in.state = ParseState::WaitingFor_SyncTwo;
                } break;
            case ParseState::WaitingFor_SyncTwo:
                if (b == SYNC_TWO) {
                    message_in.packet.header[1] = 0x65;
                    message_in.state = ParseState::WaitingFor_Descriptor;
                } break;
            case ParseState::WaitingFor_Descriptor:
                message_in.packet.header[2] = b;
                message_in.state = ParseState::WaitingFor_PayloadLength;
                break;
            case ParseState::WaitingFor_PayloadLength:
                message_in.packet.header[3] = b;
                message_in.state = ParseState::WaitingFor_Data;
                break;
            case ParseState::WaitingFor_Data:
                message_in.packet.payload[message_in.index++] = b;
                if (message_in.index >= message_in.packet.header[3]) {
                    message_in.state = ParseState::WaitingFor_Checksum;
                    message_in.index = 0;
                }
                break;
            case ParseState::WaitingFor_Checksum:
                message_in.packet.checksum[message_in.index++] = b;
                if (message_in.index >= 2) {
                    message_in.state = ParseState::WaitingFor_SyncOne;
                    message_in.index = 0;

                    if (valid_packet(message_in.packet)) {
                        handle_packet(message_in.packet);
                    }

                }

                break;
        }
        
    }
}

//gets checksum and compares it to curr packet
bool AP_ExternalAHRS_LORD::valid_packet(LORD_Packet &packet) {
    uint8_t checksumByte1 = 0;
    uint8_t checksumByte2 = 0;

    for (int i = 0; i < 4; i++) {
        checksumByte1 += packet.header[i];
        checksumByte2 += checksumByte1;
    }

    for (int i = 0; i < packet.header[3]; i++) {
        checksumByte1 += packet.payload[i];
        checksumByte2 += checksumByte1;
    }

    return packet.checksum[0] == checksumByte1 && packet.checksum[1] == checksumByte2;
}

void AP_ExternalAHRS_LORD::handle_packet(LORD_Packet &packet) {
    switch ((DescriptorSet) packet.header[2]) {
        case DescriptorSet::IMUData:
            handle_imu(packet);
            break;
        case DescriptorSet::GNSSData:
            handle_gnss(packet);
            break;
        case DescriptorSet::EstimationData:
        case DescriptorSet::BaseCommand:
        case DescriptorSet::DMCommand:
        case DescriptorSet::SystemCommand:
            break;
        
    }
}

void AP_ExternalAHRS_LORD::handle_imu(LORD_Packet &packet) {
    for (uint8_t i = 0 ; i < packet.header[3] ; i += packet.payload[i]) {
        switch (packet.payload[i]) {
            // Scaled Ambient Pressure
            case 0x17:
                break;
            // Scaled Magnetometer Vector
            case 0x06:
                break;
            // Scaled Accelerometer Vector
            case 0x04:
                break;
            // Scaled Gyro Vector
            case 0x05:
                break;
            // CF Quaternion
            case 0x0A:
                break;

        }
    }
}

void AP_ExternalAHRS_LORD::handle_gnss(LORD_Packet &packet) {
    for (uint8_t i = 0 ; i < packet.header[3] ; i += packet.payload[i]) {
        switch (packet.payload[i]) {
            // GPS Time
            case 0x09:
                break;
            // GNSS Fix Information
            case 0x0B:
                break;
            // LLH Position
            case 0x03:
                break;
            // DOP Data
            case 0x07:
                break;
            // NED Velocity
            case 0x05:
                break;
        }
    }
}


int8_t AP_ExternalAHRS_LORD::get_port(void) const {
    return 4;
};

bool AP_ExternalAHRS_LORD::healthy(void) const
{
    return true;
}

bool AP_ExternalAHRS_LORD::initialised(void) const
{
    return true;
}

bool AP_ExternalAHRS_LORD::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    return true;
}

void AP_ExternalAHRS_LORD::get_filter_status(nav_filter_status &status) const
{
    return;
}

void AP_ExternalAHRS_LORD::send_status_report(mavlink_channel_t chan) const
{
    return;
}

Vector3f AP_ExternalAHRS_LORD::populateVector3f(const uint8_t* pkt, uint8_t offset, float multiplier) {
    Vector3f data;
    uint32_t tmp[3];
    for (uint8_t j = 0; j < 3; j++) {
        tmp[j] = get4ByteField(pkt, offset + j * 4 + 2);
    }
    data.x = *reinterpret_cast<float*>( &tmp[0] );
    data.y = *reinterpret_cast<float*>( &tmp[1] );
    data.z = *reinterpret_cast<float*>( &tmp[2] );
    return data * multiplier;
}

Quaternion AP_ExternalAHRS_LORD::populateQuaternion(const uint8_t* pkt, uint8_t offset) {
    uint32_t tmp[4];
    for (uint8_t j = 0; j < 4; j++) {
        tmp[j] = get4ByteField(pkt, offset + j * 4 + 2);
    }

    Quaternion x;
    x.q1 = *reinterpret_cast<float*>( &tmp[0] );
    x.q2 = *reinterpret_cast<float*>( &tmp[1] );
    x.q3 = *reinterpret_cast<float*>( &tmp[2] );
    x.q4 = *reinterpret_cast<float*>( &tmp[3] );

    return x;
}

uint64_t AP_ExternalAHRS_LORD::get8ByteField(const uint8_t* pkt, uint8_t offset) {
    uint64_t res = 0;
    // for (int i = 0; i < 2; i++)
    //     res = res << 32 | get4ByteField(pkt,offset + 4 * i);
    memmove(&res, pkt+offset, 8);
    if (char(1) == 1)
        res = ((res & 0xff) << 56) | ((res & 0xff00000000000000) >> 56) | ((res & 0xff00) << 40) | ((res & 0xff000000000000) >> 40) | ((res & 0xff0000) << 24) | ((res & 0xff0000000000) >> 24) | ((res & 0xff000000) << 8) | ((res & 0xff00000000) >> 8);
    return res;
}

uint32_t AP_ExternalAHRS_LORD::get4ByteField(const uint8_t* pkt, uint8_t offset) {
    uint32_t res = 0;
    // for (int i = 0; i < 2; i++)
    //     res = res << 16 | get2ByteField(pkt, offset + 2 * i);
    memmove(&res, pkt+offset, 4);
    if (char(1) == 1) // Is the device little endian (need to convert big endian packet field to little endian)
        res = ((res & 0xff) << 24) | ((res & 0xff000000) >> 24) | ((res & 0xff00) << 8) | ((res & 0xff0000) >> 8);
    return res;
}

uint16_t AP_ExternalAHRS_LORD::get2ByteField(const uint8_t* pkt, uint8_t offset) {
    uint16_t res = 0;
    // for (int i = 0; i < 2; i++)
    //     res = res << 8 | pkt[offset + i];
    memmove(&res, pkt+offset, 2);
    if (char(1) == 1)
        res = ((res & 0xff) << 8) | ((res & 0xff00) >> 8);
    return res;
}



#endif  // HAL_EXTERNAL_AHRS_ENABLED