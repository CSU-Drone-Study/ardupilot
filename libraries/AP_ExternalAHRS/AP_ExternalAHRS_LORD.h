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

#pragma once

#include "AP_ExternalAHRS_backend.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_LORD: public AP_ExternalAHRS_backend {
public:

    AP_ExternalAHRS_LORD(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(mavlink_channel_t chan) const override;

    // check for new data
    void update() override {
        build_packet();
    };

private:

    enum class DescriptorSet {
        BaseCommand = 0x01,
        DMCommand = 0x0C,
        SystemCommand = 0x7F,
        IMUData = 0x80,
        GNSSData = 0x81,
        EstimationData = 0x82
    };

    enum class ParseState {
        WaitingFor_SyncOne,
        WaitingFor_SyncTwo,
        WaitingFor_Descriptor,
        WaitingFor_PayloadLength,
        WaitingFor_Data,
        WaitingFor_Checksum
    };

    void update_thread();

    AP_HAL::UARTDriver *uart;
    uint32_t baudrate;
    int8_t port_num;
    bool portOpened = false;

    const uint8_t SYNC_ONE = 0x75;
    const uint8_t SYNC_TWO = 0x65;

    struct LORD_Packet {
        uint8_t header[4];
        uint8_t payload[256];
        uint8_t checksum[2];
    };
    
    struct {
        LORD_Packet packet;
        ParseState state;
        uint8_t index;
    } message_in;

    struct {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        Quaternion quat;
        float pressure;
    } imu_data;

    void send_config();
    
    void build_packet();
    bool valid_packet(LORD_Packet &packet);
    void handle_packet(LORD_Packet &packet);
    void handle_imu(LORD_Packet &packet);
    void handle_gnss(LORD_Packet &packet);
    void post_imu();

    Vector3f populate_vector(uint8_t* data, uint8_t offset);
    Quaternion populate_quaternion(uint8_t* data, uint8_t offset);
    float extract_float(uint8_t* data, uint8_t offset);
    double extract_double(uint8_t* data, uint8_t offset);

};


#endif // HAL_EXTERNAL_AHRS_ENABLED