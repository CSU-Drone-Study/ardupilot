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
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <AP_HAL/utility/sparse-endian.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LORD::AP_ExternalAHRS_LORD(AP_ExternalAHRS *_frontend,
    AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state) {
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler -> thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD ExternalAHRS initialised");

}

void AP_ExternalAHRS_LORD::update_thread() {
    if (!portOpened) {
        portOpened = true;
        uart -> begin(baudrate);
        send_config();
    }

    while (true) {
        read_imu();
        build_packet();
        hal.scheduler -> delay(1);
    }
}

const uint8_t config_packet[] = { 0x75, 0x65, 0xC, 0x47, 0x13, 0x8, 0x1, 0x5, 0x17, 0x0, 0xA, 0x6, 0x0, 0xA, 0x4, 0x0, 0xA, 0x5, 0x0, 0xA, 0xA, 0x0, 0xA, 0x13, 0x9, 0x1, 0x5, 0x9, 0x0, 0x1, 0xB, 0x0, 0x1, 0x3, 0x0, 0x1, 0x7, 0x0, 0x1, 0x5, 0x0, 0x1, 0x3, 0x8, 0x3, 0x3, 0x9, 0x3, 0x5, 0x11, 0x1, 0x1, 0x1, 0x5, 0x11, 0x1, 0x2, 0x1, 0x5, 0x11, 0x1, 0x3, 0x0, 0x4, 0x11, 0x3, 0x1, 0x4, 0x11, 0x3, 0x2, 0x4, 0x11, 0x3, 0x3, 0xB2, 0x74, };

void AP_ExternalAHRS_LORD::send_config() {
    uart->write((const char*) config_packet);
}

// Read all available bytes into ring buffer
void AP_ExternalAHRS_LORD::read_imu() {
    uint32_t amountRead = uart -> read(tempData, bufferSize);
    buffer.write(tempData, amountRead);
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_LORD::build_packet() {
    uint64_t nbytes = buffer.available();
    while (nbytes-- > 0) {
        uint8_t* b = 0x00;
        bool goodRead = buffer.read_byte(b);
        if(!goodRead)
            continue;
        switch (message_in.state) {
            default:
            case ParseState::WaitingFor_SyncOne:
                hal.console->printf("1\n");
                if (*b == SYNC_ONE) {
                    message_in.packet.header[0] = *b;
                    message_in.state = ParseState::WaitingFor_SyncTwo;
                }
                break;
            case ParseState::WaitingFor_SyncTwo:
                hal.console->printf("2\n");
                if (*b == SYNC_TWO) {
                    message_in.packet.header[1] = *b;
                    message_in.state = ParseState::WaitingFor_Descriptor;
                }
                else {
                    message_in.state = ParseState::WaitingFor_SyncOne;
                }
                break;
            case ParseState::WaitingFor_Descriptor:
                hal.console->printf("3\n");
                message_in.packet.header[2] = *b;
                message_in.state = ParseState::WaitingFor_PayloadLength;
                break;
            case ParseState::WaitingFor_PayloadLength:
                hal.console->printf("4\n");
                message_in.packet.header[3] = *b;
                message_in.state = ParseState::WaitingFor_Data;
                break;
            case ParseState::WaitingFor_Data:
                hal.console->printf("5\n");
                message_in.packet.payload[message_in.index++] = *b;
                if (message_in.index >= message_in.packet.header[3]) {
                    message_in.state = ParseState::WaitingFor_Checksum;
                    message_in.index = 0;
                }
                break;
            case ParseState::WaitingFor_Checksum:
                hal.console->printf("6\n");
                message_in.packet.checksum[message_in.index++] = *b;
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

// returns true if the fletcher checksum for the packet is valid, else false.
bool AP_ExternalAHRS_LORD::valid_packet(LORD_Packet & packet) {
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

// Calls the correct functions based on the packet descriptor of the packet
void AP_ExternalAHRS_LORD::handle_packet(LORD_Packet& packet) {
    switch ((DescriptorSet) packet.header[2]) {
        case DescriptorSet::IMUData:
            handle_imu(packet);
            post_imu();
            break;
        case DescriptorSet::GNSSData:
            //handle_gnss(packet);
            //post_gnss();
            break;
        case DescriptorSet::EstimationData:
        case DescriptorSet::BaseCommand:
        case DescriptorSet::DMCommand:
        case DescriptorSet::SystemCommand:
            break;
    }
}

// Collects data from an imu packet into `imu_data`
void AP_ExternalAHRS_LORD::handle_imu(LORD_Packet& packet) {
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch (packet.payload[i + 1]) {
            // Scaled Ambient Pressure
            case 0x17: {
                imu_data.pressure = extract_float(packet.payload, i + 2) * 100; // Convert millibar to pascals
                break;
            }
            // Scaled Magnetometer Vector
            case 0x06: {
                imu_data.mag = populate_vector(packet.payload, i + 2) * 1000; // Convert gauss to radians
                break;
            }
            // Scaled Accelerometer Vector
            case 0x04: {
                imu_data.accel = populate_vector(packet.payload, i + 2) * 9.8; // Convert g's to m/s^2
                break;
            }
            // Scaled Gyro Vector
            case 0x05: {
                imu_data.gyro = populate_vector(packet.payload, i + 2);
                break; 
            }
            // CF Quaternion
            case 0x0A: {
                imu_data.quat = populate_quaternion(packet.payload, i + 2);
                break;
            }
        }
    }
}

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_LORD::post_imu() {
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;
        state.quat = imu_data.quat;
        state.have_quaternion = true;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins;
        ins.accel = imu_data.accel;
        ins.gyro = imu_data.gyro;
        ins.temperature = -300;
        AP::ins().handle_external(ins);
    }

    {
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = imu_data.mag;
        AP::compass().handle_external(mag);
    }

    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.instance = 0;
        baro.pressure_pa = imu_data.pressure;
        //setting temp to 25 effectively disables barometer temperature calibrations - these are already performed by lord
        baro.temperature = 25;
        AP::baro().handle_external(baro);
    }
}

// Collects data from an gnss packet into `gnss_data`
void AP_ExternalAHRS_LORD::handle_gnss(LORD_Packet &packet) {
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch (packet.payload[i + 1]) {
            // GPS Time
            case 0x09: {
                gnss_data.tow_ms = extract_double(packet.payload, i+2) * 1000; // Convert seconds to ms
                gnss_data.week = be16toh_ptr(&packet.payload[i+10]);
                break;
            }
            // GNSS Fix Information
            case 0x0B: {
                switch (packet.payload[i+2]) {
                    case (0x00): {
                        gnss_data.fix_type = GPS_FIX_TYPE_3D_FIX;
                        break;
                    }
                    case (0x01): {
                        gnss_data.fix_type = GPS_FIX_TYPE_2D_FIX;
                        break;
                    }
                    case (0x02): 
                    case (0x03): {
                        gnss_data.fix_type = GPS_FIX_TYPE_NO_FIX;
                        break;
                    }
                    default:
                    case (0x04): {
                        gnss_data.fix_type = GPS_FIX_TYPE_NO_GPS;
                        break;
                    }
                }

                gnss_data.satellites = packet.payload[i+3];
                break;
            }
            // LLH Position
            case 0x03: {
                gnss_data.lat = extract_double(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
                gnss_data.lon = extract_double(packet.payload, i+10) * 1.0e7;
                gnss_data.msl_altitude = extract_double(packet.payload, i+26) * 1.0e2; // Meters to cm
                gnss_data.horizontal_position_accuracy = extract_float(packet.payload, i + 34);
                gnss_data.vertical_position_accuracy = extract_float(packet.payload, i+38);
                break;
            }
            // DOP Data
            case 0x07: {
                gnss_data.hdop = extract_float(packet.payload, i+10);
                gnss_data.vdop = extract_float(packet.payload, i+14);
                break;
            }
            // NED Velocity
            case 0x05: {
                gnss_data.ned_velocity_north = extract_float(packet.payload, i+2);
                gnss_data.ned_velocity_east = extract_float(packet.payload, i+6);
                gnss_data.ned_velocity_down = extract_float(packet.payload, i+10);
                gnss_data.speed_accuracy = extract_float(packet.payload, i+26);
                break;
            }
        }
    }
}

// Posts data from a gnss packet to `state` and `handle_external` methods
void AP_ExternalAHRS_LORD::post_gnss() {
    AP_ExternalAHRS::gps_data_message_t gps;
    
    gps.gps_week = gnss_data.week;
    gps.ms_tow = gnss_data.tow_ms;
    gps.fix_type = gnss_data.fix_type;
    gps.satellites_in_view = gnss_data.satellites;

    gps.horizontal_pos_accuracy = gnss_data.horizontal_position_accuracy;
    gps.vertical_pos_accuracy = gnss_data.vertical_position_accuracy;

    gps.latitude = gnss_data.lat;
    gps.longitude = gnss_data.lon;
    gps.msl_altitude = gnss_data.msl_altitude;

    gps.ned_vel_north = gnss_data.ned_velocity_north;
    gps.ned_vel_east = gnss_data.ned_velocity_east;
    gps.ned_vel_down = gnss_data.ned_velocity_down;
    gps.horizontal_vel_accuracy = gnss_data.speed_accuracy;

    if (gps.fix_type >= 3 && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(gnss_data.lat),
                                int32_t(gnss_data.lon),
                                int32_t(gnss_data.msl_altitude),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    AP::gps().handle_external(gps);
}

int8_t AP_ExternalAHRS_LORD::get_port(void) const {
    if (!uart)
        return -1;
    return port_num;
};

bool AP_ExternalAHRS_LORD::healthy(void) const {
    return true;
}

bool AP_ExternalAHRS_LORD::initialised(void) const {
    return true;
}

bool AP_ExternalAHRS_LORD::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const {
    return true;
}

void AP_ExternalAHRS_LORD::get_filter_status(nav_filter_status &status) const {
    return;
}

void AP_ExternalAHRS_LORD::send_status_report(mavlink_channel_t chan) const {
    return;
}

Vector3f AP_ExternalAHRS_LORD::populate_vector(uint8_t *data, uint8_t offset) {
    Vector3f vector;
    uint32_t tmp[3];

    for (uint8_t i = 0; i < 3; i++) {
        tmp[i] = be32toh_ptr(&data[offset+i*4]);
    }

    vector.x = *reinterpret_cast<float*>(&tmp[0]);
    vector.y = *reinterpret_cast<float*>(&tmp[1]);
    vector.z = *reinterpret_cast<float*>(&tmp[2]);

    return vector;
}

Quaternion AP_ExternalAHRS_LORD::populate_quaternion(uint8_t *data, uint8_t offset) {
    Quaternion quat;
    uint32_t tmp[4];

    for (uint8_t i = 0; i < 4; i++) {
        tmp[i] = be32toh_ptr(&data[offset+i*4]);
    }

    quat.q1 = *reinterpret_cast<float*>(&tmp[0]);
    quat.q2 = *reinterpret_cast<float*>(&tmp[1]);
    quat.q3 = *reinterpret_cast<float*>(&tmp[2]);
    quat.q4 = *reinterpret_cast<float*>(&tmp[3]);

    return quat;
}

float AP_ExternalAHRS_LORD::extract_float(uint8_t *data, uint8_t offset) {
    uint32_t tmp = be32toh_ptr(&data[offset]);

    return *reinterpret_cast<float*>(&tmp);
}

double AP_ExternalAHRS_LORD::extract_double(uint8_t *data, uint8_t offset) {
    uint64_t tmp = be64toh_ptr(&data[offset]);

    return *reinterpret_cast<double*>(&tmp);
}

#endif // HAL_EXTERNAL_AHRS_ENABLED