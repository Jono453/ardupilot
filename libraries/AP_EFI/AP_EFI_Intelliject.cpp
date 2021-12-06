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
 
#include <AP_HAL/AP_HAL.h>
#include "AP_EFI_Intelliject.h"

#if HAL_EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

AP_EFI_Intelliject::AP_EFI_Intelliject(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    internal_state.estimated_consumed_fuel_volume_cm3 = 0; // Just to be sure
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI_Intelliject, 0);
}


void AP_EFI_Intelliject::update()
{
    if (!port) {
        return;
    }

    uint32_t now = AP_HAL::millis();

    const uint32_t expected_bytes = 2 + (RT_LAST_OFFSET - RT_FIRST_OFFSET) + 4;
    if (port->available() >= expected_bytes && read_incoming_realtime_data()) {
        last_response_ms = now;
        copy_to_frontend();
    }

    if (port->available() == 0 || now - last_response_ms > 200) {
        port->discard_input();
        // no request needed as ECU sends all packets automatically
    }
}

// SAMPLE FROM MEGASQUIRT SERIAL
/*
bool AP_EFI_Intelliject::read_incoming_realtime_data()
{
    // Data is parsed directly from the buffer, otherwise we would need to allocate
    // several hundred bytes for the entire realtime data table or request every
    // value individiually
    uint16_t message_length = 0;

    // reset checksum before reading new data
    checksum = 0;
    
    // Message length field begins the message (16 bits, excluded from CRC calculation)
    // Message length value excludes the message length and CRC bytes 
    message_length = port->read() << 8;
    message_length += port->read();

    if (message_length >= 256) {
        // don't process invalid messages
        // hal.console->printf("message_length: %u\n", message_length);
        return false;
    }

    // Response Flag (see "response_codes" enum)
    response_flag = read_byte_CRC32();
    if (response_flag != RESPONSE_WRITE_OK) {
        // abort read if we did not receive the correct response code;
        return false;
    }
    
    // Iterate over the payload bytes 
    for ( uint8_t offset=RT_FIRST_OFFSET; offset < (RT_FIRST_OFFSET + message_length - 1); offset++) {
        uint8_t data = read_byte_CRC32();
        float temp_float;
        switch (offset) {
            case PW1_MSB:
                internal_state.cylinder_status[0].injection_time_ms = (float)((data << 8) + read_byte_CRC32())/1000.0f;
                offset++;  // increment the counter because we read a byte in the previous line
                break;
            case RPM_MSB:
                // Read 16 bit RPM
                internal_state.engine_speed_rpm = (data << 8) + read_byte_CRC32();
                offset++;
                break;
            case ADVANCE_MSB:
                internal_state.cylinder_status[0].ignition_timing_deg = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                break;
            case ENGINE_BM:
                break;
            case BAROMETER_MSB:
                internal_state.atmospheric_pressure_kpa = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                break;
            case MAP_MSB:
                internal_state.intake_manifold_pressure_kpa = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                break;
            case MAT_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.intake_manifold_temperature = f_to_k(temp_float);
                break;
            case CHT_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.cylinder_status[0].cylinder_head_temperature = f_to_k(temp_float);
                break;
            case TPS_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.throttle_position_percent = roundf(temp_float);
                break;
            case AFR1_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.cylinder_status[0].lambda_coefficient = temp_float;
                break;
            case DWELL_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                internal_state.spark_dwell_time_ms = temp_float;
                offset++;
                break;
            case LOAD:
                internal_state.engine_load_percent = data;
                break;
            case FUEL_PRESSURE_MSB:
                // MS Fuel Pressure is unitless, store as KPA anyway
                temp_float = (float)((data << 8) + read_byte_CRC32());
                internal_state.fuel_pressure = temp_float;
                offset++;
                break;   
                
        }
    }
    
    // Read the four CRC bytes
    uint32_t received_CRC;
    received_CRC = port->read() << 24;
    received_CRC += port->read() << 16;
    received_CRC += port->read() << 8;
    received_CRC += port->read();
                        
    if (received_CRC != checksum) {
        // hal.console->printf("EFI CRC: 0x%08x 0x%08x\n", received_CRC, checksum);
        return false;
    }

    // Calculate Fuel Consumption 
    // Duty Cycle (Percent, because that's how HFE gives us the calibration coefficients)
    float duty_cycle = (internal_state.cylinder_status[0].injection_time_ms * internal_state.engine_speed_rpm)/600.0f;
    uint32_t current_time = AP_HAL::millis();
    // Super Simplified integration method - Error Analysis TBD
    // This calcualtion gives erroneous results when the engine isn't running
    if (internal_state.engine_speed_rpm > RPM_THRESHOLD) {
        internal_state.fuel_consumption_rate_cm3pm = duty_cycle*get_coef1() - get_coef2();
        internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (current_time - internal_state.last_updated_ms)/60000.0f;
    } else {
        internal_state.fuel_consumption_rate_cm3pm = 0;
    }
    internal_state.last_updated_ms = current_time;
    
    return true;
         
}
*/

float AP_EFI_Intelliject::float32ScaledFrom1UnsignedBytes(const uint8_t* bytes, int* index, float min, float invscaler)
{
    return (float)(min + invscaler*uint8FromBytes(bytes, index));
}

float AP_EFI_Intelliject::float32ScaledFrom2UnsignedBeBytes(const uint8_t* bytes, int* index, float min, float invscaler)
{
    return (float)(min + invscaler*uint16FromBeBytes(bytes, index));
}

/*!
 * \brief Decode a efiTelemetryFast_t from a byte array
 *
 * Fast telemetry, output at the fast telemetry rate.
 * \param _pg_data points to the byte array to decoded data from
 * \param _pg_bytecount points to the starting location in the byte array, and will be incremented by the number of bytes decoded
 * \param _pg_user is the data to decode from the byte array
 * \return 1 if the data are decoded, else 0.
 */

// efiTelemetryFast_t is struct that should become the EFI internal_state filled in by backend
bool AP_EFI_Intelliject::decodeefiTelemetryFast_t(const uint8_t* _pg_data, int* _pg_bytecount, efiTelemetryFast_t* _pg_user)
{
    int _pg_byteindex = *_pg_bytecount;

    // Engine speed in revolutions per minute
    // Range of rpm is 0.0 to 16383.75.
    _pg_user->rpm = float32ScaledFrom2UnsignedBeBytes(_pg_data, &_pg_byteindex, 0.0f, 1.0f/4.0f);

    // Global enable based on physical input
    _pg_user->ioEnable = (_pg_data[_pg_byteindex] >> 7);

    // User global enable.
    _pg_user->userEnable = ((_pg_data[_pg_byteindex] >> 6) & 0x1);

    // User enable for spark1.
    _pg_user->spark1Enable = ((_pg_data[_pg_byteindex] >> 5) & 0x1);

    // User enable for spark2.
    _pg_user->spark2Enable = ((_pg_data[_pg_byteindex] >> 4) & 0x1);

    // User enable for spark3.
    _pg_user->spark3Enable = ((_pg_data[_pg_byteindex] >> 3) & 0x1);

    // Source of the throttle command information. If IntelliJect is driving the throttle this is the command source. If IntelliJect is not driving the throttle this is the source of throttle position data.
    _pg_user->throttleCmdSrc = (efiThrottleSource)((_pg_data[_pg_byteindex]) & 0x7);
    _pg_byteindex += 1; // close bit field

    // Throttle command (0 to 100%) going in.
    // Range of throttleCmd is 0.0 to 127.5.
    _pg_user->throttleCmd = float32ScaledFrom1UnsignedBytes(_pg_data, &_pg_byteindex, 0.0f, 1.0f/2.0f);

    // Throttle position (0 to 100%) this may be different from the throttleCmd if a curve is applied, or if the governor is interpreting the throttle command.
    // Range of throttlePos is 0.0 to 127.5.
    _pg_user->throttlePos = float32ScaledFrom1UnsignedBytes(_pg_data, &_pg_byteindex, 0.0f, 1.0f/2.0f);

    // The third injector duty cycle in percent
    // Range of injector3Duty is 0.0 to 127.5.
    _pg_user->injector3Duty = float32ScaledFrom1UnsignedBytes(_pg_data, &_pg_byteindex, 0.0f, 1.0f/2.0f);

    // The first injector duty cycle in percent
    // Range of injector1Duty is 0.0 to 127.5.
    _pg_user->injector1Duty = float32ScaledFrom1UnsignedBytes(_pg_data, &_pg_byteindex, 0.0f, 1.0f/2.0f);

    // The second injector duty cycle in percent
    // Range of injector2Duty is 0.0 to 127.5.
    _pg_user->injector2Duty = float32ScaledFrom1UnsignedBytes(_pg_data, &_pg_byteindex, 0.0f, 1.0f/2.0f);

    *_pg_bytecount = _pg_byteindex;

    return true;

}// decodeefiTelemetryFast_t
#endif // HAL_EFI_ENABLED
