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
        // Request an update from the realtime table (7).
        // The data we need start at offset 6 and ends at 129
        send_request(7, RT_FIRST_OFFSET, RT_LAST_OFFSET);
    }
}

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

*!
 * Process a packet received from IntelliJect that reports telemetry
 * \param pkt is the packet to process
 * \param data receives the data contained in the packet
 * \return 1 if the packet was successfully decoded, else it was not a valid data packet
 */
int AP_EFI_Intelliject::processIntelliJectDataPacket(const efiPacket_t* pkt, efisdkdata_t* data)
{
    switch(pkt->type)
    {
    default:
        return 0;

    case EFI_PKT_SOFTWAREINFO:  // Software information
        return decodeefiSoftwareInfoPkt(pkt, &data->software);
        break;

    case EFI_PKT_HARDWAREINFO:  // Hardware information
        return decodeefiHardwareInfoPkt(pkt, &data->hardware);
        break;

    case EFI_PKT_ENABLE:        // Response to enable command
        {
            efiEnable_t enable;
            memset(&enable, 0, sizeof(enable));
            if(decodeefiEnablePkt(pkt, &enable))
            {
                data->fasttm.ioEnable = enable.ioEnable;
                data->fasttm.userEnable = enable.userEnable;
                data->fasttm.spark1Enable = enable.spark1Enable;
                data->fasttm.spark2Enable = enable.spark2Enable;
                data->fasttm.spark3Enable = enable.spark3Enable;
                return 1;
            }
        }
        break;

    case EFI_PKT_FUELUSED:  // Response to setting fuel used
        return decodeefiFuelUsedPkt(pkt, &data->fueltm.fuelConsumption, &data->oilinj.oilConsumption);
        break;

    case EFI_PKT_ENGINEWEAR:            // Engine wear information
        if(decodeefiEngineWearPkt(pkt, &data->wear))
            return 1;
        else
            return decodeefiEngineWearShortPkt(pkt, &data->wear.hobbs, &data->wear.revcount);
        break;

    case EFI_PKT_ENGINEWEAREXT:         // Extended engine &wear information
        return decodeefiEngineWearExtendedPkt(pkt, &data->wear.hotTime, &data->wear.highLoadTime, &data->wear.peakCHT, &data->wear.numStarts);
        break;

    case EFI_PKT_SDCARDJOURNAL:
        return decodeefiSDCardJournalPkt(pkt, &data->sdjournal);
        break;

    case EFI_PKT_RESET:                 // Reset report
        return decodeefiResetReportPkt(pkt, &data->reset);
        break;

    case EFI_PKT_STICKY_ERRORS:         // Sticky error information
        if(data->cputm.api < 5)
        {
            efiErrorsapi4_t olderrors;
            if(decodeefiErrorsapi4Pkt(pkt, &olderrors))
            {
                data->stickyerrors = convertErrorsapi4ToErrors(olderrors);
                return 1;
            }
            else
                return 0;
        }
        else
            return decodeefiErrorsPkt(pkt, &data->stickyerrors);
        break;

    case EFI_PKT_TELEMETRYERRORS:       // Dynamic error information
        if(data->cputm.api < 5)
        {
            efiErrorsapi4_t olderrors;
            if(decodeefiErrorsapi4Pkt(pkt, &olderrors))
            {
                data->dynamicerrors = convertErrorsapi4ToErrors(olderrors);
                return 1;
            }
            else
                return 0;
        }
        else
            return decodeefiErrorsPkt(pkt, &data->dynamicerrors);
        break;

    case EFI_PKT_TELEMETRYTIME:
        return decodeefiTelemetryTimePkt(pkt, &data->timetm);
        break;

    case EFI_PKT_TELEMETRYFAST:         // Fast telemetry
        return decodeefiTelemetryFastPkt(pkt, &data->fasttm);
        break;

    case EFI_PKT_TELEMETRYSENSORS:      // Telemetry information for EFI sensors
        if(data->cputm.api <= 7)
        {
            efiTelemetrySensorsapi7_t sensors = {0};
            if(decodeefiTelemetrySensorsapi7Pkt(pkt, &sensors))
            {
                data->sensorstm.map = sensors.map;
                data->sensorstm.mat = sensors.mat;
                data->sensorstm.oat = sensors.oat;
                data->sensorstm._cht = sensors._cht;
                data->sensorstm.baro = sensors.baro;
                data->sensorstm.sparetemp = sensors.sparetemp;
                data->sensorstm.throttlePosSrc = data->fasttm.throttleCmdSrc;

                if(!data->sensors2tm.expectSensors4)
                {
                    data->sensors4tm.cht = sensors._cht;
                    data->sensors4tm.cht1 = sensors._cht1;
                    data->sensors4tm.cht2 = sensors._cht;
                }

                return 1;

            }// if decoded api7

        }// if older API
        else
            return decodeefiTelemetrySensorsPkt(pkt, &data->sensorstm);
        break;

    case EFI_PKT_TELEMETRYSENSORS2:     // Telemetry2 information for EFI sensors
        if(decodeefiTelemetrySensors2Pkt(pkt, &data->sensors2tm))
        {
            return 1;
        }
        else
        {
            // These angles are unknown in the older encoding
            data->sensors2tm.measuredCrank2Angle = 0;
            data->sensors2tm.devCrank2Angle = 0;

            // Check the api7 and older encoding
            efiTelemetrySensors2api7_t sensors2;
            if(decodeefiTelemetrySensors2api7Pkt(pkt, &sensors2))
            {
                data->sensors2tm.density           = sensors2.density;
                data->sensors2tm.expectSensors4    = sensors2.expectSensors4;
                data->sensors2tm.api8              = sensors2.api8;
                data->sensors2tm.crankSense1Active = sensors2.crankSense1Active;
                data->sensors2tm.crankSense2Active = sensors2.crankSense2Active;
                data->sensors2tm.cooling2          = sensors2.cooling2;
                data->sensors2tm.tpsError          = sensors2.tpsError;
                data->sensors2tm.analogTPS         = sensors2.analogTPS;
                data->sensors2tm.pwmTPS            = sensors2.pwmTPS;

                // Special logic for older systems that will not send sensors 4
                if(!data->sensors2tm.expectSensors4)
                    data->sensors4tm.fuelp = sensors2._fuelp;

                return 1;
            }

            return 0;
        }
        break;

    case EFI_PKT_TELEMETRYSENSORS3:     // Telemetry3 information for EFI sensors
        return decodeefiTelemetrySensors3Pkt(pkt, &data->sensors3tm);
        break;

    case EFI_PKT_TELEMETRYSENSORS4:     // Telemetry4 information for EFI sensors
        return decodeefiTelemetrySensors4Pkt(pkt, &data->sensors4tm);
        break;

    case EFI_PKT_TELEMETRYFUEL:         // EFI fuel information slow telemetry
        return decodeefiTelemetryFuelPkt(pkt, &data->fueltm);
        break;

    case EFI_PKT_TELEMETRYSLOW:         // Miscellanious slow telemetry
        if(data->cputm.api <= 7)
        {
            efiTelemetrySlowapi7_t slowtm;
            memset(&slowtm, 0, sizeof(slowtm));
            if(decodeefiTelemetrySlowapi7Pkt(pkt, &slowtm))
            {
                data->extendedoutputstm.sparkAdvance2 = slowtm._sparkAdvance2;

                // Tweak to slow telemetry layout in api8
                memset(&data->slowtm, 0, sizeof(data->slowtm));
                data->slowtm.power = slowtm.power;
                data->slowtm.rpmcmd = slowtm.rpmcmd;
                data->slowtm.cooling1 = slowtm.cooling1;
                data->slowtm.pumpduty = slowtm.pumpduty;
                data->slowtm.sparkAdvance1 = slowtm.sparkAdvance1;
                data->slowtm.rpmControllerFromUser = slowtm.rpmControllerFromUser;
                data->slowtm.rpmControllerFromThrottle = slowtm.rpmControllerFromThrottle;
                return 1;
            }
            else
                return 0;
        }
        else
            return decodeefiTelemetrySlowPkt(pkt, &data->slowtm);
        break;

    case EFI_PKT_TELEMETRYINJECTOR:
        return decodeefiTelemetryInjectorPkt(pkt, &data->injectortm);
        break;

    case EFI_PKT_TELEMETRYEXTENDEDOUTPUTS:
        return decodeefiTelemetryExtendedOutputsPkt(pkt, &data->extendedoutputstm);
        break;

    case EFI_PKT_TELEMETRYCPU:          // CPU load telemetry
        if(decodeefiTelemetryCPUPkt(pkt, &data->cputm))
        {
            if(!data->cputm.gcuPresent)
            {
                // Null out the GCU data
                memset(&data->gcutm, 0, sizeof(data->gcutm));
            }

            return 1;
        }
        else
            return 0;
        break;

    case EFI_PKT_TELEMETRYCOMMS:
        return decodeefiTelemetryCommsPkt(pkt, &data->comms);
        break;

    case EFI_PKT_TELEMETRYSDCARD:
        return decodeefiTelemetrySDCardPkt(pkt, &data->sdjournal.sdcardtm);
        break;

    case EFI_PKT_TELEMETRYGCU:
        return decodeefiTelemetryGCUPkt(pkt, &data->gcutm);
        break;

    case EFI_PKT_TELEMETRYOILINJ:
        return decodeefiTelemetryOilInjectionPkt(pkt, &data->oilinj);
        break;

    case EFI_PKT_TELEMETRYSLOWSUM:

        // The slow summary packet has expanded multiple times.
        if(decodeefiTelemetrySlowSummaryPkt(pkt, &data->timetm, &data->sensorstm, &data->sensors2tm, &data->fueltm, &data->injectortm, &data->slowtm, &data->cputm, &data->sensors3tm, &data->dynamicerrors, &data->stickyerrors, &data->wear, &data->comms, &data->sensors4tm, &data->sdjournal.sdcardtm, &data->extendedoutputstm))
            return 1;
        else
            return decodeOldSlowSummaryPkt(pkt, data);
        break;

    case EFI_PKT_TELEMETRYFASTSUM:
        return decodeefiTelemetryFastSummaryPkt(pkt, &data->timetm, &data->fasttm);
        break;
    }

    return 0;

}// processIntelliJectDataPacket
#endif // HAL_EFI_ENABLED
