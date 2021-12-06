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
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

// RPM Threshold for fuel consumption estimator
#define RPM_THRESHOLD                100

class AP_EFI_Intelliject: public AP_EFI_Backend {
    
public:
    // Constructor with initialization
    AP_EFI_Intelliject(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

private:
    AP_HAL::UARTDriver *port;
    void parse_realtime_data();
    bool read_incoming_realtime_data();
    float f_to_k(float temp_f) { return (temp_f + 459.67f) * 0.55556f; };

    //! Process a packet received from IntelliJect that reports telemetry
    int processIntelliJectDataPacket(const efiPacket_t* pkt, efisdkdata_t* data);
    
    // Serial Protocol Variables
    uint32_t checksum;
    uint8_t step;
    uint8_t response_flag;
    uint16_t message_counter;
    uint32_t last_response_ms;

    /*!
     * Fast telemetry, output at the fast telemetry rate.
     */
    typedef struct
    {
        float             rpm;              //!< Engine speed in revolutions per minute
        unsigned          ioEnable : 1;     //!< Global enable based on physical input
        unsigned          userEnable : 1;   //!< User global enable.
        unsigned          spark1Enable : 1; //!< User enable for spark1.
        unsigned          spark2Enable : 1; //!< User enable for spark2.
        unsigned          spark3Enable : 1; //!< User enable for spark3.
        efiThrottleSource throttleCmdSrc;   //!< Source of the throttle command information. If IntelliJect is driving the throttle this is the command source. If IntelliJect is not driving the throttle this is the source of throttle position data.
        float             throttleCmd;      //!< Throttle command (0 to 100%) going in.
        float             throttlePos;      //!< Throttle position (0 to 100%) this may be different from the throttleCmd if a curve is applied, or if the governor is interpreting the throttle command.
        float             injector3Duty;    //!< The third injector duty cycle in percent
        float             injector1Duty;    //!< The first injector duty cycle in percent
        float             injector2Duty;    //!< The second injector duty cycle in percent
    }efiTelemetryFast_t;

    
};
