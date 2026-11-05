// ---------------------------------------------------------------------------
// Rally CAN Gauge - central data model (instance + staleness helpers)
// ---------------------------------------------------------------------------

#include "vehicle_data.h"

VehicleData g_data; // global => zero-initialized before setup()

// Has this broadcast group been heard from recently enough to trust?
//
// The subtraction (millis() - t) is deliberately done in unsigned arithmetic:
// even when millis() wraps around after ~49.7 days, unsigned wraparound makes
// the age come out correct. This is the standard embedded idiom for
// comparing timestamps - never compare millis() values with <  directly.
bool VehicleData::ecuGroupFresh(uint8_t group, uint32_t maxAgeMs) const
    {
    if (group >= MS_CAN_NUM_GROUPS) return false;
    const uint32_t t = ecu.groupLastRxMs[group];
    if (t == 0) return false; // 0 means "never received" (see decode stamp)
    return (millis() - t) <= maxAgeMs;
    }

// "Is the ECU talking at all?" Group 0 carries RPM and is configured as the
// fastest group, so it is the heartbeat we judge the whole link by.
bool VehicleData::ecuOnline() const
    {
    return ecuGroupFresh(0, ECU_OFFLINE_MS);
    }

// Same freshness idea for the BMS, indexed by our BmsFrame enum instead of a
// protocol group number.
bool VehicleData::bmsFrameFresh(uint8_t frame, uint32_t maxAgeMs) const
    {
    if (frame >= BMS_F_COUNT) return false;
    const uint32_t t = bms.frameLastRxMs[frame];
    if (t == 0) return false;
    return (millis() - t) <= maxAgeMs;
    }

// The pack-summary frame (0x356) is the BMS heartbeat: voltage/current/temp,
// sent every cycle.
bool VehicleData::bmsOnline() const
    {
    return bmsFrameFresh(BMS_F_PACK, BMS_OFFLINE_MS);
    }
