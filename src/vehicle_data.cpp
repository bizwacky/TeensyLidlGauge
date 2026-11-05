// ---------------------------------------------------------------------------
// Rally CAN Gauge - central data model (instance + staleness helpers)
// ---------------------------------------------------------------------------

#include "vehicle_data.h"

VehicleData g_data; // global => zero-initialized before setup()

// Has this ECU frame been heard from recently enough to trust?
//
// The subtraction (millis() - t) is deliberately done in unsigned arithmetic:
// even when millis() wraps around after ~49.7 days, unsigned wraparound makes
// the age come out correct. This is the standard embedded idiom for
// comparing timestamps - never compare millis() values with <  directly.
bool VehicleData::ecuFrameFresh(uint8_t frame, uint32_t maxAgeMs) const
    {
    if (frame >= ECU_F_COUNT) return false;
    const uint32_t t = ecu.frameLastRxMs[frame];
    if (t == 0) return false; // 0 means "never received" (see decode stamp)
    return (millis() - t) <= maxAgeMs;
    }

// "Is the ECU talking at all?" The rpm frame (BASE1) is the heartbeat: it
// gates "engine running", the tach and the fuel integration. All of rusEFI's
// native frames leave in one burst, so in practice they live and die
// together - but the MS dash frame with the AFR target is a separate
// broadcast, which is why NO CAN asks ecuDataComplete() instead.
bool VehicleData::ecuOnline() const
    {
    return ecuFrameFresh(ECU_F_SPEEDS);
    }

// The first decoded ECU frame that is stale (or has never arrived), or
// ECU_F_COUNT when every one is fresh.
uint8_t VehicleData::ecuFirstStaleFrame() const
    {
    for (uint8_t f = 0; f < ECU_F_COUNT; f++)
        if (!ecuFrameFresh(f)) return f;
    return ECU_F_COUNT;
    }

// Every ECU frame the gauge decodes is arriving. This is what NO CAN
// watches: losing ANY of them - a broadcast switched off in TunerStudio, one
// frame dropped - blinds some gauge or alarm, so it must be loud. (Each
// alarm additionally gates on its own frame, so a stale value can never
// trip one.)
bool VehicleData::ecuDataComplete() const
    {
    return ecuFirstStaleFrame() == ECU_F_COUNT;
    }

const char* ecuFrameName(uint8_t frame)
    {
    static const char* const names[ECU_F_COUNT] =
        { "BASE0", "BASE1", "BASE2", "BASE3", "BASE4", "BASE5", "BASE6", "BASE7", "MS-AFRTGT" };
    return (frame < ECU_F_COUNT) ? names[frame] : "-";
    }

// Same freshness idea for the BMS, indexed by our BmsFrame enum.
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
