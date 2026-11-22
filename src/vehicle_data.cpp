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

// --- the PDMs ---------------------------------------------------------------------
// Staleness is judged per row of PDM_FRAMES (config.h), the same idea as the
// ECU and BMS frames above.

bool VehicleData::pdmFrameSeen(uint8_t box, uint8_t row) const
    {
    if (box >= PDM_COUNT || row >= PDM_FRAME_COUNT) return false;
    return pdm[box].frameLastRxMs[row] != 0;
    }

bool VehicleData::pdmFrameFresh(uint8_t box, uint8_t row) const
    {
    if (!pdmFrameSeen(box, row)) return false;
    return (millis() - pdm[box].frameLastRxMs[row]) <= PDM_FRESH_MS;
    }

// "Is this box telling us everything it owes us?" A frame the firmware
// always sends must be fresh. A frame that only some firmware sends (the
// fork's rows for outputs 9-14) is owed from the moment it first arrives:
// a stock 8-output board on the bench never sends it and is still online,
// while a fork box that STOPS sending it is not.
bool VehicleData::pdmOnline(uint8_t box) const
    {
    if (box >= PDM_COUNT) return false;
    for (uint8_t row = 0; row < PDM_FRAME_COUNT; row++)
        {
        const PdmFrameNeed need = PDM_FRAMES[row].need;
        if (need == PDM_NEED_NEVER) continue;
        if (need == PDM_NEED_ONCE_SEEN && !pdmFrameSeen(box, row)) continue;
        if (!pdmFrameFresh(box, row)) return false;
        }
    return true;
    }

const char* pdmName(uint8_t box)
    {
    static const char* const names[PDM_COUNT] = { "PDME", "PDML" };
    return (box < PDM_COUNT) ? names[box] : "PDM?";
    }
