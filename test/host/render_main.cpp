// PC harness: draw the inclinometer / g-meter pages with the firmware's own
// gauge_render.cpp and save them as raw RGB565 files (raw2png.py turns them
// into a PNG). This is how docs/inclinometer_pages.png was made.
#include <Arduino.h>
#include "config.h"
#include "vehicle_data.h"
#include "gauge_render.h"
#include "alarms.h"

uint32_t g_fakeMillis = 1000;
HardwareSerial Serial, Serial7;

// stubs for the alarm table (alarms.cpp drags in the whole firmware)
const char* alarmName(uint8_t id) { return id == ALARM_OIL_LOW_CRIT ? "OIL LOW " : "FUEL LOW"; }
AlarmColor  alarmColor(uint8_t id) { return id == ALARM_OIL_LOW_CRIT ? ALARM_RED : ALARM_YELLOW; }

static uint16_t fb[SCREEN_LX * SCREEN_LY];

static void save(const char* name)
    {
    FILE* f = fopen(name, "wb");
    fwrite(fb, 2, SCREEN_LX * SCREEN_LY, f);
    fclose(f);
    }

static void scene(const char* tag, float roll, float pitch, float lon, float lat, bool live)
    {
    ImuData& u = g_data.imu;
    u.present = live; u.lastDataMs = live ? millis() : 0;
    u.rollDeg = roll; u.pitchDeg = pitch; u.lonG = lon; u.latG = lat;
    char name[64];
    for (uint8_t page = 0; page < 3; page++)
        {
        gaugeRenderMotion(fb, page, page == 0);
        snprintf(name, sizeof(name), "out_%s_%u.raw", tag, page);
        save(name);
        }
    }

int main()
    {
    ImuData& u = g_data.imu;
    g_data.alarms.top = ALARM_NONE;

    u.levelCal = true; u.maxBrakeG = 0.9f; u.maxAccelG = 0.45f; u.maxLeftG = 0.8f; u.maxRightG = 1.05f;
    u.maxPeakG = 7.3f; u.lastAirMs = 0;
    scene("level", 0, 0, 0, 0, true);

    u.lastAirMs = 420;
    scene("tilt", 18.4f, 12.0f, -0.62f, 0.48f, true);

    g_data.mode.night = true;
    scene("night", -33.0f, -24.0f, 0.3f, -0.95f, true);
    g_data.mode.night = false;

    u.levelCal = false;
    g_data.alarms.top = ALARM_FUEL_LOW;
    u.levelSetActive = true; u.levelSetPct = 62;
    scene("alarm", -7.0f, 4.0f, 0.1f, 0.2f, true);
    g_data.alarms.top = ALARM_NONE;
    u.levelSetActive = false;

    scene("dead", 0, 0, 0, 0, false);

    // the normal gauge pages too, as a sanity check that the harness draws them
    gaugeRenderScreen(fb, SCREEN_PSI_LAMBDA); save("out_gauge_psi.raw");
    return 0;
    }
