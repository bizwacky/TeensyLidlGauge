// ---------------------------------------------------------------------------
// Rally CAN Gauge - CAN bus input (ECU + BMS decode)
// ---------------------------------------------------------------------------
// Hardware: Teensy CAN3 (pins 30 RX / 31 TX) -> SN65HVD230-class transceiver
// -> the car's 500 kbps bus shared by the rusEFI ECU and the JK BMS.
//
// Two protocols, one bus:
//  * ECU: Megasquirt "Advanced Real-Time Data Broadcast" (big-endian, 50 Hz)
//    - groups/rates/sensor mappings in docs/ECU_CAN_SETUP.md.
//  * BMS: JK BMS speaking the Deye protocol (little-endian, 1 Hz)
//    - frames + JK app settings in docs/BMS_CAN_SETUP.md.
// Change the config on either device, change the matching doc + config.h.
// ---------------------------------------------------------------------------

#pragma once

// Bring the controller up. Call once from setup(), after Serial (init
// progress is printed).
void canBusInit();

// Drain every frame the interrupt has queued since the last call and decode
// them into g_data. Cheap when the queue is empty - call it every single
// pass of loop(), not just on the 20 ms tick, so the queue stays shallow.
void canBusPoll();

// Once-per-second bus duties: transmit the PCS heartbeat (0x305) the BMS
// expects from an inverter. Call from the 1 s tick.
void canBusTick1s();
