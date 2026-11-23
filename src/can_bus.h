// ---------------------------------------------------------------------------
// Rally CAN Gauge - CAN bus (ECU + BMS + PDM decode, switch/pot transmit)
// ---------------------------------------------------------------------------
// Hardware: Teensy CAN3 (pins 30 RX / 31 TX) -> TCAN1051 transceiver -> the
// car's 500 kbps bus shared by the rusEFI ECU (Super uaEFI, with its
// on-board wideband controller), the JK BMS and the two PDMs. The gauge PCB
// carries no terminator: the 120R resistors are in the harness.
//
// Four broadcasts come in, all little-endian:
//  * rusEFI's native broadcast, frames BASE0..BASE7 (layouts from rusEFI's
//    own DBC) - nearly everything the gauge shows and judges.
//  * rusEFI's "MS Simplified Broadcast" - only the target AFR byte.
//    Settings for both + frame tables: docs/ECU_CAN_SETUP.md.
//  * BMS: JK BMS speaking the Deye protocol (1 Hz)
//    - frames + JK app settings in docs/BMS_CAN_SETUP.md.
//  * PDM-E and PDM-L: dingoFW's cyclic status frames (10 Hz) - device
//    state, board temperature, and state/current/trip count per output.
//
// And the gauge sends: the BMS's PCS heartbeat (1 Hz), the roof panel's
// switches and pots for the PDMs (10 Hz + on change) - docs/PDM_CAN_SETUP.md
// - and four IMU frames (g, attitude, jumps, impacts; 65 frames/s in all) on
// ids that lose arbitration to everything else - docs/IMU_SETUP.md.
// Change the config on any device, change the matching doc + config.h.
// ---------------------------------------------------------------------------

#pragma once

// Bring the controller up. Call once from setup(), after Serial (init
// progress is printed).
void canBusInit();

// Drain every frame the interrupt has queued since the last call and decode
// them into g_data. Cheap when the queue is empty - call it every single
// pass of loop(), not just on the 20 ms tick, so the queue stays shallow.
void canBusPoll();

// Transmit the roof switch + pot frames when they are due or have changed,
// then whichever IMU frames are due. Call on the 20 ms tick, after
// inputsPoll(), modesUpdate(), motionUpdate(), fansUpdate() and
// alarmsUpdate() - it sends this tick's states, not last tick's.
void canBusTick20ms();

// Once-per-second bus duties: transmit the PCS heartbeat (0x305) the BMS
// expects from an inverter. Call from the 1 s tick.
void canBusTick1s();
