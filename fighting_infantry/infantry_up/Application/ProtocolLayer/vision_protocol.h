#ifndef  __VISION_PROTOCOL_H
#define  __VISION_PROTOCOL_H

#include "stdint.h"


/**
 * @brief Data frame sent from the embedded system to the vision module
 */
typedef __packed struct {
    uint8_t SOF;            // Frame header – start-of-frame marker
    uint8_t CRC8;           // CRC for verifying the integrity of the header itself
    __packed union {        // Status-flag union (32-bit)
        uint32_t all_flags; // Entire 32-bit flag word
        __packed struct {
            uint32_t own_color    : 1; // Bit 0: our own color
            uint32_t game_start   : 1; // Bit 1: match started
            uint32_t is_ready     : 1; // Bit 2: firing allowed (heat OK & reset done)
            uint32_t outpost_mode : 1; // Bit 3: lock only on outpost
            uint32_t engineer_mode: 1; // Bit 4: lock only on engineer
            uint32_t buff_mode    : 1; // Bit 5: lock only on buff
            uint32_t reserved     :26; // Bits 6-31: reserved for future use
        } bit;              // Bit-level access sub-structure
    } flag_union;
    float yaw;              // Current yaw angle
    float pitch;            // Current pitch angle
    float roll;             // Current gimbal roll angle
    float yaw_speed;        // Yaw-axis angular velocity
    float pitch_speed;      // Pitch-axis angular velocity
    int8_t pitch_offset;    // Pitch offset (cleared by MCU when exiting auto-aim)
    int8_t yaw_offset;      // Yaw offset (cleared by MCU when exiting auto-aim)
    float bullet_speed;     // Muzzle velocity
    uint32_t user_debug;    // User-defined debug data:
                            //   - Single-shot: latency from command to bullet passing the chrono (ms)
                            //   - Burst-shot: interval between shots (ms)
                            //   - General: for debugging purposes
    uint16_t CRC16;         // CRC for verifying the integrity of the entire data frame
} ElectricalToVisionFrame;


/**
 * @brief Data frame sent from the vision module to the embedded system
 */
typedef __packed struct {
    uint8_t SOF;            // Frame header – start-of-frame marker
    uint8_t CRC8;           // CRC for verifying the integrity of the header itself
    __packed union {        // Status-flag union (32-bit)
        uint32_t all_flags; // Entire 32-bit flag word
        __packed struct {
            uint32_t is_find_target    : 1; // Bit 0: enable vision control of pitch & yaw
            uint32_t is_keep_shooting  : 1; // Bit 1: dial – speed-loop vs. angle-loop (reserved for now)
            uint32_t is_enable_shootting : 1; // Bit 2: firing permission flag
            uint32_t detect_num        : 4; // Bits 3-6: locked target ID (0-15)
            uint32_t reserved          :25; // Bits 7-31: reserved
        } bit;              // Bit-level access sub-structure
    } flag_union;
    float yaw;              // Target yaw angle
    float pitch;            // Target pitch angle
    uint32_t user_debug;    // User-defined debug information
    uint16_t CRC16;         // CRC for verifying the integrity of the entire data frame
} VisionToElectricalFrame;



#endif


