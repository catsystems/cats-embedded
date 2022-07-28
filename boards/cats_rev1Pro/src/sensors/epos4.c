/*
 * CATS Flight Software
 * Copyright (C) 2021 Control and Telemetry Systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "epos4.h"
#include "drivers/uart.h"

static uint16_t calculate_crc(uint8_t *data, uint8_t len);
static uint8_t write_command(uint8_t *command, uint8_t *data, uint8_t *rx_buffer);
static uint8_t read_command(uint8_t *command, uint8_t *rx_buffer);

UART_BUS EPOS_BUS = {.busy = 0,
                     .initialized = 1,
                     .uart_handle = &EPOS_UART_HANDLE};

// This function calculates the crc code which is needed at the end of the command
// To successfully send a command to the EPOS4 a CRC code needs to be calculated and
// attached to the command (last 2 bytes). See Datasheet for more information
static uint16_t calculate_crc(uint8_t *data, uint8_t len) {
    uint16_t shifter, c;
    uint16_t carry;
    uint16_t crc = 0;

    for (int i = 0; i < len + 2; i += 2) {
        shifter = 0x8000;
        if (i == len) {
            c = 0;
        } else {
            c = data[i + 1] << 8 | data[i];
        }
        do {
            carry = crc & 0x8000;
            crc <<= 1;
            if (c & shifter) crc++;
            if (carry) crc ^= 0x1021;
            shifter >>= 1;
        } while (shifter);
    }
    return crc;
}


// This function is the low level command to write to the EPOS4.
// To write something, first the command bytes have to be set, in what register
// A write is needed -> command argument
// What is written into that register is saved into Data
// Then the CRC is calculated and the command is sent per UART via the DMA
// The EPOS4 always returns something and as such we check if the returned value
// yielded an error. The returned value is also returned into the rx_buffer
static uint8_t write_command(uint8_t *command, uint8_t *data, uint8_t *rx_buffer) {
    volatile uint8_t status = 1;

    uint8_t byte_stream_write[14] = {0};

    byte_stream_write[0] = 0x90;        // DLE
    byte_stream_write[1] = 0x02;        // STX
    byte_stream_write[2] = 0x68;        // Write Object
    byte_stream_write[3] = 0x04;        // Length of Data in Words
    byte_stream_write[4] = 0x01;        // Node ID
    byte_stream_write[5] = command[1];  // Index Low Byte
    byte_stream_write[6] = command[0];  // Index High byte
    byte_stream_write[7] = 0x00;        // Subindex of object
    byte_stream_write[8] = data[3];     // Data - low byte
    byte_stream_write[9] = data[2];     // Data
    byte_stream_write[10] = data[1];    // Data
    byte_stream_write[11] = data[0];    // Data - high byte

    /* CRC Calculation */
    uint8_t crc_data_array[10] = {0};
    memcpy(crc_data_array, &byte_stream_write[2],
           10 * sizeof(*byte_stream_write));

    uint16_t crc_calc = 0;
    crc_calc = calculate_crc(crc_data_array, 10);

    byte_stream_write[12] = crc_calc & 0xFF;
    ;  // CRC low byte
    byte_stream_write[13] = (crc_calc >> 8) & 0xFF;
    ;  // CRC high byte

    status = uart_transmit(&EPOS_BUS, byte_stream_write, 14);
    status = uart_receive(&EPOS_BUS, rx_buffer, 10);



    /* Check if we have an error code */
    if ((rx_buffer[7] | rx_buffer[6] | rx_buffer[5] | rx_buffer[4]) == 0) {
        status = 0;
    }

    return status;
}

// This function only implements the low level reading. This is useful
// For the reading of the current motor position.
// Like the write we need to insert the command -> being where we want to
// read the data. And what we read is then inserted into the rx_buffer
// argument
static uint8_t read_command(uint8_t *command, uint8_t *rx_buffer) {
    uint8_t status = 1;

    uint8_t byte_stream_read[10];

    byte_stream_read[0] = 0x90;        // DLE
    byte_stream_read[1] = 0x02;        // STX
    byte_stream_read[2] = 0x60;        // Read Object
    byte_stream_read[3] = 0x02;        // Length of stuff sent
    byte_stream_read[4] = 0x01;        // Node ID
    byte_stream_read[5] = command[1];  // Index Low Byte
    byte_stream_read[6] = command[0];  // Index High byte
    byte_stream_read[7] = 0x00;        // Subindex of object

    /* CRC data array */
    uint8_t crc_data_array[6] = {0};
    memcpy(crc_data_array, &byte_stream_read[2], 6 * sizeof(*byte_stream_read));

    uint16_t crc_calc = 0;
    crc_calc = calculate_crc(crc_data_array, 6);

    byte_stream_read[8] = crc_calc & 0xFF;
    ;  // CRC low byte
    byte_stream_read[9] = (crc_calc >> 8) & 0xFF;
    ;  // CRC high byte
    uart_transmit(&EPOS_BUS, byte_stream_read, 10);
    uart_receive(&EPOS_BUS, rx_buffer, 14);

    /* check if we have an error code */
    if ((rx_buffer[7] | rx_buffer[6] | rx_buffer[5] | rx_buffer[4]) == 0) {
        status = 0;
    }
    return status;
}

// This function enables the motor driver and if it is unsuccessful it returns an
// osError. If this function is not called at the beginning no motor command
// will be accepted
uint8_t enable_motor() {
    uint8_t status = 1;

    uint8_t command[2];
    uint8_t data[4];
    uint8_t rx_buffer_write[20];
    memset(rx_buffer_write,0, sizeof(rx_buffer_write));

    /* Register for Motor Control */
    command[0] = 0x60;
    command[1] = 0x40;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x06;

    status |= write_command(command, data, rx_buffer_write);

    /* Register for Motor Control */
    command[0] = 0x60;
    command[1] = 0x40;

    /* Fully Enable Controller */
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x0F;

    status |= write_command(command, data, rx_buffer_write);

    /* Check if Motor is enabled */
    uint8_t rx_buffer_read[20];
    memset(rx_buffer_read,0, sizeof(rx_buffer_read));

    command[0] = 0x60;
    command[1] = 0x41;

    status |= read_command(command, rx_buffer_read);

    if (rx_buffer_read[8] == 0x37 && rx_buffer_read[9] == 0x04) {
        status = 0;
    }

    return status;
}

// This function disables the motor driver
uint8_t disable_motor() {

    uint8_t command[2];
    uint8_t data[4];
    uint8_t rx_buffer_write[20];

    /* Register for Motor Control */
    command[0] = 0x60;
    command[1] = 0x40;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    uint8_t status = write_command(command, data, rx_buffer_write);

    return status;
}

uint8_t set_position_mode(int8_t position_mode) {

    uint8_t command[2];
    uint8_t data[4];
    uint8_t rx_buffer_write[20];

    /* Position Mode Register */
    command[0] = 0x60;
    command[1] = 0x60;

    /* Enable Cyclic Sync Position Mode */
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = position_mode;

    uint8_t status = write_command(command, data, rx_buffer_write);

    return status;
}

// This function writes the desired position into the EPOS4 position register
// Afterwards it tells the EPOS 4 to update the position and then we disable
// The movement again.
uint8_t move_to_position(int32_t position) {
    uint8_t status = 1;

    uint8_t command[2];
    uint8_t data[4];
    uint8_t rx_buffer_write[20];

    /* Write Desired Position */
    command[0] = 0x60;
    command[1] = 0x7A;

    data[0] = (position >> 24) & 0xFF;
    data[1] = (position >> 16) & 0xFF;
    data[2] = (position >> 8) & 0xFF;
    data[3] = position & 0xFF;

    status |= write_command(command, data, rx_buffer_write);

    /* Goto Position */
    command[0] = 0x60;
    command[1] = 0x40;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x3F;

    status |= write_command(command, data, rx_buffer_write);

    /* Disabe Movement Again */
    command[0] = 0x60;
    command[1] = 0x40;

    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x0F;

    status |= write_command(command, data, rx_buffer_write);

    return status;
}

uint8_t home_motor(){
    return move_to_position(0);
}
