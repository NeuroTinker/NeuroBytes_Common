#ifndef COMM_H_
#define COMM_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>

#include "HAL.h"

#define MAX_I                   NUM_INPUTS - 1
#define DEND_PING_TIME		200 // 1000 ms
#define	NID_PING_TIME		200 // 1000 ms
#define NID_PING_KEEP_ALIVE     32

#define PULSE_HEADER                0b1111
#define DOWNSTREAM_PING_HEADER      0b1011
#define BLINK_HEADER                0b1001
#define NID_PING_HEADER             0b1110
#define NID_GLOBAL_HEADER           0b1100
#define NID_SELECTED_HEADER         0b1101
#define DATA_HEADER                 0b1010

#define DATA_SUBHEAD_POT            0b000
#define DATA_SUBHEAD_TYPE           0b001
#define DATA_SUBHEAD_STATE          0b010

#define NID_PING_MESSAGE            0b11100000000000000000000000000000
#define DATA_MESSAGE                0b10100000000000000000000000000000 // (NID) (KEEP ALIVE=0) (sub-header) (CHANNEL= NONE) (DATA) (no data)

#define IDENTIFY_COMMAND            0b000001
#define VERSION_COMMAND             0b000010

#define NID_PING_DATA_LENGTH        6
#define CLOSER_PING_COUNT           3
#define IDENTIFY_TIME       500 // 250 ms
#define LPUART1_I MAX_I + 1
#define NO_NID_I MAX_I + 2

#define NID_IS_CONNECTED    (nid_i != NO_NID_I)

#define DOWNSTREAM_BUFFSIZE 3
#define ALL_BUFFSIZE        5  
#define NID_BUFFSIZE        5

#define NEXT_BIT(m)   (1 << (m.length-1)) & m.message

/*
    This and comm.c define all communication protocol
    message headers:
    0b000  -   Unused
    0b001  -   Blink (Debug)
    0b010  -   Data to NID (6-bit data header + 16-bit data packet + 3-bit parity check)
    0b011  -   Downstream Ping
    0b100  -   NID Global Command 
    0b101  -   NID Selected Command 
    0b110  -   NID Ping 
    0b111  -   Downstream Pulse
    There are three types of communication that are defined in this protocol:
    
    1. Network Interface Device (NID) broadcasting to all neurons in the network. 
    
        Messages include nid pings and identify device request. The NID commands class of messsages
        
        List of message headers:
        -NID_PING_HEADER
            +read 6-bit distance counter
                -processNIDPING
        -NID_GLOBAL_HEADER
            +read 6-bit command header
                -IDENTIFY_COMMAND
                    +read 3-bit channel
                        -processIdentifyCommand
                -VERSION_COMMAND
                    +read 13-bit packet (5-bit device id + 8-bit version)
                        -processVersionCommand
                -RESUME_COMMAND
                    -set run_flag
                -PAUSE_COMMAND
                    -clear run_flag
        -NID_SELECTED_HEADER
            +read 24-bit packet (3-bit channel + 5-bit parameter + 16-bit data)
                -SET_PARAMETER_COMMAND
                    -processParameterCommand
        -BLINK_HEADER
            -set blink_flag
            
    2. Selected neurons sending data to the NID.

    3. Upstream neurons sending pulses to downstream neurons (axon -> dendrite).

*/


typedef struct{
    uint8_t length;
    uint32_t message;
} message_t;

extern const message_t pulse_message;
extern const message_t downstream_ping_message;
extern const message_t blink_message;

typedef struct read_buffer_t read_buffer_t;
typedef bool (*read_handler_t) (read_buffer_t *);

typedef enum{
    NONE_BUFF,
    DOWNSTREAM_BUFF,
    ALL_BUFF,
    NID_BUFF
} message_buffers_t;

typedef struct{
    message_buffers_t   current_buffer;
    uint8_t             write_count;
    message_t           downstream[DOWNSTREAM_BUFFSIZE];
    uint8_t             downstream_ready_count;
    message_t           nid[NID_BUFFSIZE];
    uint8_t             nid_ready_count;
    message_t           all[ALL_BUFFSIZE];
    uint8_t             all_ready_count;
    uint8_t             source_pin;
    uint8_t             num_bits_to_write;
} write_buffer_t;

typedef struct read_buffer_t{
    uint8_t i;
    uint32_t message;
    uint8_t bits_left_to_read;
    read_handler_t callback;
} read_buffer_t;

extern uint8_t nid_i;

// flags for main()
extern volatile uint8_t blink_flag;
extern volatile uint8_t comms_flag;
extern volatile uint16_t comms_data;

extern volatile uint32_t nid_ping_time;
extern volatile uint8_t nid_distance;

extern volatile uint16_t nid_pin;
extern volatile uint16_t nid_pin_out;

extern volatile uint8_t closer_ping_count;
extern volatile uint8_t closer_distance;

extern volatile uint16_t identify_time;
extern uint8_t identify_channel;
extern volatile uint8_t nid_channel;

void readBit(uint8_t read_tick); // The readInputs() function reads the next bit for active inputs

void writeBit(void);

void writeAll(void);
void writeDownstream(void);

bool processMessageHeader(read_buffer_t * read_buffer_ptr);
bool processNIDPing(read_buffer_t * read_buffer_ptr);
bool processGlobalCommand(read_buffer_t * read_buffer_ptr);
bool processDataMessage(read_buffer_t * read_buffer_ptr);
bool processIdentifyCommand(read_buffer_t * read_buffer_ptr);
bool processSelectedCommand(read_buffer_t * read_buffer_ptr);
bool processParameterCommand(read_buffer_t * read_buffer_ptr);
bool processVersionCommand(read_buffer_t * read_buffer_ptr);

void addWrite(message_buffers_t buffer, const message_t message);
void commInit(void);

void addNIDWrite(uint32_t message);
void writeNIDByte(uint8_t byte);
uint16_t readNIDByte(void);
void readNID(void);
void writeNID(void);
#endif
