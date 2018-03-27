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

// Message templates for making NID Pings and Data-to-NID messages
#define NID_PING_MESSAGE            0b11100000000000000000000000000000
#define DATA_MESSAGE                0b10100000000000000000000000000000 
#define DATA_TYPE_MESSAGE           0b10100100000000000000000000000000

// Global commands
#define IDENTIFY_COMMAND            0b000001
#define VERSION_COMMAND             0b000010
#define PAUSE_COMMAND               0b000011

// Special NID channels
#define IDENTIFY_CLEAR              0
#define IDENTIFY_STOP               7

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
    This and comm.c define all communication protocol using the NeuroBytes protocol specifiction

    All packets are between 4 and 32 bits in length and begin with 1-bit high
    and a 3-bit message header:

    [1-bit high] [3-bit header]

    The message headers are:
    0b000  -   Unused
    0b001  -   Blink (Debug)
    0b010  -   Data to NID
    0b011  -   Downstream Ping
    0b100  -   NID Global Command 
    0b101  -   NID Selected Command 
    0b110  -   NID Ping 
    0b111  -   Downstream Pulse

    There are three types of communication that are supported in this protocol:
    
    1. Network Interface Device (NID) broadcasting to all neurons in the network. 

        Messages are sent by NID and received by all devices on the network.
        Global Commands are processed by every device on the network.
        Selected Commands are processed only by the device with the channel specified by the message.

        Included message headers:
        0b001 - Blink
        0b100 - NID Global Command - Send a command to the entire network
        0b101 - NID Selected Command - Send a command to a previously selected device
        0b110 - NID Ping - Propogate ping in order to update NID route

        ####### NID Global Command ########

        NID Global Command structure:
        [1-bit high]
        [3-bit header]
        [6-bit command]             - See list of global commands below
        [More depending on command]

        Global Commands:
        0b001 - Identify. Followed with 3-bit channel
        0b010 - Version. Followed by 16-bit board id and version number. Used to blink out-of-date boards.
        0b011 - Pause.
        0b100 - Play.
        0b101 - Zero
        0b111 - Span

            *Note: 6-bits is probably excessive for global commands. Might change in the future.

        Example NID Global Command:

        Identify new neuron on channel 2.
        1           [1-bit high]
        100         [NID Global Command Header]
        000001      [Identify Command]
        010         [Channel 2]
        _______
        = 0b1100000001010

        ######## NID Selected Command ########

        NID Selected Command:
        Used to either send a command to a specific neuron (e.g. go into learning mode)
        or change the value of an operating parameter (e.g. set dendrite 1 to 120%)

        NID Selected Command Structure:
        [1-bit high]
        [3-bit header]      - always NID Selected Command Header (0b101)
        [3-bit channel]
        [1-bit change parameter flag]
        [4-bit command] OR [4-bit parameter ID]     - depending on if [change paramater flag] is set
        (optional) [16-bit data]

        NOTE: Parameter IDs are board-specific (i.e. different for every board). 
        They are not all fully specified yet. Currently Interneurons and Motor Neurons are set.
        Their values can be found in their repos (main.c). 

        Example NID Selected Command:

        Set the value of dendrite 1 on an interneuron on channel 4 to 256
        1                   [1-bit high]
        101                 [NID Selected Command Header]
        100                 [Channel 4]
        1                   [Change Parameter]
        0010                [Dendrite 1]            *Parameter IDs can be found in main.c for each board (NOT FINAL)
        0000001000000000    [Change to 256]
        ________________
        = 0b1101100100100000001000000000

        ######## NID Ping ########

        Functional Summary:

        NID pings are periodically sent (~200 ms) by NID to the whole network in order to 
        update the shortest-route-to-NID memorized by every device.

        The NID ping packet contains a 6-bit distance field which is incremented by every successive neuron.
        The shortest route is determined by every neuronto be through the dendrite/axon that receives
        the NID ping with the shortest distance value. Memorized NID connections are lost if a NID ping
        is not received within NID_PING_TIME (~1000 ms).

        NID Ping Message Structure:
        [1-bit high]
        [3-bit header]      - always NID Ping Header (0b110)
        [6-bit distance]

        NID Ping Example Message:

        NID Ping relayed by two neurons so far
        1           [1-bit high]
        110         [NID Ping Header]
        000010      [distance = 2]
        _______
        = 0b111000010


            
    2. Selected neurons sending data to the NID.

        Messages are sent by identified devices and are relayed along the shortest path to the NID.
        The shortest path to the NID is maintained by NID pings.

        Messages sent from identified devices to the NID. Capable of sending many different types
        of data (16-bits) to the NID.

        Included message header:
        0b010 - Data to NID

        ######## Data to NID #######

        Data message structure:
        [1-bit high]            
        [3-bit header]              - 0b010 (Data Header)
        [3-bit subheader]           - specifies type of data being transmitted
        [3-bit channel]             - specifies channel sending the data
        [1-bit fire flag]           - flag indicating the neuron fired
        [4-bit parameter id]        - (optional) further specifies the type of data (e.g. dendrite weighting number)
        [16-bit data]

        Subheaders:
        0b000   - membrane potential
        0b001   - board type
        0b010   - unique id
        0b011   - operating mode
        0b100   - board-specific parameter

        At this time, board-specific parameters (for Parameter ID) have not been fully assigned.
        But some values can be found in their respective firmware repos in main.c

        Example Data Message:

        Data message from an interneuron on channel 3 sending its membrane potential:
        0b[1][010][000][011][0][0000][0000001101011100]

        Motor neuron on channel 1 sending dendrite 1 value of 2048
        1                   [1-bit High]
        010                 [Data Header]
        100                 [board-specific parameter]
        001                 [channel 1]
        0                   [fire flag 0]
        0100                [Dendrite 1 Parameter ID]   *see motor neuron repo for Parameter IDS (NOT FINAL)
        0000100000000000    [dendrite 1 = 2048]



    3. Upstream neurons sending pulses to downstream neurons (axon -> dendrite).

        All neuron->neuron communications are maintained by the downstream ping which distinguishes
        excitatory and inhibitory connections.

        Only two messages sent in this category

        0b1011 - Downstream ping message
        0b1111 - Downstream pulse message
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
extern volatile uint8_t pause_flag;
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
