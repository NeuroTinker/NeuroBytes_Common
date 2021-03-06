#ifndef NEURON_H_
#define NEURON_H_

#include "comm.h"

#define DENDRITE_I(i)         ((i-NUM_AXONS) / 2)

#define MEMBRANE_THRESHOLD      10000
#define HYPERPOLARIZATION		-10000
#define PULSE_LENGTH            12 // led white time

#define DENDRITE_ALIVE_TIME     200
#define FIRE_LED_TIME           5
#define FIRE_DELAY_TIME         10

#define LEARNING_WINDOW         100
#define LEARNING_CHANGE         500
#define MAX_WEIGHTING           18000

#define DEPRESSION_TIME         50

typedef enum{
NC =   0,
CONNECTED =    1
} dendrite_status;

typedef enum{
OFF =   0,
ON =    1
} dendrite_states;

typedef enum{
EXCITATORY = 0,
INHIBITORY = 1
} dendrite_types;

typedef struct{
uint32_t          port;
uint16_t          pin;
dendrite_states     state; // is pulse being received or not
dendrite_status     status; // not connected / connected
dendrite_types      type; // excitatory or inhibitory
uint16_t            alive_time;
int32_t             magnitude; // weighting
int32_t             base_magnitude;
int16_t             current_value; // current contribution to the membrane potential
uint8_t             nid_flag; // is this dendrite the closest to the NID
uint8_t             read_flag; // dendrite is currently getting a message
uint8_t             read_time; // message length decrementor
uint8_t             pulse_time; // time elapsed since pulse was received
uint16_t			timestamp; // last input pulse (used for learning)
} dendrite_t;

typedef enum{
INTEGRATE =       0,
FIRE =          1
} neuron_states;

typedef enum{
NONE =	0,
HEBB =	1,
SLEEP =	2
} learning_states;

typedef struct{
// mode independent vars
int16_t     potential;
dendrite_t  dendrites[NUM_DENDS];
uint16_t    dendrite_ping_time[NUM_INPUTS];
neuron_states   state;
learning_states learning_state; // Hebb learning mode

uint16_t    fire_time;
int16_t		fire_potential;

// Hebb vars
uint16_t	hebb_time;
uint8_t		time_multiple;
uint16_t	ms_count;

uint16_t    leaky_current;

} neuron_t;

void neuronInit(neuron_t *n);
void checkDendrites(neuron_t * n);
int16_t calcNeuronPotential(neuron_t *n);
void dendriteDecayStep(neuron_t * n);
void membraneDecayStep(neuron_t * n);
void dendriteSwitchOff(dendrite_t * dendrite);
void incrementHebbTime(neuron_t *n);
void calcDendriteWeightings(neuron_t *n);


#endif
