
#include "neuron.h"
#include "comm.h"

void neuronInit(neuron_t *n)
{
	uint8_t i;

	n->potential = 0;
	n->state = INTEGRATE;

	n->fire_time = 0;
	n->fire_potential = 0;

	n->hebb_time = 0;
	n->learning_state = NONE;

	n->leaky_current = 0;

	for (i=0;i<NUM_DENDS;i++){
		n->dendrites[i].state = OFF;
		n->dendrites[i].current_value = 0;
		n->dendrites[i].type = EXCITATORY;
		n->dendrites[i].timestamp = LEARNING_WINDOW;
		n->dendrites[i].pulse_time = LEARNING_WINDOW;
		n->dendrites[i].alive_time = LEARNING_WINDOW;
	}
}

void checkDendrites(neuron_t * n)
{
	uint8_t i;
	
	for (i=NUM_AXONS; i<NUM_INPUTS; i++){
		
		// check if dendrite has received a new ping
		if (dendrite_ping_flag[i] != 0){

			dendrite_ping_flag[i] = 0;

			n->dendrite_ping_time[i] = DEND_PING_TIME;
		} else if (n->dendrite_ping_time[i] == 1){	
			// dendrite ping has expired; reset the dendrite to inputs		
			setAsInput(complimentary_ports[i], complimentary_pins[i]);
			active_output_pins[COMPLIMENTARY_I(i)] = 0;
		}
		
		// incremenet dendrite_ping_time
		if (n->dendrite_ping_time[i] > 0){
			n->dendrite_ping_time[i] -= 1;
		}
		
		// check if dendrite has received a pulse
		if (dendrite_pulse_flag[i] != 0){
			dendrite_pulse_flag[i] = 0;

			n->dendrites[DENDRITE_I(i)].type = ((IS_EXCITATORY(i)) ? EXCITATORY : INHIBITORY);
			n->dendrites[DENDRITE_I(i)].state = ON;
			n->dendrites[DENDRITE_I(i)].pulse_time = 0;
		}
	}
	
	for (i=0; i < NUM_DENDS; i++){
		// switch dendrite off when pulse has expired
		if(n->dendrites[i].state == ON){
			if (n->dendrites[i].pulse_time == 0)
				n->dendrites[i].timestamp = 0;
			n->dendrites[i].pulse_time += 1;
			if (n->dendrites[i].pulse_time >= PULSE_LENGTH){
				dendriteSwitchOff(&(n->dendrites[i]));
			}
		}
		if (n->dendrites[i].timestamp < LEARNING_WINDOW)
			n->dendrites[i].timestamp++;
	}
	
}

void calcDendriteWeightings(neuron_t * n)
{
	uint8_t i;
	for (i=0; i<NUM_DENDS; i++){
		if (n->dendrites[i].timestamp < LEARNING_WINDOW){
			n->dendrites[i].magnitude += (LEARNING_WINDOW - n->dendrites[i].timestamp) * LEARNING_CHANGE / LEARNING_WINDOW;
			if (n->dendrites[i].magnitude > MAX_WEIGHTING)
				n->dendrites[i].magnitude = MAX_WEIGHTING;
		}
		n->dendrites[i].timestamp = LEARNING_WINDOW;
	}
}

void incrementHebbTime(neuron_t * n)
{
	uint8_t i;

	n->ms_count++;
	if (n->ms_count == n->time_multiple){
		n->hebb_time++;
		n->ms_count = 0;
	}

	if (n->hebb_time == UINT16_MAX - 1){
		n->hebb_time /= 2;
		for (i=0; i<NUM_DENDS; i++){
			n->dendrites[i].timestamp /= 2;
		}
		n->time_multiple *= 2;
	}
}

void dendriteSwitchOff(dendrite_t *dendrite)
{
	dendrite->state = OFF;
	dendrite->pulse_time = 0;
	
	switch(dendrite->type){
		case EXCITATORY:
			dendrite->current_value += dendrite->magnitude;
			break;
		case INHIBITORY:
			dendrite->current_value -= dendrite->magnitude;
			break;
	}
}

void dendriteDecayStep(neuron_t * n)
{
	uint8_t i;

	for(i=0; i<NUM_DENDS; i++){
		n->dendrites[i].current_value = (n->dendrites[i].current_value * 63 ) / 64;
	}
}

void membraneDecayStep(neuron_t * n)
{
	n->fire_potential = (n->fire_potential * 63) / 64;
}

int16_t calcNeuronPotential(neuron_t *n)
{
	uint8_t i;
	int16_t new_v = 0;
	for (i=0; i<NUM_DENDS; i++){
		if (n->dendrites[i].state == ON){
			switch(n->dendrites[i].type){
				case EXCITATORY:
					new_v += n->dendrites[i].magnitude;
					break;
				case INHIBITORY:
					new_v -= n->dendrites[i].magnitude;
					break;
			}
		}
		new_v += n->dendrites[i].current_value; // each dendrite contributes its decay (*.current_value) and magnitude
	}
	return new_v;
}
