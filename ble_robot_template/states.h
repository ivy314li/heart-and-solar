/*
 * states.h
 *
 *  Created on: Sep 22, 2018
 *      Author: shromonaghosh
 */

#ifndef STATES_H_
#define STATES_H_

#include <stdio.h>

typedef enum {
    OFF=0,
    NAV=1,
    TURN_90=2,
    DRIVE_STEP=3,
    TRAVERSE=4,
    TURN=5,
    DRIVE=6,
    TURN_CORRECT=7

} states;

#endif /* STATES_H_ */
