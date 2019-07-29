/*
 * hardware.h
 *
 *  Created on: Apr 24, 2019
 *      Author: raistlin
 */

#ifndef HARDWARE_H_
#define HARDWARE_H_

// uncomment this define for dummy board
#define BOARD_DUMMY
#define BUFFERED_DISPLAY
// uncomment this define for production board
//#define BOARD_PRODUCTION

//dummy board hardware defines
#ifdef BOARD_DUMMY
#define DISPLAY_NOKIA5110
#endif

//production board hardware defines
#ifdef BOARD_PRODUCTION
#define DISPLAY_0152G
#endif

#endif /* HARDWARE_H_ */
