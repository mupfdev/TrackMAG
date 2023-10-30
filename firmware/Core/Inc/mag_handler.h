/*
 * sensor_handler.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Michael Fitzmayer
 */

#ifndef MAG_HANDLER_H
#define MAG_SENSOR_HANDLER_H

typedef enum
{
  MAG_INIT = 0,
  MAG_RUN

} mag_state_t;

void mag_update(mag_state_t* state);

#endif /* MAG_HANDLER_H */
