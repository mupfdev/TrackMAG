/*
 * app_trackmag.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Michael Fitzmayer
 */

#ifndef APP_TRACKMAG_H
#define APP_TRACKMAG_H

typedef enum
{
  APP_INIT = 0,
  APP_RUN

} app_state_t;

void app_update(app_state_t* state);

#endif /* APP_TRACKMAG_H */
