#ifndef _CBC_H
#define _CBC_H

#include "daq2.h"
#include "calc2.h"

typedef enum{
  PROX_CBCS_SERVO = 1,
  HYBRID_CBCS_SERVO = 1<<1,
  TRANS_CBCS_SERVO = 1<<2,
  BUTTON_CBCS_SERVO = 1<<3,
  HYBRID_BUTTON_CBCS_SERVO = 1<<4,
  DOZE_CBCS_SERVO = 1<<5
} cbc_servo_command;

void getCBCs(DAQVarId_t base, uint16 *cbcs, uint16 len);
void setCBCs(DAQVarId_t base, uint16 *cbcs, uint16 len);
void findBestCbcs(cbc_servo_command servo_cmd);

#endif // _CBC_H
