/* -----------------------------------------------------------------
 *
 *                      COMPANY CONFIDENTIAL
 *                       INTERNAL USE ONLY
 *
 * Copyright (C) 1997 - 2015  Synaptics Incorporated.  All right reserved.
 *
 * This document contains information that is proprietary to Synaptics
 * Incorporated. The holder of this document shall treat all information
 * contained herein as confidential, shall use the information only for its
 * intended purpose, and shall protect the information in whole or part from
 * duplication, disclosure to any other party, or dissemination in any media
 * without the written permission of Synaptics Incorporated.
 *
 * Synaptics Incorporated
 * 1251 McKay Drive
 * San Jose, CA  95131
 * (408) 904-1100
 */

#include "mex.h"
#include <stdio.h>
#include "daq2.h"

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

extern mxArray *daqObject;
// static data
static char errString[1024];

static double getStructFieldValue(const mxArray *s, const char *fieldName);

double getStructFieldValue(const mxArray *s, const char *fieldName)
{
  double value;
  mxArray *field;

  field = mxGetField(s, 0, fieldName);
  if (field == NULL || (0 == mxIsNumeric(field) && 0 == mxIsLogical(field)))
  {
    snprintf(errString, 1024, "Error reading field %s", fieldName);
    mexErrMsgIdAndTxt("DAQ2:call_daq:argumentError", errString);
  }
  value = mxGetScalar(field);
  return value;
}

// DAQ2 API functions

void DAQ_startContinuousAcquisition(int16 type)
{
  mxArray *prhs[2];

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(type);

  if (prhs[1] != NULL)
  {
    mexCallMATLAB(0, NULL, 2, prhs, "startContinuousAcquisition");
    mxDestroyArray(prhs[1]);
  }
}

void DAQ_startSingleAcquisition(int16 type)
{
  mxArray *prhs[2];

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(type);

  if (prhs[1] != NULL)
  {
    mexCallMATLAB(0, NULL, 2, prhs, "startSingleAcquisition");
    mxDestroyArray(prhs[1]);
  }
}

void DAQ_stopAcquisition()
{
  mxArray *prhs[1];
  prhs[0] = daqObject;
  mexCallMATLAB(0, NULL, 1, prhs, "stopAcquisition");
}

DAQFrame_t *DAQ_getFrame(int16 frameType)
{
  mxArray *prhs[2];
  mxArray *f;
  mxArray *buf;
  double *bufPtr;
  int buflen;
  int i;
  static DAQFrame_t daqFrame;

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(frameType);
  mexCallMATLAB(1, &f, 2, prhs, "getFrame");
  // plhs is a DAQFrame struct
  if (mxIsEmpty(f))
  {
    return NULL;
  }
  daqFrame.owner = (uint16) getStructFieldValue(f, "owner");
  daqFrame.sequence = (uint16) getStructFieldValue(f, "sequence");
  daqFrame.type = (int16) getStructFieldValue(f, "type");
  daqFrame.timeStamp = (uint32) getStructFieldValue(f, "timeStamp");
  daqFrame.cycles = (uint32) getStructFieldValue(f, "cycles");
  daqFrame.length = (uint16) getStructFieldValue(f, "length");
  daqFrame.errorFlags.all = (uint16) getStructFieldValue(f, "errorFlags");

  buf = mxGetField(f, 0, "buffer");
  buflen = (int) mxGetNumberOfElements(buf);
  if (buflen > DAQ_MAX_OUTPUT_BUFFER_SIZE)
  {
    mexErrMsgIdAndTxt("DAQ2:call_daq:argumentError",
      "DAQ_getFrame() output buffer is too large\n");
  }
  bufPtr = mxGetPr(buf);
  for (i = 0; i < buflen; i++)
  {
    daqFrame.buffer[i] = (uint16) bufPtr[i];
  }
  for (;i < DAQ_MAX_OUTPUT_BUFFER_SIZE; i++)
  {
    daqFrame.buffer[i] = 0;
  }
  mxDestroyArray(f);

  return &daqFrame;
}

void DAQ_releaseFrame(DAQFrame_t *frame)
{
  mxArray *prhs[2];
  mxArray *buf;
  double *bufPtr;
  int i;
  char *fields[] = {"owner", "sequence", "type", "timeStamp",
                    "cycles", "length", "errorFlags", "buffer"};
  prhs[1] = mxCreateStructMatrix(1, 1, sizeof(fields)/sizeof(fields[0]), fields);
  mxSetField(prhs[1], 0, "owner", mxCreateDoubleScalar(frame->owner));
  mxSetField(prhs[1], 0, "sequence", mxCreateDoubleScalar(frame->sequence));
  mxSetField(prhs[1], 0, "type", mxCreateDoubleScalar(frame->type));
  mxSetField(prhs[1], 0, "timeStamp", mxCreateDoubleScalar(frame->timeStamp));
  mxSetField(prhs[1], 0, "cycles", mxCreateDoubleScalar(frame->cycles));
  mxSetField(prhs[1], 0, "length", mxCreateDoubleScalar(frame->length));
  mxSetField(prhs[1], 0, "errorFlags", mxCreateDoubleScalar(frame->errorFlags.all));

  buf = mxCreateDoubleMatrix(1, frame->length, mxREAL);
  bufPtr = mxGetPr(buf);
  for (i = 0; i < frame->length; i++)
  {
    *bufPtr++ = frame->buffer[i];
  }
  mxSetField(prhs[1], 0, "buffer", buf);

  prhs[0] = daqObject;

  mexCallMATLAB(0, NULL, 2, prhs, "releaseFrame");

  mxDestroyArray(prhs[1]);
}

void DAQ_writeVar(uint16 addr, uint16 val)
{
  mxArray *prhs[3];

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(addr);
  prhs[2] = mxCreateDoubleScalar(val);

  mexCallMATLAB(0, NULL, 3, prhs, "writeVar");
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
}

void DAQ_writeVars(DAQVarId_t *ids, uint16 *vals, uint16 n)
{
  mxArray *prhs[3];
  int i;
  double *p;

  prhs[0] = daqObject;
  prhs[1] = mxCreateNumericMatrix(1, n, mxDOUBLE_CLASS, mxREAL);
  prhs[2] = mxCreateNumericMatrix(1, n, mxDOUBLE_CLASS, mxREAL);
  p = mxGetPr(prhs[1]);
  for (i = 0; i < n; i++)
  {
    p[i] = ids[i];
  }
  p = mxGetPr(prhs[2]);
  for (i = 0; i < n; i++)
  {
    p[i] = vals[i];
  }

  mexCallMATLAB(0, NULL, 3, prhs, "writeVars");
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
}

void DAQ_writeArray(DAQVarId_t id, uint16 *vals, uint16 n)
{
  mxArray *prhs[3];
  int i;
  double *p;

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(id);
  prhs[2] = mxCreateNumericMatrix(1, n, mxDOUBLE_CLASS, mxREAL);
  p = mxGetPr(prhs[2]);
  for (i = 0; i < n; i++)
  {
    p[i] = vals[i];
  }

  mexCallMATLAB(0, NULL, 3, prhs, "writeArray");
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
}

void DAQ_writeRepeat(DAQVarId_t id, uint16 val, uint16 len)
{
  mxArray *prhs[4];

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(id);
  prhs[2] = mxCreateDoubleScalar(val);
  prhs[3] = mxCreateDoubleScalar(len);

  mexCallMATLAB(0, NULL, 4, prhs, "writeRepeat");
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
}

void DAQ_writeVarAsync(uint16 addr, uint16 val)
{
  mxArray *prhs[3];

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(addr);
  prhs[2] = mxCreateDoubleScalar(val);

  mexCallMATLAB(0, NULL, 3, prhs, "writeVarAsync");
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
}

uint16 DAQ_readVar(uint16 addr)
{
  mxArray *plhs[1], *prhs[2];
  uint16 rv;

  prhs[0] = daqObject;
  prhs[1] = mxCreateDoubleScalar(addr);

  if (prhs[1] != NULL)
  { // guaranteed never NULL when run from Matlab
    mexCallMATLAB(1, plhs, 2, prhs, "readVar");
    rv = (int16) mxGetScalar(plhs[0]);
    mxDestroyArray(prhs[1]);
    mxDestroyArray(plhs[0]);
  }
  return rv;
}

/* FIXME: Wrap getVersion */
