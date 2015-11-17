#include <string.h>
#include <stddef.h>

#include "calc2.h"
#include "cbc.h"
#include "active_frame.h"

#define LEN(x) (sizeof(x)/sizeof(x[0]))

void getCBCs(DAQVarId_t base, uint16 *cbcs, uint16 len)
{
  uint16 i;
  uint16 nRegs;
  uint16 cbcsPerVar;
  uint16 nBits;
  uint16 *outPtr = cbcs;

  uint16 mask = 0;
  if (IS_T1327)
  {
    cbcsPerVar = 4;
    nBits = 4;
    mask = 0xF;
  }
  else if (IS_TH2411 || IS_TH2432)
  {
    cbcsPerVar = 3;
    nBits = 5;
    mask = 0x1F;
  }
  else if (IS_TD4191)
  {
    cbcsPerVar = 3;
    nBits = 5;
    mask = 0x1F;
  }
  else if(IS_TD4300 || IS_TD4302)
  {
    cbcsPerVar = 4;
    nBits = 4;
    mask = 0xF;
  }
  else if(IS_TD4100)
  {
    cbcsPerVar = 4;
    nBits = 4;
    mask = 0xF;
  }

  nRegs = (len + cbcsPerVar - 1)/cbcsPerVar;

  for (i = 0; i < nRegs; i++)
  {
    uint16 v = DAQ_readVar(base + i);
    uint16 j;
    for (j = 0; j < cbcsPerVar && i * cbcsPerVar + j < len; j++)
    {
      *outPtr++ = v & mask;
      v >>= nBits;
    }
  }
}

void setCBCs(DAQVarId_t base, uint16 *cbcs, uint16 len)
{
  uint16 i;
  uint16 idx;
  uint16 cbcsPerVar;
  uint16 nBits;
  uint16 *inPtr = cbcs;
  uint16 v;
  uint16 field;
  uint16 cbcVals[8];

  if (IS_T1327)
  {
    cbcsPerVar = 4;
    nBits = 4;
  }
  else if (IS_TH2411 || IS_TH2432)
  {
    cbcsPerVar = 3;
    nBits = 5;
  }
  else if (IS_TD4191)
  {
    cbcsPerVar = 3;
    nBits = 5;
  }
  else if(IS_TD4300 || IS_TD4302)
  {
    cbcsPerVar = 4;
    nBits = 4;
  }
  else if(IS_TD4100)
  {
    cbcsPerVar = 4;
    nBits = 4;
  }


  v = 0;
  field = 0;
  idx = 0;

  for (i = 0; i < len; i++)
  {
    v += *inPtr++ << (nBits*field++);
    if (field == cbcsPerVar)
    {
      cbcVals[idx++] = v;
      if (idx == LEN(cbcVals))
      {
        DAQ_writeArray(base, cbcVals, LEN(cbcVals));
        base += LEN(cbcVals);
        idx = 0;
      }
      field = 0;
      v = 0;
    }
  }
  if (field > 0 && field != cbcsPerVar)
  {
    cbcVals[idx++] = v;
  }
  if (idx > 0)
  {
    DAQ_writeArray(base, cbcVals, idx);
  }
}

#if CONFIG_LOCAL_CBC_AUTOSCAN

#if defined(__T100X_VOID_ALIGNMENT__) && (__T100X_VOID_ALIGNMENT__ == 16)
#  define OFFSET_16(x, y) offsetof(x, y)
#else
#  define OFFSET_16(x, y) (offsetof(x, y)/sizeof(uint16))
#endif

static void addS16ScalarToU16Array(uint16 *dst, uint16 len, int16 scalar)
{
  for (; len > 0; len--)
  {
    *dst++ += scalar;
  }
}

void findBestCbcs(cbc_servo_command servo_cmd)
{
  DAQFrame_t *df;
  uint16 lcbcBits = 4;

//TODO: Add other DAQ_ReadVar too.
#if IS_AFE2430
  uint16 TmpCBCpl = 0;
  uint16 hybridMuxCbcTxPlTmp[MAX_RX] = {0};
  uint16 hybridNoMuxCbcTxPlTmp[MAX_TX] = {0};
  uint16 absPl = DAQ_readVar(ABS_ABS_PL);
  uint16 COLS = (DAQ_readVar(NUM_2D_RX_RXSET0)+DAQ_readVar(NUM_2D_RX_RXSET1));
  uint16 ROWS ATTR_UNUSED = (DAQ_readVar(NUM_2D_TX_RXSET0)+DAQ_readVar(NUM_2D_TX_RXSET1));
#else
//  uint16 COLS = DAQ_readVar(NUM_2D_RX);
  uint16 ROWS ATTR_UNUSED = DAQ_readVar(NUM_2D_TX);
#endif

#if CONFIG_ACQ_PROX_ABS
  uint16 proxXCbc[MAX_RX];
  uint16 proxYCbc[MAX_TX];
#endif
#if CONFIG_HYBRID
  uint16 hybridXCbc[MAX_RX];
  uint16 hybridYCbc[MAX_TX];
#endif
  uint16 imageCbc[MAX_RX];
  uint16 dozeCbc[MAX_RX];
#if CONFIG_BUTTON
  uint16 hybridButtonCbc[MAX_BUTTONS];
  uint16 buttonCbc[MAX_BUTTONS];
  uint16 dozeButtonCbc[MAX_BUTTONS];
#endif
#if CONFIG_GUARDS_NEED_CBC
  uint16 proxXGuardCbc[MAX_RX_GUARDS];
  uint16 hybridXGuardCbc[MAX_RX_GUARDS];
  uint16 proxYGuardCbc[MAX_TX_GUARDS];
  uint16 hybridYGuardCbc[MAX_TX_GUARDS];
#endif
  struct array_t {
    uint16 servo_enabled;
    uint16 *cbcs;
    uint16 len;
    DAQVarId_t var;
    uint16 bursts;
    uint16 offset;
    uint16 bits;
    uint16 search_length;
    uint16 column_length;
  } arrays[] = {
#if CONFIG_PROX
    { PROX_CBCS_SERVO, proxXCbc, DAQ_readVar(NUM_2D_RX), PROX_X_CBCS, DAQ_readVar(PROX_X_BURSTS), OFFSET_16(activeFrame_t, proxX), 32, 1, 1 },
    { PROX_CBCS_SERVO, proxYCbc, DAQ_readVar(NUM_2D_TX), PROX_Y_CBCS, DAQ_readVar(PROX_Y_BURSTS), OFFSET_16(activeFrame_t, proxY), 32, 1, 1 },
#endif
#if CONFIG_HYBRID
    { HYBRID_CBCS_SERVO, hybridXCbc, DAQ_readVar(NUM_2D_RX), HYBRID_X_CBCS, DAQ_readVar(HYBRID_X_BURSTS), OFFSET_16(activeFrame_t, hybridX), 16, 1, 1 },
    { HYBRID_CBCS_SERVO, hybridYCbc, DAQ_readVar(NUM_2D_TX), HYBRID_Y_CBCS, DAQ_readVar(HYBRID_Y_BURSTS), OFFSET_16(activeFrame_t, hybridY), 16, 1, 1 },
#endif
    { DOZE_CBCS_SERVO, dozeCbc, DAQ_readVar(NUM_2D_RX), DOZE_CBCS, DAQ_readVar(DOZE_BURSTS_PER_CLUSTER), 0, 16, 2, MAX_RX + MAX_BUTTONS},
#if CONFIG_BUTTON
    { HYBRID_BUTTON_CBCS_SERVO, hybridButtonCbc, MAX_BUTTONS, HYBRID_BUTTON_CBCS, DAQ_readVar(BUTTON_ABS_AXIS)? DAQ_readVar(HYBRID_Y_BURSTS) : DAQ_readVar(HYBRID_X_BURSTS), OFFSET_16(activeFrame_t, hybridButtons), 16, 1, 1 },
    { BUTTON_CBCS_SERVO, buttonCbc, DAQ_readVar(NUM_BUTTON_RX), BUTTON_CBCS, DAQ_readVar(TRANS_BURSTS_PER_CLUSTER), OFFSET_16(activeFrame_t, buttons), 16, DAQ_readVar(CONCURRENT_BUTTON_ACQUISITION)?DAQ_readVar(TRANS_CLUSTERS):1, MAX_BUTTONS },
    { DOZE_CBCS_SERVO, dozeButtonCbc, DAQ_readVar(NUM_BUTTON_RX), DOZE_BUTTON_CBCS, DAQ_readVar(DOZE_BURSTS_PER_CLUSTER), MAX_RX, 16, 2, MAX_RX + MAX_BUTTONS },
#endif
#if CONFIG_GUARDS_NEED_CBC
    { PROX_CBCS_SERVO, proxXGuardCbc, DAQ_readVar(NUM_RX_GUARDS), PROX_RX_GUARD_CBCS, DAQ_readVar(PROX_X_BURSTS), OFFSET_16(activeFrame_t, proxXGuards), 32, 1, 1 },
    { HYBRID_CBCS_SERVO, hybridXGuardCbc, DAQ_readVar(NUM_RX_GUARDS), HYBRID_RX_GUARD_CBCS, DAQ_readVar(HYBRID_X_BURSTS), OFFSET_16(activeFrame_t, hybridXGuards), 16, 1, 1 },
    { PROX_CBCS_SERVO, proxYGuardCbc, DAQ_readVar(NUM_TX_GUARDS), PROX_TX_GUARD_CBCS, DAQ_readVar(PROX_Y_BURSTS), OFFSET_16(activeFrame_t, proxYGuards), 32, 1, 1 },
    { HYBRID_CBCS_SERVO, hybridYGuardCbc, DAQ_readVar(NUM_TX_GUARDS), HYBRID_TX_GUARD_CBCS, DAQ_readVar(HYBRID_Y_BURSTS), OFFSET_16(activeFrame_t, hybridYGuards), 16, 1, 1 },
#endif
    { TRANS_CBCS_SERVO, imageCbc, DAQ_readVar(NUM_2D_RX), TRANS_CBCS, DAQ_readVar(TRANS_BURSTS_PER_CLUSTER), OFFSET_16(activeFrame_t, image), 16, DAQ_readVar(TRANS_CLUSTERS), MAX_RX },
  }; //first element in the struct must be the imagecbcs; second must be trans button cbcs

  int16 mask;
  uint16 i, j;

  if (IS_TH2411 || IS_TH2421)
    lcbcBits = 5;
  else if (IS_T1327)
    lcbcBits = 4;
  else if (IS_TH2432)
    lcbcBits = 3;

  {
    DAQVarId_t regs[] = {PROX_ENABLED, HYBRID_ENABLED, BUTTONS_ABS_ENABLED, BUTTONS_ENABLED, IMAGE_ENABLED, NOISE_ENABLED};
    uint16 vals[] =     {servo_cmd & PROX_CBCS_SERVO, servo_cmd & (HYBRID_CBCS_SERVO|HYBRID_BUTTON_CBCS_SERVO), servo_cmd & HYBRID_BUTTON_CBCS_SERVO, servo_cmd & BUTTON_CBCS_SERVO, servo_cmd & TRANS_CBCS_SERVO, 0};
    DAQ_writeVars(regs, vals, LEN(regs));
  }

  for (i = 0; i < LEN(arrays); i++)
  {
    memset(arrays[i].cbcs, 0, arrays[i].len*sizeof(arrays[i].cbcs[0]));
  }

  for (mask = (1 << (lcbcBits-1)); mask > 0; mask >>= 1)
  {
    struct array_t *arr = arrays;
    for (i = 0; i < LEN(arrays); i++, arr++)
    {
      if ( servo_cmd & arr->servo_enabled )
      {
        addS16ScalarToU16Array(arr->cbcs, arr->len, mask);
        setCBCs(arr->var, arr->cbcs, arr->len);
      }
    }

    if( servo_cmd & DOZE_CBCS_SERVO )
    {
      df = DAQ_getFrame(1);
    }
    else
    {
      df = DAQ_getFrame(0);
    }

    arr = arrays;
    for (i = 0; i < LEN(arrays); i++, arr++)
    {
      if ( servo_cmd & arr->servo_enabled )
      {
        if (arr->bits == 32)
        {
          uint32 *raw = (uint32 *) &df->buffer[arr->offset];
          uint32 threshold = (uint32) 2048*arr->bursts;
          for (j = 0; j < arr->len; j++)
          {
            uint16 k = 0;
            while(k < arr->search_length)
            {
              if (*(raw + k*arr->column_length + j) <= threshold)
              {
                arr->cbcs[j] -= mask;
                break;
              }
              k++;
            }
          }
        }
        else
        {
          uint16 *raw = (uint16 *) &df->buffer[arr->offset];
          uint16 threshold = 2048*arr->bursts;
          for (j = 0; j < arr->len; j++)
          {
            uint16 k = 0;
            while(k < arr->search_length)
            {
              if (*(raw + k*arr->column_length + j) <= threshold)
              {
                arr->cbcs[j] -= mask;
                break;
              }
              k++;
            }
          }
        }
      }
    }
    DAQ_releaseFrame(df);
  }

  //set the final CBC servo result
  {
    struct array_t *arr = arrays;
    for (i = 0; i < LEN(arrays); i++, arr++)
    {
      if ( servo_cmd & arr->servo_enabled )
      {
        setCBCs(arr->var, arr->cbcs, arr->len);
      }
    }
  }
}

#endif
