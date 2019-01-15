#ifndef __DXS_IRQ_H__
#define __DXS_IRQ_H__
/******************************************************************************

                              Copyright (c) 2014
                            Lantiq Deutschland GmbH

  For licensing information, see the file 'LICENSE' in the root folder of
  this software module.

******************************************************************************/

/**
   \file dxs_irq.h
   This file contains the declaration of the IO controls for DUSLIC XS interrupt
   handling implementation.
*/

/* ========================================================================== */
/*                                 Includes                                   */
/* ========================================================================== */

#include <linux/ioctl.h>

/* ========================================================================= */
/*                             Macros and definitions                        */
/* ========================================================================= */

#ifndef DXS_IRQ_MAJOR
#define DXS_IRQ_MAJOR 0   /* dynamic major by default */
#endif

#define DXS_MAX_DEVICES 1

/* IOCTL MAGIC */
#define DXS_IRQ_MAGIC ('d'+'x'+'s'+'_'+'i'+'r'+'q')

#define DXS_IRQ_NAME   "dxs_int"

typedef struct
{
   /* device number */
   int dev;
   /* irq number */
   int irq;
} DXS_INT_CONF_t;


/**
   Set interrupt number for a device.

   \param   DXS_INT_CONF_t Pointer to interrupt configuration
   \ref DXS_INT_CONF_t structure.

   \return Returns value as follows:
   - 0: if successful
   - negative number: in case of an error
*/
#define DXS_INT_CONF           _IOW(DXS_IRQ_MAGIC, 0x00, DXS_INT_CONF_t)


/**
   Get device on which interrupt has occurred.

   \param   int*  Pointer to integer where the device number will be written,
                  in case no interrupt occurred since last call, -1 will be
                  written

   \return Returns value as follows:
   - 0: if successful
   - negative number: in case of an error
*/
#define DXS_INT_DEV_GET        _IOR(DXS_IRQ_MAGIC, 0x01, int)

/* ========================================================================== */
/*                             Type definitions                               */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function prototypes                              */
/* ========================================================================== */

#endif /* __DXS_IRQ_H__ */
