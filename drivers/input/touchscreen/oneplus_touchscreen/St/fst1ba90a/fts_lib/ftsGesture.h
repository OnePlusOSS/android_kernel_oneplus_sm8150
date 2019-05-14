/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsGesture.h
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/


/*!
  * \file ftsGesture.h
  * \brief Contains all the macro and prototypes to handle the Gesture Detection
  * features
  */


#ifndef FTS_GESTURE_H_
#define FTS_GESTURE_H_



#include "ftsHardware.h"

#define GESTURE_MASK_SIZE               4       /* /< number of bytes of the
                                                 * gesture mask */

#define GESTURE_MAX_COORDS_PAIRS_REPORT 100     /* /< max number of gestures
                                                 * coordinates pairs reported */


int updateGestureMask(u8 *mask, int size, int en);
int disableGesture(u8 *mask, int size);
int enableGesture(u8 *mask, int size);
int enterGestureMode(int reload);
int isAnyGestureActive(void);
int readGestureCoords(u8 *event);
int getGestureCoords(u16 **x, u16 **y);

#endif    /* ! _GESTURE_H_ */
