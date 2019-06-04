/***********************************************************
* Description : OnePlus touchpanel driver
* 
* File		  : ftsGesture.h
*
* Function	  : third party interface
* 
* Source	  : provide by fts
*
* Version	  : V1.0
*
***********************************************************/
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
