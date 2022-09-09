/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_kevent_feedback.h
** Description : oplus kevent feedback data
** Version : 1.2
** Date : 2019/03/09
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Guo.Ling          2018/12/03        1.0           Build this moudle
**   LiPing-M          2019/01/29        1.1           Add SMMU for QCOM
**   GaoTing.Gan       2019/03/08        1.2           Add venus dump feedback and public the interface
******************************************************************/
#ifndef _OPLUS_MM_KEVENT_FB_
#define _OPLUS_MM_KEVENT_FB_

enum OPLUS_MM_DIRVER_FB_EVENT_ID {
	OPLUS_MM_DIRVER_FB_EVENT_ID_VIDEO_DUMP = 351,
	OPLUS_MM_DIRVER_FB_EVENT_ID_ESD = 401,
	OPLUS_MM_DIRVER_FB_EVENT_ID_VSYNC,
	OPLUS_MM_DIRVER_FB_EVENT_ID_HBM,
	OPLUS_MM_DIRVER_FB_EVENT_ID_FFLSET,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_CMDQ,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_UNDERFLOW,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_FENCE,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_JS,
	OPLUS_MM_DIRVER_FB_EVENT_ID_SMMU,
	OPLUS_MM_DIRVER_FB_EVENT_ID_GPU_FAULT,/*410*/
	OPLUS_MM_DIRVER_FB_EVENT_ID_PANEL_MATCH_FAULT,
	OPLUS_MM_DIRVER_FB_EVENT_ID_GPU_FENCE_TIMEOUT,
	OPLUS_MM_DIRVER_FB_EVENT_ID_ADSP_RESET = 801,
};

enum OPLUS_MM_DIRVER_FB_EVENT_MODULE {
	OPLUS_MM_DIRVER_FB_EVENT_MODULE_DISPLAY = 0,
	OPLUS_MM_DIRVER_FB_EVENT_MODULE_AUDIO,
	OPLUS_MM_DIRVER_FB_EVENT_MODULE_VIDEO
};

int upload_mm_kevent_feedback_data(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, unsigned char *payload);

#endif /* _OPLUS_MM_KEVENT_FB_ */
