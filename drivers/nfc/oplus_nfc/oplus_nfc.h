#ifndef __OPLUS_NFC_H__
#define __OPLUS_NFC_H__

#include <stdbool.h>

#define CHECK_NFC_CHIP(chip) \
pr_err("%s : enter\n", __func__); \
if (!is_support_chip(chip)) { \
    pr_err("%s device not support, exit\n", __func__); \
    return -ENOMEM; \
}

typedef enum{
    UNKNOWN = 0,
    NQ330,
    SN100T,
    SN100F,
    ST21H,
    ST54H,
    PN557
} chip_type;

bool is_support_chip(chip_type chip);
bool is_nfc_support(void);

#endif