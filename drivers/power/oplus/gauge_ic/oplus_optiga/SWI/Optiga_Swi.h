#ifndef OPTIGA_SWI_H
#define OPTIGA_SWI_H

#include "../oplus_optiga.h"

#define SWI_BAUDRATE_DEF  (100000UL)
#define SWI_BAUDRATE_MAX (500000UL)
#define SWI_BAUDRATE_MIN  (10000UL)

/* Transaction Elements */
/* BroadCast */
#define SWI_BC              (0x08u)    /* Bus Command */
#define SWI_EDA             (0x09u)    /* Extended Device Address */
#define SWI_SDA             (0x0Au)    /* Slave Device Address */
#define SWI_MDA             (0x0Bu)    /* Master Device Address */

/* MultiCast */
#define SWI_WD              (0x04u)    /* Write Data */
#define SWI_ERA             (0x05u)    /* Extended Register Address */
#define SWI_WRA             (0x06u)    /* Write Register Address */
#define SWI_RRA             (0x07u)    /* Read Register Address */

/* Unicast */
#define SWI_RD_ACK          (0x0Au)    /* ACK and not End of transmission */
#define SWI_RD_NACK         (0x08u)    /* ACK and not End of transmission */
#define SWI_RD_ACK_EOT      (0x0Bu)    /* ACK and not End of transmission */
#define SWI_RD_NACK_EOT     (0x09u)    /* ACK and not End of transmission */

/* Bus Command */
#define SWI_BRES            (0x00u)    /* Bus Reset */
#define SWI_PDWN            (0x02u)    /* Bus Power down */
#define SWI_EXBC            (0x08u)    /* Extent 00001xxx */
#define SWI_ESSM            (0x08u)    /* Enter Standard Speed Mode */
#define SWI_EHSM            (0x09u)    /* Enter High Speed Mode */
#define SWI_EINT            (0x10u)    /* Enable Interrupt */
#define SWI_RBL0            (0x20u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL1            (0x21u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL2            (0x22u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL3            (0x23u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL4            (0x24u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL5            (0x25u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL6            (0x26u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_RBL7            (0x27u)    /* RBLn Set Read Burst Length 2^n */
#define SWI_DISS            (0x30u)    /* Device ID Search Start */
#define SWI_DIMM            (0x32u)    /* Device ID Search Memory */
#define SWI_DIRC            (0x33u)    /* Device ID Search Recall */
#define SWI_DIE0            (0x34u)    /* Device ID Search Enter 0 */
#define SWI_DIE1            (0x35u)    /* Device ID Search Enter 1 */
#define SWI_DIP0            (0x36u)    /* Device ID Search Probe 0 */
#define SWI_DIP1            (0x37u)    /* Device ID Search Probe 1 */
#define SWI_DI00            (0x38u)    /* DIS Enter 0 Probe 0 (DIE0 + DIP0) */
#define SWI_DI01            (0x39u)    /* DIS Enter 0 Probe 1 (DIE0 + DIP1) */
#define SWI_DI10            (0x3Au)    /* DIS Enter 1 Probe 0 (DIE1 + DIP0) */
#define SWI_DI11            (0x3Bu)    /* DIS Enter 1 Probe 1 (DIE1 + DIP1) */
#define SWI_DASM            (0x40u)    /* Device Activation Stick Mode */
#define SWI_DACL            (0x41u)    /* Device Activation Clear */
#define SWI_ECFG            (0x50u)    /* Enable Configuration Space */
#define SWI_EREG            (0x51u)    /* Enable Register Space */
#define SWI_DMAC            (0x60u)    /* Default Master Activation */
#define SWI_CURD            (0x70u)    /* Call for Un-registered Devices */
#define SWI_DCXX            (0xC0u)    /* Device Specific Commands DC00-DC31 11xxxxxx */

/* Configration Register Address */
#define SWI_CTRL0           (0x00u)    /* Control Register 0 */
#define SWI_CTRL1           (0x01u)    /* Control Register 1 */
#define SWI_CTRL2           (0x02u)    /* Control Register 2 */
#define SWI_CTRL7           (0x07u)   /* Control Register 7 */
#define SWI_CAP0            (0x08u)    /* Capability Register 0 */
#define SWI_CAP1            (0x09u)    /* Capability Register 1 */
#define SWI_CAP4            (0x0Cu)    /* Capability Register 4 */
#define SWI_CAP5            (0x0Du)    /* Capability Register 5 */
#define SWI_CAP6            (0x0Eu)    /* Capability Register 6 */
#define SWI_CAP7            (0x0Fu)    /* Capability Register 7 */
#define SWI_INT0            (0x18u)    /* Interrupt Register 0 */
#define SWI_INT4            (0x1Cu)    /* Interrupt Register 4 */
#define SWI_DADR0           (0x20u)    /* Device Address Register 0 */
#define SWI_DADR1           (0x21u)    /* Device Address Register 1 */
#define SWI_DADR2           (0x22u)    /* Device Address Register 2 */
#define SWI_DADR3           (0x23u)    /* Device Address Register 3 */

#define SWI_STAT8           (0x38u)    /* Status Register 8 */
#define SWI_STAT12          (0x3Cu)    /* Status Register 12 */
#define SWI_UID0            (0x40u)    /* UID Register 0 */
#define SWI_UID1            (0x41u)    /* UID Register 1 */
#define SWI_UID2            (0x42u)    /* UID Register 2 */
#define SWI_UID3            (0x43u)    /* UID Register 3 */
#define SWI_UID4            (0x44u)    /* UID Register 4 */
#define SWI_UID5            (0x45u)    /* UID Register 5 */
#define SWI_UID6            (0x46u)    /* UID Register 6 */
#define SWI_UID7            (0x47u)    /* UID Register 7 */
#define SWI_UID8            (0x48u)    /* UID Register 8 */
#define SWI_UID9            (0x49u)    /* UID Register 9 */
#define SWI_UID10           (0x4Au)    /* UID Register 10 */
#define SWI_UID11           (0x4Bu)    /* UID Register 11 */

#define SWI_VNUM            (0x50u)    /* Version Number Register */

/* OPTIGA Bus Command */
#define SWI_OPTIGA_ECCSTART  (0xC0u)    /* ECC Start Command */
#define SWI_OPTIGA_ECCESTART (0xC1u)    /* ECCE Start Command */

/* OPTIGA Register */
#define SWI_OPTIGA_CTRL_SPACE		(0x0266u)
#define SWI_OPTIGA_NVM_CAL           (0x0268u)  /* NVM_Cal*/
#define SWI_OPTIGA_BG_CAL            (0x0269u)  /* BG_Cal */
#define SWI_OPTIGA_VCO_CAL           (0x026Bu)  /* VCO_Cal */
#define SWI_OPTIGA_PTAT_CAL          (0x026Du)  /* PTAT_Cal */
#define SWI_OPTIGA_CUR_CAL           (0x026Eu)  /* Cur_Cal */

#define SWI_OPTIGA_CTRL0_CLK         (0x0270u)  /* CTRL0_CLK */
#define SWI_OPTIGA_CTRL1_ADC         (0x0271u)  /* CTRL1_ADC */
#define SWI_OPTIGA_CTRL2_NVM         (0x0272u)  /* CTRL2_NVM */
#define SWI_OPTIGA_ST_CTRL           (0x0273u)  /* ST_CTRL */
#define SWI_OPTIGA_NVM_ADDR          (0x0274u)  /* NVM_ADDR */
#define SWI_OPTIGA_NVM_LOCK          (0x0275u)  /* NVM Lock control */
#define SWI_OPTIGA_NVM_LOCK_ST       (0x0276u)  /* NVM lock status */

#define SWI_ECC_BASE                (0x0000u)  /* ECC Area Base Address */
#define SWI_NVM_BASE                (0x0100u)  /* NVM Area Base Address */
#define SWI_CRYPTO_BASE             (0x0300u)  /* CRY Area Base Address */

/* NVM */
#define SWI_NVM_WIP3                (0x0018u)  /* Page Address of WIP3 */
#define SWI_NVM_WIP2                (0x0010u)  /* Page Address of WIP2 */
#define SWI_NVM_WIP1                (0x0028u)  /* Page Address of WIP1 */
#define SWI_NVM_WIP0                (0x0020u)  /* Page Address of WIP0 */


typedef struct _S_OPTIGA_PUID
{
	uint16_t uwVendorId;
	uint16_t uwProductId;
	uint32_t ulIdHigh;
	uint32_t ulIdLow;
}S_OPTIGA_PUID;

/* *** SWI High Level Functions *** */
BOOL Swi_ReadActualSpace( uint8_t * ubp_Data );
BOOL Swi_ReadRegisterSpace( uint16_t uw_Address, uint8_t * ubp_Data);
BOOL Swi_WriteRegisterSpace( uint16_t uw_Address, uint8_t ub_Data, uint8_t ub_BitSelect, BOOL b_WaitForInterrupt, BOOL b_ImmediateInterrupt, BOOL * bp_IrqDetected);
BOOL Swi_WriteRegisterSpaceNoIrq( uint16_t uw_Address, uint8_t ub_Data, uint8_t ub_BitSelect );
BOOL Swi_ReadConfigSpace( uint16_t uw_Address, uint8_t * ubp_Data );
BOOL Swi_WriteConfigSpace( uint16_t uw_Address, uint8_t ub_Data, uint8_t ub_BitSelect );
BOOL Swi_SearchPuid( uint8_t ub_BitsToSearch, S_OPTIGA_PUID * stp_DetectedPuid );
BOOL Swi_SearchMultiplePuid(uint8_t ub_BitsToSearch,  S_OPTIGA_PUID *stp_DetectedPuid, uint8_t * ubp_DevCnt );
BOOL Swi_SelectByPuid( uint8_t ub_BitsToExecute,  S_OPTIGA_PUID * stp_DeviceToSelect, uint16_t uw_AssignDevAdr );
void Swi_SelectByAddress( uint16_t uw_DeviceAddress );
void Swi_SetAddress(uint16_t uw_DeviceAddress);
void Swi_PowerDown( void );
void Swi_PowerUp( void );
void Swi_Reset( void );
void Swi_PowerDownCmd (void);
BOOL Swi_ReadUID(uint8_t* UID);
BOOL Swi_SearchMultiplePuid(uint8_t ub_BitsToSearch,  S_OPTIGA_PUID *p_DetectedPuid, uint8_t * ubp_DevCnt );

/* *** SWI Low Level Functions *** */
/* *** SWI Low Level Functions *** */
void Swi_SendRawWord( uint8_t ub_Code, uint8_t ub_Data, BOOL b_WaitForInterrupt, BOOL b_ImmediateInterrupt, BOOL * bp_IrqDetected );
void Swi_SendRawWordNoIrq( uint8_t ub_Code, uint8_t ub_Data );
BOOL Swi_ReceiveRawWord( uint8_t * up_Byte );
BOOL Swi_TreatInvertFlag(uint8_t Code, uint8_t Data);
void Swi_AbortIrq( void );
void Swi_WaitForIrq( BOOL * bp_IrqDetected, BOOL b_Immediate );

#endif
