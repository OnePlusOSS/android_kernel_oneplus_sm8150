#ifndef OPTIGA_NVM_H
#define OPTIGA_NVM_H

#include "../oplus_optiga.h"
#include "Optiga_Swi.h"

typedef enum Nvm_UID
{
	Optiga_UID           = 0x00,
	Oplus_UID            = 0x01,
} UID;

BOOL Nvm_ProgramData( BOOL b_WaitForFinish, BOOL b_VerifyData, uint16_t uw_Address, uint8_t ub_BytesToProgram, uint8_t * ubp_Data );
BOOL Nvm_ReadData( uint16_t uw_Address, uint8_t ub_BytesToRead, uint8_t * ubp_Data );

BOOL Nvm_DecreaseLifeSpanCounter( void );
BOOL Nvm_VerifyLifeSpanCounter( BOOL * bp_IsValid );
BOOL Nvm_ReadLifeSpanCounter( uint32_t * ulp_LifeSpanCounter);
BOOL Nvm_LockNVM( uint8_t ub_PageNumber );
BOOL Nvm_ReadLockNVM( uint8_t * ubp_PageNumber );

void Nvm_SwitchUID( uint32_t id );

#endif
