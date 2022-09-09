#include "Optiga_Nvm.h"

extern uint32_t g_culNvmTimeout;	//6ms
static uint8_t ubUID;
//Forcefully, update the current register pointer at 0x275 to ECFG space
//Note: This code only applies to lower byte of vendor ID = 0x29
void Nvm_SwitchUID( uint32_t uid ) {
	switch (uid) {
	case Optiga_UID:
		ubUID = 0x2a;
		break;
	case Oplus_UID:
		ubUID = 0x35;
		break;
	default:
		ubUID = 0x35;
		break;
	}
}

BOOL move_register_pointer()
{
	uint8_t ubUID10=0x00, cnt = 0;

	do
	{
		ubUID10=0x00;
		if ((Swi_ReadConfigSpace(SWI_UID10, &ubUID10)==FALSE) || (cnt++ > 30))
		{
			return FALSE;
		}
	}while(ubUID10 != ubUID);

	return TRUE;
}


/* ****************************************************************************
   name:      Nvm_ProgramData()

   function:  program desired data values into OPTIGA NVM address space.
              the option b_VerifyData can be used to re-read the programmed
              data to be sure that the data was written correct.

   input:     IN: b_WaitForFinish
                if 'true', then the host waits for the NVM state-mashine to
                finish before returning from that function.
              IN: b_VerifyData
                if 'true', then the written data is cross-checked with the
                content of the write buffer.
              IN: uw_Address
                start address within NVM to program data beginning from.
              IN: ub_BytesToProgram
                number of bytes to program.
              IN: * ubp_Data
                pointer to buffer holding values to program into NVM.
   output:    bool

   return:    'true', if write access was executed without any problems.
              'false', if failures happened

   date:			2010-03-04: time out added.
 ************************************************************************* */
BOOL Nvm_ProgramData( BOOL b_WaitForFinish, BOOL b_VerifyData, uint16_t uw_Address, uint8_t ub_BytesToProgram, uint8_t * ubp_Data )
{

	BOOL bResult;
	uint8_t ubData;
	uint16_t uwDataCount;

	uint16_t uwAddress;
	uint8_t ubAddressH;
	uint8_t ubAddressL;

	uint32_t ulNvmTimeOut;


	/* check not allowed settings */
	if( (b_WaitForFinish == FALSE) && (b_VerifyData == TRUE) )
	{
		return FALSE;
	}

	/* remove NVM offset */
	uw_Address &= 0x00FFu;

	for( uwDataCount = 0u; uwDataCount < ub_BytesToProgram; uwDataCount++ )
	{

		uwAddress = uw_Address + uwDataCount;
		ubAddressH = (uint8_t)((uwAddress >> 3u) & 0x1Fu);
		ubAddressH |= 0xC0u;
		ubAddressL = (uint8_t)(uwAddress & 0x07u);

		/* wait for NVM state-machine to be ready for a new command */
		ulNvmTimeOut = g_culNvmTimeout;
		do
		{
			if( Swi_ReadRegisterSpace( SWI_OPTIGA_CTRL2_NVM, &ubData ) == FALSE )
			{
				return FALSE;
			}
			/* check for timeout */
			if( ulNvmTimeOut == 0u )
			{
				return FALSE;
			}
			ulNvmTimeOut--;
		}while( (ubData & 0x80u) != 0u );

		/* functions return always true, if mask is set to 0xFF */
		(void)Swi_WriteRegisterSpaceNoIrq( SWI_NVM_WIP0 | ubAddressL, ubp_Data[uwDataCount], 0xFFu );
		(void)Swi_WriteRegisterSpaceNoIrq( SWI_OPTIGA_NVM_ADDR, ubAddressL, 0xFFu );
		if (move_register_pointer()==FALSE)
			return FALSE;
		(void)Swi_WriteRegisterSpaceNoIrq( SWI_OPTIGA_CTRL2_NVM, ubAddressH, 0xFFu );
	}

	/* if user wants to wait until write is done, then poll as long as programming requires */
	if( b_WaitForFinish == TRUE )
	{
		ulNvmTimeOut = g_culNvmTimeout;
		do
		{
			if( Swi_ReadRegisterSpace( SWI_OPTIGA_CTRL2_NVM, &ubData ) == FALSE )
			{
				return FALSE;
			}
			/* check for timeout */
			if( ulNvmTimeOut == 0u )
			{
				return FALSE;
			}
			ulNvmTimeOut--;
		}while( (ubData & 0x80u) != 0u );
	}


	if( b_VerifyData == TRUE )
	{
		for( uwDataCount = 0u; uwDataCount < ub_BytesToProgram; uwDataCount++ )
		{
			bResult = Nvm_ReadData( uw_Address + uwDataCount, 1u, &ubData );
			if( (bResult == FALSE) || (ubData != ubp_Data[uwDataCount]) )
			{
				return FALSE;
			}
		}
	}

	/* all ok */
	return TRUE;

}



/* ****************************************************************************
   name:      Nvm_ReadData()

   function:  read data from requested NVM address and store data into
              provided buffer.

   input:     IN: uw_Address
                start address to read data from NVM.
              IN: ub_BytesToRead
                number of bytes to read from NVM.
              OUT: * ubp_Data 
                pointer to buffer to store read data into.
   output:    bool

   return:    'true', if reading was ok  .
              'false', if reading failed.

   date:			2010-03-04: time out added.
 ************************************************************************* */
BOOL Nvm_ReadData( uint16_t uw_Address, uint8_t ub_BytesToRead, uint8_t * ubp_Data )
{

	uint16_t uwDataCount;
	uint8_t ubData;

	uint16_t uwAddress;
	uint8_t ubAddressH;
	uint8_t ubAddressL;

	uint32_t ulNvmTimeOut;


	/* remove NVM offset */
	uw_Address &= 0x00FFu;

	for( uwDataCount = 0u; uwDataCount < ub_BytesToRead; uwDataCount++ )
	{
		uwAddress = uw_Address + uwDataCount;
		ubAddressH = (uint8_t)((uwAddress >> 3u) & 0x1Fu);
		ubAddressH |= 0x80u;
		ubAddressL = (uint8_t)(uwAddress & 0x07u);

		ulNvmTimeOut = g_culNvmTimeout;
		do
		{
			if( Swi_ReadRegisterSpace( SWI_OPTIGA_CTRL2_NVM, &ubData ) == FALSE )
			{
				return FALSE;
			}
			/* check for timeout */
			if( ulNvmTimeOut == 0u )
			{
				return FALSE;
			}
			ulNvmTimeOut--;
		}while( (ubData & 0x80u) != 0u );


		/* functions return always true, if mask is set to 0xFF */
		(void)Swi_WriteRegisterSpaceNoIrq( SWI_OPTIGA_NVM_ADDR, ubAddressL, 0xFFu );
		if (move_register_pointer()==FALSE)
			return FALSE;
		(void)Swi_WriteRegisterSpaceNoIrq( SWI_OPTIGA_CTRL2_NVM, ubAddressH, 0xFFu );

		ulNvmTimeOut = g_culNvmTimeout;
		do
		{
			if( Swi_ReadRegisterSpace(SWI_OPTIGA_CTRL2_NVM, &ubData) == FALSE )
			{
				return FALSE;
			}
			/* check for timeout */
			if( ulNvmTimeOut == 0u )
			{
				return FALSE;
			}
			ulNvmTimeOut--;
		}while( (ubData & 0x80u) != 0u );

		if( Swi_ReadRegisterSpace( SWI_NVM_WIP2 | ubAddressL, &ubData ) == FALSE )
		{
			return FALSE;
		}
		ubp_Data[uwDataCount] = ubData;
	}

	/* all ok */
	return TRUE;

}



/* ****************************************************************************
   name:      Nvm_DecreaseLifeSpanCounter()

   function:  try to decrease LifeSpanCounter by one.

   input:     -
   output:    bool

   return:    'true', if decrease was triggered.
              'false', if decrease failed.

   date:			2010-03-04: time out added.
 ************************************************************************* */
BOOL Nvm_DecreaseLifeSpanCounter( void )
{

	uint8_t ubData;

	uint32_t ulNvmTimeOut;


	/* wait for NVM state-machine to be ready for a new command */
	ulNvmTimeOut = g_culNvmTimeout;
	do
	{
		if( Swi_ReadRegisterSpace( SWI_OPTIGA_CTRL2_NVM, &ubData ) == FALSE )
		{
			return FALSE;
		}
		/* check for timeout */
		if( ulNvmTimeOut == 0u )
		{
			return FALSE;
		}
		ulNvmTimeOut--;
	}while( (ubData & 0x80u) != 0u );

	/* function returns always true, if mask is set to 0xFF */
	(void)Swi_WriteRegisterSpaceNoIrq( SWI_OPTIGA_NVM_ADDR, 0x20u, 0xFFu );
	if (move_register_pointer()==FALSE)
		return FALSE;


	return TRUE;

}



/* ****************************************************************************
   name:      Nvm_VerifyLifeSpanCounter()

   function:  check that the LifeSpanCounter is within a valid nuber range

   input:     OUT: * bp_IsValid
                pointer to bool to store verification state of the 
                LifeSpanCounter into.
   output:    bool

   return:    'true', if read access was ok.
              'false', if read access failed.

   date:      .
 ************************************************************************* */
BOOL Nvm_VerifyLifeSpanCounter( BOOL * bp_IsValid )
{

	uint32_t ulLifeSpanCount;


	/* for case of fail, the LSP also will set to false */
	*bp_IsValid = FALSE;

	if( Nvm_ReadLifeSpanCounter( &ulLifeSpanCount ) == TRUE )
	{
		if( ulLifeSpanCount <= 100000u )
		{
			*bp_IsValid = TRUE;
		}
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}



/* ****************************************************************************
   name:      Nvm_ReadLifeSpanCounter()

   function:  read current setting of the LifeSpanCounter.

   input:     OUT: * ulp_LifeSpanCounter
                pointer to uint32_t to store current LifeSpanCounter state
                into.
   output:    bool

   return:    'true', if LifeSpanCounter reading was ok.
              'false', if LifeSpanCounter reading failed.

   date:      v0.93; 2009-05-25: size optimized
 ************************************************************************* */
BOOL Nvm_ReadLifeSpanCounter( uint32_t * ulp_LifeSpanCounter )
{

	uint8_t ubCount;
	uint8_t ubData[ 4 ];
	uint32_t ulResult = 0UL;


	/* read data from NVM */
	if( Nvm_ReadData( 0x148u, 4u, ubData ) == FALSE )
	{
		return FALSE;
	}

	/* create result */
	ubCount = 4;
	do
	{
		ubCount--;	/* NOTE: prevent MISRA-C rule 12.13 violation. */
		ulResult = (ulResult << 8u) | ubData[ubCount];
	}while( ubCount != 0u );
	*ulp_LifeSpanCounter =  ulResult;


	/* all done well */
	return TRUE;

}

/* ****************************************************************************
   name:      Nvm_LockNVM()

   function:  Lock NVM

   input:     IN: ub_PageNumber
                page number to be locked.

   return:    'true', if lock was ok.
              'false', if lock failed.

 ************************************************************************* */
BOOL Nvm_LockNVM( uint8_t ub_PageNumber )
{

	return Swi_WriteRegisterSpaceNoIrq (SWI_OPTIGA_NVM_LOCK, (1<<ub_PageNumber), 0xff);

}

/* ****************************************************************************
   name:      Nvm_ReadLockNVM()

   function:  Read Lock NVM page

   input:     IN: *ubp_PageNumber
                pointer to page number that is locked.

   return:    'true', if lock was ok.
              'false', if lock failed.

 ************************************************************************* */
BOOL Nvm_ReadLockNVM( uint8_t * ubp_PageNumber )
{

	return Swi_ReadRegisterSpace (SWI_OPTIGA_NVM_LOCK, ubp_PageNumber);

}

