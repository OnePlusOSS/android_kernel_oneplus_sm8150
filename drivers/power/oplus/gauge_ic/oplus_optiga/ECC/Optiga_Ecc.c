/**
 * @file   Optiga_Ecc.h
 * @date   Mar, 2016
 * @brief  Implementation of ECC algorithm
 *
 */

#include "Optiga_Math.h"
#include "Optiga_Curve.h"
#include "Optiga_Ecc.h"
#include "../SWI/Optiga_Auth.h"
#include <linux/random.h>

#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
static uint8_t ChallengeByteEncode(uint8_t byteIn);
static uint8_t RespByteDecode(uint8_t byteIn);
static BOOL Ecc_EncodeChlg( dwordvec_t Chlg );
static BOOL Ecc_DecodeRes( dwordvec_t Res, mac_t Mac );
#endif

void Ecc_SwitchKey( uint32_t id ) {
	switch (id) {
	case Optiga_KeyID:
		ECCE_131_Curve = &ECCE_131_Curve_Optiga;
		ODC_163_PubKeyXY = &ODC_163_PubKeyXY_Optiga;
		break;
	case Oplus_KeyID:
		ECCE_131_Curve = &ECCE_131_Curve_Oplus;
		ODC_163_PubKeyXY = &ODC_163_PubKeyXY_Oplus;
		break;
	default:
		ECCE_131_Curve = &ECCE_131_Curve_Oplus;
		ODC_163_PubKeyXY = &ODC_163_PubKeyXY_Oplus;
		break;
	}
}
/** !!!!!!!!!! W A R N I N G !!!!!!!!!!
 *
 * this implementation calls the system function rand() for random number generation.
 * rand() is not a cryptographically strong random number generator.
 *
 * !!!!! this implementation must be replaced by a program using an unpredictable
 * true or pseudo random number generator with good statistics !!!!!
 */

/** generates a 128 bit random number
 * \param[out] erg generated random number
 */
void Ecc_Rng128( uint32_t * gf2n_RandomValue )
{
	uint8_t ubCount;
	//uint32_t uwRand;

	for( ubCount = 0u; ubCount < 4u; ubCount++ )
	{
		get_random_bytes(&gf2n_RandomValue[ubCount], sizeof(uint32_t));
		//uwRand = rand();
		//gf2n_RandomValue[ubCount] = uwRand;
	}
	gf2n_RandomValue[4] = 0u;
}

#if defined (SLE95100) || defined (SLE95300)
/* ****************************************************************************
   name:      Ecc_DoAuthentication()

   function:  execute a complete ECCS authentication sequence.

   input:     nil
   output:    bool

   return:    true, if all was ok.
              false, if errors detected.

   date:      .
 ************************************************************************* */
BOOL Ecc_DoAuthentication( )
{
	dwordvec_t gf2nRandomValue = {0,0,0,0,0,0};
	dwordvec_t gf2nChallenge = {0,0,0,0,0,0};
	dwordvec_t gf2nReturnX = {0,0,0,0,0,0};
	dwordvec_t gf2nReturnZ = {0,0,0,0,0,0};
	dwordvec_t gf2nCheck = {0,0,0,0,0,0};

	if( Ecc_GenerateChallenge( gf2nChallenge, gf2nRandomValue, MODE_ECCS ) == FALSE)
	{
		return FALSE;
	}

	if( Ecc_GenerateCheckValue( gf2nCheck, gf2nRandomValue, g_public_key_131, MODE_ECCS )== FALSE)
	{
		return FALSE;
	}

	if( Ecc_StartECC( gf2nChallenge, gf2nReturnX, gf2nReturnZ, FALSE, MODE_ECCS ) == FALSE )
	{
		return FALSE;
	}

	if( Ecc_VerifyResponse( gf2nReturnX, gf2nReturnZ, gf2nCheck, g_public_key_131, MODE_ECCS ) == FALSE )
	{
		return FALSE;
	}

	return TRUE;
}

#endif

#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
/* ****************************************************************************
   name:      Ecc_DoAuthenticationEnhanced()

   function:  execute a complete ECCE authentication sequence.

   input:     IN: gf2nUid
    				array to store the 96bits UID
   output:    bool

   return:    true, if all was ok.
              false, if errors detected.

   date:      .
 ************************************************************************* */
BOOL Ecc_DoAuthenticationEnhanced(uint8_t *gf2nUid )
{
	uint8_t i;
	uint8_t uid[12];
	dwordvec_t gf2nRandomValue;
	dwordvec_t gf2nChallenge={0x0, 0x0, 0x0, 0x0, 0x0};

	dwordvec_t gf2nReturnX={0x0, 0x0, 0x0, 0x0, 0x0 };
	dwordvec_t gf2nReturnZ={0x0, 0x0, 0x0, 0x0 , 0x0};
	dwordvec_t gf2nCheck={0x0, 0x0, 0x0, 0x0, 0x0};

	dwordvec_t gf2nPublicKey={0x0, 0x0, 0x0, 0x0, 0x0};
#if defined (SLE95250) || defined (SLE95300)
	uint32_t   gf2nODC[12];
#endif

	// reverse uid
	for(i=0; i<12; i++)
	{
		uid[12-i-1] = gf2nUid[i];
	}

#if defined (SLE95250) || defined (SLE95300)

	if( Ecc_ReadODC (gf2nODC, gf2nPublicKey) == FALSE)
	{
		return FALSE;
	}

	if( Ecc_VerifyODC (gf2nODC, gf2nPublicKey, uid) == FALSE)
	{
		return FALSE;
	}

	/* mask off the random padding bits */
	gf2nPublicKey[4] = gf2nPublicKey[4] & 0x7;
	gf2nPublicKey[5] = 0;
#else
	gf2nPublicKey[0] = g_public_key_131[0];
	gf2nPublicKey[1] = g_public_key_131[1];
	gf2nPublicKey[2] = g_public_key_131[2];
	gf2nPublicKey[3] = g_public_key_131[3];
	gf2nPublicKey[4] = g_public_key_131[4];
	gf2nPublicKey[5] = g_public_key_131[5];
#endif

	if( Ecc_GenerateChallenge( gf2nChallenge, gf2nRandomValue, MODE_ECCE ) == FALSE)
	{
		return FALSE;
	}

	if( Ecc_GenerateCheckValue( gf2nCheck, gf2nRandomValue, gf2nPublicKey, MODE_ECCE )== FALSE)
	{
		return FALSE;
	}

	if( Ecc_StartECC( gf2nChallenge, gf2nReturnX, gf2nReturnZ, FALSE, MODE_ECCE ) == FALSE )
	{
		return FALSE;
	}

	if( Ecc_VerifyResponse( gf2nReturnX, gf2nReturnZ, gf2nCheck, gf2nPublicKey, MODE_ECCE ) == FALSE )
	{
		return FALSE;
	}

	return TRUE;
}
#endif
/* ****************************************************************************
   name:      Ecc_GenerateChallenge()

   function:  create a new challenge to be sent to OPTIGA.

   input:     OUT: gf2n_Challenge
                gf2n_t array for challenge to be stored into.  
              OUT: gf2n_RandomValue
                gf2n_t array for random values to be stored into.
   output:    bool

   return:    true, if all was ok.
              false, if errors detected.

   date:      .
 ************************************************************************* */
BOOL Ecc_GenerateChallenge( dwordvec_t gf2n_Challenge, dwordvec_t gf2n_RandomValue, uint8_t bEccMode )
{
	/* get new random value and calculate challenge */
	Ecc_Rng128( gf2n_RandomValue );

	if(bEccMode == MODE_ECCS)
		generate_challenge(gf2n_Challenge, &ECCS_131_Curve, gf2n_RandomValue);
	else
		generate_challenge(gf2n_Challenge, ECCE_131_Curve, gf2n_RandomValue);

	return TRUE;
}

/* ****************************************************************************
   name:      Ecc_GenerateCheckValue()

   function:  get check value that is needed to verify the OPTIGA response.
              the checkvalue is linked to the random value and public key

   input:     OUT: gf2n_CheckValue
                gf2n_t array for checkvalue to be stored into.  
              IN: gf2n_RandomValue
                random values that are used for the current challenge.
              IN: gf2n_PublicKey
                public key that is used for the current challenge.

   output:    bool

   return:    true, if all was ok.
              false, if errors detected.

   date:      .
 ************************************************************************* */
BOOL Ecc_GenerateCheckValue( dwordvec_t gf2n_CheckValue, dwordvec_t gf2n_RandomValue, dwordvec_t gf2n_PublicKey, uint8_t bEccMode )
{
	if(bEccMode == MODE_ECCS)
		generate_checkvalue(gf2n_CheckValue, gf2n_PublicKey, &ECCS_131_Curve, gf2n_RandomValue);
	else
		generate_checkvalue(gf2n_CheckValue, gf2n_PublicKey, ECCE_131_Curve, gf2n_RandomValue);

	return TRUE;
}

/* ****************************************************************************
   name:      Ecc_VerifyResponse()

   function:  check, if response to sent challenge was valid.

   input:     IN: gf2n_XResponse
                gf2n_t array holding the x part of the OPTIGA response.
              IN: gf2n_ZResponse  
                gf2n_t array holding the z part of the OPTIGA response.
              IN: gf2n_CheckValue
                gf2n_t array holding checkvalue.
              IN: gf2n_PublicKey
                gf2n_t array holding public key.
              IN: bEccMode
                mode of authentication. 1=ECCE, 0=ECCS
   output:    bool

   return:    true, if authentication is valid.
              false, if authentication failed.

   date:      .
 ************************************************************************* */
BOOL Ecc_VerifyResponse( dwordvec_t gf2n_XResponse, dwordvec_t gf2n_ZResponse, dwordvec_t gf2n_CheckValue, dwordvec_t gf2n_PublicKey, uint8_t bEccMode )
{
#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
	mac_t hostMac;
#endif
	if(bEccMode == MODE_ECCS)
		return verify_response( gf2n_XResponse, gf2n_ZResponse, gf2n_CheckValue, &ECCS_131_Curve );
#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
	else
		return verify_mac64(gf2n_XResponse, gf2n_ZResponse, gf2n_CheckValue, gf2n_PublicKey, ECCE_131_Curve, hostMac);
#endif
}
#if defined (SLE95250) || defined (SLE95300)
/* ****************************************************************************
   name:      Ecc_VerifyODC()

   function:  check, if ODC is authentic.

   input:     IN: gf2n_ODC
                gf2n_t array holding OPTIGA digital certificate (ODC).
              IN: gf2n_PublicKey
                gf2n_t array holding the public key.
              IN: gf2n_Uid
                gf2n_t array holding the UID.

   output:    bool

   return:    true, if authentication is valid.
              false, if authentication failed.

   date:      .
 ************************************************************************* */
BOOL Ecc_VerifyODC( uint32_t *gf2n_ODC, uint32_t *gf2n_PublicKey, uint8_t *gf2n_Uid )
{
	uint32_t	 	ubHashOut[8];
	uint32_t	 	ubHashIn[8];
	signature_t 	sigSignature[1];

	memset(ubHashOut, 0, sizeof(ubHashOut));
	memset(ubHashIn, 0, sizeof(ubHashIn));
	memset((uint8_t*)(sigSignature[0].r_value), 0, sizeof(sigSignature[0].r_value));
	memset((uint8_t*)(sigSignature[0].s_value), 0, sizeof(sigSignature[0].s_value));

	memcpy((uint8_t*)(sigSignature[0].r_value), (uint8_t*)gf2n_ODC, 21);
	memcpy((uint8_t*)(sigSignature[0].s_value), ((uint8_t*)gf2n_ODC)+24, 21);
	/* Remove the random bit padding from the last locations */
	sigSignature[0].r_value[5] = sigSignature[0].r_value[5] & 0x7;
	sigSignature[0].s_value[5] = sigSignature[0].s_value[5] & 0x7;

	memcpy((uint8_t*)ubHashIn, (uint8_t*)gf2n_PublicKey, 18);
	memcpy(((uint8_t*)ubHashIn)+18, (uint8_t*)gf2n_Uid, 10);
	sha256((uint8_t*)ubHashOut, (uint8_t*)ubHashIn, 28);

	return ECDSA_verify( sigSignature, (uint8_t*)ubHashOut, ODC_163_PubKeyXY, &ODC_163_Curve);
}
#endif

/* ****************************************************************************
   name:      Ecc_StartECC()

   function:  send a calculated challenge to OPTIGA and start ECC engine of
              the OPTIGA IC.
              if OPTIGA indicates calculation finished, the results are read
              from OPTIGA memory space into the arrays gf2n_XResponse and
              gf2n_ZResponse.

   input:     IN: gf2n_Challenge
                gf2n_t array holding the challenge to be issued.
              OUT: gf2n_XResponse
                gf2n_t array holding the x part of the OPTIGA response.
              OUT: gf2n_ZResponse
                gf2n_t array holding the z part of the OPTIGA response.
              IN: bPolling
                polling mode to check OPTIGA engine for calculation finished
                state.
                if 'true', then a wait of at least 32ms is done before the data is
                read back, else the host waits for SWI IRQ signal.
                NOTE: please use FALSE, as that is the most efficient way
                      to handle the ECC engine.
              IN: bEccMode
                mode of authentication. 1 = ECCE, 0 = ECCS
   output:    bool

   return:    true, if all was ok.
              false, if errors detected.

   date:
 ************************************************************************* */
BOOL Ecc_StartECC( dwordvec_t gf2n_Challenge, dwordvec_t gf2n_XResponse, dwordvec_t gf2n_ZResponse, BOOL bPolling, uint8_t bEccMode)
{
	BOOL ret;
#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
	if(bEccMode == MODE_ECCE)
		Ecc_EncodeChlg(gf2n_Challenge);
#endif

	ret = Ecc_SendChallengeAndGetResponse( (uint16_t *)gf2n_Challenge, (uint16_t *)gf2n_XResponse, (uint16_t *)gf2n_ZResponse, bPolling, bEccMode) ;

	if(ret == FALSE)
		return ret;
#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
	if(bEccMode == MODE_ECCE)
		Ecc_DecodeRes(gf2n_ZResponse, gf2n_XResponse);
#endif
	return ret;
}

#if defined (SLE95150) || defined (SLE95250) || defined (SLE95300)
static uint8_t ChallengeByteEncode(uint8_t byteIn)
{
	uint8_t ByteEncode = 0;
	uint8_t data;

	// bit 7 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB7) & 0x01;
	ByteEncode |= data << 7;

	// bit 6 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB6) & 0x01;
	ByteEncode |= data << 6;

	// bit 5 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB5) & 0x01;
	ByteEncode |= data << 5;

	// bit 4 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB4) & 0x01;
	ByteEncode |= data << 4;

	// bit 3 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB3) & 0x01;
	ByteEncode |= data << 3;

	// bit 2 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB2) & 0x01;
	ByteEncode |= data << 2;

	// bit 1 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB1) & 0x01;
	ByteEncode |= data << 1;

	// bit 0 after encoding
	data = byteIn;
	data= (data >> ChanEncodeB0) & 0x01;
	ByteEncode |= data;

	return ByteEncode;

}

// response decoding
// data Bit		location after encoding
// 0				1
// 1				7
// 2				3
// 3				6
// 4				5
// 5				4
// 6				2
// 7				0
static uint8_t RespByteDecode(uint8_t byteIn)
{
	uint8_t ByteDecode = 0;
	uint8_t data;

	// bit 7 after decoding
	data = byteIn;
	data= (data >> RespEncodeB7) & 0x01;
	ByteDecode |= data << 7;

	// bit 6 after decoding
	data = byteIn;
	data= (data >> RespEncodeB6) & 0x01;
	ByteDecode |= data << 6;

	// bit 5 after decoding
	data = byteIn;
	data= (data >> RespEncodeB5) & 0x01;
	ByteDecode |= data << 5;

	// bit 4 after decoding
	data = byteIn;
	data= (data >> RespEncodeB4) & 0x01;
	ByteDecode |= data << 4;

	// bit 3 after decoding
	data = byteIn;
	data= (data >> RespEncodeB3) & 0x01;
	ByteDecode |= data << 3;

	// bit 2 after decoding
	data = byteIn;
	data= (data >> RespEncodeB2) & 0x01;
	ByteDecode |= data << 2;

	// bit 1 after decoding
	data = byteIn;
	data= (data >> RespEncodeB1) & 0x01;
	ByteDecode |= data << 1;

	// bit 0 after decoding
	data = byteIn;
	data= (data >> RespEncodeB0) & 0x01;
	ByteDecode |= data;

	return ByteDecode;

}

// challenge encoding
// data Bit		location after encoding
// 0				4
// 1				2
// 2				7
// 3				1
// 4				6
// 5				3
// 6				5
// 7				0
static BOOL Ecc_EncodeChlg( dwordvec_t Chlg )
{
	uint8_t i;

	for(i=0;i<17;i++)
	{
		*(((uint8_t*)Chlg)+i) = ChallengeByteEncode(*(((uint8_t*)Chlg)+i));
	}

	return TRUE;
}

static BOOL Ecc_DecodeRes( dwordvec_t Res, mac_t Mac )
{
	uint8_t i;

	for(i=0;i<17;i++)
	{
		*(((uint8_t*)Res)+i) = RespByteDecode(*(((uint8_t*)Res)+i));
	}
	for(i=0;i<8;i++)
	{
		*(((uint8_t*)Mac)+i) = RespByteDecode(*(((uint8_t*)Mac)+i));
	}

	return TRUE;
}
#endif


