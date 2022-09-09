#ifndef OPTIGA_ECC_H
#define OPTIGA_ECC_H

#include "../oplus_optiga.h"

#define SLE95250

#define MODE_ECCS		0
#define MODE_ECCE		1

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
#define ChanEncodeB7 2
#define ChanEncodeB6  4
#define ChanEncodeB5  6
#define ChanEncodeB4  0
#define ChanEncodeB3  5
#define ChanEncodeB2  1
#define ChanEncodeB1  3
#define ChanEncodeB0  7
// Response encoding
// data Bit		location after encoding
// 0				1
// 1				7
// 2				3
// 3				6
// 4				5
// 5				4
// 6				2
// 7				0
#define RespEncodeB7  1
#define RespEncodeB6  3
#define RespEncodeB5  4
#define RespEncodeB4  5
#define RespEncodeB3  2
#define RespEncodeB2  6
#define RespEncodeB1  0
#define RespEncodeB0  7

typedef enum Ecc_KeyID
{
	Optiga_KeyID           = 0x00,
	Oplus_KeyID      = 0x01,
} KeyID;

void Ecc_Rng128( dwordvec_t gf2n_RandomValue );
void Ecc_Fixed( uint32_t * gf2n_RandomValue );
BOOL Ecc_DoAuthentication( void );
BOOL Ecc_DoAuthenticationEnhanced(uint8_t *gf2nUid );
BOOL Ecc_GenerateChallenge( dwordvec_t gf2n_Challenge, dwordvec_t gf2n_RandomValue, uint8_t bEccMode );
BOOL Ecc_GenerateCheckValue( dwordvec_t gf2n_CheckValue, dwordvec_t gf2n_RandomValue, dwordvec_t gf2n_PublicKey, uint8_t bEccMode );
BOOL Ecc_StartECC( dwordvec_t gf2n_Challenge, dwordvec_t gf2n_XResponse, dwordvec_t gf2n_ZResponse, BOOL bPolling, uint8_t bEccMode) ;
BOOL Ecc_VerifyResponse( dwordvec_t gf2n_XResponse, dwordvec_t gf2n_ZResponse, dwordvec_t gf2n_CheckValue, dwordvec_t gf2n_PublicKey, uint8_t bEccMode );
BOOL Ecc_VerifyODC( uint32_t *gf2n_ODC, uint32_t *gf2n_PublicKey, uint8_t *gf2n_Uid );
void Ecc_SwitchKey( uint32_t id );
#endif

