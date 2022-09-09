/**
 * @file   Optiga_Math.h
 * @date   August, 2012
 * @brief  ECC crypto engine
 *
 */

#ifndef _OPTIGA_MATH_H_
#define _OPTIGA_MATH_H_

/*************************** Include Files ***************************/
#include "../oplus_optiga.h"


/************************* Constant Define ***************************/
#define CH(x, y, z) ((z) ^ ((x) & ((y) ^ (z))))
#define MAJ(x, y, z) (((x) & ((y) ^ (z))) ^ ((y) & (z)))
#define SR(A,B)((A)>>(B)) // shift right operation
#define ROTR(A,B) (((A)>>(B))|((A)<<(32-(B)))) // rotate right operation
#define BIG_SIG_0(x) ((ROTR((x), 2) ^ ROTR((x), 13)) ^ ROTR((x), 22))
#define BIG_SIG_1(x) (ROTR((x), 6) ^ ROTR((x), 11) ^ ROTR((x), 25))
#define LIT_SIG_0(x) (ROTR((x), 7) ^ ROTR((x), 18) ^ SR((x), 3))
#define LIT_SIG_1(x) (ROTR((x), 17) ^ ROTR((x), 19) ^ SR((x), 10))
//hash without UID
#define HASH_163_IN		(22u) // ODC163
#define ODC_PK163_LEN (21u) // in bytes	(163)
#define DOUBLE_ARRAY_LEN(A) ((2*(A)+31)/32)
#define dwordvec_l_shift(A, B) actual_dwordvec_l_shift(A, B)
#define ONE8 0xFFU
#define T8(x)((x) & ONE8)

/** conversion character string to word big endian */
#define BIG_U8TO32(c) (((uint32_t)T8(*(c))<<24) | ((uint32_t)T8(*((c)+1))<<16) | \
		((uint32_t)T8(*((c)+2))<<8) | ((uint32_t)T8(*((c)+3))))

/** conversion word to character string big endian */
#define BIG_U32TO8(c, v) do {            \
		uint32_t common_h_tmp_x = (v);       \
		uint8_t *common_h_tmp_d = (c);                               \
		common_h_tmp_d[0] = (uint8_t)T8(common_h_tmp_x >> 24);       \
		common_h_tmp_d[1] = (uint8_t)T8(common_h_tmp_x >> 16);       \
		common_h_tmp_d[2] = (uint8_t)T8(common_h_tmp_x >> 8);        \
		common_h_tmp_d[3] = (uint8_t)T8(common_h_tmp_x);             \
} while (0)

#define GF2_131 (131) /** sizes of finite fields GF(2^n) */
#define GF2_163 (163)
#define GF2_193 (193)
//#define MAX_DEGREE (163)
//#define ARRAY_LEN(A) (((A)+31)/32)
#define gf2n_add(A, B, C) actual_gf2n_add(A, B, C) /* addition: out = op1 + op2 */
#define gf2n_sum(A, B) actual_gf2n_sum(A, B) /* addition: out += op */
#define gf2n_mul(A, B, C) actual_gf2n_mul(A, B, C) /* multiplication: out = op1 * op2 */
#define gf2n_square(A, B) actual_gf2n_square(A, B) /* squaring: out = op^2 */
#define dwordvec_copy(A, B) actual_dwordvec_copy(A, B) /* copy of a GF(2^131)-element */

/************************* Type Define ***************************/
/** data type for points on elliptic curve */
/** data type for elements in GF(2^n) and GF(p) */
//typedef uint32_t dwordvec_t[ARRAY_LEN(MAX_DEGREE)];

typedef struct 
{
	dwordvec_t x_coord;
	dwordvec_t y_coord;
} eccpoint_t;

/** data type for elliptic curve parameters */
/** data type for signature */
typedef struct
{
	/** r value of signature */
	dwordvec_t r_value;
	/** s value of signature */
	dwordvec_t s_value;
} signature_t;

typedef uint32_t mac_t[3];  

typedef struct 
{
	unsigned int degree;    /* extension degree n of the finite field GF(2^n) */
	uint32_t *coeff_a;      /* parameter a of the elliptic curve y^2+xy = x^3+ax^2+b */
	uint32_t *coeff_sqrt_b; /* square root of parameter b of the elliptic curve y^2+xy = x^3+ax^2+b */
	uint32_t *base_point_x; /* x-coordinate of the base point */
	uint32_t *base_point_y; /* y-coordinate of the base point */
	uint32_t *order;        /* prime order of the base point */
} curve_parameter_t;

typedef void (*func2_pt)(uint32_t *, const uint32_t *);
typedef void (*func3_pt)(uint32_t *, const uint32_t *, const uint32_t *);

/* brief definition of sha256 context */
typedef struct
{
	uint32_t H[8];      /*!< 8 working variables/final hash value */
	uint32_t length;    /*!< length counter */
	unsigned int next;  /*!< next unused byte in message buffer */
	uint8_t M[64];      /*!< message buffer */
} sha256_context_t;

typedef uint32_t double_dwordvec_t[DOUBLE_ARRAY_LEN(MAX_DEGREE)];

typedef enum _E_MAC_IDX
{
	MAC_NVM           = 0x00,
	MAC_DEV_ADDR      = 0x01,
	MAC_TEMP_DATA	  = 0x02,
	MAC_TEMP_HTHRESH  = 0x03,
	MAC_TEMP_LTHRESH  = 0x04,
	MAC_TRAS_COUNTER  = 0x05,
	MAC_LIFE_COUNTER  = 0x06
} E_MAC_IDX;

/*********************** Function Prototypes *************************/
// Authentication Functions
BOOL generate_challenge (dwordvec_t xA, const curve_parameter_t *curve, dwordvec_t lambda);
BOOL generate_checkvalue (dwordvec_t xC, const dwordvec_t xT, const curve_parameter_t *curve, dwordvec_t lambda);
BOOL verify_response( dwordvec_t gf2n_XResponse, dwordvec_t gf2n_ZResponse, dwordvec_t gf2n_CheckValue, const curve_parameter_t *curve );
BOOL verify_mac80(const mac_t mac_value, const dwordvec_t Z, const dwordvec_t xC, const mac_t data, const curve_parameter_t *curve, mac_t host_mac_value);
BOOL verify_mac64(const mac_t mac_value, const dwordvec_t Z, const dwordvec_t xC, const mac_t data, const curve_parameter_t *curve, mac_t host_mac_value);
BOOL ECDSA_verify(const signature_t *sig, const uint8_t *hash_data, const eccpoint_t *pub_key, const curve_parameter_t *curve);
void mac_algorithm_64(mac_t mac_value, const mac_t data, const mac_t session_key);
void mac_algorithm_80(mac_t mac_value, const mac_t data, const mac_t session_key);

// GF2N Functions
void gf2_131_sum(dwordvec_t io, const dwordvec_t op);
void gf2_163_sum(dwordvec_t io, const dwordvec_t op);
void gf2_193_sum(dwordvec_t io, const dwordvec_t op);
void gf2_131_reduction(dwordvec_t out, const double_dwordvec_t temp);
void gf2_163_reduction(dwordvec_t out, const double_dwordvec_t temp);
void gf2_193_reduction(dwordvec_t out, const double_dwordvec_t temp);
void gf2_131_square(dwordvec_t out, const dwordvec_t op);
void gf2_163_square(dwordvec_t out, const dwordvec_t op);
void gf2_193_square(dwordvec_t out, const dwordvec_t op);
void dwordvec_131_copy(dwordvec_t copy, const dwordvec_t in);
void dwordvec_163_copy(dwordvec_t copy, const dwordvec_t in);
void dwordvec_193_copy(dwordvec_t copy, const dwordvec_t in);
void gf2_131_add(dwordvec_t out, const dwordvec_t op1, const dwordvec_t op2);
void gf2_163_add(dwordvec_t out, const dwordvec_t op1, const dwordvec_t op2);
void gf2_193_add(dwordvec_t out, const dwordvec_t op1, const dwordvec_t op2);
void gf2_131_mul(dwordvec_t out, const dwordvec_t op1, const dwordvec_t op2);
void gf2_163_mul(dwordvec_t out, const dwordvec_t op1, const dwordvec_t op2);
void gf2_193_mul(dwordvec_t out, const dwordvec_t op1, const dwordvec_t op2);
BOOL gf2n_divide(dwordvec_t t1, const dwordvec_t op1, const dwordvec_t op2);
void gf2n_init(const unsigned int degree);
void dwordvec_swap(dwordvec_t el1, dwordvec_t el2);
int dwordvec_cmp(const dwordvec_t a, const dwordvec_t b);
BOOL dwordvec_iszero(const dwordvec_t el);
void dwordvec_add(dwordvec_t c, const dwordvec_t a, const dwordvec_t b);
uint32_t dwordvec_sub(dwordvec_t c, const dwordvec_t a, const dwordvec_t b);
void dwordvec_l_shift_163(dwordvec_t out, const dwordvec_t in);
void dwordvec_l_shift_193(dwordvec_t out, const dwordvec_t in);
void r_shift(dwordvec_t el);
void remove_x_power(dwordvec_t a, dwordvec_t b);
void remove_2_power(dwordvec_t a, dwordvec_t b);
void precompute(dwordvec_t table[], const dwordvec_t el);
void gfp_divide(dwordvec_t x, const dwordvec_t a, const dwordvec_t b);

// ECC Functions
void ecc_init(const curve_parameter_t *p);
void mont_ecc_add(dwordvec_t E, dwordvec_t F, const dwordvec_t A, const dwordvec_t B, const dwordvec_t C, const dwordvec_t D, const dwordvec_t point_x);
void mont_ecc_double(dwordvec_t E, dwordvec_t F, const dwordvec_t A, const dwordvec_t B);
void mont_ecc_mul(dwordvec_t A, dwordvec_t B, dwordvec_t C, dwordvec_t D, const dwordvec_t point_x, const dwordvec_t scalar);
BOOL ecc_point_on_curve(const eccpoint_t *p);
void ecc_mul_projective(dwordvec_t X, dwordvec_t Y, dwordvec_t Z, const eccpoint_t *p, const dwordvec_t scalar);
BOOL ecc_add_point_x(dwordvec_t point_x, const dwordvec_t X1, const dwordvec_t Y1, const dwordvec_t Z1, const dwordvec_t X2, const dwordvec_t Y2, const dwordvec_t Z2);

// SHA256
void sha256_compress(sha256_context_t *context);
void sha256_final(uint8_t *hash_value, sha256_context_t *context);
void sha256_update(const uint8_t *input_data, const uint32_t input_length, sha256_context_t *context);
void sha256_init(sha256_context_t *context);
void sha256(uint8_t *hash_value, const uint8_t *input_data, const uint32_t input_length);


#endif				/* _BIF_AUTHENTICATION_H_ */

