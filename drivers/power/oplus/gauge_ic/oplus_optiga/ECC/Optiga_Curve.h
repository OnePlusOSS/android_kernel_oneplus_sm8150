/**
 * @file   Optiga_Curve.h
 * @date   Mar, 2016
 * @brief  Definition of the curve parameters
 *
 */
#ifndef OPTIGA_CURVE_H_
#define OPTIGA_CURVE_H_

#include "../oplus_optiga.h"

curve_parameter_t *ECCE_131_Curve;
eccpoint_t *ODC_163_PubKeyXY;
//SLE95100
/* Engineering ECCS Curve Parameters*/
const uint32_t g_coeff_sqrt_b_131[ARRAY_LEN(GF2_131)] =
{ 0x2c3633d9u,  0xe070a5f8u,  0x49d33c69u,  0xe4808f89u, 0x0u};
const uint32_t g_xP_131[ARRAY_LEN(GF2_131)] =
{ 0x281ec1f5u, 0x6c20c9b7u, 0xae22c4fcu, 0x651a4282u, 0x3u};

const curve_parameter_t ECCS_131_Curve =
{
		/* degree       */ GF2_131,
		/* coeff_a      */ NULL, /* not needed for authentication protocol */
		/* coeff_sqrt_b */ (uint32_t *)g_coeff_sqrt_b_131,
		/* base_point_x */ (uint32_t *)g_xP_131,
		/* base_point_y */ NULL, /* not needed for authentication protocol */
		/* order        */ NULL
};

/* ECCS Engineering Public key */
uint32_t g_public_key_131[ARRAY_LEN(GF2_131)] =
{0x6f34dd1du, 0xd9589d92u, 0xc5502efcu, 0x15fee224u, 0x02u };

// SLE95250
curve_parameter_t ECCE_131_Curve_Optiga =
{
		/* degree       */ GF2_131,
		/* coeff_a      */ NULL, /* not needed for authentication protocol */
		/* coeff_sqrt_b */ (uint32_t *)g_coeff_sqrt_b_131,
		/* base_point_x */ (uint32_t *)g_xP_131,
		/* base_point_y */ NULL, /* not needed for authentication protocol */
		/* order        */ NULL
};

/* ODC curve*/
const uint32_t g_coeff_sqrt_b_163[ARRAY_LEN(GF2_163)] =
  {0x797268e9u, 0x5bdcae9au, 0xdd55c8e8u, 0x70fbf823u, 0xdbdc9384u, 0x0u};
const uint32_t g_coeff_a_163[ARRAY_LEN(GF2_163)] =
  {0x0u, 0x0u, 0x0u, 0x0u, 0x0u, 0x0u};
const uint32_t g_xP_163[ARRAY_LEN(GF2_163)] =
  {0xc24277c4u, 0xc5a16dabu, 0x2a645a3du, 0xf295bf05u, 0xd52dedc9u, 0x5u};
const uint32_t g_yP_163[ARRAY_LEN(GF2_163)] =
  {0x1f06ba90u, 0x6cc79db6u, 0x2280c10fu, 0x227cd1a2u, 0xc0efb574u, 0x7u};
const uint32_t g_order_163[ARRAY_LEN(GF2_163)] =
  {0x99b2cc87u, 0x2cab1c18u, 0x72c8u, 0x0u, 0x0u, 0x2u};

const curve_parameter_t ODC_163_Curve =
{
		/* degree       */ GF2_163,
		/* coeff_a      */ (uint32_t *)g_coeff_a_163,
		/* coeff_sqrt_b */ (uint32_t *)g_coeff_sqrt_b_163,
		/* base_point_x */ (uint32_t *)g_xP_163,
		/* base_point_y */ (uint32_t *)g_yP_163,
		/* order        */ (uint32_t *)g_order_163
};


/* ODC 163-bit public key */
eccpoint_t ODC_163_PubKeyXY_Optiga =
{
	{0x4e1c2cf6u, 0x03fb02a8u, 0x17f74149u, 0xdabba0a5u, 0xde5bcd40u, 0x00000004u},
	{0x7a549b52u, 0x398336b6u, 0xa35fe797u, 0x0b1f22f3u, 0x12e3b51cu, 0x00000002u}
};

/*add by zhouhaikang, add key for oplus*/
/*Personalized ECC Parameter*/
/** square root of curve parameter b */
const uint32_t Personalized_g_coeff_sqrt_b_131[ARRAY_LEN(GF2_131)] =
  {0xa99beb1cu, 0x7b254b79u, 0xf596e1b7u, 0x8e8ab51eu, 0x0u};
/** affine x-coordinate of the base point */
const uint32_t Personalized_g_xP_131[ARRAY_LEN(GF2_131)] =
  {0x7fb86b33u, 0xbe5a5064u, 0xdb404fa6u, 0x1fda9855u, 0x5u};

curve_parameter_t ECCE_131_Curve_Oplus =
{
              /* degree       */ GF2_131,
              /* coeff_a      */ NULL, /* not needed for authentication protocol */
              /* coeff_sqrt_b */ (uint32_t *) Personalized_g_coeff_sqrt_b_131,
              /* base_point_x */ (uint32_t *) Personalized_g_xP_131,
              /* base_point_y */ NULL, /* not needed for authentication protocol */
              /* order        */ NULL
};

/*Personalized ODC Public Key*/
eccpoint_t ODC_163_PubKeyXY_Oplus =
{
       { 0x6cd2fdd7,0x0e66a649,0x67fbb5c1,0x7a2d92c9,0x7b075189,0x00000006 },
       { 0xca16167c,0x1d14f1ac,0x48edb1d6,0x819e2f11,0x24ab1342,0x00000000 }
};

#endif
// OPTIGA_CURVE_H_
