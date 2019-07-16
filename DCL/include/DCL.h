/* DCL.h - C2000 Digital Controller Library header file - version 2.1.1
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * ALL RIGHTS RESERVED 
 */

#ifndef _C_DCL_H
#define _C_DCL_H

//! \file   		DCL.h
//! \brief  		Contains the public interface to the
//!         		Digital Controller Library functions

#include <stdint.h>

#ifndef C2000_IEEE754_TYPES
#define C2000_IEEE754_TYPES
typedef float float32_t;
#endif

//! \brief 			Function name compatibility with version 1.0
//!
#define	DCL_runPID		DCL_runPID_C1
#define	DCL_runPIDc		DCL_runPID_L1
#define DCL_runPI		DCL_runPI_C1
#define DCL_runPIc		DCL_runPI_L1
#define	DCL_runDF13		DCL_runDF13_C1
#define	DCL_runDF13i	DCL_runDF13_C2
#define	DCL_runDF13p	DCL_runDF13_C3
#define	DCL_runDF13c	DCL_runDF13_L1
#define	DCL_runDF13ic	DCL_runDF13_L2
#define	DCL_runDF13pc	DCL_runDF13_L3
#define	DCL_runDF22		DCL_runDF22_C1
#define	DCL_runDF22i	DCL_runDF22_C2
#define	DCL_runDF22p	DCL_runDF22_C3
#define	DCL_runDF22c	DCL_runDF22_L1
#define	DCL_runDF22ic	DCL_runDF22_L2
#define	DCL_runDF22pc	DCL_runDF22_L3
#define	DCL_runDF23		DCL_runDF23_C1
#define	DCL_runDF23i	DCL_runDF23_C2
#define	DCL_runDF23p	DCL_runDF23_C3
#define	DCL_runDF23c	DCL_runDF23_L1
#define	DCL_runDF23ic	DCL_runDF23_L2
#define	DCL_runDF23pc	DCL_runDF23_L3

//! \brief          Un-comment for structure compatibility with version 2.0
//!
//#define PID   DCL_PID
//#define PI    DCL_PI
//#define DF13  DCL_DF13
//#define DF22  DCL_DF22
//#define DF23  DCL_DF23

//--- Linear PID controller ---------------------------------------------------

//! \brief 			Defines the DCL_PID controller structure
//!
typedef volatile struct {
	float32_t Kp;		//!< Proportional gain
	float32_t Ki;		//!< Integral gain
	float32_t Kd;		//!< Derivative gain
	float32_t Kr;		//!< Set point weight
	float32_t c1;		//!< D-term filter coefficient 1
	float32_t c2;		//!< D-term filter coefficient 2
	float32_t d2;		//!< D-term filter intermediate storage 1
	float32_t d3;		//!< D-term filter intermediate storage 2
	float32_t i10;		//!< I-term intermediate storage
	float32_t i14;		//!< Intermediate saturation storage
	float32_t Umax;		//!< Upper saturation limit
	float32_t Umin;		//!< Lower saturation limit
} DCL_PID;

//! \brief 			Defines default values to initialise the DCL_PID structure
//!
#define	PID_DEFAULTS {  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
						0.0f, 0.0f, 1.0f, -1.0f }

//! \brief     		Executes an ideal form PID controller on the FPU32
//! \param[in] p	Pointer to the DCL_PID structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \param[in] lk	External output clamp flag
//! \return    		The control effort
//!
extern float32_t DCL_runPID_C1(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk);

//! \brief     		Executes an inline ideal form PID controller on the FPU32
//! \param[in] p	Pointer to the DCL_PID structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \param[in] lk	External output clamp flag
//! \return    		The control effort
//!
static inline float32_t DCL_runPID_C2(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk)
{
	float32_t v1, v4, v5, v8, v9, v10, v12;

	v5 = (p->Kr * rk) - yk;
	v8 = ((rk - yk) * p->Ki * p->Kp * p->i14) + p->i10;
	p->i10 = v8;
	v1 = yk * p->Kd * p->c1;
	v4 = v1 - p->d2 - p->d3;
	p->d2 = v1;
	p->d3 = v4 * p->c2;
	v9 = ((v5 - v4) * p->Kp) + v8;
	v10 = (v9 > p->Umax) ? p->Umax : v9;
	v10 = (v10 < p->Umin) ? p->Umin : v10;
	v12 = (v10 == v9) ? 1.0f : 0.0f;
	p->i14 = v12 * lk;

	return v10;
}

//! \brief     		Executes an parallel form PID controller on the FPU32
//! \param[in] p	Pointer to the DCL_PID structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \param[in] lk	External output clamp flag
//! \return    		The control effort
//!
static inline float32_t DCL_runPID_C3(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk)
{
	float32_t v1, v4, v5, v6, v8, v9, v10, v12;

	v5 = rk - yk;
	v6 = v5 * p->Kp;
	v8 = v5 * p->Ki * p->i14 + p->i10;
	p->i10 = v8;
	v1 = v5 * p->Kd * p->c1;
	v4 = v1 - p->d2 - p->d3;
	p->d2 = v1;
	p->d3 = v4 * p->c2;
	v9 = v6 + v8 + v4;
	v10 = (v9 > p->Umax) ? p->Umax : v9;
	v10 = (v10 < p->Umin) ? p->Umin : v10;
	v12 = (v10 == v9) ? 1.0f : 0.0f;
	p->i14 = v12 * lk;

	return v10;
}

//! \brief     		Executes a parallel form PID controller on the FPU32
//! \param[in] p	Pointer to the DCL_PID structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \param[in] lk	External output clamp flag
//! \return    		The control effort
//!
extern float32_t DCL_runPID_C4(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk);

//! \brief     		Executes an ideal form PID controller on the CLA
//! \param[in] p	Pointer to the DCL_PID structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \param[in] lk	External output clamp flag
//! \return    		The control effort
//!
extern float32_t DCL_runPID_L1(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk);

//! \brief     		Executes a parallel form PID controller on the CLA
//! \param[in] p	Pointer to the DCL_PID structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \param[in] lk	External output clamp flag
//! \return    		The control effort
//!
extern float32_t DCL_runPID_L2(DCL_PID *p, float32_t rk, float32_t yk, float32_t lk);


//--- Linear PI controller ----------------------------------------------------

//! \brief 			Defines the DCL_PI controller structure
//!
typedef volatile struct {
	float32_t Kp;		//!< Proportional gain
 	float32_t Ki;		//!< Integral gain
 	float32_t i10;		//!< I storage
	float32_t Umax;		//!< Upper control saturation limit
	float32_t Umin;		//!< Lower control saturation limit
	float32_t i6;		//!< Saturation storage
    float32_t Imax;     //!< Upper integrator saturation limit
    float32_t Imin;     //!< Lower integrator saturation limit
} DCL_PI;

//! \brief 	Defines default values to initialise the PI structure
//!
#define	PI_DEFAULTS { 1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f }

//! \brief     		Executes a series form PI controller on the FPU32
//! \param[in] p	Pointer to the DCL_PI structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \return    		The control effort
//!
extern float32_t DCL_runPI_C1(DCL_PI *p, float32_t rk, float32_t yk);

//! \brief     		Executes an inline series form PI controller on the FPU32
//! \param[in] p	Pointer to the DCL_PI structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \return    		The control effort
//!
static inline float32_t DCL_runPI_C2(DCL_PI *p, float32_t rk, float32_t yk)
{
	float32_t v2, v4, v5, v9;

	v2 = p->Kp * (rk - yk);
	v4 = p->i10 + (p->Ki * p->i6 * v2);
	v5 = v2 + v4;
	v9 = (v5 > p->Umax) ? p->Umax : v5;
	v9 = (v9 < p->Umin) ? p->Umin : v9;
	p->i10 = v4;
	p->i6 = (v5 == v9) ? 1.0f : 0.0f;

	return v9;
}

//! \brief     		Executes an parallel form PI controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_PI structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \return    		The control effort
//!
static inline float32_t DCL_runPI_C3(DCL_PI *p, float32_t rk, float32_t yk)
{
	float32_t v1, v2, v4, v5, v9;

	v1 = rk - yk;
	v2 = p->Kp * v1;
	v4 = (v1 * p->Ki * p->i6) + p->i10;
	p->i10 = v4;
	v5 = v2 + v4;
	v9 = (v5 > p->Umax) ? p->Umax : v5;
	v9 = (v9 < p->Umin) ? p->Umin : v9;
	p->i6 = (v5 == v9) ? 1.0f : 0.0f;

	return v9;
}

//! \brief     		Executes a parallel form PI controller on the FPU32
//! \param[in] p	Pointer to the DCL_PI structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \return    		The control effort
//!
extern float32_t DCL_runPI_C4(DCL_PI *p, float32_t rk, float32_t yk);

//! \brief          Executes an parallel form PI controller on the FPU32
//!                 Contains enhanced AWR logic
//!                 Implemented as inline C function
//! \param[in] p    Pointer to the DCL_PI structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback value
//! \return         The control effort
//!
static inline float32_t DCL_runPI_C5(DCL_PI *p, float32_t rk, float32_t yk)
{
    float32_t v1, v5, v7, v8;
    uint16_t l11, l12, l14, l17, l18, l19;

    v1 = rk - yk;
    v5 = (v1 * p->Ki * p->i6) + p->i10;
    p->i10 = v5;
    v7 = (v1 * p->Kp) + v5;
    v8 = (v7 > p->Umax) ? p->Umax : v7;
    v8 = (v8 < p->Umin) ? p->Umin : v8;
    l17 = ((v7 - v8) == 0) ? 1U : 0U;
    l11 = (v5 >= p->Imax) ? 1U : 0U;
    l12 = (v5 <= p->Imin) ? 1U : 0U;
    l19 = (v5 > 0) ? 1U : 0U;
    l14 = (v1 > 0) ? 1U : 0U;
    l18 = l17 & (!(l11 | l12) | (l19 ^ l14));
    p->i6 = (l18 == 0U) ? 0.0f : 1.0f;

    return v8;
}

//! \brief     		Executes a series form PI controller on the CLA
//! \param[in] p	Pointer to the DCL_PI structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \return    		The control effort
//!
extern float32_t DCL_runPI_L1(DCL_PI *p, float32_t rk, float32_t yk);

//! \brief     		Executes a parallel form PI controller on the CLA
//! \param[in] p	Pointer to the DCL_PI structure
//! \param[in] rk	The controller set-point reference
//! \param[in] yk	The measured feedback value
//! \return    		The control effort
//!
extern float32_t DCL_runPI_L2(DCL_PI *p, float32_t rk, float32_t yk);


//--- Linear PI2 controller ---------------------------------------------------

//! \brief          Defines the DCL_PI2 controller structure
//!
typedef volatile struct {
    float32_t Kp;       //!< Proportional gain
    float32_t Ki;       //!< Integral gain
    float32_t i6;       //!< Integrator 1 storage
    float32_t i9;       //!< Integrator 2 storage
    float32_t i12;      //!< Saturation 1 storage
    float32_t i13;      //!< Saturation 2 storage
    float32_t Umax;     //!< Upper saturation limit
    float32_t Umin;     //!< Lower saturation limit
} DCL_PI2;

//! \brief  Defines default values to initialise the DCL_PI2 structure
//!
#define PI2_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f, -1.0f }

//! \brief          Executes an inline series form PI2 controller on the FPU32
//! \param[in] p    Pointer to the DCL_PI2 structure
//! \param[in] rk   The controller set-point reference
//! \param[in] yk   The measured feedback
//! \return         The control effort
//!
static inline float32_t DCL_runPI2_C1(DCL_PI2 *p, float32_t rk, float32_t yk)
{
    float32_t v1, v2, v5, v8, v10, v11, v14;
    uint16_t l1, l2, l3, l4, l5, l6;

    v1 = rk - yk;
    v2 = p->Kp * v1;
    v5 = (v1 * p->Ki * p->i12) + p->i6;
    p->i6 = v5;
    v8 = (v5 * p->i13) + p->i9;
    p->i9 = v8;
    v10 = v2 + v8;

    v11 = (v10 > p->Umax) ? p->Umax : v10;
    v11 = (v11 < p->Umin) ? p->Umin : v11;
    v14 = v10 - v11;
    l1 = (v1 > 0.0f) ? 1U : 0U;
    l2 = (v14 > 0.0f) ? 1U : 0U;
    l3 = (v14 == 0.0f) ? 1U : 0U;
    l4 = (v5 > 0.0f) ? 1U : 0U;
    l5 = l3 | (l1 ^ l2);
    l6 = l3 | (l4 ^ l2);
    p->i12 = (l5 == 1U) ? 1.0f : 0.0f;
    p->i13 = (l6 == 1U) ? 1.0f : 0.0f;

    return v11;
}

//! \brief          Resets the integrators of the PI2 controller on the FPU32
//! \param[in] p    Pointer to the DCL_PI2 structure
//! \return         None
//!
static inline void DCL_resetPI2_C1(DCL_PI2 *p)
{
    p->i6 = 0.0f;
    p->i9 = 0.0f;
    p->i12 = 1.0f;
    p->i13 = 1.0f;
}


//--- Direct Form 1 - 1st order -----------------------------------------------

//! \brief          Defines the DCL_DF11 controller structure
//!
typedef volatile struct {
    float32_t b0;   //!< b0
    float32_t b1;   //!< b1
    float32_t a1;   //!< a1
    float32_t d1;   //!< e(k-1)
    float32_t d2;   //!< u(k-1)
} DCL_DF11;

//! \brief          Defines default values to initialise the DCL_DF11 structure
//!
#define DF11_DEFAULTS { 0.5f, 0.5f, 1.0f, 0.0f, 0.0f }

//! \brief          Executes a 1st order Direct Form 1 controller on the FPU32
//! \param[in] p    Pointer to the DCL_DF11 controller structure
//! \param[in] ek   The servo error
//! \return         The control effort
//!
extern float32_t DCL_runDF11_C1(DCL_DF11 *p, float32_t ek);

//! \brief          Executes a 1st order Direct Form 1 controller on the FPU32
//!                 Implemented as inline C function
//! \param[in] p    Pointer to the DCL_DF11 controller structure
//! \param[in] ek   The servo error
//! \return         The control effort
//!
static inline float32_t DCL_runDF11_C2(DCL_DF11 *p, float32_t ek)
{
    p->d2 = (ek * p->b0) + (p->d1 * p->b1) - (p->d2 * p->a1);
    p->d1 = ek;

    return p->d2;
}

//! \brief          Executes a 1st order Direct Form 1 controller on the CLA
//! \param[in] p    Pointer to the DCL_DF11 controller structure
//! \param[in] ek   The servo error
//! \return         The control effort
//!
extern float32_t DCL_runDF11_L1(DCL_DF11 *p, float32_t ek);


//--- Direct Form 1 - 3rd order -----------------------------------------------

//! \brief 			Defines the DCL_DF13 controller structure
//!
typedef volatile struct {
	// coefficients
	float32_t b0;	//!< b0
	float32_t b1;	//!< b1
	float32_t b2;	//!< b2
	float32_t b3;	//!< b3
	float32_t a0;	//!< a0
	float32_t a1;	//!< a1
	float32_t a2;	//!< a2
	float32_t a3;	//!< a3

	//data
	float32_t d0;	//!< e(k)
	float32_t d1;	//!< e(k-1)
	float32_t d2;	//!< e(k-2)
	float32_t d3;	//!< e(k-3)
	float32_t d4;	//!< u(k)
	float32_t d5;	//!< u(k-1)
	float32_t d6;	//!< u(k-2)
	float32_t d7;	//!< u(k-3)
} DCL_DF13;

//! \brief 			Defines default values to initialise the DCL_DF13 structure
//!
#define	DF13_DEFAULTS {	0.25f, 0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f, 0.0f, \
						0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

//! \brief     		Executes a full 3rd order Direct Form 1 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF13_C1(DCL_DF13 *p, float32_t ek);

//! \brief     		Executes an immediate 3rd order Direct Form 1 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \param[in] vk	The partial pre-computed control effort
//! \return    		The control effort
//!
extern float32_t DCL_runDF13_C2(DCL_DF13 *p, float32_t ek, float32_t vk);

//! \brief     		Executes a partial pre-computed 3rd order Direct Form 1 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//! \return    		The control effort
//!
extern float32_t DCL_runDF13_C3(DCL_DF13 *p, float32_t ek, float32_t uk);

//! \brief     		Executes a full 3rd order Direct Form 1 controller on the FPU32
//!					Implemented as inline C function
//!					Note: d0 not used
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
static inline float32_t DCL_runDF13_C4(DCL_DF13 *p, float32_t ek)
{
	p->d4 = (ek * p->b0) + (p->d1 * p->b1) + (p->d2 * p->b2) + (p->d3 * p->b3) - (p->d5 * p->a1) - (p->d6 * p->a2) - (p->d7 * p->a3);
	p->d3 = p->d2;
	p->d2 = p->d1;
	p->d1 = ek;
	p->d7 = p->d6;
	p->d6 = p->d5;
	p->d5 = p->d4;

	return p->d4;
}

//! \brief     		Executes an immediate 3rd order Direct Form 1 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \param[in] vk	The partial pre-computed control effort
//! \return    		The control effort
//!
static inline float32_t DCL_runDF13_C5(DCL_DF13 *p, float32_t ek, float32_t vk)
{
	p->d4 = (ek * p->b0) + vk;

	return p->d4;
}

//! \brief     		Executes a partial pre-computed 3rd order Direct Form 1 controller on the FPU32
//!					Implemented as inline C function
//!					Note: d0 not used
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//! \return    		The control effort
//!
static inline float32_t DCL_runDF13_C6(DCL_DF13 *p, float32_t ek, float32_t uk)
{
	float32_t v9;

	v9 = (ek * p->b1) + (p->d1 * p->b2) + (p->d2 * p->b3) - (uk * p->a1) - (p->d5 * p->a2) - (p->d6 * p->a3);
	p->d2 = p->d1;
	p->d1 = ek;
	p->d6 = p->d5;
	p->d5 = uk;

	return v9;
}

//! \brief     		Executes a full 3rd order Direct Form 1 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF13_L1(DCL_DF13 *p, float32_t ek);

//! \brief     		Executes an immediate 3rd order Direct Form 1 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \param[in] vk	The partial pre-computed control effort
//! \return    		The control effort
//!
extern float32_t DCL_runDF13_L2(DCL_DF13 *p, float32_t ek, float32_t vk);

//! \brief     		Executes a partial pre-computed 3rd order Direct Form 1 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF13 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//! \return    		The control effort
//!
extern float32_t DCL_runDF13_L3(DCL_DF13 *p, float32_t ek, float32_t uk);


//--- Direct Form 2 - 2nd order -----------------------------------------------

//! \brief 			Defines the DCL_DF22 controller structure
//!
typedef volatile struct {
	float32_t b0;	//!< b0
	float32_t b1;	//!< b1
	float32_t b2;	//!< b2
	float32_t a1;	//!< a1
	float32_t a2;	//!< a2
	float32_t x1;	//!< x1
	float32_t x2;	//!< x2
} DCL_DF22;

//! \brief 			Defines default values to initialise the DCL_DF22 structure
//!
#define	DF22_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

//! \brief     		Executes a full 2nd order Direct Form 2 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF22_C1(DCL_DF22 *p, float32_t ek);

//! \brief     		Executes an immediate 2nd order Direct Form 2 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF22_C2(DCL_DF22 *p, float32_t ek);

//! \brief     		Executes a partial pre-computed 2nd order Direct Form 2 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//!
extern void DCL_runDF22_C3(DCL_DF22 *p, float32_t ek, float32_t uk);

//! \brief     		Executes a full 2nd order Direct Form 2 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
static inline float32_t DCL_runDF22_C4(DCL_DF22 *p, float32_t ek)
{
	float32_t v7;

	v7 = (ek * p->b0) + p->x1;
	p->x1 = (ek * p->b1) + p->x2 - (v7 * p->a1);
	p->x2 = (ek * p->b2) - (v7 * p->a2);

	return v7;
}

//! \brief     		Executes an immediate 2nd order Direct Form 2 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
static inline float32_t DCL_runDF22_C5(DCL_DF22 *p, float32_t ek)
{
	return ((ek * p->b0) + p->x1);
}

//! \brief     		Executes a partial pre-computed 2nd order Direct Form 2 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//!
static inline void DCL_runDF22_C6(DCL_DF22 *p, float32_t ek, float32_t uk)
{
	p->x1 = (ek * p->b1) + p->x2 - (uk * p->a1);
	p->x2 = (ek * p->b2) - (uk * p->a2);
}

//! \brief     		Executes a full 2nd order Direct Form 2 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF22_L1(DCL_DF22 *p, float32_t ek);

//! \brief     		Executes an immediate 2nd order Direct Form 2 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF22_L2(DCL_DF22 *p, float32_t ek);

//! \brief     		Executes a partial pre-computed 2nd order Direct Form 2 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF22 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//!
extern void DCL_runDF22_L3(DCL_DF22 *p, float32_t ek, float32_t uk);


//--- Direct Form 2 - 3rd order -----------------------------------------------

//! \brief 			Defines the DCL_DF23 controller structure
//!
typedef volatile struct {
	float32_t b0;	//!< b0
	float32_t b1;	//!< b1
	float32_t b2;	//!< b2
	float32_t b3;	//!< b3
	float32_t a1;	//!< a1
	float32_t a2;	//!< a2
	float32_t a3;	//!< a3
	float32_t x1;	//!< x1
	float32_t x2;	//!< x2
	float32_t x3;	//!< x3
} DCL_DF23;

//! \brief 			Defines default values to initialise the DCL_DF23 structure
//!
#define	DF23_DEFAULTS { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }

//! \brief     		Executes a full 3rd order Direct Form 2 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF23_C1(DCL_DF23 *p, float32_t ek);

//! \brief     		Executes an immediate 3rd order Direct Form 2 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF23_C2(DCL_DF23 *p, float32_t ek);

//! \brief     		Executes a partial pre-computed 3rd order Direct Form 2 controller on the FPU32
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//!
extern void DCL_runDF23_C3(DCL_DF23 *p, float32_t ek, float32_t uk);

//! \brief     		Executes a full 3rd order Direct Form 2 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
static inline float32_t DCL_runDF23_C4(DCL_DF23 *p, float32_t ek)
{
	float32_t v7;

	v7 = (ek * p->b0) + p->x1;
	p->x1 = (ek * p->b1) + p->x2 - (v7 * p->a1);
	p->x2 = (ek * p->b2) + p->x3 - (v7 * p->a2);
	p->x3 = (ek * p->b3) - (v7 * p->a3);

	return v7;
}

//! \brief     		Executes an immediate 3rd order Direct Form 2 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
static inline float32_t DCL_runDF23_C5(DCL_DF23 *p, float32_t ek)
{
	return ((ek * p->b0) + p->x1);
}

//! \brief     		Executes a partial pre-computed 3rd order Direct Form 2 controller on the FPU32
//!					Implemented as inline C function
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//!
static inline void DCL_runDF23_C6(DCL_DF23 *p, float32_t ek, float32_t uk)
{
	p->x1 = (ek * p->b1) + p->x2 - (uk * p->a1);
	p->x2 = (ek * p->b2) + p->x3 - (uk * p->a2);
	p->x3 = (ek * p->b3) - (uk * p->a3);
}

//! \brief     		Executes a full 3rd order Direct Form 2 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF23_L1(DCL_DF23 *p, float32_t ek);

//! \brief     		Executes an immediate 3rd order Direct Form 2 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \return    		The control effort
//!
extern float32_t DCL_runDF23_L2(DCL_DF23 *p, float32_t ek);

//! \brief     		Executes a partial pre-computed 2nd order Direct Form 2 controller on the CLA
//! \param[in] p	Pointer to the DCL_DF23 controller structure
//! \param[in] ek	The servo error
//! \param[in] uk	The controller output in the previous sample interval
//!
extern void DCL_runDF23_L3(DCL_DF23 *p, float32_t ek, float32_t uk);


//--- Direct Form 2 - clamp ---------------------------------------------------

//! \brief 			Saturates a control variable and returns 1 if either limit is exceeded
//!
//! \details		Can be used to saturate a pre-computed Direct Form 2 controller.
//!					If the immediate result is in range it can be used, otherwise
//!					it can be clamped and the next partial pre-computation skipped.
//!					An example of use with a pre-computed DF22 controller follows:
//!
//! \code
//! uk = DCL_runDF22_C2(&arma2, rk);				// immediate result from pre-computed controller
//! i = DCL_runClamp_C1(&uk, 1.0f, -1.0f);			// clamp immediate result to +/-1.0
//! // ...use uk here...
//! if (0 == i)										// if immediate result is in range...
//! {
//!		DCL_runDF22_C3(&arma2, rk, uk);				// ...pre-compute the next partial result
//! }
//! \endcode
//!
//! \param[in] data	The address of the data variable
//! \param[in] Umax	The upper limit
//! \param[in] Umin	The lower limit
//! \return			Returns 0 if (Umin < data < Umax), else 1
//!
extern uint16_t DCL_runClamp_C1(float32_t *data, float32_t Umax, float32_t Umin);

//! \brief 			Saturates a control variable and returns 1 if either limit is exceeded
//! \param[in] data	The address of the data variable
//! \param[in] Umax	The upper limit
//! \param[in] Umin	The lower limit
//! \return			Returns 0 if (Umin < data < Umax), else 1
//!
static inline uint16_t DCL_runClamp_C2(float32_t *data, float32_t Umax, float32_t Umin)
{
	float32_t iv = *(data);

	*(data) = (*(data) > Umax) ? Umax : *(data);
	*(data) = (*(data) < Umin) ? Umin : *(data);

	return ((iv < Umax) && (iv > Umin)) ? 0 : 1;
}

//! \brief 			Saturates a control variable and returns 1.0f if either limit is exceeded
//!
//! \details		Can be used to saturate a pre-computed Direct Form 2 controller.
//!					If the immediate result is in range it can be used, otherwise
//!					it can be clamped and the next partial pre-computation skipped.
//!					An example of use with a pre-computed DF22 controller follows:
//!
//! \code
//! uk = DCL_runDF22_L2(&arma2, rk);				// immediate result from pre-computed controller
//! f = DCL_runClamp_L1(&uk, 1.0f, -1.0f);			// clamp immediate result to +/-1.0
//! // ...use uk here...
//! if (0.5f > f)									// if immediate result is in range...
//! {
//!		DCL_runDF22_L3(&arma2, rk, uk);				// ...pre-compute the next partial result
//! }
//! \endcode
//!
//! \param[in] data	The address of the data variable
//! \param[in] Umax	The upper limit
//! \param[in] Umin	The lower limit
//! \return			Returns 0.0f if (Umin < data < Umax), else 1.0f
//!
extern float32_t DCL_runClamp_L1(float32_t *data, float32_t Umax, float32_t Umin);

#endif // _C_DCL_H

/* end of file */
