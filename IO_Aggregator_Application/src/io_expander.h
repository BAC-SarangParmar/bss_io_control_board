/*
 *      Copyright (c) 2024 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Bacancy System LLP
 *      Date & Time: 29/03/2024 11:20
 *
 *
 *                      CONFIDENTIAL INFORMATION
 *                      ------------------------
 *      This Document contains Confidential Information or Trade Secrets,
 *      or both, which are the property of Bacancy System LLP.
 *      This document may not be copied, reproduced, reduced to any
 *      electronic medium or machine readable form or otherwise
 *      duplicated and the information herein may not be used,
 *      disseminated or otherwise disclosed, except with the prior
 *      written consent of Bacancy System LLP.
 *
 */
 /*!
 * \file : io_expander.h
 * \brief : includes prototype of public function defined in io_expander file,
 *          definitions of structure and Macros for io_expander.
 */

#ifndef __IO_EXPANDER_H_
#define __IO_EXPANDER_H_

/***********************************************************************
 * Include Header Files
 **********************************************************************/
#include "io_expander.h"
#include "app.h"
#include "definitions.h"

/***********************************************************************
* PUBLIC FUNCTION PROTOTYPES
***********************************************************************/
void vConfigureIOexpanders(void);
uint8_t TCA9539_ReadRegister(uint8_t reg);
/***********************************************************************
* END OF FILE
***********************************************************************/
#endif  //__IO_EXPANDER_H_
