/*!*****************************************************************
 * \file    sigfox_ep_flags.h
 * \brief   Sigfox End-Point library compilations flags definition.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#ifndef __SIGFOX_EP_FLAGS_H__
#define __SIGFOX_EP_FLAGS_H__

/*** Library compilation flags ***/

#define RC1 CONFIG_SIGFOX_RC1
#define RC2 CONFIG_SIGFOX_RC2
#define RC3C CONFIG_SIGFOX_RC3C
#define RC3D CONFIG_SIGFOX_RC3D
#define RC4 CONFIG_SIGFOX_RC4
#define RC5 CONFIG_SIGFOX_RC5
#define RC6 CONFIG_SIGFOX_RC6
#define RC7 CONFIG_SIGFOX_RC7
#define APPLICATION_MESSAGES CONFIG_SIGFOX_APPLICATION_MESSAGES
#define CONTROL_KEEP_ALIVE_MESSAGE CONFIG_SIGFOX_CONTROL_KEEP_ALIVE_MESSAGE
#define BIDIRECTIONAL CONFIG_SIGFOX_BIDIRECTIONAL
#define ASYNCHRONOUS CONFIG_SIGFOX_ASYNCHRONOUS
#define LOW_LEVEL_OPEN_CLOSE CONFIG_SIGFOX_LOW_LEVEL_OPEN_CLOSE
#define REGULATORY CONFIG_SIGFOX_REGULATORY
#define LATENCY_COMPENSATION CONFIG_SIGFOX_LATENCY_COMPENSATION
#define PARAMETERS_CHECK CONFIG_SIGFOX_PARAMETERS_CHECK
#define CERTIFICATION CONFIG_SIGFOX_CERTIFICATION
#define PUBLIC_KEY_CAPABLE CONFIG_SIGFOX_PUBLIC_KEY_CAPABLE
#define VERBOSE CONFIG_SIGFOX_VERBOSE
#define ERROR_CODES CONFIG_SIGFOX_ERROR_CODES
#define ERROR_STACK CONFIG_SIGFOX_ERROR_STACK

#endif /* __SIGFOX_EP_FLAGS_H__ */
