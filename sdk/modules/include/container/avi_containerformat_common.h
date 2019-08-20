/****************************************************************************
 * modules/include/container/avi_containerformat_common.h
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef MODULES_INCLUDE_CONTAINER_AVI_CONTAINERFORMAT_COMMON_H
#define MODULES_INCLUDE_CONTAINER_AVI_CONTAINERFORMAT_COMMON_H

/* Required chunk. */

#define CHUNKID_RIFF     0x46464952  /*!< RIFF */
#define FORMAT_AVI       0x20495641  /*!< AVI  */


#define SUBCHUNKID_LIST  0x5453494C  /*!< LIST */
#define SUBCHUNKID_JUNK  0x4B4E554A  /*!< JUNK */

#define SUBCHUNKID_HDRL  0x6c726468  /*!< hdrl */
#define SUBCHUNKID_AVIH  0x68697661  /*!< avih */
#define SUBCHUNKID_STRL  0x6c727473  /*!< strl */
#define SUBCHUNKID_STRH  0x68727473  /*!< strh */
#define SUBCHUNKID_STRF  0x66727473  /*!< strf */
#define SUBCHUNKID_MOVI  0x69766f6d  /*!< movi */

#define FCCTYPE_VIDEO    0x73646976  /*!< strh */
#define FCCTYPE_AUDIO    0x73647561  /*!< strh */

#define FCCHANDLER_MJPG  0x67706a6d  /*!< mjpg */

#define MOVICHANKID_VIDEO 0x63643030  /*!< 00dc */
#define MOVICHANKID_AUDIO 0x67706a6d  /*!< 01wb */

#endif /* MODULES_INCLUDE_CONTAINER_AVI_CONTAINERFORMAT_COMMON_H */
