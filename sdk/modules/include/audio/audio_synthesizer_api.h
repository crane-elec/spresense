/****************************************************************************
 * modules/include/audio/audio_synthesizer_api.h
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

#ifndef __MODULES_INCLUDE_AUDIO_AUDIO_RECORDER_API_H
#define __MODULES_INCLUDE_AUDIO_AUDIO_RECORDER_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_recorder_api Audio Synthesizer API
 * @{
 *
 * @file       audio_synthesizer_api.h
 * @brief      CXD5602 Audio Synthesizer API
 * @author     CXD5602 Audio SW Team
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "audio/audio_common_defs.h"
#include "audio/audio_object_common_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AS_FEATURE_SYNTHESIZER_ENABLE

/** @name Packet length of command*/
/** @{ */

/*! \brief InitRecorder command (#AUDCMD_INITSYN) packet length */

#define LENGTH_INIT_SYNTHESIZER    10 

/*! \brief StartSyn command (#AUDCMD_STARTSYN) packet length */

#define LENGTH_START_SYNTHESIZER    2

/*! \brief StopSyn command (#AUDCMD_STOPSYN) packet length */

#define LENGTH_STOP_SYNTHESIZER     2


/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Event type of Recorder */

typedef enum
{
  /*! \brief Activate */

  AsSynthesizerEventAct = 0,

  /*! \brief Init */

  AsSynthesizerEventInit,

  /*! \brief Start */

  AsSynthesizerEventStart,

  /*! \brief Stop */

  AsSynthesizerEventStop,

  /*! \brief Deactivate */

  AsSynthesizerEventDeact,

  /*! \brief SetMicGain */

  AsSynthesizerEventSet

} AsSynthesizerEvent;

/* SetSynthesizerStatus */

/* InitSynthesizer */

/** SetSynthesizerStatus Command (#AUDCMD_SETSYNTHESIZERSTATUS) parameter */

typedef struct
{
	
} AsActivateSynthesizerParam;

typedef bool (*SynthesizerCallback)(AsSynthesizerEvent evtype, uint32_t result, uint32_t sub_result);

typedef struct
{
  AsActivateSynthesizerParam param;
	
} AsActivateSynthesizer;

/** InitSynthesizer Command (#AUDCMD_INITSYN) parameter */

typedef struct
{
  WaveType         type;
  uint8_t          channel_no;
  uint32_t         frequency;
  AudioPcmBitWidth bit_width;

  char dsp_path[AS_AUDIO_DSP_PATH_LEN];
	
} AsInitSynthesizerParam;

/** RecorderCommand definition */
typedef union
{
  /*! \brief [in] for ActivateRecorder
   * (Object Interface==AS_ActivateMediaRecorder)
   */
 
  AsActivateRecorder act_param;


  /*! \brief [in] for InitRecorder
   * (Object Interface==AS_InitMediaRecorder)
   */

  AsInitRecorderParam init_param;

  /*! \brief [in] for SetMicGain
   * (Object Interface==AS_SetMicGainMediaRecorder)
   */

} RecorderCommand;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of recorder */

  uint8_t recorder;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;
} AsRecorderMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of input data */

  uint8_t input;

  /*! \brief [in] Memory pool id of output data */

  uint8_t output;

  /*! \brief [in] Memory pool id of dsp command data */

  uint8_t dsp;
} AsRecorderPoolId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsRecorderMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsRecorderPoolId_t   pool_id;
} AsCreateRecorderParam_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Create audio recorder
 *
 * @param[in] param: Parameters of resources used by audio recorder
 * @param[in] attcb: Attention callback of Recorder. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateMediaRecorder(FAR AsCreateRecorderParam_t *param,
                            AudioAttentionCb attcb);

__attribute__((deprecated(
                 "\n \
                  \n Deprecated create API is used. \
                  \n Use \"AS_CreateMediaRecorder(AsCreateRecorderParam_t, \
                  \n                             AudioAttentionCb)\". \
                  \n \
                  \n")))
bool AS_CreateMediaRecorder(FAR AsCreateRecorderParam_t *param);

/**
 * @brief Activate audio recorder
 *
 * @param[in] param: Activation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateMediaRecorder(FAR AsActivateRecorder *actparam);

/**
 * @brief Init audio recorder
 *
 * @param[in] param: Initialization parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_InitMediaRecorder(FAR AsInitRecorderParam *initparam);

/**
 * @brief Start audio recorder
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StartMediaRecorder(void);

/**
 * @brief Stop audio recorder
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StopMediaRecorder(void);

/**
 * @brief Deactivate audio recorder
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateMediaRecorder(void);

/**
 * @brief Deactivate audio recorder
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteMediaRecorder(void);

/**
 * @brief Set mic gain for audio recorder
 *
 * @param[in] gain    : Mic gain
 *
 * @retval     true  : success
 * @retval     false : failure
 * @note Refer to AsRecorderMicGainParam for gain setting range.
 */

bool AS_SetMicGainMediaRecorder(FAR AsRecorderMicGainParam *micgain_param);

#endif  /* __MODULES_INCLUDE_AUDIO_AUDIO_RECORDER_API_H */
/**
 * @}
 */

/**
 * @}
 */
