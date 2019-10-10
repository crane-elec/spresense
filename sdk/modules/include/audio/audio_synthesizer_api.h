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

#ifndef __MODULES_INCLUDE_AUDIO_SYNTHESIZER_API_H
#define __MODULES_INCLUDE_AUDIO_SYNTHESIZER_API_H

/**
 * @defgroup audioutils Audio Utility
 * @{
 */

/**
 * @defgroup audioutils_audio_synthesizer_api Audio Synthesizer API
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

/*! \brief InitSynthesizer command (#AUDCMD_INITSYN) packet length */

#define LENGTH_INIT_SYNTHESIZER    10 

/*! \brief StartSyn command (#AUDCMD_STARTSYN) packet length */

#define LENGTH_START_SYNTHESIZER    2

/*! \brief StopSyn command (#AUDCMD_STOPSYN) packet length */

#define LENGTH_STOP_SYNTHESIZER     2


/** @} */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/** Event type of Synthesizer */

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

/** Waveform of Synthesizer */

typedef enum
{
  InvalidWave          = 0xffffffff,

  /*! \brief Waveform */

  AsSynthesizerSinWave = 0,

  AsSynthesizerWaveModeNum

} AsSynthesizerWaveMode;

/* SetSynthesizerStatus */

/* InitSynthesizer */

/** SetSynthesizerStatus Command (#AUDCMD_SETSYNTHESIZERSTATUS) parameter */

typedef bool (*SynthesizerCallback)(AsSynthesizerEvent evtype, uint32_t result, void *param);

typedef struct
{
  /*! \brief [in] Processing completion callback */

  SynthesizerCallback cb;

  /*! \brief [in] General parameter to callback arguments */

  void               *param;

} AsActivateSynthesizer;

/** InitSynthesizer Command (#AUDCMD_INITSYN) parameter */

typedef struct
{
  /*! \brief [in] Waveform type */

  AsSynthesizerWaveMode type;

  /*! \brief [in] Number of channels */

  uint8_t               channel_num;

  /*! \brief [in] Sound frequency */

  uint32_t              sampling_rate;

  /*! \brief [in] Bit depth */

  uint8_t               bit_width;

  /*! \brief [in] Audio DSP path */

  char dsp_path[AS_AUDIO_DSP_PATH_LEN];

} AsInitSynthesizerParam;

typedef struct
{
  /*! \brief [in] Processing stop flag */

  bool  is_end;

} AsCompleteSynthesizer;

typedef struct
{
  /*! \brief [in] Channel number */

  uint8_t               channel_no;

  /*! \brief [in] Sound frequency */

  uint32_t              frequency;

} AsSetSynthesizer;

/** SynthesizerCommand definition */

typedef union
{
  /*! \brief [in] for AsExecSynthesizer
   * (Object Interface==AsExecSynthesizer)
   */
 
  AsCompleteSynthesizer  comp_param;

  /*! \brief [in] for ActivateSynthesizer
   * (Object Interface==AS_ActivateMediaSynthesizer)
   */
 
  AsActivateSynthesizer  act_param;


  /*! \brief [in] for InitSynthesizer
   * (Object Interface==AS_InitMediaSynthesizer)
   */

  AsInitSynthesizerParam init_param;

  /*! \brief [in] for SetMicGain
   * (Object Interface==AS_SetMicGainMediaSynthesizer)
   */

  /*! \brief [in] for SetSynthesizer
   * (Object Interface==AS_SetMediaSynthesizer)
   */

  AsSetSynthesizer       set_param;

} SynthesizerCommand;

/** Message queue ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Message queue id of output mixer */

  uint8_t mixer;

  /*! \brief [in] Message queue id of synthesizer */

  uint8_t synthesizer;

  /*! \brief [in] Message queue id of audio_manager */

  uint8_t mng;

  /*! \brief [in] Message queue id of DSP */

  uint8_t dsp;

} AsSynthesizerMsgQueId_t;

/** Pool ID parameter of activate function */

typedef struct
{
  /*! \brief [in] Memory pool id of input data */

  MemMgrLite::PoolId input;

  /*! \brief [in] Memory pool id of output data */

  MemMgrLite::PoolId output;

  /*! \brief [in] Memory pool id of dsp command data */

  MemMgrLite::PoolId dsp;

} AsSynthesizerPoolId_t;

/** Activate function parameter */

typedef struct
{
  /*! \brief [in] ID for sending messages to each function */

  AsSynthesizerMsgQueId_t msgq_id;

  /*! \brief [in] ID of memory pool for processing data */

  AsSynthesizerPoolId_t   pool_id;

} AsCreateSynthesizerParam_t;

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
 * @brief Create audio synthesizer
 *
 * @param[in] param: Parameters of resources used by audio synthesizer
 * @param[in] attcb: Attention callback of Synthesizer. NULL means no callback.
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_CreateMediaSynthesizer(FAR AsCreateSynthesizerParam_t *param,
                                   AudioAttentionCb            attcb);

__attribute__((deprecated(
                 "\n \
                  \n Deprecated create API is used. \
                  \n Use \"AS_CreateMediaSynthesizer(AsCreateSynthesizerParam_t, \
                  \n                                 AudioAttentionCb)\". \
                  \n \
                  \n")))
bool AS_CreateMediaSynthesizer(FAR AsCreateSynthesizerParam_t *param);

/**
 * @brief Activate audio synthesizer
 *
 * @param[in] param: Activation parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_ActivateMediaSynthesizer(FAR AsActivateSynthesizer *actparam);

/**
 * @brief Init audio synthesizer
 *
 * @param[in] param: Initialization parameters
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_InitMediaSynthesizer(FAR AsInitSynthesizerParam *initparam);

/**
 * @brief Start audio synthesizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StartMediaSynthesizer(void);

/**
 * @brief Stop audio synthesizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_StopMediaSynthesizer(void);

/**
 * @brief Set audio synthesizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_SetMediaSynthesizer(FAR AsSetSynthesizer *set_param);

/**
 * @brief Deactivate audio synthesizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeactivateMediaSynthesizer(void);

/**
 * @brief Deactivate audio synthesizer
 *
 * @retval     true  : success
 * @retval     false : failure
 */

bool AS_DeleteMediaSynthesizer(void);

#endif  /* __MODULES_INCLUDE_AUDIO_SYNTHESIZER_API_H */
/**
 * @}
 */

/**
 * @}
 */
