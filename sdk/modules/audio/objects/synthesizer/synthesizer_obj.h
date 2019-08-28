/****************************************************************************
 * modules/audio/objects/synthesizer/synthesizer_obj.h
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

#ifndef __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H
#define __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/message/Message.h"
#include "memutils/s_stl/queue.h"
#include "memutils/memory_manager/MemHandle.h"
#include "audio/audio_high_level_api.h"
#include "audio/audio_message_types.h"
#include "components/oscillator/oscillator_component.h"
#include "audio_state.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

class SynthsizerObject {
public:
  static void create(AsSynthsizerMsgQueId_t msgq_id,
                     AsSynthsizerPoolId_t pool_id);

private:
  SynthsizerObject(AsSynthsizerMsgQueId_t msgq_id,
                   AsSynthsizerPoolId_t pool_id);
    m_msgq_id(msgq_id),
    m_pool_id(pool_id),
  {}

  enum SynthsizerState_e
  {
    SynthsizerStateInactive = 0,
    SynthsizerStateReady,
    SynthsizerStateExecute,
    SynthsizerStateStopping,
    SynthsizerStateErrorStopping,
    SynthsizerStateWaitStop,
    SynthsizerStateNum
  };

  AsSynthsizerMsgQueId_t m_msgq_id;
  AsSynthsizerPoolId_t   m_pool_id;

  AudioState<SynthsizerState_e> m_state;

  OscllicatorComponentHandler m_osc_hdlr;

  SynthsizerCallback m_callback;

  void run();
  void parse(MsgPacket *);

  void reply(AsSynthsizerEvent evtype,
             MsgType msg_type,
             uint32_t result);

  void illegal(MsgPacket *);
  void activate(MsgPacket *);
  void deactivate(MsgPacket *);

  void init(MsgPacket *);
  void execOnReady(MsgPacket *);
  void stopOnExec(MsgPacket *);
  void set(MsgPacket *);
//  void stopOnWait(MsgPacket *);

  void nextReqOnExec(MsgPacket *msg);
  void nextReqOnStopping(MsgPacket *msg);
  void cmpDoneOnExec(MsgPacket *msg);
  void cmpDoneOnStopping(MsgPacket *msg);


/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

__WIEN2_END_NAMESPACE

#endif /* __MODULES_AUDIO_OBJECTS_MEDIA_RECORDER_MEDIA_RECORDER_OBJ_H */

