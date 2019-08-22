/****************************************************************************
 * modules/audio/components/oscillator/oscillator_component.h
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

#ifndef WIEN2_OSCILLATOR_COMPONENT_H
#define WIEN2_OSCILLATOR_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"

#include "wien2_common_defs.h"
#include "memutils/s_stl/queue.h"
#include "apus/apu_cmd.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "debug/dbg_log.h"
#include "components/common/component_common.h"

using namespace MemMgrLite;

__WIEN2_BEGIN_NAMESPACE

typedef bool (*EncCompCallback)(void*);

struct InitOscParam
{
  WaveType         type;
  uint8_t          channel_no;
  uint32_t         frequency;
  AudioPcmBitWidth bit_width;
  OscCompCallback  callback;
};


struct ExecOscParam
{
  BufferHeader buffer;
  uint8_t      channel_no;
};

struct StopOscParam
{
};

struct SetOscParam
{
  uint8_t          channel_no;
  uint32_t         frequency;
};

struct OscCmpltParam
{
  Apu::ApuEventType event_type;
  bool              result;

  union
  {
    InitOscParam init_enc_cmplt;
    ExecOscParam exec_enc_cmplt;
    StopOscParam stop_enc_cmplt;
  };
};

class OscillatorComponent : public ComponentCommon<Apu::InternalResult>
{
public:
  OscillatorComponent(MsgQueId apu_dtq,PoolId apu_pool_id)
  {
    m_apu_dtq = apu_dtq;
    m_apu_pool_id = apu_pool_id;
  }
  ~OscillatorComponent() {}

  uint32_t activate(const char *path, uint32_t *dsp_inf);
  bool     deactivate();
  uint32_t init(const InitOscParam& param, uint32_t *dsp_inf);
  bool     exec(const ExecOscParam& param);
  bool     flush(const StopOscParam& param);
  bool     set(const SetOscParam& param);
  bool     recv(void *p_param);
  bool     done(void)
           {
             if (!m_apu_cmd_mh_que.pop())
             {
               ENCODER_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_FREE_ERROR);
               return false;
              }
             return true;
           };

private:
  typedef s_std::Queue<MemMgrLite::MemHandle, APU_COMMAND_QUEUE_SIZE> ApuCmdMhQue;
  ApuCmdMhQue m_apu_cmd_mh_que;

  MsgQueId m_apu_dtq;
  PoolId m_apu_pool_id;

  void send_apu(Apu::Wien2ApuCmd*);
  EncCompCallback m_callback;

  void *m_dsp_handler;
};

__WIEN2_END_NAMESPACE

#endif /* WIEN2_OSCILLATOR_COMPONENT_H */

