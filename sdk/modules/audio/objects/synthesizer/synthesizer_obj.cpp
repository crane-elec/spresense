/****************************************************************************
 * modules/audio/objects/synthesizer/synthesizer_obj.cpp
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <nuttx/arch.h>
#include <stdlib.h>
#include <arch/chip/cxd56_audio.h>
#include "memutils/common_utils/common_assert.h"
#include "synthesizer_obj.h"
#include "components/oscillator/oscillator_component.h"
#include "debug/dbg_log.h"

__USING_WIEN2
using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool osc_done_callback(OscCmpltParam *param, void *instance)
{
  SynthesizerObject *obj = (SynthesizerObject *)instance;

  SynthesizerCommand cmd;
  MsgType            type = MSG_AUD_SYN_CMD_NEXT_REQ;

  if (param->event_type == Apu::FlushEvent)
    {
      cmd.comp_param.is_end = true;
    }
  else if (param->event_type == Apu::SetParamEvent)
    {
      type = MSG_AUD_SYN_CMD_SET_DONE;
    }
  else
    {
      cmd.comp_param.is_end = false;
    }

  err_t er = obj->send(type, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
static void pcm_proc_done_callback(int32_t identifier, bool is_end)
{
  SynthesizerObject*  obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  cmd.comp_param.is_end = is_end;

  err_t er = obj->send(MSG_AUD_SYN_CMD_DONE, cmd);

  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
static int AS_SynthesizerObjectEntry(int argc, char *argv[])
{
  struct SynthesizerObject *inst = (SynthesizerObject *)strtoul(argv[1], NULL, 16);

  inst->run();

  return 0;
}

/*--------------------------------------------------------------------------*/
static bool CreateSynthesizer(AsSynthesizerMsgQueId_t msgq_id, AsSynthesizerPoolId_t pool_id, AudioAttentionCb attcb)
{
  /* Register attention callback */

  SYNTHESIZER_REG_ATTCB(attcb);

  /* Clear */

  FAR MsgQueBlock *que;

  F_ASSERT(MsgLib::referMsgQueBlock(msgq_id.synthesizer, &que) == ERR_OK);

  que->reset();

  /* Create SynthesizerObject singleton instance */

  SynthesizerObject *inst;

  inst = SynthesizerObject::create(msgq_id, pool_id);

  /* Initialize task parameter. */

  char *argv[2];
  char  instatnce[16];

  snprintf(instatnce, 16, "%p", inst);

  argv[0] = instatnce;
  argv[1] = NULL;

  pid_t syn_pid = task_create("SYN_OBJ",
                              150,
                              1024 * 2,
                              AS_SynthesizerObjectEntry,
                              (FAR char* const*)argv);
  if (syn_pid < 0)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  SynthesizerObject::set_pid(syn_pid);

  return true;
}

/*--------------------------------------------------------------------------*/
SynthesizerObject *SynthesizerObject::create(AsSynthesizerMsgQueId_t msgq_id,
                                             AsSynthesizerPoolId_t   pool_id)
{
  SynthesizerObject*  inst = SynthesizerObject::get_instance();

  inst->m_msgq_id = msgq_id;
  inst->m_pool_id = pool_id;

  return inst;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::run(void)
{
  err_t        err_code;
  MsgQueBlock *que;
  MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_msgq_id.synthesizer, &que);
  F_ASSERT(err_code == ERR_OK);

  while(1)
    {
      err_code = que->recv(TIME_FOREVER, &msg);
      F_ASSERT(err_code == ERR_OK);

      parse(msg);

      err_code = que->pop();
      F_ASSERT(err_code == ERR_OK);
    }
}

/*--------------------------------------------------------------------------*/
SynthesizerObject::MsgProc SynthesizerObject::MsgProcTbl[AUD_SYN_MSG_NUM][SynthsizerStateNum] =
{
  /* Message type: MSG_AUD_SYN_CMD_ACT. */

  {                                          /* Synthesizer status: */
    &SynthesizerObject::activate,            /*   Booted            */
    &SynthesizerObject::illegalEvt,          /*   Ready             */
    &SynthesizerObject::illegalEvt,          /*   PreActive         */
    &SynthesizerObject::illegalEvt,          /*   Active            */
    &SynthesizerObject::illegalEvt,          /*   Stopping          */
    &SynthesizerObject::illegalEvt,          /*   ErrorStopping     */
    &SynthesizerObject::illegalEvt           /*   WaitStop          */
  },

  /* Message type: MSG_AUD_SYN_CMD_INIT. */

  {                                          /* Synthesizer status: */
    &SynthesizerObject::illegalEvt,          /*   Booted            */
    &SynthesizerObject::init,                /*   Ready             */
    &SynthesizerObject::illegalEvt,          /*   PreActive         */
    &SynthesizerObject::illegalEvt,          /*   Active            */
    &SynthesizerObject::illegalEvt,          /*   Stopping          */
    &SynthesizerObject::illegalEvt,          /*   ErrorStopping     */
    &SynthesizerObject::illegalEvt           /*   WaitStop          */
  },

  /* Message type:  MSG_AUD_SYN_CMD_START. */

  {                                          /* Synthesizer status: */
    &SynthesizerObject::illegalEvt,          /*   Booted            */
    &SynthesizerObject::execOnReady,         /*   Ready             */
    &SynthesizerObject::illegalEvt,          /*   PreActive         */
    &SynthesizerObject::illegalEvt,          /*   Active            */
    &SynthesizerObject::illegalEvt,          /*   Stopping          */
    &SynthesizerObject::illegalEvt,          /*   ErrorStopping     */
    &SynthesizerObject::illegalEvt           /*   WaitStop          */
  },

  /* Message type: MSG_AUD_SYN_CMD_STOP. */

  {                                          /* Synthesizer status: */
    &SynthesizerObject::illegalEvt,          /*   Booted            */
    &SynthesizerObject::illegalEvt,          /*   Ready             */
    &SynthesizerObject::illegalEvt,          /*   PreActive         */
    &SynthesizerObject::stopOnExec,          /*   Active            */
    &SynthesizerObject::illegalEvt,          /*   Stopping          */
    &SynthesizerObject::illegalEvt,          /*   ErrorStopping     */
    &SynthesizerObject::stopOnWait           /*   WaitStop          */
  },

  /* Message type: MSG_AUD_SYN_CMD_DEACT. */

  {                                          /* Synthesizer status: */
    &SynthesizerObject::illegalEvt,          /*   Booted            */
    &SynthesizerObject::deactivate,          /*   Ready             */
    &SynthesizerObject::illegalEvt,          /*   PreActive         */
    &SynthesizerObject::illegalEvt,          /*   Active            */
  	&SynthesizerObject::illegalEvt,          /*   Stopping          */
    &SynthesizerObject::illegalEvt,          /*   ErrorStopping     */
    &SynthesizerObject::illegalEvt           /*   WaitStop          */
  },

  /* Message type: MSG_AUD_SYN_CMD_SET */
  {                                          /* Synthesizer status: */
    &SynthesizerObject::illegalEvt,          /*   Booted            */
    &SynthesizerObject::set,                 /*   Ready             */
    &SynthesizerObject::illegalEvt,          /*   PreActive         */
    &SynthesizerObject::set,                 /*   Active            */
    &SynthesizerObject::illegalEvt,          /*   Stopping          */
    &SynthesizerObject::illegalEvt,          /*   ErrorStopping     */
    &SynthesizerObject::illegalEvt,          /*   WaitStop          */
  }
};

/*--------------------------------------------------------------------------*/
SynthesizerObject::MsgProc SynthesizerObject::MsgResultTbl[AUD_SYN_RST_MSG_NUM][SynthsizerStateNum] =
{
  /* Message type: MSG_AUD_SYN_CMD_NEXT_REQ. */

  {
    &SynthesizerObject::illegalCompDone,     /*   Booted            */
    &SynthesizerObject::illegalCompDone,     /*   Ready             */
    &SynthesizerObject::nextReqOnExec,       /*   PreActive         */
    &SynthesizerObject::cmpDoneOnExec,       /*   Active            */
    &SynthesizerObject::nextReqOnStopping,   /*   Stopping          */
    &SynthesizerObject::illegalCompDone,     /*   ErrorStopping     */
    &SynthesizerObject::illegalCompDone      /*   WaitStop          */

  },

  /* Message type: MSG_AUD_SYN_CMD_DONE. */

  {
    &SynthesizerObject::illegalCompDone,     /*   Booted            */
    &SynthesizerObject::illegalCompDone,     /*   Ready             */
    &SynthesizerObject::illegalCompDone,     /*   PreActive         */
    &SynthesizerObject::cmpDoneOnExec,       /*   Active            */
    &SynthesizerObject::cmpDoneOnStopping,   /*   Stopping          */
    &SynthesizerObject::illegalCompDone,     /*   ErrorStopping     */
    &SynthesizerObject::illegalCompDone      /*   WaitStop          */
  },

  /* Message type: MSG_AUD_SYN_CMD_SET_DONE. */

  {
    &SynthesizerObject::illegalCompDone,     /*   Booted            */
    &SynthesizerObject::cmpDoneOnSet,        /*   Ready             */
    &SynthesizerObject::cmpDoneOnSet,        /*   PreActive         */
    &SynthesizerObject::cmpDoneOnSet,        /*   Active            */
    &SynthesizerObject::illegalCompDone,     /*   Stopping          */
    &SynthesizerObject::illegalCompDone,     /*   ErrorStopping     */
    &SynthesizerObject::illegalCompDone      /*   WaitStop          */
  },
};

/*--------------------------------------------------------------------------*/
void SynthesizerObject::parse(MsgPacket *msg)
{
  uint event = MSG_GET_SUBTYPE(msg->getType());

  if (MSG_IS_REQUEST(msg->getType()) != 0)
    {
      F_ASSERT(event < AUD_SYN_MSG_NUM);
      (this->*MsgProcTbl[event][m_state.get()])(msg);
    }
  else
    {
      F_ASSERT(event < AUD_SYN_RST_MSG_NUM);
      (this->*MsgResultTbl[event][m_state.get()])(msg);
    }
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::reply(AsSynthesizerEvent evtype, MsgType msg_type, uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, m_param);
    }
  else if (m_msgq_id.mng != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_SYNTHESIZER_OBJ,
                           result);
      err_t er = MsgLib::send<AudioObjReply>(m_msgq_id.mng,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             m_msgq_id.synthesizer,
                                             cmplt);
      if (ERR_OK != er)
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::illegalEvt(MsgPacket *msg)
{
  uint msgtype = msg->getType();

  msg->moveParam<SynthesizerCommand>();

  /* Extract and abandon message data */

  uint32_t idx = msgtype - MSG_AUD_SYN_CMD_ACT;
  
  AsSynthesizerEvent table[] =
  {
    AsSynthesizerEventAct,
    AsSynthesizerEventInit,
    AsSynthesizerEventStart,
    AsSynthesizerEventStop,
    AsSynthesizerEventDeact,
    AsSynthesizerEventSet
  };

  reply(table[idx], (MsgType)msgtype, AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::stopOnExec(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  MemHandle     mh;

  uint32_t  max_pcm_buff_size =
    (Manager::getPoolSize(m_pool_id.output)) / (Manager::getPoolNumSegs(m_pool_id.output));

  if (mh.allocSeg(m_pool_id.output, max_pcm_buff_size) != ERR_OK)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return;
    }

  if (!m_pcm_buf_mh_que.push(mh))
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  if (!m_oscillator.flush())
    {
      SYNTHESIZER_OBJ_ERR(AS_ECODE_DSP_STOP_ERROR);
    }

  m_state = SynthsizerStateStopping;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::stopOnWait(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  reply(AsSynthesizerEventStop, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::activate(MsgPacket *msg)
{
  AsActivateSynthesizer act    = msg->moveParam<SynthesizerCommand>().act_param;
  uint32_t              result = AS_ECODE_OK;

  /* Set event callback */

  m_callback = act.cb;
  m_param    = act.param;

  /* Active */

  reply(AsSynthesizerEventAct, msg->getType(), result);

  m_state = SynthsizerStateReady;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::deactivate(MsgPacket *msg)
{
  uint32_t  result = AS_ECODE_OK;

  msg->moveParam<SynthesizerCommand>();

  SYNTHESIZER_OBJ_DBG("DEACT:\n");

  if (!m_oscillator.deactivate())
    {
      result = AS_ECODE_DSP_UNLOAD_ERROR;
    }

  memset(m_dsp_path, 0, sizeof(m_dsp_path));

  reply(AsSynthesizerEventDeact, msg->getType(), result);

  m_state = SynthsizerStateBooted;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::init(MsgPacket *msg)
{
  AsInitSynthesizerParam param   = msg->moveParam<SynthesizerCommand>().init_param;
  uint32_t               result  = AS_ECODE_OK;
  uint32_t               dsp_inf = 0;

  SYNTHESIZER_OBJ_DBG("INIT: type %d, ch num %d, bit len %d, sampling_rate %d\n",
                      param.type,
                      param.channel_num,
                      param.bit_width,
                      param.sampling_rate);

  if (strcmp(m_dsp_path, param.dsp_path) != 0)
    {
      if (m_dsp_path[0] != '\0')
        {
          m_oscillator.deactivate();
        }

      result = m_oscillator.activate(m_msgq_id.dsp, m_pool_id.dsp, param.dsp_path, &dsp_inf);
    }

  if (result == AS_ECODE_OK)
    {
      strcpy(m_dsp_path, param.dsp_path);

      InitOscParam  osc_init;

      /* Waveform setting */

      if (param.type == AsSynthesizerSinWave)
        {
          osc_init.type = SinWave;
        }
      else
        {
          result = AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST;
          SYNTHESIZER_OBJ_ERR(result);
        }

      /* Number of channels setting */

      osc_init.channel_num = param.channel_num;

      /* Bit depth setting */

      if (param.bit_width == AS_BITLENGTH_16)
        {
          osc_init.bit_length = AudPcm16Bit;
        }
      else if (param.bit_width == AS_BITLENGTH_24)
        {
          osc_init.bit_length = AudPcm24Bit;
        }
      else if (param.bit_width == AS_BITLENGTH_32)
        {
          osc_init.bit_length = AudPcm32Bit;
        }
      else
        {
          result = AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST;
          SYNTHESIZER_OBJ_ERR(result);
        }

      m_bit_length = param.bit_width;

      /* Sampling rate setting */
#if 0
      if (param.sampling_rate == AS_SAMPLINGRATE_48000)
        {
          osc_init.sampling_rate = AudFs_48000;
        }
      else if (param.sampling_rate == AS_SAMPLINGRATE_96000)
        {
          osc_init.sampling_rate = AudFs_96000;
        }
      else if (param.sampling_rate == AS_SAMPLINGRATE_192000)
        {
          osc_init.sampling_rate = AudFs_192000;
        }
      else
        {
          result = AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST;
          SYNTHESIZER_OBJ_ERR(result);
        }
#else
      osc_init.sampling_rate = param.sampling_rate;
#endif

      /* Callback, parameter setting */

      osc_init.callback = osc_done_callback;
      osc_init.instance = this;

      /* Set frequency value */

      if (result == AS_ECODE_OK)
        {
          m_oscillator.init(osc_init, &dsp_inf);
        }
    }

  reply(AsSynthesizerEventInit, msg->getType(), result);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::illegalCompDone(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();
  SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::execOnReady(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  /* First oscillator request */

  oscillator_exec();

  reply(AsSynthesizerEventStart, msg->getType(), AS_ECODE_OK);

  m_state = SynthsizerStatePreActive;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::set(MsgPacket *msg)
{
  AsSetSynthesizer param = msg->moveParam<SynthesizerCommand>().set_param;

  SetOscParam osc_set;

  osc_set.channel_no = param.channel_no;
  osc_set.frequency  = param.frequency;

  m_oscillator.set(osc_set);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::nextReqOnExec(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  uint32_t  max_pcm_buff_size =
    (Manager::getPoolSize(m_pool_id.output)) / (Manager::getPoolNumSegs(m_pool_id.output));

  if (m_pcm_buf_mh_que.size() < MAX_OUT_BUFF_NUM)
    {
      /* Next oscillator request */

      oscillator_exec();
    }
  else
    {
      AsPcmDataParam  data;

      while (!m_pcm_buf_mh_que.empty())
        {
          /* Next mixer request */

          data.mh         = m_pcm_buf_mh_que.top();
          data.size       = max_pcm_buff_size;
          data.is_end     = false;
          data.is_valid   = 1;

          sendPcmToOwner(data);

          if (!m_pcm_buf_mh_que.pop())
            {
              SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
              break;
            }
        }

      /* Next request */

      oscillator_exec();

      m_state = SynthsizerStateActive;
    }
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::nextReqOnStopping(MsgPacket *msg)
{
  AsCompleteSynthesizer param = msg->moveParam<SynthesizerCommand>().comp_param;

  uint32_t  max_pcm_buff_size =
    (Manager::getPoolSize(m_pool_id.output)) / (Manager::getPoolNumSegs(m_pool_id.output));

  AsPcmDataParam  data;

  /* End mixer request */

  data.is_end   = param.is_end;
  data.mh       = m_pcm_buf_mh_que.top();
  data.is_valid = 1;
  data.size     = max_pcm_buff_size;

  m_pcm_buf_mh_que.pop();

  sendPcmToOwner(data);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::cmpDoneOnExec(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  uint32_t  max_pcm_buff_size =
    (Manager::getPoolSize(m_pool_id.output)) / (Manager::getPoolNumSegs(m_pool_id.output));

  AsPcmDataParam  data;

  while (!m_pcm_buf_mh_que.empty())
    {
      data.mh         = m_pcm_buf_mh_que.top();
      data.size       = max_pcm_buff_size;
      data.is_end     = false;
      data.is_valid   = 1;

      sendPcmToOwner(data);

      if (!m_pcm_buf_mh_que.pop())
        {
          SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_POP_ERROR);
          break;
        }
    }

  /* Next request */

  if (Manager::getPoolNumAvailSegs(m_pool_id.output) > 1)
    {
      oscillator_exec();
    }
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::cmpDoneOnStopping(MsgPacket *msg)
{
  AsCompleteSynthesizer param = msg->moveParam<SynthesizerCommand>().comp_param;

  if (param.is_end)
    {
      reply(AsSynthesizerEventStop, msg->getType(), AS_ECODE_OK);

      m_state = SynthsizerStateReady;
    }
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::cmpDoneOnSet(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  reply(AsSynthesizerEventSet, msg->getType(), AS_ECODE_OK);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::sendPcmToOwner(AsPcmDataParam& data)
{
  /* Send message for PCM data notify */

  data.identifier = 0;
  data.callback   = pcm_proc_done_callback;
  data.sample     = AS_SAMPLINGRATE_48000;
  data.bit_length = m_bit_length;

  err_t er = MsgLib::send<AsPcmDataParam>(m_msgq_id.mixer,
                                          MsgPriNormal,
                                          MSG_AUD_MIX_CMD_DATA,
                                          m_msgq_id.synthesizer,
                                          data);
  F_ASSERT(er == ERR_OK);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::oscillator_exec(void)
{
  ExecOscParam  param;
  MemHandle     mh;

  uint32_t  max_pcm_buff_size =
    (Manager::getPoolSize(m_pool_id.output)) / (Manager::getPoolNumSegs(m_pool_id.output));

  if (mh.allocSeg(m_pool_id.output, max_pcm_buff_size) != ERR_OK)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_MEMHANDLE_ALLOC_ERROR);
      return;
    }

  if (!m_pcm_buf_mh_que.push(mh))
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_QUEUE_PUSH_ERROR);
      return;
    }

  param.buffer.size     = mh.getSize();
  param.buffer.p_buffer = (unsigned long *)mh.getPa();

  if (!m_oscillator.exec(param))
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*--------------------------------------------------------------------------*/
bool AS_CreateMediaSynthesizer(FAR AsCreateSynthesizerParam_t *param,
                                   AudioAttentionCb            attcb)
{
  /* Parameter check */

  if (param == NULL)
    {
      return false;
    }

  return CreateSynthesizer(param->msgq_id, param->pool_id, attcb);
}

/*--------------------------------------------------------------------------*/
bool AS_CreateMediaSynthesizer(FAR AsCreateSynthesizerParam_t *param)
{
  return AS_CreateMediaSynthesizer(param, NULL);
}

/*--------------------------------------------------------------------------*/
bool AS_ActivateMediaSynthesizer(FAR AsActivateSynthesizer *actparam)
{
  /* Parameter check */

  if (actparam == NULL)
    {
      return false;
    }

  /* Activate */

  SynthesizerObject *obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  cmd.act_param = *actparam;

  err_t er = obj->send(MSG_AUD_SYN_CMD_ACT, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_InitMediaSynthesizer(FAR AsInitSynthesizerParam *initparam)
{
  /* Parameter check */

  if (initparam == NULL)
    {
      return false;
    }

  /* Init */

  SynthesizerObject *obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  cmd.init_param = *initparam;

  err_t er = obj->send(MSG_AUD_SYN_CMD_INIT, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StartMediaSynthesizer(void)
{
  /* Start */

  SynthesizerObject *obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  err_t er = obj->send(MSG_AUD_SYN_CMD_START, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_SetMediaSynthesizer(FAR AsSetSynthesizer *set_param)
{
  /* Set */

  SynthesizerObject *obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  cmd.set_param = *set_param;

  err_t er = obj->send(MSG_AUD_SYN_CMD_SET, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StopMediaSynthesizer(void)
{
  /* Stop */

  SynthesizerObject *obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  err_t er = obj->send(MSG_AUD_SYN_CMD_STOP, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateMediaSynthesizer(void)
{
  SynthesizerObject *obj = SynthesizerObject::get_instance();

  SynthesizerCommand cmd;

  err_t er = obj->send(MSG_AUD_SYN_CMD_DEACT, cmd);

  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteMediaSynthesizer(void)
{
  pid_t pid = SynthesizerObject::get_pid();

  task_delete(pid);

  /* Unregister attention callback */

  SYNTHESIZER_OBJ_UNREG_ATTCB();

  return true;
}
