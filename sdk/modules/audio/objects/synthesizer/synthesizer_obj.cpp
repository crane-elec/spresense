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
#include "media_recorder_obj.h"
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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t s_syn_pid;
static AsSynthesizerMsgQueId_t s_msgq_id;
static AsSynthesizerPoolId_t   s_pool_id;

static SynthesizerObject *s_syn_obj = NULL;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int AS_SynthesizerObjectEntry(int argc, char *argv[])
{
  SynthesizerObject::create(s_msgq_id,
                           s_pool_id);
  return 0;
}

/*--------------------------------------------------------------------------*/
bool CreateSynthesizer(AsSynthesizerMsgId msgq_id, AsSynthesizerPoolId_t pool_id, AudioAttentionCb attcb)
{
  /* Register attention callback */

//  MEDIA_PLAYER_REG_ATTCB(attcb);

  /* Parameter check */

  /* Create */

  s_msgq_id = msgq_id;
  s_pool_id = pool_id;
	
  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(s_msgq_id, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  s_syn_pid = task_create("SYN_OBJ",
                           150, 1024 * 2,
                           AS_SynthesizerObjectEntry,
                           NULL);
      if (s_syn_pid < 0)
        {
          _ERR(id, AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
          return false;
        }
  return true;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::create(FAR void **obj,
                              AsSynthesizerMsgQueId_t msgq_id,
                              AsSynthesizerPoolId_t pool_id) /* なんでタイプが必要なんだっけ？ */
{
  FAR SynthesizerObject *syn_obj = new SynthesizerObject(msgq_id, pool_id);

  if (syn_obj != NULL)
    {
      *obj = reinterpret_cast<void*>(syn_obj); /* これはなんで必要だったっけ？*/
      syn_obj->run();
    }
  else
    {
      _ERR(player_id, AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return;
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*--------------------------------------------------------------------------*/
void SynthesizerObject::run(void)
{
  err_t        err_code;
  MsgQueBlock *que;
  MsgPacket   *msg;

  err_code = MsgLib::referMsgQueBlock(m_msgq_id, &que);
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
void SynthesizerObject::parse(MsgPacket *msg)
{
  uint event = MSG_GET_SUBTYPE(msg->getType());

  if (MSG_IS_REQUEST(msg->getType()) != 0)
    {
      F_ASSERT(event < AUD_PLY_MSG_NUM); /* eventの定義どこ？*/
      (this->*MsgProcTbl[event][m_state.get()])(msg);
    }
  else
    {
      F_ASSERT(event < AUD_PLY_RST_MSG_NUM); /* eventの定義どこ？*/
      (this->*MsgResultTbl[event][m_state.get()])(msg);
    }
}

/*--------------------------------------------------------------------------*/
SynthesizerObject::MsgProc SynthesizerObject::MsgProcTbl[_MSG_NUM][SynStateNum] =
{
  /* Message type: MSG_AUD_SYN_CMD_ACT. */

  {                                  /* Synthesizer status:    */
    &SynthesizerObject::activate,            /*   BootedState.        */
    &SynthesizerObject::illegalEvt,          /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */ /*これ必要？*/
  },

  /* Message type: MSG_AUD_SYN_CMD_INIT. */

  {                                  /* Synthesizer status:    */
    &SynthesizerObject::illegalEvt,          /*   BootedState.        */
    &SynthesizerObject::init,                /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type:  MSG_AUD_PLY_CMD_PLAY. */

  {                                  /* Synthesizer status:    */
    &SynthesizerObject::illegalEvt,          /*   BootedState.        */
    &SynthesizerObject::execOnReady,         /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_STOP. */

  {                                  /* Synthesizer status:    */
    &SynthesizerObject::illegalEvt,          /*   BootedState.        */
    &SynthesizerObject::illegalEvt,          /*   ReadyState.         */
    &SynthesizerObject::stopOnExec,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::stopOnWait           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_DEACT. */

  {                                  /* Synthesizer status:    */
    &SynthesizerObject::illegalEvt,          /*   BootedState.        */
    &SynthesizerObject::deactivate,          /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
  	&SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_SET */
  {                                  /* Synthesizer status:    */
    &SynthesizerObject::illegalEvt,          /*   BootedState.        */
    &SynthesizerObject::set,                 /*   ReadyState.         */
    &SynthesizerObject::set,                 /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   WaitStopState.      */
  }
};

/*--------------------------------------------------------------------------*/
SynthesizerObject::MsgProc SynthesizerObject::MsgResultTbl[_MSG_NUM][SynStateNum] =
{
  /* Message type: MSG_AUD_SYN_CMD_NEXT_REQ. */

  {
    &SynthesizerObject::illegalSinkDone,     /*   BootedState.        */
    &SynthesizerObject::illegalSinkDone,     /*   ReadyState.         */
    &SynthesizerObject::nextReqOnExec,       /*   ExecuteState.       */
    &SynthesizerObject::nextReqOnStopping,   /*   StoppingState.      */
    &SynthesizerObject::illegalSinkDone      /*   WaitStopState.      */

  },

  /* Message type: MSG_AUD_SYN_CMD_CMP_DONE. */

  {
    &SynthesizerObject::illegalCmpDone,      /*   BootedState.        */
    &SynthesizerObject::illegalCmpDone,      /*   ReadyState.         */
    &SynthesizerObject::cmpDoneOnexec,       /*   PlayState.          */
    &SynthesizerObject::cmpDoneOnStopping,   /*   StoppingState.      */
    &SynthesizerObject::illegalCmpDone       /*   WaitStopState.      */
  },
};


/*--------------------------------------------------------------------------*/
void SynthesizerObject::reply(AsSynthesizerEvent evtype, MsgType msg_type, uint32_t result)
{
  if (m_callback != NULL)
    {
      m_callback(evtype, result, 0);
    }
  else if (m_msgq_id.mng != MSG_QUE_NULL)
    {
      AudioObjReply cmplt((uint32_t)msg_type,
                           AS_OBJ_REPLY_TYPE_REQ,
                           AS_MODULE_ID_Synthesizer_OBJ,
                           result);
      err_t er = MsgLib::send<AudioObjReply>(m_msgq_id.mng,
                                             MsgPriNormal,
                                             MSG_TYPE_AUD_RES,
                                             m_msgq_id,
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

  reply(msgtype, (MsgType)msgtype, AS_ECODE_STATE_VIOLATION);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::activate(MsgPacket *msg)
{
  AsActivateSynthesizer act = msg->moveParam<SynthesizerCommand>().act_param;
  bool result;

/*  _DBG("ACT: indev %d, outdev %d\n",
                   act.param.input_device,
                   act.param.output_device);*/

  /* Set event callback */

	
	/* ocsillatorにactivateを発行?*/
	
	
  m_callback = act.cb;

  if (!checkAndSetMemPool())
    {
      reply(AsSynthesizerEventAct,
            msg->getType(),
            AS_ECODE_CHECK_MEMORY_POOL_ERROR);
      return;
    }
	
	
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::deactivate(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  DBG("DEACT:\n");

  if (AS_ECODE_OK != unloadCodec())
    {
      reply(AsEventDeact, msg->getType(), AS_ECODE_DSP_UNLOAD_ERROR);
      return;
    }

  reply(AsSynthesizerEventDeact, msg->getType(), AS_ECODE_OK);
  m_state = BootedState;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::init(MsgPacket *msg)
{
  AsInitSynthesizerParam param = msg->moveParam<SynthesizerCommand>().init_param;
  uint8_t result;
  uint32_t dsp_inf;

/*  _DBG("INIT: ch num %d, bit len %d, codec %d(%s), fs %d\n",
                   param.channel_number,
                   param.bit_length,
                   param.codec_type,
                   param.dsp_path,
                   param.sampling_rate);*/

/*パラメータチェック*/
	
	
  if (result == AS_ECODE_OK)
    {
      /* Update codec accordingt to audio data type. */

      /* NOTE : Codec loading process was Moved to here from play()
       *        for reducing memory usage. Exclude Gapless once.
       */

              result = loadCodec(next_codec, &param, &dsp_inf);
    }
  else
    {
      MEDIA_PLAYER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
    }

  reply(AsSynthesizerEventInit, msg->getType(), result);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::execOnReady(MsgPacket *msg)
{}
/*--------------------------------------------------------------------------*/
void SynthesizerObject::stopOnExec(MsgPacket *msg)
{}
/*--------------------------------------------------------------------------*/
void SynthesizerObject::set(MsgPacket *msg)
/*--------------------------------------------------------------------------*/
void SynthesizerObject::nextReqOnExec(MsgPacket *msg)
{}
/*--------------------------------------------------------------------------*/
void SynthesizerObject::nextReqOnStopping(MsgPacket *msg)
{}
/*--------------------------------------------------------------------------*/
void SynthesizerObject::cmpDoneOnExec(MsgPacket *msg)
{}
/*--------------------------------------------------------------------------*/
void SynthesizerObject::cmpDoneOnStopping(MsgPacket *msg)
{}

