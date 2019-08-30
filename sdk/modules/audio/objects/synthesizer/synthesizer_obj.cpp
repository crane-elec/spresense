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
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t                   s_syn_pid;
static AsSynthesizerMsgQueId_t s_msgq_id;
static AsSynthesizerPoolId_t   s_pool_id;
static SynthesizerObject      *s_syn_obj = NULL;

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
bool CreateSynthesizer(AsSynthesizerMsgQueId_t msgq_id, AsSynthesizerPoolId_t pool_id, AudioAttentionCb attcb)
{
  /* Register attention callback */

//  MEDIA_PLAYER_REG_ATTCB(attcb);

  /* Parameter check */

  /* Create */

  s_msgq_id = msgq_id;
  s_pool_id = pool_id;
	
  FAR MsgQueBlock *que;
  err_t err_code = MsgLib::referMsgQueBlock(s_msgq_id.synthesizer, &que);
  F_ASSERT(err_code == ERR_OK);
  que->reset();

  s_syn_pid = task_create("SYN_OBJ",
                           150, 1024 * 2,
                           AS_SynthesizerObjectEntry,
                           NULL);
  if (s_syn_pid < 0)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::create(AsSynthesizerMsgQueId_t   msgq_id,
                               AsSynthesizerPoolId_t     pool_id)
{
  s_syn_obj = new SynthesizerObject(msgq_id, pool_id);

  if (s_syn_obj != NULL)
    {
      s_syn_obj->run();
    }
  else
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
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

  {                                          /* Synthesizer status:   */
    &SynthesizerObject::activate,            /*   InactiveState.      */
    &SynthesizerObject::illegalEvt,          /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   ErrorStoppingState. */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_INIT. */

  {                                          /* Synthesizer status:   */
    &SynthesizerObject::illegalEvt,          /*   InactiveState.      */
    &SynthesizerObject::init,                /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   ErrorStoppingState. */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type:  MSG_AUD_SYN_CMD_PLAY. */

  {                                          /* Synthesizer status:   */
    &SynthesizerObject::illegalEvt,          /*   InactiveState.      */
    &SynthesizerObject::execOnReady,         /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   ErrorStoppingState. */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_STOP. */

  {                                          /* Synthesizer status:   */
    &SynthesizerObject::illegalEvt,          /*   InactiveState.      */
    &SynthesizerObject::illegalEvt,          /*   ReadyState.         */
    &SynthesizerObject::stopOnExec,          /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   ErrorStoppingState. */
    &SynthesizerObject::stopOnWait           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_DEACT. */

  {                                          /* Synthesizer status:   */
    &SynthesizerObject::illegalEvt,          /*   InactiveState.      */
    &SynthesizerObject::deactivate,          /*   ReadyState.         */
    &SynthesizerObject::illegalEvt,          /*   ExecuteState.       */
  	&SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   ErrorStoppingState. */
    &SynthesizerObject::illegalEvt           /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_SET */
  {                                          /* Synthesizer status:   */
    &SynthesizerObject::illegalEvt,          /*   InactiveState.      */
    &SynthesizerObject::set,                 /*   ReadyState.         */
    &SynthesizerObject::set,                 /*   ExecuteState.       */
    &SynthesizerObject::illegalEvt,          /*   StoppingState.      */
    &SynthesizerObject::illegalEvt,          /*   ErrorStoppingState. */
    &SynthesizerObject::illegalEvt,          /*   WaitStopState.      */
  }
};

/*--------------------------------------------------------------------------*/
SynthesizerObject::MsgProc SynthesizerObject::MsgResultTbl[AUD_SYN_RST_MSG_NUM][SynthsizerStateNum] =
{
  /* Message type: MSG_AUD_SYN_CMD_NEXT_REQ. */

  {
    &SynthesizerObject::illegalSinkDone,     /*   InactiveState.      */
    &SynthesizerObject::illegalSinkDone,     /*   ReadyState.         */
    &SynthesizerObject::nextReqOnExec,       /*   ExecuteState.       */
    &SynthesizerObject::nextReqOnStopping,   /*   StoppingState.      */
    &SynthesizerObject::illegalSinkDone,     /*   ErrorStoppingState. */
    &SynthesizerObject::illegalSinkDone      /*   WaitStopState.      */

  },

  /* Message type: MSG_AUD_SYN_CMD_DONE. */

  {
    &SynthesizerObject::illegalCompDone,      /*   InactiveState.      */
    &SynthesizerObject::illegalCompDone,      /*   ReadyState.         */
    &SynthesizerObject::cmpDoneOnExec,       /*   ExecuteState.       */
    &SynthesizerObject::cmpDoneOnStopping,   /*   StoppingState.      */
    &SynthesizerObject::illegalSinkDone,     /*   ErrorStoppingState. */
    &SynthesizerObject::illegalCompDone      /*   WaitStopState.      */
  },

  /* Message type: MSG_AUD_SYN_CMD_SET_DONE. */

  {
    &SynthesizerObject::illegalCompDone,     /*   InactiveState.      */
    &SynthesizerObject::illegalCompDone,      /*   ReadyState.         */
    &SynthesizerObject::cmpDoneOnExec,       /*   ExecuteState.       */
    &SynthesizerObject::cmpDoneOnStopping,   /*   StoppingState.      */
    &SynthesizerObject::illegalSinkDone,     /*   ErrorStoppingState. */
    &SynthesizerObject::illegalCompDone      /*   WaitStopState.      */
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
      m_callback(evtype, result, 0);
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

  reply(AsSynthesizerEventStop, msg->getType(), AS_ECODE_OK);
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
  bool                  result = AS_ECODE_OK;

  /* Set event callback */

  m_callback = act.cb;

  /* Active */

  reply(AsSynthesizerEventAct, msg->getType(), result);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::deactivate(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();

  SYNTHESIZER_OBJ_DBG("DEACT:\n");

  reply(AsSynthesizerEventDeact, msg->getType(), AS_ECODE_OK);

  m_state = SynthsizerStateInactive;
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::init(MsgPacket *msg)
{
  AsInitSynthesizerParam param  = msg->moveParam<SynthesizerCommand>().init_param;
  uint8_t                result = AS_ECODE_OK;

  SYNTHESIZER_OBJ_DBG("INIT: ch num %d, bit len %d, codec %d(%s), fs %d\n",
                      param.channel_no,
                      param.bit_length,
                      param.type,
                      param.frequency);

  reply(AsSynthesizerEventInit, msg->getType(), result);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::illegalSinkDone(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();
  SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::illegalCompDone(MsgPacket *msg)
{
  msg->moveParam<SynthesizerCommand>();
  SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::execOnReady(MsgPacket *msg)
{}

/*--------------------------------------------------------------------------*/
void SynthesizerObject::set(MsgPacket *msg)
{}

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

  SynthesizerCommand cmd;

  cmd.act_param = *actparam;

  err_t er = MsgLib::send<SynthesizerCommand>(s_msgq_id.synthesizer,
                                              MsgPriNormal,
                                              MSG_AUD_SYN_CMD_ACT,
                                              NULL,
                                              cmd);
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

  SynthesizerCommand cmd;

  cmd.init_param = *initparam;

  err_t er = MsgLib::send<SynthesizerCommand>(s_msgq_id.synthesizer,
                                              MsgPriNormal,
                                              MSG_AUD_SYN_CMD_INIT,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StartMediaSynthesizer(void)
{
  /* Start */

  SynthesizerCommand cmd;

  err_t er = MsgLib::send<SynthesizerCommand>(s_msgq_id.synthesizer,
                                              MsgPriNormal,
                                              MSG_AUD_MRC_CMD_START,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_StopMediaSynthesizer(void)
{
  /* Stop */

  SynthesizerCommand cmd;

  err_t er = MsgLib::send<SynthesizerCommand>(s_msgq_id.synthesizer,
                                              MsgPriNormal,
                                              MSG_AUD_MRC_CMD_STOP,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeactivateMediaSynthesizer(void)
{
  SynthesizerCommand cmd;

  err_t er = MsgLib::send<SynthesizerCommand>(s_msgq_id.synthesizer,
                                              MsgPriNormal,
                                              MSG_AUD_SYN_CMD_DEACT,
                                              NULL,
                                              cmd);
  F_ASSERT(er == ERR_OK);

  return true;
}

/*--------------------------------------------------------------------------*/
bool AS_DeleteMediaSynthesizer(void)
{
  if (s_syn_obj == NULL)
    {
      SYNTHESIZER_OBJ_ERR(AS_ATTENTION_SUB_CODE_TASK_CREATE_ERROR);
      return false;
    }

  task_delete(s_syn_pid);
  delete s_syn_obj;
  s_syn_obj = NULL;

  /* Unregister attention callback */

  SYNTHESIZER_OBJ_UNREG_ATTCB();

  return true;
}
