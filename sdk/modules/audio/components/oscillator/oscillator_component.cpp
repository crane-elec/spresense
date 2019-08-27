/****************************************************************************
 * modules/audio/components/oscillator/oscillator_component.cpp
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#include <arch/chip/backuplog.h>
#include <sdk/debug.h>

#include "encoder_component.h"
#include "apus/cpuif_cmd.h"

#include "memutils/message/Message.h"
#include "memutils/message/MsgPacket.h"
#include "audio/audio_message_types.h"
#include "dsp_driver/include/dsp_drv.h"
#include "apus/dsp_audio_version.h"
#include "wien2_internal_packet.h"

#define DBG_MODULE DBG_MODULE_AS

__WIEN2_BEGIN_NAMESPACE


/*--------------------------------------------------------------------*/
/* callback function for DSP Driver */
/*--------------------------------------------------------------------*/
void osc_dsp_done_callback(void *p_response, void *p_instance)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;

  switch (p_param->process_mode)
    {
      case Apu::CommonMode:
          if (p_param->event_type == Apu::BootEvent)
            {
              err_t er = MsgLib::send<uint32_t>(((EncoderComponent*)p_instance)->get_apu_mid(),
                                                MsgPriNormal,
                                                MSG_ISR_APU0,
                                                0,
                                                p_param->data.value);
              F_ASSERT(er == ERR_OK);
            }
          else if (p_param->event_type == Apu::ErrorEvent)
            {
              ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_ASSETION_FAIL);
            }
          break;

      case Apu::EncMode:
          AS_encode_recv_apu(p_response);
          break;

      default:
          ENCODER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
          break;
    }
}

/*--------------------------------------------------------------------
    Class Methods
  --------------------------------------------------------------------*/
uint32_t OscillatorComponent::activate(const char *path,
                                       uint32_t *dsp_inf)
{
  char filepath[64];
  uint32_t osc_dsp_version;

  /* Load DSP binary */

  int ret = DD_Load(filepath, 
                    osc_dsp_done_callback, 
                    (void *)this, 
                    &m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_Load() failure. %d\n", ret);
      OSCILLATOR_ERR(AS_ATTENTION_SUB_CODE_DSP_LOAD_ERROR);
      return AS_ECODE_DSP_LOAD_ERROR;
    }

  /* wait for DSP boot up... */

  dsp_boot_check(m_apu_dtq, dsp_inf);

  /* DSP version check */

  if (osc_dsp_version != *dsp_inf)
    {
      logerr("DSP version unmatch. expect %08x / actual %08x",
              osc_dsp_version, *dsp_inf);

      OSCILLATOR_ERR(AS_ATTENTION_SUB_CODE_DSP_VERSION_ERROR);
    }

  OSCILLATOR_INF(AS_ATTENTION_SUB_CODE_DSP_LOAD_DONE);

  return AS_ECODE_OK;
}

/*--------------------------------------------------------------------*/
bool OscillatorComponent::deactivate(void)
{
  OSCILLATOR_DBG("DEACT:\n");

  int ret = DD_Unload(m_dsp_handler);

  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_UnLoad() failure. %d\n", ret);
      OSCILLATOR_ERR(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_ERROR);
      return false;
    }
  OSCILLATOR_INF(AS_ATTENTION_SUB_CODE_DSP_UNLOAD_DONE);

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t OscillatorComponent::init(const InitOscParam& param, uint32_t *dsp_inf)
{
//  OSCILLATOR_DBG("INIT: codec %d, infs %d, outfs %d, bit len %d, ch num %d, complexity %d, bit rate %d, cb %08x\n",
//                  param.codec_type, param.input_sampling_rate, param.output_sampling_rate, param.bit_width,
//                  param.channel_num, param.complexity, param.bit_rate, param.callback);

  m_callback = param.callback;

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return AS_ECODE_OSCILLATOR_LIB_INITIALIZE_ERROR;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::InitEvent;

  p_apu_cmd->init_osc_cmd.channel_no    =  param.channel_no;
  p_apu_cmd->init_osc_cmd.type          =  param.type;
  p_apu_cmd->init_osc_cmd.frequency     =  param.frequency;
  p_apu_cmd->init_osc_cmd.bit_width     =  param.bit_width;

  p_apu_cmd->init_osc_cmd.debug_dump_info.addr = NULL;
  p_apu_cmd->init_osc_cmd.debug_dump_info.size = 0;

  send_apu(p_apu_cmd);

  /* Wait init completion and receive reply information */

  Apu::InternalResult internal_result;
  uint32_t rst = dsp_init_check(m_apu_dtq, &internal_result);
  *dsp_inf = internal_result.value;

  return rst;
}

/*--------------------------------------------------------------------*/
bool OscillatorComponent::exec(const ExecOscParam& param)
{
  /* Data check */

  if (param.buffer.p_buffer == NULL) {
     OSCILLATOR_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
     return false;
  }

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::ExecEvent;

  p_apu_cmd->exec_enc_cmd.channel_no = param.channel_no;
  p_apu_cmd->exec_enc_cmd.buffer     = param.buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
uint32_t OscillatorComponent::set(const SetOscParam& param)
{

  /* You should check date */


  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());
  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::SetParamEvent;

  p_apu_cmd->exec_osc_cmd.channel_no = param.channel_no;
  p_apu_cmd->exec_osc_cmd.buffer     = param.buffer;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool OscillatorComponent::flush()
{
  /* Regardless of output buffer is not allocated, send Flush Request
   * to DSP. Because it is needed by DSP to finish process correctly.
   */

  /* Flush */

  Apu::Wien2ApuCmd* p_apu_cmd = static_cast<Apu::Wien2ApuCmd*>(getApuCmdBuf());

  if (p_apu_cmd == NULL)
    {
      return false;
    }

  memset(p_apu_cmd, 0x00, sizeof(Apu::Wien2ApuCmd));

  p_apu_cmd->header.process_mode = Apu::OscMode;
  p_apu_cmd->header.event_type   = Apu::FlushEvent;

  send_apu(p_apu_cmd);

  return true;
}

/*--------------------------------------------------------------------*/
bool OscillatorComponent::recv(void *p_response)
{
  DspDrvComPrm_t *p_param = (DspDrvComPrm_t *)p_response;
  Apu::Wien2ApuCmd *packet = static_cast<Apu::Wien2ApuCmd*>(p_param->data.pParam);

  if (Apu::ApuExecOK != packet->result.exec_result)
    {
      ENCODER_WARN(AS_ATTENTION_SUB_CODE_DSP_EXEC_ERROR);
    }

  if (Apu::InitEvent == packet->header.event_type)
    {
      /* Notify init completion to myself */

      Apu::InternalResult internal_result = packet->result.internal_result[0];
      dsp_init_complete(m_apu_dtq, packet->result.exec_result, &internal_result);

      return true;
    }

  return m_callback(p_param);
}

/*--------------------------------------------------------------------*/
void OscillatorComponent::send_apu(Apu::Wien2ApuCmd* p_cmd)
{
  DspDrvComPrm_t com_param;
  com_param.event_type = p_cmd->header.event_type;
  com_param.process_mode = p_cmd->header.process_mode;
  com_param.type = DSP_COM_DATA_TYPE_STRUCT_ADDRESS;
  com_param.data.pParam = reinterpret_cast<void*>(p_cmd);

  int ret = DD_SendCommand(m_dsp_handler, &com_param);
  if (ret != DSPDRV_NOERROR)
    {
      logerr("DD_SendCommand() failure. %d\n", ret);
      ENCODER_ERR(AS_ATTENTION_SUB_CODE_DSP_SEND_ERROR);
      return;
    }
}

__WIEN2_END_NAMESPACE

