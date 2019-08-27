/****************************************************************************
 * audio_recorder/worker/userproc/src/oscillator.cpp
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

#include "oscillator.h"

/*--------------------------------------------------------------------*/
/*                                                                    */
/*--------------------------------------------------------------------*/

Oscillator::CtrlProc Oscillator::CtrlFuncTbl[Wien2::Apu::ApuEventTypeNum][OscStateNum] =
{
                /* Booted */          /* Ready */           /* Active */
/* boot   */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::illegal },
/* Init   */  { &Oscillator::init,    &Oscillator::init,    &Oscillator::illegal },
/* Exec   */  { &Oscillator::illegal, &Oscillator::exec,    &Oscillator::exec    },
/* Flush  */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::flush   },
/* Set    */  { &Oscillator::illegal, &Oscillator::set,     &Oscillator::set     },
/* tuning */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::illegal },
/* error  */  { &Oscillator::illegal, &Oscillator::illegal, &Oscillator::illegal }
};

/*--------------------------------------------------------------------*/
void Oscillator::parse(Wien2::Apu::Wien2ApuCmd *cmd)
{
  if (cmd->header.process_mode != Wien2::Apu::OscMode)
    {
      cmd->result.exec_result = Wien2::Apu::ApuExecError;
      return;
    }
	
  (this->*CtrlFuncTbl[cmd->header.event_type][m_state])(cmd);
}

/*--------------------------------------------------------------------*/
void Oscillator::illegal(Wien2::Apu::Wien2ApuCmd *cmd)
{
  cmd->result.exec_result = Wien2::Apu::ApuExecError; /* データエラートの区別ができるようにコードを分ける
	*/
}

/*--------------------------------------------------------------------*/
void Oscillator::init(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Init signal process. */
  /* データチェック */


	for(int i = 0;i<cmd->channel_num;i++){
		m_frequency[i] = -1;
		m_theta[i] = 0;
		m_omega[i] = 0;
	}

	m_type = cmd->type;
	m_channel_num = cmd->channel_num;
	m_bit_length = cmd->bit_length;
	m_sampling_rate = cmd->sampling_rate;

  m_state = Ready;

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void Oscillator::exec(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Execute process to input audio data. */

	for(int i = 0;i<m_channel_num;i++){
		q15_t* ptr = (q15_t*)cmd->buffer->p_buffer + i;
 		for(int j = 0;j<cmd->buffer->size;j++){ /* サンプルに変える必要ある？*/
 			*ptr = arm_sin_q15 (m_theta[i]);
 			(m_theta[i] + m_omega[i]) < 1 ? m_theta[i] = (m_theta[i] + m_omega[i]) :  m_theta[i] = (m_theta[i] + m_omega[i] -1);
 		}
	}

  m_state = Active;

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void Oscillator::flush(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Flush process. */

  m_state = Ready;

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void Oscillator::set(Wien2::Apu::Wien2ApuCmd *cmd)
{
  /* Set process parameters. */
	m_omega[cmd->channel_no] = cmd->frequency * 0x7fffffff / m_sampling_rate;
	
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}


