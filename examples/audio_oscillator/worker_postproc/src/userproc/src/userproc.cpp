/****************************************************************************
 * audio_oscillator/worker_postproc/src/userproc/src/userproc.cpp
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

#include "userproc.h"

/*--------------------------------------------------------------------*/
/*                                                                    */
/*--------------------------------------------------------------------*/

/*--------------------------------------------------------------------*/
void UserProc::init(InitParam *param)
{
  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::exec(ExecParam *param)
{
  /* !!tentative!! simply copy from input to output */

  uint16_t  input_ch   = MAX_CHANNEL_NUMBER;
  uint16_t  bit_length = 2;
  int16_t  *input      = (int16_t *)param->exec_cmd.input.addr;
  int16_t  *output     = (int16_t *)param->exec_cmd.output.addr;
  int32_t   data[MAX_CHANNEL_NUMBER];
  int32_t   data_l;
  int32_t   data_r;
  int32_t   dev;

  /* Currently, input data bit_length = 16bit, 8ch fixed */

  param->exec_cmd.output.size = param->exec_cmd.input.size / 4;

  uint32_t sample_byte = bit_length * input_ch;

  for (uint32_t j = 0; j < param->exec_cmd.input.size / sample_byte; j++)
    {
      for (int i = 0; i < input_ch; i++)
        {
          data[i] = *input++;
        }

      data_l = 0;
      data_r = 0;

      for (int i = 0; i < input_ch; i += 2)
        {
          data_l += data[i];
          data_r += data[i + 1];

          /* When ch num is 1, Extend L1 -> L1L1 */

          if (input_ch < 2)
            {
              data_l = data_r;
            }
        }

      if (input_ch >= 4)
        {
          dev = input_ch / 2;
        }
      else
        {
          dev = 1;
        }

      *output++ = (data_l / dev);
      *output++ = (data_r / dev);
    }

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::flush(FlushParam *param)
{
  param->flush_cmd.output.size = 0;

  param->result.result_code = CustomprocCommand::ExecOk;
}

/*--------------------------------------------------------------------*/
void UserProc::set(SetParam *param)
{
  m_toggle = param->postswitch;

  param->result.result_code = CustomprocCommand::ExecOk;
}

