/****************************************************************************
 * audio_oscillator/audio_oscillator_main.cxx
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

#include <sdk/config.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <asmp/mpshm.h>
#include <sys/stat.h>
#include <arch/board/board.h>

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/simple_fifo/CMN_SimpleFifo.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"
#include "audio/audio_high_level_api.h"
#include <audio/utilities/wav_containerformat.h>
#include <audio/utilities/frame_samples.h>
#include "include/msgq_id.h"
#include "include/mem_layout.h"
#include "include/memory_layout.h"
#include "include/msgq_pool.h"
#include "include/pool_layout.h"
#include "include/fixed_fence.h"
#ifdef EXAMPLES_AUDIO_OSCILLATOR_USEPPOSTPROC
#include "userproc_command.h"
#endif
#include <arch/chip/cxd56_audio.h>

/* Section number of memory layout to use */

#define AUDIO_SECTION   SECTION_NO0

using namespace MemMgrLite;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSP file path */

#define DSPBIN_PATH "/mnt/sd0/BIN"

/* Output time(sec). */

#define OSCILLATOR_REC_TIME 20

/* Default Volume. -20dB */

#define VOLUME  -200

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* For share memory. */

static mpshm_t s_shm;

static sem_t   s_sem;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void outputmixer0_done_callback(MsgQueId requester_dtq,
                                       MsgType reply_of,
                                       AsOutputMixDoneParam *done_param)
{
  sem_post(&s_sem);
}

static bool synthesizer_callback(AsSynthesizerEvent evtype,
                                 uint32_t           result,
                                 void              *instance)
{
  sem_post(&s_sem);

  return true;
}

static bool app_create_audio_sub_system(void)
{
  bool result = true;

  /* Create Oscillator. */

  AsCreateSynthesizerParam_t  syn;

  syn.msgq_id.synthesizer = MSGQ_AUD_OSCILLATOR;
  syn.msgq_id.mng         = MSGQ_AUD_APP;
  syn.msgq_id.dsp         = MSGQ_AUD_DSP;
  syn.msgq_id.mixer       = MSGQ_AUD_OUTPUT_MIX;
  syn.pool_id.input       = S0_NULL_POOL;
  syn.pool_id.output      = S0_REND_PCM_BUF_POOL;
  syn.pool_id.dsp         = S0_OSC_APU_CMD_POOL;

  result = AS_CreateMediaSynthesizer(&syn, NULL);
  if (!result)
    {
      printf("Error: AS_CreateMediaSynthesizer() failed. system memory insufficient!\n");
      return false;
    }

  /* Create mixer feature. */

  AsCreateOutputMixParams_t output_mix_act_param;
  output_mix_act_param.msgq_id.mixer = MSGQ_AUD_OUTPUT_MIX;
  output_mix_act_param.msgq_id.mng   = MSGQ_AUD_APP;
  output_mix_act_param.msgq_id.render_path0_filter_dsp = MSGQ_AUD_PFDSP0;
  output_mix_act_param.msgq_id.render_path1_filter_dsp = MSGQ_AUD_PFDSP1;
  output_mix_act_param.pool_id.render_path0_filter_pcm = S0_PF0_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path1_filter_pcm = S0_PF1_PCM_BUF_POOL;
  output_mix_act_param.pool_id.render_path0_filter_dsp = S0_PF0_APU_CMD_POOL;
  output_mix_act_param.pool_id.render_path1_filter_dsp = S0_PF1_APU_CMD_POOL;

  result = AS_CreateOutputMixer(&output_mix_act_param, NULL);
  if (!result)
    {
      printf("Error: AS_CreateOutputMixer() failed. system memory insufficient!\n");
      return false;
    }

  /* Create renderer feature. */

  AsCreateRendererParam_t renderer_create_param;
  renderer_create_param.msgq_id.dev0_req  = MSGQ_AUD_RND_PLY;
  renderer_create_param.msgq_id.dev0_sync = MSGQ_AUD_RND_PLY_SYNC;
  renderer_create_param.msgq_id.dev1_req  = 0xFF;
  renderer_create_param.msgq_id.dev1_sync = 0xFF;

  result = AS_CreateRenderer(&renderer_create_param);
  if (!result)
    {
      printf("Error: AS_CreateRenderer() failure. system memory insufficient!\n");
      return false;
    }

  return result;
}

static void app_deact_audio_sub_system(void)
{
  /* Delete OutputMixer. */

  AS_DeleteOutputMix();

  /* Delete Renderer. */

  AS_DeleteRenderer();

  /* Delete Oscillator. */

  AS_DeleteMediaSynthesizer();
}

static bool app_activate_baseband(void)
{
  CXD56_AUDIO_ECODE error_code;

  /* Power on audio device */

  error_code = cxd56_audio_poweron();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_poweron() error! [%d]\n", error_code);
      return false;
    }

  /* Activate OutputMixer */

  AsActivateOutputMixer mixer_act;

  mixer_act.output_device = HPOutputDevice;
  mixer_act.mixer_type    = MainOnly;
#ifdef EXAMPLES_AUDIO_OSCILLATOR_USEPPOSTPROC
  mixer_act.post_enable   = PostFilterEnable;
#else
  mixer_act.post_enable   = PostFilterDisable;
#endif
  mixer_act.cb            = outputmixer0_done_callback;

  AS_ActivateOutputMixer(OutputMixer0, &mixer_act);
  sem_wait(&s_sem);

  return true;
}

#ifdef EXAMPLES_AUDIO_OSCILLATOR_USEPPOSTPROC

static bool app_send_initpostproc_command(void)
{
  AsInitPostProc  init;
  InitParam       initpostcmd;

  init.addr = reinterpret_cast<uint8_t *>(&initpostcmd);
  init.size = sizeof(initpostcmd);

  AS_InitPostprocOutputMixer(OutputMixer0, &init);
  sem_wait(&s_sem);

  return true;
}

static bool app_send_setpostproc_command(void)
{
  AsSetPostProc set;
  static SetParam      setpostcmd;
  static bool   s_toggle = false;

  s_toggle = (s_toggle) ? false : true;

  setpostcmd.postswitch = s_toggle;

  set.addr = reinterpret_cast<uint8_t *>(&setpostcmd);
  set.size = sizeof(SetParam);

  AS_SetPostprocOutputMixer(OutputMixer0, &set);
  sem_wait(&s_sem);

  return true;
}

#endif

static bool app_deactivate_baseband(void)
{
  /* Deactivate OutputMixer */

  AsDeactivateOutputMixer mixer_deact;

  AS_DeactivateOutputMixer(OutputMixer0, &mixer_deact);
  sem_wait(&s_sem);

  CXD56_AUDIO_ECODE error_code;

  /* Power off audio device */

  error_code = cxd56_audio_poweroff();

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_poweroff() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

static bool app_set_oscillator_status(void)
{
  AsActivateSynthesizer act;

  act.cb = synthesizer_callback;

  AS_ActivateMediaSynthesizer(&act);
  sem_wait(&s_sem);

  return true;
}

static bool app_init_oscillator()
{
  AsInitSynthesizerParam  init;

  init.type          = AsSynthesizerSinWave;
  init.channel_num   = AS_CHANNEL_6CH;
  init.sampling_rate = AS_SAMPLINGRATE_48000;
  init.bit_width     = AS_BITLENGTH_16;

  sprintf(init.dsp_path, "%s/%s", DSPBIN_PATH, "OSCPROC");

  AS_InitMediaSynthesizer(&init);
  sem_wait(&s_sem);

  return true;
}

static bool app_start_oscillator(void)
{
  AS_StartMediaSynthesizer();
  sem_wait(&s_sem);

  return true;
}

static bool app_stop_oscillator(void)
{
  AS_StopMediaSynthesizer();
  sem_wait(&s_sem);

  return true;
}

static bool app_deactive_oscillator(void)
{
  AS_DeactivateMediaSynthesizer();
  sem_wait(&s_sem);

  return true;
}

static bool app_set_frequency_oscillator(uint8_t channel_number, uint32_t *frequency)
{
  AsSetSynthesizer set_param;
  bool             res = true;

  for (int i = 0; i < channel_number; i++)
    {
      set_param.channel_no = i;
      set_param.frequency  = frequency[i];

      AS_SetMediaSynthesizer(&set_param);
      sem_wait(&s_sem);
    }

  return res;
}

static bool app_set_clkmode(void)
{
  CXD56_AUDIO_ECODE error_code;

  error_code = cxd56_audio_set_clkmode(CXD56_AUDIO_CLKMODE_NORMAL);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_clkmode() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

static bool app_set_volume(int master_db)
{
  /* Set volume to audio driver */

  CXD56_AUDIO_ECODE error_code;

  error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_OUT, master_db);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_vol() error! [%d]\n", error_code);
      return false;
    }

  error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_IN1, 0);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_vol() error! [%d]\n", error_code);
      return false;
    }

  error_code = cxd56_audio_set_vol(CXD56_AUDIO_VOLID_MIXER_IN2, 0);

  if (error_code != CXD56_AUDIO_ECODE_OK)
    {
      printf("cxd56_audio_set_vol() error! [%d]\n", error_code);
      return false;
    }

  return true;
}

static bool app_init_libraries(void)
{
  int ret;
  uint32_t addr = AUD_SRAM_ADDR;

  /* Initialize shared memory.*/

  ret = mpshm_init(&s_shm, 1, AUD_SRAM_SIZE);
  if (ret < 0)
    {
      printf("Error: mpshm_init() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_remap(&s_shm, (void *)addr);
  if (ret < 0)
    {
      printf("Error: mpshm_remap() failure. %d\n", ret);
      return false;
    }

  /* Initalize MessageLib. */

  err_t err = MsgLib::initFirst(NUM_MSGQ_POOLS, MSGQ_TOP_DRM);
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = MsgLib::initPerCpu();
  if (err != ERR_OK)
    {
      printf("Error: MsgLib::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  void* mml_data_area = translatePoolAddrToVa(MEMMGR_DATA_AREA_ADDR);
  err = Manager::initFirst(mml_data_area, MEMMGR_DATA_AREA_SIZE);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initFirst() failure. 0x%x\n", err);
      return false;
    }

  err = Manager::initPerCpu(mml_data_area, static_pools, pool_num, layout_no);
  if (err != ERR_OK)
    {
      printf("Error: Manager::initPerCpu() failure. 0x%x\n", err);
      return false;
    }

  /* Create static memory pool of VoiceCall. */

  const uint8_t sec_no = AUDIO_SECTION;
  const NumLayout layout_no = MEM_LAYOUT_OSCILLATOR;
  void* work_va = translatePoolAddrToVa(S0_MEMMGR_WORK_AREA_ADDR);
  const PoolSectionAttr *ptr  = &MemoryPoolLayouts[AUDIO_SECTION][layout_no][0];
  err = Manager::createStaticPools(sec_no,
                                   layout_no,
                                   work_va,
                                   S0_MEMMGR_WORK_AREA_SIZE,
                                   ptr);
  if (err != ERR_OK)
    {
      printf("Error: Manager::createStaticPools() failure. %d\n", err);
      return false;
    }

  return true;
}

static bool app_finalize_libraries(void)
{
  /* Finalize MessageLib. */

  MsgLib::finalize();

  /* Destroy static pools. */

  MemMgrLite::Manager::destroyStaticPools(AUDIO_SECTION);

  /* Finalize memory manager. */

  MemMgrLite::Manager::finalize();

  /* Destroy shared memory. */

  int ret;
  ret = mpshm_detach(&s_shm);
  if (ret < 0)
    {
      printf("Error: mpshm_detach() failure. %d\n", ret);
      return false;
    }

  ret = mpshm_destroy(&s_shm);
  if (ret < 0)
    {
      printf("Error: mpshm_destroy() failure. %d\n", ret);
      return false;
    }

  return true;
}

bool app_play_process(void)
{
  bool  res = true;

  /* Define beep scale */

  struct {
    uint32_t  fs[6];
  }
  node[] = {
    {131,  523,    0,    0,  523,  262},
    {131,  523,    0,    0,    0,  294},
    {147,  587,    0,    0,  587,  330},
    {147,  587,    0,    0,    0,  349},
    {165,  659,    0, 1046,  659,  392},
    {165,  659,    0, 1174,    0,  440},
    {175,  697,    0, 1318,  697,  494},
    {175,  697,    0, 1397,    0,  523},
    {196,  784,  262, 1568,  784,    0},
    {196,  784,  294, 1760,    0,    0},
    {220,  880,  330, 1976,  880,    0},
    {220,  880,  349, 2093,    0,    0},
    {247,  988,  392,    0,  988,    0},
    {247,  988,  440,    0,    0,    0},
    {262, 1046,  494,    0, 1046,    0},
    {262, 1046,  523,    0,    0,    0},
    {  0,    0,    0,    0,    0,    0}   /* Terminate */
  },
  *p_node = node;

  /* Initial value setting */

  if (!(res = app_set_frequency_oscillator(6, p_node->fs)))
    {
      printf("Error: app_set_frequency_oscillator() failure.\n");
      goto errout_app_play_process;
    }

  /* Start oscillator operation. */

  if (!(res = app_start_oscillator()))
    {
      printf("Error: app_start_oscillator() failure.\n");
      goto errout_app_play_process;
    }

  /* Running... */

  printf("Running...\n");

  for (; p_node->fs[0]; p_node++)
    {
      /* Set frequency. */

      if (!(res = app_set_frequency_oscillator(6, p_node->fs)))
        {
          printf("Error: app_set_frequency_oscillator() failure.\n");
          break;
        }

      usleep(1000 * 1000);

#ifdef EXAMPLES_AUDIO_OSCILLATOR_USEPPOSTPROC
      app_send_setpostproc_command();
#endif
    }

  /* Stop oscillator operation. */

  if (!(res = app_stop_oscillator()))
    {
      printf("Error: app_stop_operation() failure.\n");
    }

errout_app_play_process:

  return res;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int audio_oscillator_main(int argc, char *argv[])
#endif
{
  printf("Start AudioOscillator example\n");

  sem_init(&s_sem, 0, 0);

  /* Waiting for SD card mounting. */

  sleep(2);

  /* First, initialize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_init_libraries())
    {
      printf("Error: init_libraries() failure.\n");
      return 1;
    }

  /* Next, Create the features used by AudioSubSystem. */

  if (!app_create_audio_sub_system())
    {
      printf("Error: act_audiosubsystem() failure.\n");
      return 1;
    }

  /* Change AudioSubsystem to Ready state so that I/O parameters can be changed. */

  if (!app_activate_baseband())
    {
      printf("Error: app_activate_baseband() failure.\n");
      return 1;
    }

  /* Set audio clock mode. */

  if (!app_set_clkmode())
    {
      printf("Error: app_set_clkmode() failure.\n");
      return 1;
    }

  /* Set oscillator operation mode. */

  if (!app_set_oscillator_status())
    {
      printf("Error: app_set_oscillator_status() failure.\n");
      return 1;
    }

#ifdef EXAMPLES_AUDIO_OSCILLATOR_USEPPOSTPROC

  /* Init Postproc. */

  app_send_initpostproc_command();

#endif

  /* Initialize Oscillator. */

  if (!app_init_oscillator())
    {
      printf("Error: app_init_oscillator() failure.\n");
      return 1;
    }

  /* Cancel output mute. */

  app_set_volume(VOLUME);

  /* Set output mute. */

  if (board_external_amp_mute_control(false) != OK)
    {
      printf("Error: board_external_amp_mute_control(false) failuer.\n");
      return 1;
    }

  /* Running... */

  if (!app_play_process())
    {
      printf("Error: app_play_process() failure.\n");
      return 1;
    }

  /* Set output mute. */

  if (board_external_amp_mute_control(true) != OK)
    {
      printf("Error: board_external_amp_mute_control(true) failuer.\n");
      return 1;
    }

  /* Unload oscillator operation. */

  if (!app_deactive_oscillator())
    {
      printf("Error: app_deactive_oscillator failuer.\n");
      return 1;
    }

  /* Deactivate baseband */

  if (!app_deactivate_baseband())
    {
      printf("Error: app_deactivate_baseband() failure.\n");
      return 1;
    }

  /* Deactivate the features used by AudioSubSystem. */

  app_deact_audio_sub_system();

  /* finalize the shared memory and memory utility used by AudioSubSystem. */

  if (!app_finalize_libraries())
    {
      printf("Error: finalize_libraries() failure.\n");
      return 1;
    }

  printf("Exit AudioOscillator example\n");

  sem_destroy(&s_sem);

  return 0;
}
