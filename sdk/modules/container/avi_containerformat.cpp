/****************************************************************************
 * modules/container/wav_containerformat.cpp
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

#include <string.h>
#include <container/avi_containerformat.h>

/*--------------------------------------------------------------------------*/
bool AviContainerFormat::init(uint32_t framerate,uint32_t width, uint32_t height)
{
  m_framerate = framerate;
  m_width = width;
  m_height = height;

  return true;
}

/*--------------------------------------------------------------------------*/
bool AviContainerFormat::getHeader(AVIHEADER *header, uint32_t bytes, uint32_t frames)
{
  if (header == NULL)
    {
      return false;
    }

  header->riff       = CHUNKID_RIFF;
  header->total_size = bytes + sizeof(AVIHEADER) + sizeof(MOVIHEADER) - 8;
  header->avi        = FORMAT_AVI;

  header->list_hdrl  = SUBCHUNKID_LIST;
  header->hdrl_size  = sizeof(AVIMAINHEADER)+4;
  header->hdrl       = SUBCHUNKID_HDRL;
	
  header->main_header.fcc                   = SUBCHUNKID_AVIH;
  header->main_header.cb                    = sizeof(AVIMAINHEADER)-8;
  header->main_header.dwMicroSecPerFrame    = 1000*1000/m_framerate;
  header->main_header.dwMaxBytesPerSec      = 0x2000; /*“K“–*/
  header->main_header.dwPaddingGranularity  = 0;
  header->main_header.dwFlags               = 0; /* all off */
  header->main_header.dwTotalFrames         = frames;
  header->main_header.dwInitialFrames       = 0;
  header->main_header.dwStreams             = 1; /*Audio‚ð“ü‚ê‚éê‡‚Í2*/
  header->main_header.dwSuggestedBufferSize = 0x4000; /*“K“–*/
  header->main_header.dwWidth               = m_width;
  header->main_header.dwHeight              = m_height;
  memset(header->main_header.dwReserved, 0 , sizeof(header->main_header.dwReserved));


  header->list_strl  = SUBCHUNKID_LIST;
  header->strl_size  = sizeof(AVISTREAMHEADER)+4;
  header->strl       = SUBCHUNKID_STRL;

  header->stream_header.fcc                   = SUBCHUNKID_STRH;
  header->stream_header.cb                    = sizeof(AVISTREAMHEADER)-8;
  header->stream_header.fccType               = FCCTYPE_VIDEO;
  header->stream_header.fccHandler            = FCCHANDLER_MJPG;
  header->stream_header.dwFlags               = 0;
  header->stream_header.wPriority             = 0;
  header->stream_header.wLanguage             = 0;
  header->stream_header.dwInitialFrames       = 0;
  header->stream_header.dwScale               = 1;
  header->stream_header.dwRate                = m_framerate;
  header->stream_header.dwStart               = 0;
  header->stream_header.dwLength              = frames;
  header->stream_header.dwSuggestedBufferSize = 0x4000;
  header->stream_header.dwQuality             = -1;
  header->stream_header.dwSampleSize          = 0;
  header->stream_header.left =0;
  header->stream_header.top =0;
  header->stream_header.right                 = (uint16_t)m_width;
  header->stream_header.bottom                = (uint16_t)m_height;

  header->stream_format.fcc                   = SUBCHUNKID_STRF;
  header->stream_format.cb                    = sizeof(AVISTREAMFORMAT)-8;
  header->stream_format.biSize = sizeof(AVISTREAMFORMAT)-8;
  header->stream_format.biWidth = m_width;
  header->stream_format.biHeight = m_height;
  header->stream_format.biPlanes = 1;

  header->stream_format.biBitCount = 0;
  header->stream_format.biCompression = 4;  /* BI_JPEG */
  header->stream_format.biSizeImage = bytes;
  header->stream_format.biXPelsPerMeter = 0;
  header->stream_format.biYPelsPerMeter = 0;
  header->stream_format.biClrUsed = 3;
  header->stream_format.biClrImportant = 0;

  return true;
}
/*--------------------------------------------------------------------------*/
bool AviContainerFormat::getMoviHeader(MOVIHEADER *header, uint32_t bytes)
{
  header->list  = SUBCHUNKID_LIST;
  header->cb  = bytes+4;
  header->fcc   = SUBCHUNKID_MOVI;

  return true;
}
