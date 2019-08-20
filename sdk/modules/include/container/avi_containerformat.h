/****************************************************************************
 * modules/include/container/avi_containerformat.h
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

#ifndef MODULES_INCLUDE_CONTAINER_AVI_CONTAINERFORMAT_H
#define MODULES_INCLUDE_CONTAINER_AVI_CONTAINERFORMAT_H

#include <sys/types.h>
#include "avi_containerformat_common.h"

typedef struct _avimainheader {
    uint32_t  fcc;
    uint32_t  cb;
    uint32_t  dwMicroSecPerFrame;
    uint32_t  dwMaxBytesPerSec;
    uint32_t  dwPaddingGranularity;
    uint32_t  dwFlags;
    uint32_t  dwTotalFrames;
    uint32_t  dwInitialFrames;
    uint32_t  dwStreams;
    uint32_t  dwSuggestedBufferSize;
    uint32_t  dwWidth;
    uint32_t  dwHeight;
    uint32_t  dwReserved[4];
} AVIMAINHEADER;

typedef struct _avistreamheader {
     uint32_t  fcc;
     uint32_t  cb;
     uint32_t  fccType;
     uint32_t  fccHandler;
     uint32_t  dwFlags;
     uint16_t  wPriority;
     uint16_t  wLanguage;
     uint32_t  dwInitialFrames;
     uint32_t  dwScale;
     uint32_t  dwRate;
     uint32_t  dwStart;
     uint32_t  dwLength;
     uint32_t  dwSuggestedBufferSize;
     uint32_t  dwQuality;
     uint32_t  dwSampleSize;
     uint16_t  left;
     uint16_t  top;
     uint16_t  right;
     uint16_t  bottom;
} AVISTREAMHEADER;

typedef struct _avistreamformat {   /* In movie case, BITMAPINFOHEADER structure */ 
     uint32_t  fcc;
     uint32_t  cb;
    uint32_t  biSize;
    uint32_t   biWidth;  
    uint32_t   biHeight;
    uint16_t   biPlanes;            // ==1
    uint16_t   biBitCount;   
    uint32_t  biCompression;       // BI_JPEG = 4
    uint32_t  biSizeImage;         
    uint32_t   biXPelsPerMeter;   
    uint32_t   biYPelsPerMeter; 
    uint32_t  biClrUsed;       
    uint32_t  biClrImportant;
} AVISTREAMFORMAT;

typedef struct _moviheader {

     uint32_t list;
     uint32_t cb;
     uint32_t fcc;
	
} MOVIHEADER;


typedef struct _aviheader {
  /*! \brief character "RIFF" */

  uint32_t riff;

  /*! \brief Whole size exclude "RIFF" */

  uint32_t total_size;

  /*! \brief character "AVI " */

  uint32_t avi;

  uint32_t list_hdrl;
  uint32_t hdrl_size;
  uint32_t hdrl;
  AVIMAINHEADER   main_header;

  uint32_t list_strl;
  uint32_t strl_size;
  uint32_t strl;
  AVISTREAMHEADER stream_header;
  AVISTREAMFORMAT stream_format;
} AVIHEADER;


class AviContainerFormat
{
private:
  uint32_t m_framerate;
  uint32_t m_width;
  uint32_t m_height;

public:
  bool init(uint32_t framerate,uint32_t width, uint32_t height);
  bool getHeader(AVIHEADER *header, uint32_t bytes, uint32_t frames);
  bool getMoviHeader(MOVIHEADER *header, uint32_t bytes);
};

#endif /* MODULES_INCLUDE_CONTAINER_AVI_CONTAINERFORMAT_H */
