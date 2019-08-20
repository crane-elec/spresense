/****************************************************************************
 * mjpeg/mjpeg_main.c
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
#include <sys/stat.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <time.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/mkfatfs.h>
#include "video/video.h"

#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <sys/mount.h>

#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include <arch/chip/cisif.h>

#include <container/avi_containerformat.h>

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include "nximage.h"

#  ifdef CONFIG_IMAGEPROC
#    include <imageproc/imageproc.h>
#  endif

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Note: Buffer size must be multiple of 32. */

#define IMAGE_JPG_SIZE     (32*1024)  /* 100kB */

#define IMAGE_BUFNUM       (10)

#define AVI_FILENAME_LEN (32)

#ifdef CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD
#ifndef CONFIG_EXAMPLES_MJPEG_LCD_DEVNO
#  define CONFIG_EXAMPLES_MJPEG_LCD_DEVNO 0
#endif

#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))
#endif /* CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD */

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct uyvy_s
{
  uint8_t u0;
  uint8_t y0;
  uint8_t v0;
  uint8_t y1;
};

struct v_buffer {
  uint32_t             *start;
  uint32_t             length;
};
typedef struct v_buffer v_buffer_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *save_dir;

#ifdef CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD
struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  0              /* exit code */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_nximage.hnx = nx_open(dev);
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_nximage.hnx, &color);
  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_close(g_nximage.hnx);
      return ERROR;
    }

  while (!g_nximage.havepos)
    {
      (void) sem_wait(&g_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n",
         g_nximage.xres, g_nximage.yres);

  return 0;
}

#  ifndef CONFIG_IMAGEPROC
static inline void ycbcr2rgb(uint8_t y,  uint8_t cb, uint8_t cr,
                             uint8_t *r, uint8_t *g, uint8_t *b)
{
  int _r;
  int _g;
  int _b;
  _r = (128 * (y-16) +                  202 * (cr-128) + 64) / 128;
  _g = (128 * (y-16) -  24 * (cb-128) -  60 * (cr-128) + 64) / 128;
  _b = (128 * (y-16) + 238 * (cb-128)                  + 64) / 128;
  *r = itou8(_r);
  *g = itou8(_g);
  *b = itou8(_b);
}

static inline uint16_t ycbcrtorgb565(uint8_t y, uint8_t cb, uint8_t cr)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ycbcr2rgb(y, cb, cr, &r, &g, &b);
  r = (r >> 3) & 0x1f;
  g = (g >> 2) & 0x3f;
  b = (b >> 3) & 0x1f;
  return (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
}

/* Color conversion to show on display devices. */

static void yuv2rgb(void *buf, uint32_t size)
{
  struct uyvy_s *ptr;
  struct uyvy_s uyvy;
  uint16_t *dest;
  uint32_t i;

  ptr = buf;
  dest = buf;
  for (i = 0; i < size / 4; i++)
    {
      /* Save packed YCbCr elements due to it will be replaced with
       * converted color data.
       */

      uyvy = *ptr++;

      /* Convert color format to packed RGB565 */

      *dest++ = ycbcrtorgb565(uyvy.y0, uyvy.u0, uyvy.v0);
      *dest++ = ycbcrtorgb565(uyvy.y1, uyvy.u0, uyvy.v0);
    }
}
#  endif /* !CONFIG_IMAGEPROC */
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */


static void free_buffer(struct v_buffer  *buffers, uint8_t bufnum)
{
  uint8_t cnt;
  if (buffers)
    {
      for (cnt = 0; cnt < bufnum; cnt++)
        {
          if (buffers[cnt].start)
            {
              free(buffers[cnt].start);
            }
        }

      free(buffers);
    }
}

AVIHEADER avihead;
MOVIHEADER movihead;
AviContainerFormat *s_container_format = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
extern "C" int main(int argc, FAR char *argv[])
#else
extern "C" int mjpeg_main(int argc, char *argv[])
#endif
{
  int ret;
  int exitcode = -1;
  int v_fd;
  struct stat stat_buf;
  uint32_t buf_type = V4L2_BUF_TYPE_STILL_CAPTURE;
  uint32_t format   = V4L2_PIX_FMT_JPEG;
  struct v4l2_requestbuffers req = {0};
  struct v4l2_format fmt = {0};
  struct v4l2_buffer buf = {0};
  struct v_buffer  *buffers = NULL;
  uint32_t total_size = 0;
  FILE *fp;
  int cnt;

  /* select capture mode */

#ifdef CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD
  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
      return ERROR;
    }
#  ifdef CONFIG_IMAGEPROC
  imageproc_initialize();
#  endif
#endif /* CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD */

  /* In SD card is available, use SD card.
   * Otherwise, use SPI flash.
   */

  ret = stat("/mnt/sd0", &stat_buf);
  if (ret < 0)
    {
      save_dir = "/mnt/spif";
    }
  else
    {
      save_dir = "/mnt/sd0";
    }

  const char *filename = "/mnt/sd0/mjeg.avi";
  fp = fopen(filename, "wb");

  s_container_format = new AviContainerFormat;

  s_container_format->init(15, VIDEO_HSIZE_QVGA, VIDEO_VSIZE_QVGA);
  s_container_format->getHeader(&avihead, 0, 10); /* size is set later */

  fwrite(&avihead, 1, sizeof(avihead), fp);

  s_container_format->getMoviHeader(&movihead, 0 ); /* size is set later  */

  fwrite(&movihead, 1, sizeof(movihead), fp);

  ret = video_initialize("/dev/video");
  if (ret != 0)
    {
      printf("ERROR: Failed to initialize video: errno = %d\n", errno);
      goto errout_with_nx;
    }

  v_fd = open("/dev/video", 0);
  if (v_fd < 0)
    {
      printf("ERROR: Failed to open video.errno = %d\n", errno);
      goto errout_with_video_init;
    }

  /* Initialize user pointer I/O */

  req.type   = buf_type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = IMAGE_BUFNUM;
  req.mode   = V4L2_BUF_MODE_FIFO;

  ret = ioctl(v_fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
      goto errout_with_buffer;
    }

  /* Set image format */

  fmt.type                = buf_type;
  fmt.fmt.pix.width       = VIDEO_HSIZE_QVGA;
  fmt.fmt.pix.height      = VIDEO_VSIZE_QVGA;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = format;

  ret = ioctl(v_fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret < 0)
    {
      printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
      goto errout_with_buffer;
    }

  /* Allocate memory for image buffer */

  buffers = (v_buffer*)malloc(sizeof(v_buffer_t) * IMAGE_BUFNUM);

  for (cnt = 0; cnt < IMAGE_BUFNUM; cnt++)
    {
      buffers[cnt].start  = (uint32_t *)memalign(32, IMAGE_JPG_SIZE);
      buffers[cnt].length = IMAGE_JPG_SIZE;

      buf.type      = buf_type;
      buf.memory    = V4L2_MEMORY_USERPTR;
      buf.index     = cnt;
      buf.m.userptr = (unsigned long)buffers[cnt].start;
      buf.length    = buffers[cnt].length;

      ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          goto errout_with_buffer;
        }
    }

  ret = ioctl(v_fd, VIDIOC_TAKEPICT_START, 0);
  if (ret < 0)
    {
      printf("Failed to start taking picture\n");
      return ret;
    }

  for (cnt = 0; cnt < IMAGE_BUFNUM; cnt++)
    {
      /* Note: VIDIOC_DQBUF acquire capture data. */

      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = buf_type;
      buf.memory = V4L2_MEMORY_USERPTR;

      ret = ioctl(v_fd, VIDIOC_DQBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail DQBUF %d\n", errno);
          goto errout_with_buffer;
        }

      const char chunk[4] = {0x30, 0x30, 0x64, 0x63};
      fwrite(chunk, 1, 4, fp);
      fwrite((uint8_t *)buf.m.userptr, 1, (size_t)buf.bytesused, fp);
      total_size += (buf.bytesused + 4);

printf("JPEG size = %d\n", buf.bytesused);

#ifdef CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD
      if (format == V4L2_PIX_FMT_UYVY)
        {
          /* Convert YUV color format to RGB565 */

#  ifdef CONFIG_IMAGEPROC
          imageproc_convert_yuv2rgb((void *)buf.m.userptr,
                       VIDEO_HSIZE_QVGA,
                       VIDEO_VSIZE_QVGA);
#  else
          yuv2rgb((void *)buf.m.userptr, buf.bytesused);
#  endif
          nximage_image(g_nximage.hbkgd, (void *)buf.m.userptr);
        }
#endif /* CONFIG_EXAMPLES_MJPEG_OUTPUT_LCD */

      /* Note: VIDIOC_QBUF reset released buffer pointer. */

      ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          goto errout_with_buffer;
        }
    }

  ret = ioctl(v_fd, VIDIOC_TAKEPICT_STOP, false);
  if (ret < 0)
    {
      printf("Failed to start taking picture\n");
    }

  /* Return to top and reset data size */

  fseek(fp, SEEK_SET, 0);

  s_container_format->getHeader(&avihead, total_size, 10);

  fwrite(&avihead, 1, sizeof(avihead), fp);

  s_container_format->getMoviHeader(&movihead, total_size);

  fwrite(&movihead, 1, sizeof(movihead), fp);

  fclose(fp);
 
  exitcode = OK;

errout_with_buffer:
  close(v_fd);

  free_buffer(buffers, IMAGE_BUFNUM);

errout_with_video_init:

  video_uninitialize();

errout_with_nx:
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
#  ifdef CONFIG_IMAGEPROC
  imageproc_finalize();
#  endif
  nx_close(g_nximage.hnx);
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */

  return exitcode;
}
