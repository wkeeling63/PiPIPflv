/*
 *  PiPIPflv.c 
 * This program creates a 2 camera components and tunnels them to the
 * Hardware Video Scaler (HVS) component to overlay the second camera 
 * frame on the frame of the main camera. The HVS output is tunneled to 
 * the encoder to create a video H.264 stream.  The video stream is 
 * created using MMAL api and run on the GPU.  The audio stream is 
 * created from ALSA api using Adafruit I2S MEMS Microphone 
 * (SPH0645LM4H).  The audio stream is encoded to ACC using FFPMEG aps 
 * and both streams are added to flash video container by FFMPEG api.
 *  
 */

#include <getopt.h>
#include <string.h>
#include <ctype.h>
#include <alsa/asoundlib.h>
#include <sys/time.h>
#include <sys/signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <sysexits.h>
#include <sys/socket.h>
#include <time.h>
#include <semaphore.h>
#include <stdbool.h>

#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include "libavformat/avio.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/frame.h"
#include "libavutil/opt.h"
#include "libswresample/swresample.h"
#include <libavutil/channel_layout.h>
#include <libavutil/mathematics.h>
#include <libavutil/timestamp.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include <bcm2835.h>

#include "raspiCamUtilities.h"
#include "mmalcomponent.h"
#include "GPSUtil.h"
#ifndef LLONG_MAX
#define LLONG_MAX    9223372036854775807LL
#endif

#define DEFAULT_FORMAT		SND_PCM_FORMAT_S32_LE
#define DEFAULT_SPEED 		44100
#define BUFFER_SIZE			262144
#define DEFAULT_CHANNELS_IN	2
#define GPIO_LED			RPI_BPLUS_GPIO_J8_13 
#define GPIO_SWT			RPI_BPLUS_GPIO_J8_15

char *command;
snd_pcm_t *handle;
struct {
	snd_pcm_format_t format;
	unsigned int channels;
	unsigned int rate;
} hwparams, rhwparams;
int timelimit = 0;
snd_pcm_stream_t stream = SND_PCM_STREAM_PLAYBACK;
u_char *audiobuf = NULL, *rlbufs = NULL;
snd_pcm_uframes_t chunk_size = 0;
snd_pcm_uframes_t buffer_frames = 0;
size_t bits_per_sample, bits_per_frame;
size_t chunk_bytes;
int badparm=0;
int height = 1080;
int width = 1920;
int ovl_height = 540;
int ovl_width = 960;
int loc_x = -1;
int loc_y = -1;
int m_flip_h = MMAL_FALSE;
int m_flip_v = MMAL_FALSE;
int o_flip_h = MMAL_TRUE;
int o_flip_v = MMAL_FALSE;
int camera = 1;
int intraframe = 30;
int initQ = 25, minQ = 20, maxQ = 40;
int abort_flg = 0;
int channel_num = 1;
int fps = 25;
int stop_flag=0;
int check_switch=0;
int gpio_init=0;
int atmaxQ=0;

int64_t start_time, write_target_time, write_variance = 0, pbrec_count = LLONG_MAX;

char file_name[128];
char pcm_name[17];

GPS_T gps_data;
pthread_t tid;

RASPIVID_STATE *pstate = NULL;

AVFormatContext *flv_frmtctx = NULL;
AVIOContext *io_ctx = NULL;
AVCodecContext *raw_codec_ctx = NULL, *aac_codec_ctx = NULL, *h264_codec_ctx = NULL;
AVCodec *aac_codec = NULL, *raw_codec = NULL, *h264_codec = NULL;
AVStream *aac_audio_strm = NULL, *h264_video_strm = NULL;
SwrContext *resample_ctx = NULL;
AVAudioFifo *fifo = NULL;
AVFrame *infrm, *outfrm;
int64_t audio_samples=0; 
char datestr[32];

static void prg_exit(int code);

void *gps_thread(void *argp)
{
   GPS_T *gps = (GPS_T *)argp;
   gps_data.active=1;
   gps_data.speed=-1; 
  
   open_gps(gps);
    
   while (gps->active) 
      {  
      read_gps(gps);
      }
 
   close_gps(gps);
}

static char* get_time_str(char* time_str)
{
	time_t time_uf;
	struct tm *time_fmt;
	time(&time_uf);
	time_fmt = localtime(&time_uf);
    strftime(time_str, 32,"%a %d %b %Y %I:%M:%S %p %Z", time_fmt);
	return time_str;
} 

static void usage(char *command)
{
	printf(
("Usage: %s [OPTION]... URL"
"\n"
"  URL in the form Protocol:loc (valid protocols are RTMP and FILE)\n"
"    for RTMP loc is URL string\n"
"    for FILE loc is path and filename string\n"
"\n"
"Program paramaters:\n"
"-?, --help              	help\n"
"-d, --duration=#        	interrupt after # seconds - defaults to forever (0) -1 is use GPIO switch\n"
"Audio paramaters:\n"
"-D, --device=NAME       	select PCM by name\n"  
"-n, --number=#          	number of audio channels\n"
"Camera paramaters:\n"
"-c, --camera=#          	main camera # - defaults to 1\n"
"-h, --height=#          	height #:# pixels - defaults to max 1080 if 2nd number is omitted 2nd camera is 1/2 height\n"
"-w, --width=#        	 	width #:# pixels - defaults to max 1920 if 2nd number is omitted 2nd camera is 1/2 height\n"
"-l, --location=#:#      	2nd camera PIP location h:v pixel camera # - defaults to lower left\n"
"-f, --flip=h.v:h.v      	flip image of camera - example 0.0:1.0 flips the main camera vertically & horizontally\n"
"                        	and 2nd camera vertically\n"
"-F, --fps=#				frames per second #\n"
"Encoder paramaters:\n"
"-q, --quantisation=#:#:#	quantisation Init:Min:Max parameters\n"
"-i, --intraframe=#      	intra key frame rate # frame\n"  )   
		, command);
} 
int init_avapi(char *filename)
{
	int status=0;
	AVDictionary *options = NULL;
	
//  setup format context and io context
	avformat_alloc_output_context2(&flv_frmtctx, NULL, "flv", NULL);
	if (!flv_frmtctx) 
		{
		fprintf(stderr, "Could not allocate output format context\n");
		return -1;
		}
	if (!(flv_frmtctx->url = av_strdup(filename))) 
		{
        fprintf(stderr, "Could not copy url.\n");
        return -1;
		}
		
// Setup  H264 codec
	h264_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
	if (!h264_codec)
		{
		fprintf(stderr, "H264 codec id not found!\n");
		return -1;
		}	

	h264_video_strm = avformat_new_stream(flv_frmtctx, NULL);
	if (!h264_video_strm) 
		{
		fprintf(stderr, "Could not allocate H264 stream\n");
		return -1;
		}
        
	h264_codec_ctx = avcodec_alloc_context3(h264_codec); 
	if (!h264_codec_ctx) 
		{
		fprintf(stderr, "Could not alloc an video encoding context\n");
		return -1;
		}	

   	h264_codec_ctx->codec_id = AV_CODEC_ID_H264;
	h264_codec_ctx->bit_rate = 0;
	h264_codec_ctx->qmin = 20;
	h264_codec_ctx->qmax = 40;
	h264_codec_ctx->width = h264_codec_ctx->coded_width = width;
	h264_codec_ctx->height = h264_codec_ctx->coded_height = height;
	h264_codec_ctx->sample_rate    = fps;
	h264_codec_ctx->gop_size      = intraframe;                  
	h264_codec_ctx->pix_fmt       = AV_PIX_FMT_YUV420P; 
	
	status = avcodec_parameters_from_context(h264_video_strm->codecpar, h264_codec_ctx);
	if (status < 0) 
		{
		fprintf(stderr, "Could not initialize stream parameters\n");
		return -1;
		}
    
	
	h264_video_strm->time_base.den = fps;   // Set the sample rate for the container
	h264_video_strm->time_base.num = 1;
	h264_video_strm->avg_frame_rate.num = fps;   // Set the sample rate for the container
	h264_video_strm->avg_frame_rate.den = 1;
	h264_video_strm->r_frame_rate.num = fps;   // Set the sample rate for the container
	h264_video_strm->r_frame_rate.den = 1;

	if (flv_frmtctx->oformat->flags & AVFMT_GLOBALHEADER) { // Some container formats (like MP4) require global headers to be present.
		h264_codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;}
		
	if ((status = av_dict_set(&options, "rtmp_live", "live", 0)) < 0) {
        fprintf(stderr, "rtmp live option: %s\n", av_err2str(status));}

    if ((status = avio_open2(&io_ctx, filename, AVIO_FLAG_WRITE, NULL, &options)))
		{
		fprintf(stderr, "Could not open output file '%s' (error '%s')\n", filename, av_err2str(status));
		return -1;
		}
        
	flv_frmtctx->pb = io_ctx;
		
//  setup AAC codec and stream context
	aac_codec = avcodec_find_encoder(AV_CODEC_ID_AAC);
	if (!aac_codec)
		{
		fprintf(stderr, "AAC codec id not found!\n");
		return -1;
		}	
	aac_audio_strm = avformat_new_stream(flv_frmtctx, NULL);
	if (!aac_audio_strm) 
		{
		fprintf(stderr, "Could not allocate AAC stream\n");
		return -1;
		}
        
	aac_codec_ctx = avcodec_alloc_context3(aac_codec); 
	if (!aac_codec_ctx) 
		{
		fprintf(stderr, "Could not alloc an encoding context\n");
		return -1;
		}
    
	aac_codec_ctx->channels       = channel_num;
	aac_codec_ctx->channel_layout = av_get_default_channel_layout(channel_num);
	aac_codec_ctx->sample_rate    = DEFAULT_SPEED;
	aac_codec_ctx->sample_fmt     = aac_codec->sample_fmts[0];
	aac_codec_ctx->bit_rate       = 64000;
	aac_codec_ctx->strict_std_compliance = FF_COMPLIANCE_EXPERIMENTAL;  // Allow the use of the experimental AAC encoder.

	aac_audio_strm->time_base.den = DEFAULT_SPEED;   // Set the sample rate for the container
	aac_audio_strm->time_base.num = 1;
    
	if (flv_frmtctx->oformat->flags & AVFMT_GLOBALHEADER)  // Some container formats (like MP4) require global headers to be present.
		aac_codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        
	if ((status = avcodec_open2(aac_codec_ctx, aac_codec, NULL) < 0)) 
		{
		fprintf(stderr, "Could not open output codec (error '%s')\n", av_err2str(status));
		return -1;
		}
	status = avcodec_parameters_from_context(aac_audio_strm->codecpar, aac_codec_ctx);
	if (status < 0) 
		{
		fprintf(stderr, "Could not initialize stream parameters\n");
		return -1;
		}

//  	av_dump_format(flv_frmtctx, 0, "stdout", 1);

//  setup RAW codec and context
	raw_codec = avcodec_find_encoder(AV_CODEC_ID_PCM_S32LE_PLANAR);
	if (!raw_codec)
		{
		fprintf(stderr, "PCM_S32_LE codec id not found!\n");
		return -1;
		}	
	raw_codec_ctx = avcodec_alloc_context3(raw_codec); 
	if (!aac_codec_ctx) 
		{
		fprintf(stderr, "Could not alloc RAW context\n");
		return -1;
		}
    
	raw_codec_ctx->channels       = DEFAULT_CHANNELS_IN;
	raw_codec_ctx->channel_layout = av_get_default_channel_layout(DEFAULT_CHANNELS_IN);
	raw_codec_ctx->sample_rate    = DEFAULT_SPEED;
	raw_codec_ctx->sample_fmt     = raw_codec->sample_fmts[0];  // AV_SAMPLE_FMT_S32
	raw_codec_ctx->bit_rate       = 2822400;  // or 64000
	raw_codec_ctx->time_base.num  = 1;
	raw_codec_ctx->time_base.den  = DEFAULT_SPEED;
	raw_codec_ctx->strict_std_compliance = FF_COMPLIANCE_EXPERIMENTAL;   // Allow the use of the experimental AAC encoder.
    
//  setup resampler context
	resample_ctx = swr_alloc_set_opts(NULL, av_get_default_channel_layout(aac_codec_ctx->channels), aac_codec_ctx->sample_fmt,
		aac_codec_ctx->sample_rate, av_get_default_channel_layout(raw_codec_ctx->channels), raw_codec_ctx->sample_fmt,
		raw_codec_ctx->sample_rate, 0, NULL);
	if (!resample_ctx) 
		{
		fprintf(stderr, "Could not allocate resample context\n");
		return -1;
		}
	if ((status = swr_init(resample_ctx)) < 0) 
		{
		fprintf(stderr, "Could not open resample context\n");
		swr_free(&resample_ctx);
		return -1;
		}

//  write flv header 
	flv_frmtctx->start_time_realtime=get_microseconds64();  // flv_frmtctx->start_time_realtime=0;  // 0 should user system clock
	status = avformat_init_output(flv_frmtctx, &options);  // null if AVDictionary is unneeded????
	if (status < 0)
		{
		fprintf(stderr, "Write ouput header failed! STATUS %d\n", status);
		return -1;
		} 

	status = avformat_write_header(flv_frmtctx, &options);  // null if AVDictionary is unneeded????
	if (status < 0)
		{
		fprintf(stderr, "Write ouput header failed! STATUS %d\n", status);
		return -1;
		}

// setup fifo sample queue
	if (!(fifo = av_audio_fifo_alloc(AV_SAMPLE_FMT_S32P, DEFAULT_CHANNELS_IN, 1))) 
		{
		fprintf(stderr, "Could not allocate FIFO\n");
		return -1;
		}
    
// allocate and init work frames
	infrm=av_frame_alloc();	
	if (!infrm) {fprintf(stderr, "unable to allocate in frame!\n");}

	infrm->channel_layout=raw_codec_ctx->channel_layout;
	infrm->sample_rate=raw_codec_ctx->sample_rate;
	infrm->format=raw_codec_ctx->sample_fmt;
	infrm->nb_samples=aac_codec_ctx->frame_size;  
    
	status=av_frame_get_buffer(infrm, 0);  
	if (status) {fprintf(stderr, "unable to allocate in frame data! %d %s\n", status, av_err2str(status));}
    
	outfrm=av_frame_alloc();	
	if (!outfrm) {fprintf(stderr, "unable to allocate out frame!\n");}
	outfrm->channel_layout=aac_codec_ctx->channel_layout;
	outfrm->sample_rate=aac_codec_ctx->sample_rate;
	outfrm->format=aac_codec_ctx->sample_fmt;
	outfrm->nb_samples=aac_codec_ctx->frame_size;

	status=av_frame_get_buffer(outfrm, 0);
	if (status) {fprintf(stderr, "unable to allocate out frame data!\n");}

	return 0; 
}

int close_avapi(void)
{
	int status;

	if (outfrm) {av_frame_free(&outfrm);}
	if (infrm) {av_frame_free(&infrm);}
	
	if (fifo) {av_audio_fifo_free(fifo);}

	if (resample_ctx) {swr_init(resample_ctx);}

	if (aac_codec_ctx)
		{
		AVPacket packet;
		av_init_packet(&packet); 
		packet.data = NULL;
		packet.size = 0;
		avcodec_send_frame(aac_codec_ctx, NULL); 
		avcodec_receive_packet(aac_codec_ctx, &packet);
		do 	{
			packet.pts = packet.dts = get_microseconds64()/1000-start_time;
			av_write_frame(flv_frmtctx, &packet);
			status = avcodec_receive_packet(aac_codec_ctx, &packet);
			} 
		while (!status);
		}

	if (raw_codec_ctx) {avcodec_free_context(&raw_codec_ctx);}
	if (h264_codec_ctx) {avcodec_free_context(&h264_codec_ctx);}
	if (aac_codec_ctx) {avcodec_free_context(&aac_codec_ctx);}
		
	if (flv_frmtctx)
		{
		if (io_ctx && io_ctx->seekable == 1)
			{
			status = av_write_trailer(flv_frmtctx);  
			if (status < 0) {fprintf(stderr, "Write ouput trailer failed! STATUS %d\n", status);}
			}  
		avformat_free_context(flv_frmtctx);
		}
	if (io_ctx)
		{	
		status = avio_close(io_ctx);	
		if (status < 0)
			{
			fprintf(stderr, "Could not close output file (error '%s')\n", av_err2str(status));
			return -1; 
			}
		} 
	return 0;
}
static void hvs_input_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
	if (buffer->user_data)
		{
		cairo_surface_destroy(buffer->user_data);
		}
	mmal_buffer_header_release(buffer);
}
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{

	MMAL_BUFFER_HEADER_T *new_buffer;
	static int64_t framecnt=0;
	static int64_t pts = -1;
	
	PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
	int *ap = pData->abort_ptr;
//	if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) fprintf(stderr, "config buffer\n");
//	fprintf(stderr, "buffer size %d %d\n", buffer->length, buffer->flags);
//	fprintf(stderr, "Callback %lld ",  get_microseconds64()/1000-start_time);
	if (*ap) 
		{
		fprintf(stderr, "Abort flag for callback\n");
		return;
		}

	if (pData)
		{
		int bytes_written = buffer->length;
		if (buffer->length)
			{
			mmal_buffer_header_mem_lock(buffer);
			if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO)
				{
				bytes_written = buffer->length;
				fprintf(stderr, "skipped due to flag %d \n", buffer->flags);
				}
			else
				{			
				AVPacket *packet=pData->vpckt;
				int status;
				if (buffer->pts != MMAL_TIME_UNKNOWN && buffer->pts != pData->pstate->lasttime)
					{
					if (pData->pstate->frame == 0)
						pData->pstate->starttime = buffer->pts;
					pData->pstate->lasttime = buffer->pts;
					pts = buffer->pts - pData->pstate->starttime;
					pData->pstate->frame++;
					}	
				if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) 
					{
					if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME)
						{
						packet->flags=AV_PKT_FLAG_KEY+AV_PKT_FLAG_TRUSTED;
						}
					else
						{
						packet->flags=AV_PKT_FLAG_TRUSTED;
						}
					if (pData->vbuf_ptr == 0)
						{
						packet->data=buffer->data;
						packet->size=buffer->length;
						} 
					else
						{
						memcpy(pData->vbuf+pData->vbuf_ptr, buffer->data+buffer->offset, buffer->length);
						pData->vbuf_ptr += buffer->length;
						packet->data=pData->vbuf;
						packet->size=pData->vbuf_ptr;
						pData->vbuf_ptr=0;
						}
					packet->dts = packet->pts = pts/1000;
					if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG)
						{
						if (packet->side_data) {
							av_packet_free_side_data(packet);}
						uint8_t *side_data = NULL;
						side_data = av_packet_new_side_data(packet, AV_PKT_DATA_NEW_EXTRADATA, buffer->length);
						if (!side_data) {
							fprintf(stderr, "%s\n", AVERROR(ENOMEM));
							prg_exit(EXIT_FAILURE);
							}
						memcpy(side_data, buffer->data+buffer->offset, buffer->length);
						}
					int64_t wstart = get_microseconds64();
					sem_wait(pData->mutex);
					status=av_write_frame(flv_frmtctx, packet);
					sem_post(pData->mutex);
					*pData->wvariance = *pData->wvariance + (get_microseconds64() - wstart)-*pData->wtargettime;
					if (status)
						{
						fprintf(stderr, "frame write error or flush %d\n", status);
						bytes_written = 0;
						}
					else 
						{
						++framecnt;
						bytes_written = buffer->length;
						}				
					}

				else
					{
					if (buffer->length >  BUFFER_SIZE - pData->vbuf_ptr) 
						{
						fprintf(stderr, "save vbuf to small\n");
						*ap = 1;
						}
					else
						{
						memcpy(pData->vbuf+pData->vbuf_ptr, buffer->data+buffer->offset, buffer->length);
						pData->vbuf_ptr+=buffer->length;
						bytes_written = buffer->length;	
						}
					}
				}

				mmal_buffer_header_mem_unlock(buffer);
				if (bytes_written != buffer->length)
					{
					vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
					*ap = 1;
					}
			}
		}
	else
		{
		vcos_log_error("Received a encoder buffer callback with no state");
		}

	mmal_buffer_header_release(buffer);
	if (port->is_enabled)
		{
		MMAL_STATUS_T status;
		new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);
		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);
		if (!new_buffer || status != MMAL_SUCCESS)
			vcos_log_error("Unable to return a buffer to the encoder port");
		}
}

int encode_and_write(u_char **data_in, int flush, sem_t *mutex)
{
	int status; 
	int min_samples=infrm->nb_samples; 
	AVPacket packet;
	int64_t save_pts=0, calc_pts;

	if (flush) {min_samples=1;}

	while (av_audio_fifo_size(fifo) >= min_samples)
	{
	outfrm->pts = outfrm->pkt_dts = save_pts = get_microseconds64()/1000-start_time;
    calc_pts = (double) audio_samples * 1000 / DEFAULT_SPEED;
	status = av_audio_fifo_read(fifo, (void **)infrm->data, infrm->nb_samples);
	if (status < 0) 
		{
		fprintf(stderr, "fifo read failed! %d %s\n", status, av_err2str(status));
		return -1;
		}
	else
		{
		audio_samples+=status;
		}
	
	status = swr_convert_frame(resample_ctx, outfrm, infrm);
	if (status) {fprintf(stderr, "Frame convert %d (error '%s')\n", status, av_err2str(status));}
	
	av_init_packet(&packet); // Set the packet data and size so that it is recognized as being empty. 
	packet.data = NULL;
	packet.size = 0;


	status = avcodec_send_frame(aac_codec_ctx, outfrm);  
	if (status == AVERROR_EOF) // The encoder signals that it has nothing more to encode.
		{
		status = 0;
		fprintf(stderr, "EOF at send frame\n");
		goto cleanup;
		}
	 else 
		if (status < 0)
			{
			fprintf(stderr, "Could not send packet for encoding (error '%s')\n", av_err2str(status));
			return status;
			}
	status = avcodec_receive_packet(aac_codec_ctx, &packet);

	if (status == AVERROR(EAGAIN)) // If the encoder asks for more data to be able to provide an encoded frame, return indicating that no data is present.
		{
		status = 0;
		} 
	else 
		if (status == AVERROR_EOF) // If the last frame has been encoded, stop encoding.
			{
			status = 0;
			fprintf(stderr, "EOF at receive packet\n");
			goto cleanup;
			} 
		else 
			if (status < 0) 
				{
				fprintf(stderr, "Could not encode frame (error '%s')\n", av_err2str(status));  //get this if not loaded frame
				goto cleanup;
    			} 
			else 
				{
				packet.duration=0;
				packet.pos=-1;
				packet.dts=packet.pts=save_pts-250;
				packet.stream_index = 1;
				sem_wait(mutex);
				status = av_write_frame(flv_frmtctx, &packet);
				sem_post(mutex); 
				if (status < 0) 
					{
					fprintf(stderr, "Could not write frame (error '%s')\n", av_err2str(status));
					goto cleanup;
					}

				}
	}
	return status;


cleanup:
	av_packet_unref(&packet);
	return status;
}

static void set_params(void)
{
	snd_pcm_hw_params_t *params;
	snd_pcm_uframes_t buffer_size;
	int err;

	snd_pcm_hw_params_alloca(&params);
	err = snd_pcm_hw_params_any(handle, params);
	if (err < 0) 
		{
		fprintf(stderr, "Broken configuration for this PCM: no configurations available\n");
		prg_exit(EXIT_FAILURE);
		}
	err = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);

	if (err < 0) 
		{
		fprintf(stderr, "Access type not available\n");
		prg_exit(EXIT_FAILURE);
		}
	err = snd_pcm_hw_params_set_format(handle, params, hwparams.format);
	if (err < 0) 
		{
		fprintf(stderr, "Sample format non available\n");
		prg_exit(EXIT_FAILURE);
		}
	err = snd_pcm_hw_params_set_channels(handle, params, hwparams.channels);
	if (err < 0) 
		{
		fprintf(stderr, "Channels count non available\n");
		prg_exit(EXIT_FAILURE);
		}


	err = snd_pcm_hw_params_set_rate_near(handle, params, &hwparams.rate, 0);
	assert(err >= 0);

	err = snd_pcm_hw_params(handle, params);
	if (err < 0) 
		{
		fprintf(stderr, "Unable to install hw params\n");
		prg_exit(EXIT_FAILURE);
		}
		
	snd_pcm_hw_params_get_period_size(params, &chunk_size, 0);
	snd_pcm_hw_params_get_buffer_size(params, &buffer_size);
	if (chunk_size == buffer_size) 
		{
		fprintf(stderr, "Can't use period equal to buffer size (%lu == %lu)\n", 
		      chunk_size, buffer_size);
		prg_exit(EXIT_FAILURE);
		}

	bits_per_sample = snd_pcm_format_physical_width(hwparams.format);
	bits_per_frame = bits_per_sample * hwparams.channels;
	chunk_bytes = chunk_size * bits_per_frame / 8;

	audiobuf = (u_char *)malloc(chunk_bytes);
	if (audiobuf == NULL) 
		{
		fprintf(stderr, "not enough memory\n");
		prg_exit(EXIT_FAILURE);
		}
	rlbufs = (u_char *)malloc(chunk_bytes);
	if (rlbufs == NULL) 
		{
		fprintf(stderr, "not enough memory\n");
		prg_exit(EXIT_FAILURE);
		}

	buffer_frames = buffer_size;	/* for position test */
}

/* I/O error handler */
static void xrun(void)
{
	snd_pcm_status_t *status;
	int res;
	snd_pcm_status_alloca(&status);
	if ((res = snd_pcm_status(handle, status))<0) 
		{
		fprintf(stderr, "status error: %s\n", snd_strerror(res));
		prg_exit(EXIT_FAILURE);
		}
	
	if (snd_pcm_status_get_state(status) == SND_PCM_STATE_XRUN) 
		{
		if ((res = snd_pcm_prepare(handle))<0) 
			{
			fprintf(stderr, "xrun: prepare error: %s\n", snd_strerror(res));
			prg_exit(EXIT_FAILURE);
			}
		return;		/* ok, data should be accepted again */
		} 
		if (snd_pcm_status_get_state(status) == SND_PCM_STATE_DRAINING) 
			{
			if (stream == SND_PCM_STREAM_CAPTURE) 
				{
				fprintf(stderr, "capture stream format change? attempting recover...\n");
				if ((res = snd_pcm_prepare(handle))<0) 
					{
					fprintf(stderr, "xrun(DRAINING): prepare error: %s\n", snd_strerror(res));
					prg_exit(EXIT_FAILURE);
					}
				return;
				}
			}
		fprintf(stderr, "read/write error, state = %s\n", snd_pcm_state_name(snd_pcm_status_get_state(status)));
		prg_exit(EXIT_FAILURE);
}
/* I/O suspend handler */
static void suspend(void)
{
	int res;
	while ((res = snd_pcm_resume(handle)) == -EAGAIN)
		sleep(1);	/* wait until suspend flag is released */
	if (res < 0) {
		if ((res = snd_pcm_prepare(handle)) < 0) {
			fprintf(stderr, "suspend: prepare error: %s\n", snd_strerror(res));
			prg_exit(EXIT_FAILURE);
		}
	}
}
/*
 *  read function
 */
static ssize_t pcm_read(u_char *data, u_char **data_out,size_t rcount)
{
	ssize_t r, size;
	size_t result = 0;
	size_t count = rcount;
	u_char *data_in=data;
	if (count != chunk_size) 
		{
		count = chunk_size;
		}
	

	while (count > 0) 
		{
		r = snd_pcm_readi(handle, data, count);
		if (r == -EAGAIN || (r >= 0 && (size_t)r < count)) 
			{
			fprintf(stderr, "wait\n");
			snd_pcm_wait(handle, 100);
			}
		else if (r == -EPIPE) 
			{
			xrun();
			} 
		else if (r == -ESTRPIPE)
			{
			suspend();
			} 
		else if (r < 0) 
			{
			fprintf(stderr, "read error: %s\n", snd_strerror(r));
			prg_exit(EXIT_FAILURE);	
			}
			if (r > 0) 
				{
				result += r;
				count -= r;
				data += r * bits_per_frame / 8;
				}
		}
	size = r * bits_per_frame / 8;


	size_t i;   
	int s, x, lr=0;
	u_char *lptr=data_out[0], *rptr=data_out[1];
	x=chunk_bytes/4;
	for (i=0; i < x; ++i) {
		for (s=0;s < 4; ++s) {
			if (lr) {
				*rptr = *data_in;
				++rptr;}
			else {
				*lptr = *data_in;
				++lptr;}
			++data_in;}
			if (lr) {lr=0;}
			else {lr=1;}}

	int status;		
	status=av_audio_fifo_write(fifo, (void **)data_out, r);
	if (status < 0)
		{
		fprintf(stderr, "fifo write failed!\n");
		}
	else
		if (status != r) 
			{
			fprintf(stderr, "fifo did not write all! to write %d written %d\n", r, status);
			}
	return rcount;
}

/* calculate the data count to read from/to dsp */
static int64_t calc_count(void)
{
	int64_t count;

	if (timelimit == 0) 
		{
		count = pbrec_count;
		} 
	else 
		{
		count = snd_pcm_format_size(hwparams.format, hwparams.rate * hwparams.channels);
		count *= (int64_t)timelimit;
		}
	return count < pbrec_count ? count : pbrec_count;
}

MMAL_STATUS_T setup_components(RASPIVID_STATE *state)
{
   
	MMAL_STATUS_T status = MMAL_SUCCESS;
	MMAL_PORT_T *camera_video_port = NULL;
	MMAL_PORT_T *camera2_video_port = NULL;
	MMAL_PORT_T *encoder_input_port = NULL;
	MMAL_PORT_T *encoder_output_port = NULL;
	MMAL_PORT_T *hvs_main_input_port = NULL;
	MMAL_PORT_T *hvs_ovl_input_port = NULL;
	MMAL_PORT_T *hvs_text_input_port = NULL;
	MMAL_PORT_T *hvs_output_port = NULL;
   
    // Setup for sensor specific parameters, only set W/H settings if zero on entry
	int cam = state->common_settings.cameraNum, cam2 = 0, max_width = 0, max_height = 0;
	get_sensor_defaults(state->common_settings.cameraNum, state->common_settings.camera_name,
                       &max_width, &max_height);
                       
    if (width > max_width || ovl_width > max_width || height > max_height || ovl_height > max_height) {
		fprintf(stdout, "Resolution larger than sensor %dX%d\n", max_width, max_height);
		return -128;}
		
    state->camera_parameters.stereo_mode.mode = MMAL_STEREOSCOPIC_MODE_NONE;

	if (!cam) {cam2 = 1;}

	if ((status = create_camera_component(state)) != MMAL_SUCCESS)
		{
		vcos_log_error("%s: Failed to create camera component", __func__);
		return -128;
		}
		
	state->common_settings.width = ovl_width;
	state->common_settings.height = ovl_height;
	state->common_settings.cameraNum = cam2;
	state->camera_parameters.vflip = o_flip_v;
	state->camera_parameters.hflip = o_flip_h;
   
	if ((status = create_camera_component(state)) != MMAL_SUCCESS)
		{
		vcos_log_error("%s: Failed to create camera component", __func__);
		return -128;
		}

	state->common_settings.width = width;
	state->common_settings.height = height;
	state->common_settings.cameraNum = cam;
	state->camera_parameters.vflip = m_flip_v;
	state->camera_parameters.hflip = m_flip_h;

    if ((status = create_hvs_component(state)) != MMAL_SUCCESS)
		{
		vcos_log_error("%s: Failed to create hvs component", __func__);
		destroy_camera_component(state);
		return -128;
		} 

	if ((status = create_encoder_component(state)) != MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to create encode component", __func__);
		destroy_camera_component(state);
		destroy_hvs_component(state);
		return -128; 
	}	
  
    camera_video_port   = state->camera_component->output[MMAL_CAMERA_VIDEO_PORT];
    camera2_video_port   = state->camera2_component->output[MMAL_CAMERA_VIDEO_PORT];

	hvs_main_input_port = state->hvs_component->input[0];
    hvs_ovl_input_port  = state->hvs_component->input[1];
    hvs_text_input_port  = state->hvs_component->input[2];
    hvs_output_port     = state->hvs_component->output[0];
    encoder_input_port  = state->encoder_component->input[0];
    encoder_output_port = state->encoder_component->output[0];
    
    if ((status = connect_ports(camera_video_port, hvs_main_input_port, &state->hvs_main_connection)) != MMAL_SUCCESS)
    {
		vcos_log_error("%s: Failed to connect camera video port to hvs input", __func__); 
		state->hvs_main_connection = NULL;
		return -128;
	}
	
	if ((status = connect_ports(camera2_video_port, hvs_ovl_input_port, &state->hvs_ovl_connection)) != MMAL_SUCCESS)
    {
		vcos_log_error("%s: Failed to connect camera2 video port to hvs input", __func__); 
		state->hvs_ovl_connection = NULL;
		return -128;
	} 
	
	if ((status = connect_ports(hvs_output_port, encoder_input_port, &state->encoder_connection)) != MMAL_SUCCESS)
    {
		vcos_log_error("%s: Failed to connect hvs to encoder input", __func__); 
		state->encoder_connection = NULL;
		return -128;
	} 
	
	hvs_text_input_port->buffer_num = hvs_text_input_port->buffer_num_min+1;
	hvs_text_input_port->buffer_size = hvs_text_input_port->buffer_size_min;
//	pool_in = mmal_pool_create(hvs_text_input_port->buffer_num, hvs_text_input_port->buffer_size);
	state->hvs_textin_pool = mmal_pool_create(hvs_text_input_port->buffer_num, hvs_text_input_port->buffer_size);

	if ((status = mmal_port_enable(hvs_text_input_port, hvs_input_callback)) != MMAL_SUCCESS)
	{
		vcos_log_error("%s: Failed to enable hvs text input", __func__); 
		return -128;
	} 	
	return 0;
}

void close_components(RASPIVID_STATE *state)
{
    // Disable all our ports that are not handled by connections
    if (state->encoder_component)
		check_disable_port(state->encoder_component->output[0]);
    
    if (state->camera_component) {
		check_disable_port(state->camera_component->output[MMAL_CAMERA_PREVIEW_PORT]);  
		check_disable_port(state->camera_component->output[MMAL_CAMERA_CAPTURE_PORT]);}
	if (state->camera2_component) {
		check_disable_port(state->camera2_component->output[MMAL_CAMERA_PREVIEW_PORT]); 
		check_disable_port(state->camera2_component->output[MMAL_CAMERA_CAPTURE_PORT]);}  
   
	if (state->encoder_connection)
		mmal_connection_destroy(state->encoder_connection);
	if (state->hvs_main_connection)
		mmal_connection_destroy(state->hvs_main_connection);
	if (state->hvs_ovl_connection)
		mmal_connection_destroy(state->hvs_ovl_connection);
	if (state->hvs_component->input[2]->is_enabled)
		mmal_port_disable(state->hvs_component->input[2]);

    // Disable and destroy components 
	if (state->encoder_component)
		mmal_component_disable(state->encoder_component);
	if (state->encoder_component)
		mmal_component_disable(state->hvs_component);
	if (state->camera_component)
		mmal_component_disable(state->camera_component);
	if (state->camera2_component)
		mmal_component_disable(state->camera2_component);

	destroy_encoder_component(state);
	destroy_hvs_component(state);
	destroy_camera_component(state);
	
	pstate=NULL;
      
	return;
}
/*
 *	Subroutine to clean up before exit.
 */
static void close_it(void)  
{
//	fprintf(stdout, "\n");
	// stop gps thread
	gps_data.active=0;
	pthread_join(tid, NULL); 
	
	sem_t *psem=pstate->callback_data.mutex;	
	close_components(pstate);
	close_avapi();
	sem_destroy(psem);
	if (gpio_init) {
		bcm2835_gpio_write(GPIO_LED, LOW);
		bcm2835_close();}
	if (handle) {
		snd_pcm_close(handle);
		handle = NULL;}
	free(audiobuf);
	free(rlbufs);
	snd_config_update_free_global();
	fprintf(stdout, "%s PiPIPflv ending\n", get_time_str(datestr));
}
static void prg_exit(int code)  
{
	close_it();
	exit(code);
}
static void signal_handler(int sig)
{
	static int in_aborting;

	if (in_aborting)
		return;

	in_aborting = 1;

	if (stream == SND_PCM_STREAM_CAPTURE) 
		{
		stream = -1;
		}
	prg_exit(EXIT_FAILURE);
}

static void capture(char *orig_name)
{
	u_char *bufs[2];
	int i;
	size_t vsize;
	RASPIVID_STATE state;
	pstate = &state;
	
	// start gps thread
	pthread_create(&tid, NULL, gps_thread, (void *)&gps_data);

	AVPacket video_packet;
	av_init_packet(&video_packet);
	video_packet.stream_index=0;
	video_packet.duration=0;
	video_packet.pos=-1;

	char *name = orig_name;	/* current filename */
	int64_t count;		/* number of bytes to capture */
	int status=0;

	/* get number of bytes to capture */
	count = calc_count();
	if (count == 0)
		count = LLONG_MAX;

	/* setup sound hardware */
	set_params();
	
	// init ffmpeg  
	status=init_avapi(name);
	if (status) {prg_exit(EXIT_FAILURE);}
	
	// init GPIO library bcm2835
	if (bcm2835_init()) {gpio_init = 1;}
	else {fprintf (stderr, "bcm2835 init failed\n");}
    bcm2835_gpio_fsel(GPIO_SWT, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(GPIO_LED, BCM2835_GPIO_FSEL_OUTP);

	vcos_log_register("PiPIPflv", VCOS_LOG_CATEGORY);
	
    start_time = get_microseconds64()/1000; 
    
	default_status(&state);

	state.common_settings.width = width;
	state.common_settings.height = height;
	state.common_settings.ovl_width = ovl_width;
	state.common_settings.ovl_height = ovl_height;
	state.common_settings.ovl_x = loc_x;
	state.common_settings.ovl_y = loc_y;
	state.intraperiod = intraframe;
	state.framerate = fps;
	state.common_settings.cameraNum = camera;
	state.camera_parameters.vflip = m_flip_v;
	state.camera_parameters.hflip = m_flip_h;
	state.quantisationParameter = initQ;
	state.quantisationMin = minQ;
	state.quantisationMax = maxQ;
	
	// init MMAL components 
	status = setup_components(&state);
	
	state.callback_data.pstate = &state;
	state.callback_data.abort_ptr = &abort_flg;
	state.callback_data.stime = &start_time;
	state.callback_data.wtargettime = &write_target_time;
	state.callback_data.wvariance = &write_variance;
	state.callback_data.fctx = flv_frmtctx;
	state.callback_data.vpckt=&video_packet;
	state.callback_data.vbuf_ptr=0;
	
	//calc write target
	write_target_time = 250000/fps;

	state.callback_data.vbuf = (u_char *)malloc(BUFFER_SIZE);
	if (state.callback_data.vbuf == NULL) 
		{
		fprintf(stderr, "not enough memory vbuf\n");
		prg_exit(EXIT_FAILURE);
		}
	sem_t def_mutex;
	sem_init(&def_mutex, 0, 1);
	state.callback_data.mutex=&def_mutex;

    // Set up our userdata - this is passed though to the callback where we need the information.
	state.encoder_component->output[0]->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

    // Enable the encoder output port and tell it its callback function
	status = mmal_port_enable(state.encoder_component->output[0], encoder_buffer_callback);
	if (status) 
		{
		fprintf(stderr, "enable port failed\n");
		}
	
	// Send all the buffers to the encoder output port
	int num = mmal_queue_length(state.encoder_pool->queue);    
	int q;
	for (q=0; q<num; q++)
		{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);
		if (!buffer)
			vcos_log_error("Unable to get a required buffer %d from pool queue", q);
		if (mmal_port_send_buffer(state.encoder_component->output[0], buffer)!= MMAL_SUCCESS)
			vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
		}
	
	vsize = chunk_bytes / 2;
	for (i = 0; i < 2; ++i)
		bufs[i] = rlbufs + vsize * i;

	if (count > 2147483648LL)
		count = 2147483648LL;

	// capture 
	snd_pcm_drop(handle);
	snd_pcm_prepare(handle);

	mmal_port_parameter_set_boolean(state.camera_component->output[MMAL_CAMERA_VIDEO_PORT], MMAL_PARAMETER_CAPTURE, 1);
	mmal_port_parameter_set_boolean(state.camera2_component->output[MMAL_CAMERA_VIDEO_PORT], MMAL_PARAMETER_CAPTURE, 1);
		
	bcm2835_gpio_write(GPIO_LED, HIGH);

	int64_t stop_time;
	
	int last_speed=-2, speed, font_size=state.common_settings.height/21, font_space;
	MMAL_BUFFER_HEADER_T *buffer_header=NULL;
	cairo_surface_t *temp_surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, TEXTW, TEXTH);
	cairo_t *temp_context =  cairo_create(temp_surface);
	cairo_rectangle(temp_context, 0, 0, TEXTW, TEXTH);
	cairo_select_font_face(temp_context, "cairo:serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
	cairo_set_font_size(temp_context, font_size);
	cairo_text_extents_t extents;
	cairo_text_extents(temp_context, "888 mph", &extents);
	font_space=TEXTW-extents.x_advance;
	cairo_destroy(temp_context);
	cairo_surface_destroy(temp_surface);

	if (timelimit)
		{
		stop_time = get_microseconds64()+(timelimit*1000000);
		}
	else
		{
		stop_time = 9223372036854775807LL;
		}
		
	while (stop_time > get_microseconds64() && !(stop_flag))
		{
		size_t c = (count <= (int64_t)chunk_bytes) ? (size_t)count : chunk_bytes;
		size_t f = c * 8 / bits_per_frame;

		if (last_speed != gps_data.speed)
			{
			if ((buffer_header = mmal_queue_get(state.hvs_textin_pool->queue)) != NULL)
				{
				if (gps_data.speed < 0)
					{
					buffer_header->length=buffer_header->alloc_size=0;
					buffer_header->user_data=NULL;
					} 
				else
					{
					cairo_surface_t *image=cairo_text(gps_data.speed, font_size, font_space);	
					buffer_header->data=cairo_image_surface_get_data(image);
					buffer_header->length=buffer_header->alloc_size=
						cairo_image_surface_get_height(image)*cairo_image_surface_get_stride(image);
					} 
				buffer_header->cmd=buffer_header->offset=0;
				int satus=mmal_port_send_buffer(state.hvs_component->input[2], buffer_header);
				if (status) printf("buffer send of text overlay failed\n");
				}
			last_speed=gps_data.speed;
			}

		if (pcm_read(audiobuf, bufs, f) != f)   //? if
			break;
		if (encode_and_write(bufs, 0, &def_mutex) < 0) 
			{
			fprintf(stderr, "encode/write failed!\n");
			prg_exit(EXIT_FAILURE);
			}
		if (write_variance > (write_target_time*fps*4) || write_variance < (write_target_time*fps*-4))
			{
			MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, 0};
			status = mmal_port_parameter_get(state.encoder_component->output[0], &param.hdr);
			if (status != MMAL_SUCCESS) {vcos_log_error("Unable to get current QP");}
			if (write_variance < 0 && param.value > state.quantisationMin)
				{
				param.value--;
				atmaxQ = MMAL_FALSE;
				fprintf(stdout, "%s Quantization %d\n", get_time_str(datestr), param.value);
				status = mmal_port_parameter_set(state.encoder_component->output[0], &param.hdr);
				if (status != MMAL_SUCCESS) {vcos_log_error("Unable to reset QP");}
				write_variance = 0;
				}
			else 
				{
				if (write_variance > 0 && param.value < state.quantisationMax)
					{
					param.value++;
					fprintf(stdout, "%s Quantization %d\n", get_time_str(datestr), param.value);
					status = mmal_port_parameter_set(state.encoder_component->output[0], &param.hdr);
					if (status != MMAL_SUCCESS) {vcos_log_error("Unable to reset QP");}
					write_variance = 0;
					}
				else
					{
					if (param.value == state.quantisationMax && !(atmaxQ))
						{
						fprintf(stdout, "%s At max Quantization %d\n", get_time_str(datestr), param.value);	
						atmaxQ = MMAL_TRUE;
						}
					write_variance = 0;
					}
				}
			}
		count -= c;
		if (abort_flg) {count = 0; fprintf(stderr, "abort\n");}
		if (check_switch) {
			stop_flag = bcm2835_gpio_lev(GPIO_SWT);
			if (stop_flag) {count =0;}
			}				
		}
	bcm2835_gpio_write(GPIO_LED, LOW);
	
	mmal_port_parameter_set_boolean(state.camera_component->output[MMAL_CAMERA_VIDEO_PORT], MMAL_PARAMETER_CAPTURE, 0);
	mmal_port_parameter_set_boolean(state.camera2_component->output[MMAL_CAMERA_VIDEO_PORT], MMAL_PARAMETER_CAPTURE, 0);
	
	if (encode_and_write(bufs, 1, &def_mutex) < 0) 
		{
		fprintf(stderr, "encode/write last failed!\n");
		prg_exit(EXIT_FAILURE);
		}
	close_it();
}

int main(int argc, char *argv[])
{

	int option_index;
	static const char short_options[] = "?D:d:h:w:c:q:i:n:l:f:F:";
	static const struct option long_options[] = {
		{"help", 0, 0, '?'},
		{"device", 1, 0, 'D'},
		{"duration", 1, 0,'d'},
		{"height", 1, 0, 'h'},
		{"width", 1, 0, 'w'},
		{"camera", 1, 0, 'c'},
		{"quantisation", 1, 0, 'q'},
		{"intraframe", 1, 0, 'i'},
		{"number", 1, 0, 'n'},
		{"location", 1, 0, 'l'},
		{"flip", 1, 0, 'f'},
		{"fps", 1, 0, 'F'},
		{0, 0, 0, 0}
		};

	strncpy(pcm_name, "dmic_sv", 17);
	int err, c, num_parms = 0;
	snd_pcm_info_t *info;

	snd_pcm_info_alloca(&info);

	assert(err >= 0);

	command = argv[0];
	stream = SND_PCM_STREAM_CAPTURE;

	while ((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1) {
		switch (c) {
		case '?':
			usage(command);
			return 0;
		case 'D':
			strncpy(pcm_name, optarg, 17);
			break;
		case 'd':
			timelimit = strtol(optarg, NULL, 0);
			break;
		case 'h':
			num_parms = sscanf(optarg, "%d:%d", &height, &ovl_height);
            switch(num_parms) {
			case EOF:
				fprintf(stdout, "Height sscanf failed\n");
				badparm=1;
				break;
			case 1: 
				if (height > 1080 || height < 1) {
					fprintf(stdout, "Height > than 1080 or < 1 %d\n", height);
					badparm=1;}
				else {
					ovl_height = height/2;}
				break;
			case 2:
				if (height > 1080 || height < 1  || ovl_height > 1080 || ovl_height < 1) {
					fprintf(stdout, "Height > than 1080 or < 1 %d:%d\n", height, ovl_height);
					badparm=1;}
				break;} 
			break;
		case 'w':
			num_parms = sscanf(optarg, "%d:%d", &width, &ovl_width);
            switch(num_parms) {
			case EOF:
				fprintf(stderr, "Width sscanf failed\n");
				badparm=1;
				break;
			case 1: 
				if (height > 1920 || width < 1) {
					fprintf(stdout, "Width > than 1920 or < 1 %d\n", width);
					badparm=1;}
				else {
					ovl_width = width/2;}
				break;
			case 2:
				if (width > 1920 || width < 1  || ovl_width > 1920 || ovl_width < 1) {
					fprintf(stdout, "Width > than 1920 or < 1 %d:%d\n", width, ovl_width);
					badparm=1;}
				break;} 
			break;
		case 'l':
			num_parms = sscanf(optarg, "%d:%d", &loc_x, &loc_y);
            switch(num_parms) {
			case EOF:
				fprintf(stderr, "overlay X/Y sscanf failed\n");
				badparm=1;
				break;
			case 1: 
				fprintf(stdout, "Must have both X and Y location\n");
				badparm=1;
				break;
			case 2:
				if (loc_y > 1080 || loc_y < 0  || loc_x > 1920 || loc_x < 0) {
					fprintf(stdout, "X must be between  0 & 1920 Y must be between 0 & 1080 %d:%d\n", loc_x, loc_y);
					badparm=1;}
				break;} 
			break;
		case 'f':
			num_parms = sscanf(optarg, "%d.%d:%d.%d", &m_flip_h, &m_flip_v, &o_flip_h, &o_flip_v);
            switch(num_parms) {
			case EOF:
				fprintf(stderr, "filp sscanf failed\n");
				badparm=1;
				break;
			case 4:
				if (m_flip_h < 0 || m_flip_h > 1 || m_flip_v < 0 || m_flip_v > 1 ||
					o_flip_h < 0 || o_flip_h > 1 || o_flip_v < 0 || o_flip_v > 1) {
					fprintf(stdout, "flip flag must be 1 or 0 (true or false) %d.%d:%d.%d\n", m_flip_h, m_flip_v, o_flip_h, o_flip_v);
					badparm=1;}
				break;
			default:
				fprintf(stdout, "flip does not have 4 params\n");
				badparm=1;
				break;}
			break;
		case 'c':
			camera = strtol(optarg, NULL, 0);
			if (camera > 1 || camera < 0) {
				fprintf(stdout, "Camera must be 0 or 1 %d\n", camera);
				badparm=1;}
			break;
		case 'q':
			num_parms = sscanf(optarg, "%d:%d:%d", &initQ, &minQ, &maxQ);
            switch(num_parms) {
			case EOF:
				fprintf(stderr, "quantisation sscanf failed\n");
				badparm=1;
				break;
			case 1:
				if (initQ > 40 || initQ < 20) {
					fprintf(stdout, "quantisation parameters must be 20 to 40 %d\n", initQ);
					badparm=1;}
				break;
			case 2:
				if (initQ > 40 || initQ < 20 || minQ > 40 || minQ < 20) {
					fprintf(stdout, "quantisation parameters must be 20 to 40 %d:%d\n", initQ, minQ);
					badparm=1;}
				break;
			case 3:
				if (initQ > 40 || initQ < 20 || minQ > 40 || minQ < 20|| maxQ > 40 || maxQ < 20) {
					fprintf(stdout, "quantisation parameters must be 20 to 40 %d:%d:%d\n", initQ, minQ, maxQ);
					badparm=1;}
				break;}
			break;
		case 'n':
			channel_num = strtol(optarg, NULL, 0);
			if (channel_num > 2 || channel_num < 1) {
				fprintf(stdout, "number of audio channels must be 1 or 2 %d\n", channel_num);
				badparm=1;}
			break;
		case 'i':
			intraframe = strtol(optarg, NULL, 0);
			if (intraframe > 60) {
				fprintf(stdout, "Intra frame rate should be < 60  %d\n", intraframe);
				badparm=1;}
			break;
		case 'F':
			fps = strtol(optarg, NULL, 0);
			if (fps < 0 || fps > 31) {
				fprintf(stdout, "frames per second must be > 0 and < 31 %d\n", fps);
				badparm=1;}
			break;
		default:
			fprintf(stdout, "Try `%s --help' for more information.\n", command);
			return 1;	
		}
	}

	if (timelimit == -1) {
		check_switch = 1;
		timelimit = 0;}

	if (badparm == 1) {
		return 1;}
		
	if (ovl_height >= height || ovl_width >= width) {
		fprintf(stdout, "overlay window must be smaller than main window\n");
		return 1;}
	if (loc_x >= (width-ovl_width) || loc_y >= height-ovl_height) {
		fprintf(stderr, "Warning: The overlay window falls partly outside the main window and will not be visible\n");}
	if (minQ > initQ || minQ > maxQ || maxQ < initQ || maxQ < minQ) {
		fprintf(stderr, "quantalizer values not logical\n");
		return 1;}
	if (loc_x == -1) {
		loc_x = 0;
		loc_y = height-ovl_height;}

	int i;
	char *cp = argv[argc-1];
    for (i=0;!(cp[i]==58)&&!(cp[i]==0);i++){}
	if ((cp[i] == 58) && (i==4)) 
		{
		if (strncmp(cp,"file", i) && strncmp(cp,"rtmp", i))
			{
			fprintf(stdout, "Protocol is incorrect %s\n", cp);
			return 1;
			}
		}		
	err = snd_pcm_open(&handle, pcm_name, stream, 0);
	if (err < 0) {
		fprintf(stdout, "%s open error: %s\n", pcm_name, snd_strerror(err));
		return 1;
	}
	
	fprintf(stdout, "%s PiPIPflv started %dX%d Q=%d>%d<%d camera=%d\n", get_time_str(datestr), width, height, minQ, initQ, maxQ, camera);

	rhwparams.format = DEFAULT_FORMAT;
	rhwparams.rate = DEFAULT_SPEED;
	rhwparams.channels = DEFAULT_CHANNELS_IN;
	chunk_size = 1024;
	hwparams = rhwparams;

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGABRT, signal_handler);

	if (optind > argc - 1)  
		{
	    fprintf(stdout, "Must have file or stream address\n");
	    return 1;
		} 
	else 
		{
		capture(argv[optind++]);
		}
	
	return EXIT_SUCCESS;
}

