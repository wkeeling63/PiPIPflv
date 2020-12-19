#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <semaphore.h>
 
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

#include "raspiCamUtilities.h"
#include "mmalcomponent.h"

#include <stdbool.h>

MMAL_STATUS_T create_camera_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
   MMAL_STATUS_T status;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create camera component");
      goto error;
   }

   status = raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
   status += raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
   status += raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set stereo mode : error %d", status);
      goto error;
   }

   MMAL_PARAMETER_INT32_T camera_num =
   {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

   status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", status);
      goto error;
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera doesn't have output ports");
      goto error;
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
      goto error;
   }

   preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, default_camera_control_callback);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable control port : error %d", status);
      goto error;
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->common_settings.width,
         .max_stills_h = state->common_settings.height,
         .stills_yuv422 = 0,
         .one_shot_stills = 0,
         .max_preview_video_w = state->common_settings.width,
         .max_preview_video_h = state->common_settings.height,
         .num_preview_video_frames = 3 + vcos_max(0, (state->framerate-30)/10),
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 0,
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
      };
      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   format = preview_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(preview_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera viewfinder format couldn't be set");
      goto error;
   }

   // Set the encode format on the video  port

   format = video_port->format;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(video_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera video format couldn't be set");
      goto error;
   }
   

   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   // Set the encode format on the still  port

   format = still_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }

  // Ensure there are enough buffers to avoid dropping frames 
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


   // Enable component 
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera component couldn't be enabled");
      goto error;
   }

   // Note: this sets lots of parameters that were not individually addressed before.
   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   if (!state->camera_component)
   {
      state->camera_component = camera;
   }
   else
   {
      state->camera2_component = camera;
   }

   return status;
   
error:

   if (camera)
      mmal_component_destroy(camera);

   return status;
} 

void destroy_camera_component(RASPIVID_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
   if (state->camera2_component)
   {
      mmal_component_destroy(state->camera2_component);
      state->camera_component = NULL;
   }
} 

MMAL_STATUS_T create_hvs_component(RASPIVID_STATE *state)
{
      
   MMAL_COMPONENT_T *hvs = 0; 
   MMAL_PORT_T *hvs_main_input = NULL, *hvs_ovl_input, *hvs_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;
   int opacity = 255;
   
        
   status = mmal_component_create("vc.ril.hvs", &hvs);
   
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create hvs component");
      goto error;
   }
   
   if (!hvs->input_num || !hvs->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("HVS doesn't have input/output ports");
      goto error;
   }

   hvs_main_input = hvs->input[0];
   hvs_ovl_input = hvs->input[1];
   hvs_output = hvs->output[0];
  
   mmal_format_copy(hvs_main_input->format, state->camera_component->output[MMAL_CAMERA_VIDEO_PORT]->format); 

   hvs_output->format->encoding = MMAL_ENCODING_RGB24;

   MMAL_DISPLAYREGION_T param = {{MMAL_PARAMETER_DISPLAYREGION, sizeof(param)}, MMAL_DISPLAY_SET_DUMMY};
   param.set = MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_LAYER | MMAL_DISPLAY_SET_ALPHA;
   param.dest_rect.width = state->common_settings.width;
   param.dest_rect.height = state->common_settings.height;
   param.layer = 0;
   param.alpha = opacity;
 
   status = mmal_port_parameter_set(hvs_main_input, &param.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set displayregion hvs main input");
      goto error;
   }
  
   param.set = MMAL_DISPLAY_SET_DUMMY;  
   status = mmal_port_parameter_get(hvs_main_input, &param.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to get displayregion hvs main input");
      goto error;
   }
    
   // Commit the port changes to the hvs main input port
   status = mmal_port_format_commit(hvs_main_input);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on hvs main input port");
      goto error;
   }

   param.set =  MMAL_DISPLAY_SET_FULLSCREEN | MMAL_DISPLAY_SET_DEST_RECT | MMAL_DISPLAY_SET_LAYER | MMAL_DISPLAY_SET_ALPHA;
   param.fullscreen = MMAL_FALSE;
   param.dest_rect.x = state->common_settings.ovl_x;  
   param.dest_rect.y = state->common_settings.ovl_y;
   param.dest_rect.width = state->common_settings.ovl_width;
   param.dest_rect.height = state->common_settings.ovl_height;
   param.layer=  1;
   param.alpha = opacity | MMAL_DISPLAY_ALPHA_FLAGS_DISCARD_LOWER_LAYERS;
      
   status = mmal_port_parameter_set(hvs_ovl_input, &param.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set displayregion hvs overlay input");
      goto error;
   }
   
   param.set = MMAL_DISPLAY_SET_DUMMY;  
   status = mmal_port_parameter_get(hvs_ovl_input, &param.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to get displayregion hvs main input");
      goto error;
   }
   
   // Commit the port changes to the hvs ovl input port
   status = mmal_port_format_commit(hvs_ovl_input);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on hvs ovl input port");
      goto error;
   }
   
   // Commit the port changes to the hvs output port
   hvs_output->format->encoding = MMAL_ENCODING_RGB24;
   hvs_output->format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   hvs_output->format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   hvs_output->format->es->video.crop.x = 0;
   hvs_output->format->es->video.crop.y = 0;
   hvs_output->format->es->video.crop.width = state->common_settings.width;
   hvs_output->format->es->video.crop.height = state->common_settings.height;
   hvs_output->format->es->video.frame_rate.num = state->framerate;
   hvs_output->format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;
   
   status = mmal_port_format_commit(hvs_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on hvs output port");
      goto error;
   }
   
   //  Enable component
   status = mmal_component_enable(hvs);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable hvs component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(hvs_output, hvs_output->buffer_num, hvs_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for hvs output port %s", hvs_output->name);
   }

   state->hvs_pool = pool;
   state->hvs_component = hvs;

   return status;

error:
   if (hvs)
      mmal_component_destroy(hvs);

   state->hvs_component = NULL;

   return status;
}

void destroy_hvs_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->hvs_pool)
   {
      mmal_port_pool_destroy(state->hvs_component->output[0], state->hvs_pool);
   }

   if (state->hvs_component)
   {
      mmal_component_destroy(state->hvs_component);
      state->hvs_component = NULL;
   }
}


MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;
   
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create video encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Video encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output ??
   mmal_format_copy(encoder_output->format, encoder_input->format);
     
   mmal_format_copy(encoder_input->format, state->hvs_component->output[0]->format);
      
   status = mmal_port_format_commit(encoder_input);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder input port");
      goto error;
   }

   encoder_output->format->encoding = state->encoding;
   encoder_output->format->bitrate = state->bitrate;
   
   encoder_output->buffer_size = encoder_output->buffer_size_recommended;
   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;
      
   encoder_output->buffer_num = encoder_output->buffer_num_recommended;
// remove when hang is resolved
   encoder_output->buffer_num = 100;  // if hang not fix on real Pi hardware
   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;
     
   // set the frame rate on output to 0, to ensure it gets
   // updated correctly from the input framerate when port connected
   encoder_output->format->es->video.frame_rate.num = 0;
   encoder_output->format->es->video.frame_rate.den = 1;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      goto error;
   }

   MMAL_PARAMETER_VIDEO_RATECONTROL_T paramrc = {{ MMAL_PARAMETER_RATECONTROL, sizeof(paramrc)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
   status = mmal_port_parameter_set(encoder_output, &paramrc.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set ratecontrol");
      goto error;
   }

   MMAL_PARAMETER_UINT32_T param = {{ MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
   status = mmal_port_parameter_set(encoder_output, &param.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set intraperiod");
      goto error;
   }

   MMAL_PARAMETER_UINT32_T param1 = {{ MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param1)}, state->quantisationParameter};
   status = mmal_port_parameter_set(encoder_output, &param1.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set initial QP");
      goto error;
   }

   MMAL_PARAMETER_UINT32_T param2 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param2)}, state->quantisationMin};
   status = mmal_port_parameter_set(encoder_output, &param2.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set min QP");
      goto error;
   }

   MMAL_PARAMETER_UINT32_T param3 = {{ MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param3)}, state->quantisationMax};
   status = mmal_port_parameter_set(encoder_output, &param3.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set max QP");
      goto error;
   }

   MMAL_PARAMETER_VIDEO_PROFILE_T  paramvp;
   paramvp.hdr.id = MMAL_PARAMETER_PROFILE;
   paramvp.hdr.size = sizeof(paramvp);
   paramvp.profile[0].profile = state->profile;
   paramvp.profile[0].level = state->level;
   status = mmal_port_parameter_set(encoder_output, &paramvp.hdr);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set H264 profile");
      goto error;
   }

   if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set immutable input flag");
   }


   //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set INLINE HEADER FLAG parameters");
   }

      //set flag for add SPS TIMING
   if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
   {
      vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
   }
 
   //  Enable component
   status = mmal_component_enable(encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   state->encoder_pool = pool;
   state->encoder_component = encoder;
      
   return status;

error:
   if (encoder)
      mmal_component_destroy(encoder);

   state->encoder_component = NULL;

   return status;
}

void destroy_encoder_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->encoder_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}



MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}

void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}

void get_sensor_defaults(int camera_num, char *camera_name, int *width, int *height )
{
   MMAL_COMPONENT_T *camera_info;
   MMAL_STATUS_T status;

   // Default to the OV5647 setup
   strncpy(camera_name, "OV5647", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);

   // Try to get the camera name and maximum supported resolution
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
   if (status == MMAL_SUCCESS)
   {
      MMAL_PARAMETER_CAMERA_INFO_T param;
      param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
      param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware version
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);

      if (status != MMAL_SUCCESS)
      {
         // Running on newer firmware
         param.hdr.size = sizeof(param);
         status = mmal_port_parameter_get(camera_info->control, &param.hdr);
         if (status == MMAL_SUCCESS && param.num_cameras > camera_num)
         {
            // Take the parameters from the first camera listed.
            if (*width == 0)
               *width = param.cameras[camera_num].max_width;
            if (*height == 0)
               *height = param.cameras[camera_num].max_height;
            strncpy(camera_name, param.cameras[camera_num].camera_name, MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
            camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN-1] = 0;
         }
         else
            vcos_log_error("Cannot read camera info, keeping the defaults for OV5647");
      }
      else
      {
         // Older firmware
         // Nothing to do here, keep the defaults for OV5647
      }

      mmal_component_destroy(camera_info);
   }
   else
   {
      vcos_log_error("Failed to create camera_info component");
   }

   // default to OV5647 if nothing detected..
   if (*width == 0)
      *width = 2592;
   if (*height == 0)
      *height = 1944;
}
void default_status(RASPIVID_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));
   
   strncpy(state->common_settings.camera_name, "(Unknown)", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
   // We dont set width and height since these will be specific to the app being built.

   state->common_settings.filename = NULL;
   state->common_settings.verbose = 0;  //remove
   state->common_settings.cameraNum = 1;
   state->common_settings.sensor_mode = 5;

   // Now set anything non-zero
   state->timeout = -1; 
   state->common_settings.width = 1920;    
   state->common_settings.height = 1080;     
   state->encoding = MMAL_ENCODING_H264;
   state->bitrate = 0; // 0 for variable bit rate
//   state->framerate = VIDEO_FRAME_RATE_NUM;
   state->intraperiod = 15;    // Not set
   state->quantisationParameter = 30;
   state->quantisationMin = 20;
   state->quantisationMax = 40;
   state->immutableInput = 1;
   state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
   state->level = MMAL_VIDEO_LEVEL_H264_41;
   state->waitMethod = 0;     //remove
   state->bCapturing = 0;     //remove
   state->bInlineHeaders = 0;
   state->frame = 0;             //remove??
   state->addSPSTiming = MMAL_FALSE;
   state->slices = 1;

   // Set up the camera_parameters to default
   state->camera_parameters.sharpness = 0;
   state->camera_parameters.contrast = 0;
   state->camera_parameters.brightness = 50;
   state->camera_parameters.saturation = 0;
   state->camera_parameters.ISO = 0;                    // 0 = auto
   state->camera_parameters.videoStabilisation = 0;
   state->camera_parameters.exposureCompensation = 0;
   state->camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_AUTO;
   state->camera_parameters.flickerAvoidMode = MMAL_PARAM_FLICKERAVOID_OFF;
   state->camera_parameters.exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT;  //from MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE
   state->camera_parameters.awbMode = MMAL_PARAM_AWBMODE_AUTO;
   state->camera_parameters.imageEffect = MMAL_PARAM_IMAGEFX_NONE;
   state->camera_parameters.colourEffects.enable = 0;
   state->camera_parameters.colourEffects.u = 128;
   state->camera_parameters.colourEffects.v = 128;
   state->camera_parameters.rotation = 0;
   state->camera_parameters.hflip = state->camera_parameters.vflip = 1;
   state->camera_parameters.roi.x = state->camera_parameters.roi.y = 0.0;
   state->camera_parameters.roi.w = state->camera_parameters.roi.h = 1.0;
   state->camera_parameters.shutter_speed = 0;          // 0 = auto
   state->camera_parameters.awb_gains_r = 0;      // Only have any function if AWB OFF is used.
   state->camera_parameters.awb_gains_b = 0;
   state->camera_parameters.drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;
   state->camera_parameters.stats_pass = MMAL_FALSE;
   state->camera_parameters.enable_annotate = 0;
   state->camera_parameters.annotate_string[0] = '\0';
   state->camera_parameters.annotate_text_size = 0;	//Use firmware default
   state->camera_parameters.annotate_text_colour = -1;   //Use firmware default
   state->camera_parameters.annotate_bg_colour = -1;     //Use firmware default
   state->camera_parameters.stereo_mode.mode = MMAL_STEREOSCOPIC_MODE_NONE;
   state->camera_parameters.stereo_mode.decimate = MMAL_FALSE;
   state->camera_parameters.stereo_mode.swap_eyes = MMAL_FALSE;
}


