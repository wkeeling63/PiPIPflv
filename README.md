Purpose: To build and front and rear facing live stream race camera

Description PiPIPflv: Creates a 2 camera components and tunnels them to the 
Hardware Video Scaler (HVS) component to overlay the second camera 
frame on the frame of the main camera. The HVS output is tunneled to 
the encoder to create a video H.264 stream.  The video stream is 
created using MMAL api and run on the GPU.  The audio stream is 
created from ALSA api using Adafruit I2S MEMS Microphone 
(SPH0645LM4H).  The audio stream is encoded to ACC using FFPMEG aps 
and both streams are added to flash video container by FFMPEG api.


Install: cmake . 

Hardware required:

2 Camera Raspberry Pi Compute Module 
	(with carrier board - Raspberry CM3 for CM4 I/O board, StereoPi or WaveShare POE board)

Adafruit I2S MEMS Microphone (SPH0645LM4H)

