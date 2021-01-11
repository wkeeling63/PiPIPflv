Purpose: To build and front and rear facing live stream race camera

Description PiPIPflv: Creates a 2 camera components and tunnels them to the 
Hardware Video Scaler (HVS) component to overlay the second camera 
frame on the frame of the main camera. The HVS output is tunneled to 
the encoder to create a video H.264 stream.  The video stream is 
created using MMAL api and run on the GPU.  The audio stream is 
created from ALSA api using Adafruit I2S MEMS Microphone 
(SPH0645LM4H).  The audio stream is encoded to ACC using FFPMEG api 
and both streams are added to flash video container by FFMPEG api.


Install: 

	cmake .
	
	make 

Software required:
* FFMPEG library sudo apt-get install libavformat-dev
* ALSA library sudo apt-get install libasound2-dev
* bcm2835 library http://www.airspayce.com/mikem/bcm2835/index.html (tested with version 1.68)
* Cairo library sudo apt-get install libcairo2-dev
* cmake for build sudo apt-get install cmake

  	(to install all needed software other than bcm2835 "sudo apt-get install cmake ibavformat-dev libasound2-dev libcairo2-dev")

Hardware required:
* 2 Camera Raspberry Pi Compute Module 

	(with carrier board - Raspberry CM3 for CM4 I/O board, StereoPi or WaveShare POE board)
* Adafruit I2S MEMS Microphone (SPH0645LM4H) 

	Must use "Old Kernel Install Method" with Compute Module https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/raspberry-pi-wiring-and-test
	
	with CM4 you must compile my_loader.c with fe203000.i2s values (same as Pi4) in place of 3f203000.i2s
	
	PiPIPflv expects .asoundrc to be setup as pre the above page (after Nov 2020 Pi OS will remove .asoundrc in the user home will be removed when desktop GUI start the work around for this is a golbal ALSA configuration /etc/asound.conf with the same config as .asoundrc)

	
* SixFab Raspberry Pi 3G/4G & LTE Base HAT (for Cellular modem and GPS) 

	optional if GPS module is not install you will see open/close errors on GPS like "Open of GPS data failed! RC=-1"

