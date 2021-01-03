# PiPIPflv

SET(COMPILE_DEFINITIONS -Werror)

# Set --no-as-needed to stop the linker discarding mmal_vc_client
# as it can't see that the constructor registers a load of functionality
# with the MMAL core.
# SET( CMAKE_EXE_LINKER_FLAGS "-Wl,--no-as-needed" )
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-L/opt/vc/lib,--no-as-needed")

include_directories(/opt/vc/include)
include_directories(/usr/include/alsa)
# include_directories(/alsa-utils/include)

# Find the commit hash of the build and pass to the compiler
execute_process(
  COMMAND git log -1 --abbrev=12 --format=%h
  OUTPUT_VARIABLE GIT_COMMIT_ID
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
add_definitions("-DGIT_COMMIT_ID=\"${GIT_COMMIT_ID}\"")

# Determine if we are tainted
execute_process(
  COMMAND bash "-c" "git ls-files -m | wc -l"
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_TAINTED
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
add_definitions("-DGIT_TAINTED=${GIT_TAINTED}")
 
set (PIPCAM_SOURCES
   raspiCamUtilities.c
   mmalcomponent.c
   GPSUtil.c)
   
add_executable(PiPIPflv ${PIPCAM_SOURCES} PiPIPflv.c)

set (MMAL_LIBS mmal_core mmal_util mmal_vc_client)

target_link_libraries(PiPIPflv pthread asound avformat avcodec avutil swresample ${MMAL_LIBS} vcos bcm_host bcm2835 cairo)

install(TARGETS PiPIPflv RUNTIME DESTINATION bin)
