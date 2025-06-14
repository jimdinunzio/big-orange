conda create -n BigOrange python==3.7.12
conda activate BigOrange
cd %CONDA_PREFIX%
mkdir src
cd src
git clone https://github.com/jimdinunzio/big-orange
cd big-orange

git submodule update --init --recursive

REM For Google Assistant related code
REM conda install google-auth
REM conda install google-auth-oauthlib
REM pip install google-assistant-grpc
pip install --upgrade click

REM For TTS (Text To Speech)
pip install git+https://github.com/DeepHorizons/tts

REM For Word2Number used by robot main
conda install -c conda-forge word2number
pip install parse

REM For Loading shared objects (and specifically 64-bit python bridge to 32-bit DLL)
pip install msl.loadlib

pip install pyaudio

REM for coral Edge TPU
REM pip3 install https://dl.google.com/coral/python/tflite_runtime-2.1.0.post1-cp37-cp37m-win_amd64.whl
REM conda install Pillow

REM For Depth AI
pip install depthai==2.30.0.0



REM For Alexa Skill integration
pip install awsiotsdk

pip install -r python\pyFirmata\requirements.txt

pip install -r python\depthai_blazepose\requirements.txt
# for windows pip doesn't install all the binary dependencies, but conda does.
pip uninstall open3d
conda install -c conda-forge open3d

pip install playsound
pip install opencv-python
pip install pygame
pip install keyboard
pip install openai
pip install websockets
pip install imutils

cd python\speech_recognition
python setup.py install
cd ..

REM For Respeaker Mic Array V2.0 see git/usb_4_mic_array/requirements.txt
pip install -r python\usb_4_mic_array\requirements.txt

REM Install these prerequisites using conda for the Mycroft Precise wake word detection because pip couldn't do it alone
@REM conda install -c conda-forge hdf5
@REM conda install -c conda-forge netcdf4

@REM conda install tensorflow=1.13.1
REM For Mycroft Precise wake word detection

@REM cd mycroft-precise
@REM python setup.py install

@REM cd runner
@REM python setup.py install
@REM cd ..

pip install vosk

REM Install voices for Windows SAPI from .reg files in windows-10-voices-add

REM install driver for SEEED DFU and SEEED Control  for LEDs on respeaker
REM https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/#install-dfu-and-led-control-driver

REM Install Touch Screen driver TouchPanel_1.3.2.0

REM Install Autologon from Sysinternals
REM https://docs.microsoft.com/en-us/sysinternals/downloads/autologon

REM Install depthcam_depthcam_publish_demo
REM Install Xbox360_64Eng driver for off brand XBOX 360 receiver - must switch to test signing mode 
REM with "bcdedit /set testsigning on" and reboot and reverse afterwards.

REM Must install VC Redist 2010 x86, 2015-2022 x64
REM Copy model folder with vosk model

REM Flash built in Arduino Leonardo with modified firmata (with PULSE_IN) _arduino example StandardFirmata.ino 
REM Install openai API key