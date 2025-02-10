# YanBot-Interact

Human Robot Interaction Part. User information publisher. Voice interaction, screen UI display. Currently containing 4 module components.

## basic

Basic prerequisites for running subsequent programs. Need to be prepared first.

```bash
# YanBot-Interact/
source scripts/install_deps_basic.sh
catkin_make
source devel/setup.bash
```

## interact

Comprehensive node, including launch file.

```bash
# YanBot-Interact/
## Docker paddlespeech_dev T1: /mnt/
bash scripts/install_deps_paddle_speech_in_docker.sh
source scripts/run_stt_paddle_speech_in_docker.sh
## Docker paddlespeech_dev T2: /mnt/
source scripts/run_tts_paddle_speech_in_docker.sh
## T3: YanBot-Interact/
roslaunch interact_pkg simu.launch
```

## wakeup

Hotword detection. Currently using snowboy.

```bash
# YanBot-Interact/
bash scripts/install_deps_snowboy.sh
## demo: wakeup/snowboy/
python examples/Python3/demo.py resources/models/nihaoyanyan.pmdl
## ROS: YanBot-Interact/
rosrun wakeup_pkg wakeup_snowboy.py
```

## stt

Using [paddlespeech]((https://hub.docker.com/r/paddlecloud/paddlespeech/tags)) docker container.
```bash
# YanBot-Interact/
bash scripts/install_deps_paddle_speech.sh
## ROS: YanBot-Interact/
rosrun stt_pkg client_paddle_speech.py
## paddlespeech_dev T1: /mnt/
bash scripts/install_deps_paddle_speech_in_docker.sh
source scripts/run_stt_docker_paddle_speech.sh
```

## tts

Using [paddlespeech]((https://hub.docker.com/r/paddlecloud/paddlespeech/tags)) docker container.
```bash
# YanBot-Interact/
bash scripts/install_deps_paddle_speech.sh
## ROS: YanBot-Interact/
rosrun tts_pkg client_paddle_speech.py
## paddlespeech_dev T1: /mnt/
bash scripts/install_deps_paddle_speech_in_docker.sh
source scripts/run_stt_docker_paddle_speech.sh
```
