# YanBot-Interact
Interaction Part of YanBot.

```bash
bash scripts/install_deps_basic.sh
```

## wakeup

### snowboy

```bash
# at YanBot-Interact/
bash scripts/install_deps_snowboy.sh
# at wakeup/snowboy/
python examples/Python3/demo.py resources/models/nihaoyanyan.pmdl
```

## stt

### paddle_speech

https://hub.docker.com/r/paddlecloud/paddlespeech/tags

```bash
宿主机 (Host)                          Docker 容器
+-------------------+                 +-------------------+
|                   |                 |                   |
|   STT客户端脚本     | HTTP请求        |  STT服务 (端口8888)|
|  (client_stt.py)  | --------------->|                   |
|                   |                 |                   |
|                   |                 |                   |
|   TTS客户端脚本     | HTTP请求        |  TTS服务 (端口8889)|
|  (client_tts.py)  | --------------->|                   |
+-------------------+                 +-------------------+
```

```bash
# at YanBot-Interact/
bash scripts/install_deps_paddle_speech.sh
rosrun stt_pkg client_paddle_speech.py
# at docker: /mnt/
bash scripts/install_deps_docker_paddle_speech.sh
source scripts/run_stt_docker_paddle_speech.sh
```
