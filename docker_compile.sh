#!/bin/bash
# 启动 Docker 容器进行编译，共享宿主机已下载好的 ESP32 环境
docker run --rm -v "$(pwd):/workspace" \
  -v "$HOME/.arduino15:/root/.arduino15" \
  -v "$HOME/Arduino:/root/Arduino" \
  drone-builder compile --fqbn esp32:esp32:esp32 drone_meishi/drone_meishi.ino
