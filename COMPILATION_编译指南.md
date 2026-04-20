# ESP32 迷你无人机飞控 - 编译指南 (2026年版)

本指南详细说明了如何在受限网络环境下，利用国内加速镜像和 Docker 成功编译本项目。

---

## 1. 快速开始 (推荐方案：Docker 挂载模式)

由于 ESP32 工具链体积庞大（约 300MB+），在 Docker 内部直接下载极易失败。推荐在宿主机下载环境后，通过 Docker 挂载运行。

### 准备环境 (宿主机)
1. **安装工具**：确保 `arduino-cli` 已安装（本项目路径为 `/home/j/bin/arduino-cli`）。
2. **配置国内加速镜像** (乐鑫官方极狐镜像)：
   ```bash
   arduino-cli config init --overwrite
   arduino-cli config add board_manager.additional_urls https://jihulab.com/esp-mirror/espressif/arduino-esp32/-/raw/gh-pages/package_esp32_index_cn.json
   arduino-cli core update-index
   ```
3. **安装核心库** (必须带 `-cn` 后缀以触发国内加速)：
   ```bash
   arduino-cli core install esp32:esp32@3.0.7-cn
   ```
4. **安装依赖库**：
   ```bash
   arduino-cli lib install "ESPAsyncWebServer" "AsyncTCP"
   ```

### 使用 Docker 编译
运行项目根目录下的脚本：
```bash
chmod +x docker_compile.sh
./docker_compile.sh
```
该脚本会将宿主机的 `~/.arduino15`（环境）和 `~/Arduino/libraries`（库）挂载到容器中，实现环境隔离与高速编译的平衡。

---

## 2. 关键补丁与修复 (已在本项目中应用)

在编译过程中，我们解决了以下两个核心冲突，后续开发请务必注意：

### 2.1 修复代码重定义错误
在 `drone_meishi.ino` 中，`kEnableSerialAttDebug` 曾被定义了两次，导致编译失败。已删除冗余定义。

### 2.2 修复 ESPAsyncWebServer 兼容性
由于 ESP32 Arduino Core 3.0.x 升级了 `mbedtls` 库，旧版 `ESPAsyncWebServer` 中的加密函数后缀不匹配。需对库文件进行如下替换：
- `mbedtls_md5_starts_ret` -> `mbedtls_md5_starts`
- `mbedtls_md5_update_ret` -> `mbedtls_md5_update`
- `mbedtls_md5_finish_ret` -> `mbedtls_md5_finish`
*注：本项目宿主机环境已完成此修复。*

---

## 3. 故障排除

### 网络超时
- **Docker 拉取镜像慢**：已配置 `/etc/docker/daemon.json` 使用国内镜像（如 `docker.xuanyuan.me`）。
- **Arduino 核心库下载失败**：请检查是否使用了 `package_esp32_index_cn.json` 并选择了 `-cn` 版本。

### 编译命令参考
- **完整编译指令**：
  ```bash
  arduino-cli compile --fqbn esp32:esp32:esp32 drone_meishi/drone_meishi.ino
  ```

---

## 4. 文件说明
- `Dockerfile.build`：用于构建基础编译镜像。
- `docker_compile.sh`：一键调用 Docker 进行编译的入口。
- `arduino-cli-bin`：为 Docker 构建准备的本地工具二进制文件。
