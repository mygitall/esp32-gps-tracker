# ESP32 GPS 追踪器

ESP32 + ATGM336H GPS + Air780EX 4G LTE Cat.1 实时定位追踪系统。

## 仓库说明

本项目拆分为两个独立仓库：

| 仓库 | 内容 | 本地路径 |
|------|------|---------|
| `esp32-gps-tracker` | 固件（.ino 草图） | `~/esp32-gps-tracker/` |
| `esp32` | 服务端（PHP + HTML） | `~/Desktop/AI开发/esp32/` |

## 拉取到本地

```bash
# 固件仓库
git clone https://github.com/mygitall/esp32-gps-tracker.git ~/esp32-gps-tracker

# 服务端仓库
git clone https://github.com/mygitall/esp32.git ~/Desktop/AI开发/esp32
```

## 硬件接线

| ESP32 GPIO | 连接 |
|-----------|------|
| GPIO4 | Air780EX TX |
| GPIO5 | Air780EX RX |
| GPIO16 | ATGM336H TX |
| GPIO17 | ATGM336H RX |
| GPIO27 | S8050 基极 → Air780EX PWRKEY |
| GPIO2 | 状态 LED |

- Air780EX 波特率: 115200
- ATGM336H 波特率: 9600
- 供电: ESP32 DevKit 需 5V，单节 18650（3.7-4.2V）需升压模块

## 编译与烧录

```bash
# 编译
arduino-cli compile --fqbn esp32:esp32:esp32 full_firmware_4g/full_firmware_4g.ino

# USB 烧录（设备在 /dev/cu.usbserial-0001）
arduino-cli upload --fqbn esp32:esp32:esp32 -p /dev/cu.usbserial-0001 full_firmware_4g/full_firmware_4g.ino

# WiFi OTA 烧录（设备和电脑同局域网）
python3 ~/Library/Arduino15/packages/esp32/hardware/esp32/3.3.8/tools/espota.py \
  -i 192.168.31.141 -p 3232 -a "12345678" -f full_firmware_4g.ino.bin
```

## 服务端部署

服务端 PHP 文件通过 FTP 上传到虚拟主机：

```bash
# FTP 上传示例
python3 -c "
import ftplib
ftp = ftplib.FTP()
ftp.connect('gao367888125.zfkwp02.guaixing.cn', 21, timeout=15)
ftp.login('gao367888125', 'c6f57i8j')
ftp.storbinary('STOR /esp32/mmq/receiver.php', open('receiver.php','rb'))
ftp.quit()
"
```

服务器文件路径: `/esp32/mmq/`

## 架构

```
ATGM336H GPS → ESP32 → Air780EX 4G → HTTP → receiver.php → MySQL → api.php → map.html
```

- 固件: GPS 每秒采集，位置变化存入缓冲区，每 10 秒批量 GET 发送
- 服务端: receiver.php 接收批量数据入库，api.php 提供查询，map.html 展示地图
