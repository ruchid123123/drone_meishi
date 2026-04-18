# Communication Architecture


---

## 1. システム全体図

操縦者(スマホ) ↔ ドローン本体(ESP32) ↔ センサ/アクチュエータ、およびオプションのPC接続を含む全体像。

```mermaid
flowchart LR
    subgraph Operator["操縦側"]
        Phone["スマホ / PCブラウザ<br/>Web UI"]
        ProcPC["PC<br/>Processing Viewer<br/>(姿勢可視化)"]
    end

    subgraph Drone["ドローン本体"]
        ESP["ESP32 DevKit<br/>Flight Controller"]
        IMU["MPU6050<br/>6軸IMU"]
        ESC["4× Motor / ESC<br/>(M0–M3)"]
        NVS[("NVS Flash<br/>PID/Cal保存")]
    end

    Phone <-->|"Wi-Fi 2.4GHz<br/>SoftAP: ESP32-DRONE<br/>HTTP/WebSocket :80"| ESP
    ESP <-->|"I2C 400kHz<br/>SDA=21 / SCL=22<br/>addr 0x68"| IMU
    ESP -->|"PWM 20kHz / 11bit<br/>GPIO 15,13,12,14"| ESC
    ESP <-->|"内蔵SPI Flash"| NVS
    ESP -.->|"USB UART 115200bps<br/>(オプション)"| ProcPC
```

---

## 2. 機体プロペラ配置（Xクアッド）

WebSocket経由の `roll/pitch/yaw/throttle` をモータミキシングで4基に配分。

```mermaid
flowchart TB
    subgraph Frame[" "]
        direction TB
        FRONT(("▲ Front"))
        M0["M0 (FL)<br/>GPIO15<br/>CCW"]
        M1["M1 (FR)<br/>GPIO13<br/>CW"]
        M3["M3 (RL)<br/>GPIO14<br/>CW"]
        M2["M2 (RR)<br/>GPIO12<br/>CCW"]
        FRONT -.- M0
        FRONT -.- M1
    end

    Mix["Motor Mix (X)<br/>M0 = T + R + P − Y<br/>M1 = T − R + P + Y<br/>M2 = T − R − P − Y<br/>M3 = T + R − P + Y"]
    Mix --> M0
    Mix --> M1
    Mix --> M2
    Mix --> M3
```

---

## 3. 制御ループ（500 Hz）

メインループの内部データフロー。IMU読み出しからモータ出力までを2ms周期で回す。

```mermaid
flowchart LR
    A["① IMU読み出し<br/>I2C 14byte"] --> B["② AHRS推定<br/>Mahony / Madgwick / Comp"]
    B --> C["③ 角度PID<br/>(roll/pitch/yaw)"]
    C --> D["④ レートPID"]
    D --> E["⑤ モータミキシング<br/>X-quad"]
    E --> F["⑥ PWM出力<br/>LEDC 20kHz/11bit"]

    G["WebSocket入力<br/>throttle/roll/pitch/yaw"] --> C
    H["フェイルセーフ判定<br/>WS切断/タイムアウト/傾き>80°"] -.-> E
```

---

## 4. WebSocket メッセージ仕様

ブラウザ ↔ ESP32 (`ws://192.168.4.1/ws`) はテキストCSV/JSON。

### Client → Drone

| Type | Format | 用途 | レート |
|------|--------|------|--------|
| `C` | `C,throttle,roll,pitch,yaw,arm` | 操縦入力 | 50 Hz |
| `PID` | `PID,ANGLE\|RATE\|YAW,kp,ki,kd` | PIDゲイン更新 | 任意 |
| `CFG` | `CFG,KEY,VALUE` | 設定変更 | 任意 |
| `CAL` | `CAL,LEVEL\|GYRO` | キャリブレーション要求 | 任意 |
| `MTEST` | `MTEST,idx,thr,dur_ms` / `MTEST,STOP` | モータテスト | 任意 |
| `GET` | `GET` | 設定同期要求 | 接続時 |
| `SAVE` | `SAVE` | NVSへ保存 | 任意 |
| `K` | `K` | 強制Disarm | 任意 |

### Drone → Client (JSON)

| Type | 主なフィールド | 用途 | レート |
|------|--------------|------|--------|
| `tel` | `t, armed, roll, pitch, yaw, m0..m3, vbatt, loop_hz, fs, cmd_age` | テレメトリ | 20 Hz |
| `cfg` | PID/制限/オフセット一式 | 接続時の設定通知 | 接続時 |
| `ack` | `op, tag, ok, msg` | コマンド応答 | 都度 |

### 操縦シーケンス例

```mermaid
sequenceDiagram
    participant B as 📱 Browser
    participant E as ESP32 (FC)
    participant I as MPU6050
    participant M as Motors

    B->>E: HTTP GET /  (UI HTML)
    B->>E: WS connect /ws
    E-->>B: cfg (現在のPID/設定)
    loop 50Hz
        B->>E: C,throttle,roll,pitch,yaw,arm
    end
    loop 500Hz (内部)
        E->>I: I2C read 14B
        I-->>E: ax,ay,az,gx,gy,gz
        E->>M: PWM (M0..M3)
    end
    loop 20Hz
        E-->>B: tel JSON
    end
    Note over B,E: WS切断 → 即Disarm<br/>cmd_age > 300ms → Disarm
```

---

## 5. ピン/ポート割当

| 機能 | 信号 | GPIO / ポート | 備考 |
|------|------|---------------|------|
| I2C IMU | SDA | GPIO 21 | 400 kHz |
| I2C IMU | SCL | GPIO 22 | addr 0x68 |
| Motor M0 (FL) | PWM | GPIO 15 | LEDC ch0 |
| Motor M1 (FR) | PWM | GPIO 13 | LEDC ch1 |
| Motor M2 (RR) | PWM | GPIO 12 | LEDC ch2 |
| Motor M3 (RL) | PWM | GPIO 14 | LEDC ch3 |
| Web Server | TCP | :80 (HTTP / WS) | ESPAsyncWebServer |
| Wi-Fi | SoftAP | 2.4GHz | SSID `ESP32-DRONE` / IP `192.168.4.1` |
| Debug | UART0 | USB | 115200 bps |

---

## 6. フェイルセーフ

| トリガ | 動作 | 備考 |
|--------|------|------|
| WebSocket切断 | 即Disarm | `FS_WS_DISCONNECT` |
| 操縦コマンド未受信 > 300ms | Disarm | `cmd_to` で変更可 |
| 機体傾き > 80° | Auto Disarm | `tilt_dis` で変更可 |
| IMU異常 | Disarm | 姿勢推定値が不正な場合 |
| Armラッチ後の再Arm抑止 | キル状態保持 | 明示的にDisarm要求が必要 |

---

## 7. 通信
操縦は **Wi-Fi WebSocket**
ライブラリ依存は `ESPAsyncWebServer` / `AsyncTCP`

以下は使用していません．
- SBUS（プロポ受信機）
- ESP-NOW
- Bluetooth
- MAVLink / CRSF / S.Port 等のテレメトリ規格


