# M5Bala2-Unified

製品そのものの概要は [README.md](README.md)（英語、上流のもの）を参照してください。本ドキュメントはフォークでの変更点と開発手順をまとめたものです。

[m5stack/M5Bala2](https://github.com/m5stack/M5Bala2) のフォークです。デバイス層を M5Unified に移行し、ESP-IDF 5.x ベースの ESP32 core 3.x でビルドできるようにしています。

上流は `M5Stack` ライブラリ（0.4.6）を使っており、同ライブラリは 2023 年で更新が止まっているため core 3.x でビルドできません（ESP-IDF 5.x で削除された `rom/miniz.h` に依存）。上流の README にも「ボードマネージャを 2.1.4 に下げてください」という注意書きがあります。本フォークはこの制約を解消します。

MPU6886 の姿勢角を Madgwick フィルタで推定し、角度 PID と速度 PID の 2 段構成で車輪を制御します。傾き角の波形は M5Canvas によるちらつきのない描画で、姿勢制御とは別のコアで動きます。実機で上流と同等の安定性を確認済みです。

## 対象ハードウェア

| 項目           | 値                                                                                                   |
| -------------- | ---------------------------------------------------------------------------------------------------- |
| 製品           | Bala2-Fire (SKU: K014-E)                                                                             |
| 本体           | M5Stack Fire (ESP32, 16MB flash, PSRAM 4MB)                                                          |
| モーターベース | BALA2（メインコントローラ STM32F030C8T6、N20 エンコーダ付減速モーター 2 基、1200mAh バッテリー内蔵） |
| ベースとの通信 | I2C アドレス `0x3A`、内部 I2C バス、100kHz                                                           |
| IMU            | MPU6886                                                                                              |
| FQBN           | `m5stack:esp32:m5stack_fire`                                                                         |

関連ドキュメント: [Bala2-Fire](https://docs.m5stack.com/en/app/bala2fire) / [Bala2](https://docs.m5stack.com/en/app/bala2)

## セットアップ

[Arduino CLI](https://arduino.github.io/arduino-cli/) が必要です。

```bash
brew install arduino-cli
```

本リポジトリの作成には Claude Code 用スキル [m5stack-arduino-cli-skill](https://github.com/YuyaIwata/m5stack-arduino-cli-skill) を活用しています。このスキルをサブモジュールとして含めているため、クローン時は `--recurse-submodules` を付けてください。

```bash
git clone --recurse-submodules git@github.com:YuyaIwata/M5Bala2-Unified.git
```

クローン済みの場合は次のコマンドで取得できます。

```bash
git submodule update --init --recursive
```

ボードパッケージとライブラリは [sketch.yaml](sketch.yaml) のプロファイルに固定されており、初回ビルド時に自動で導入されます。手動でのボードマネージャ URL 追加は不要です。

- プラットフォーム: `m5stack:esp32@3.3.8`（M5Stack 公式インデックス、ESP-IDF v5.5.4）
- ライブラリ: `M5Unified@0.2.19`、`M5GFX@0.2.26`

## ビルドと書き込み

```bash
# ビルド
arduino-cli compile --profile m5stack_fire .

# ポート確認
arduino-cli board list

# 書き込み
arduino-cli upload --profile m5stack_fire -p /dev/cu.usbserial-XXXXXXXX .

# シリアルモニタ
arduino-cli monitor -p /dev/cu.usbserial-XXXXXXXX -c baudrate=115200
```

M5Stack Fire は CH9102 または CP210x の USB シリアルブリッジを搭載しており、macOS では `/dev/cu.usbserial-*` または `/dev/cu.wchusbserial*` として見えます。`arduino-cli board list` の Board Name 列が `Unknown` になるのは正常で、ポート検出とボード自動判別は別物です。macOS 11 以降はどちらのブリッジもカーネル内蔵ドライバで動作するため、サードパーティ製ドライバの導入は不要です。

## 操作方法

| 操作                      | 動作                                                                                  |
| ------------------------- | ------------------------------------------------------------------------------------- |
| ボタン A                  | 目標角度を +0.25°                                                                     |
| ボタン C                  | 目標角度を -0.25°                                                                     |
| ボタン B を押しながら起動 | キャリブレーションモード。ジャイロ補正後、B 押下で現在の角度を中心角として NVS に保存 |
| ボタン C を押しながら起動 | 充電モード                                                                            |

制御ゲインは [M5Bala2-Unified.ino](M5Bala2-Unified.ino) の先頭で定義しています。

```cpp
float kp = 24.0f, ki = 0.0f, kd = 90.0f;        // 角度 PID
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 0.0f; // 速度 PID
```

## ファイル構成

| ファイル                                                                                | 役割                                                      |
| --------------------------------------------------------------------------------------- | --------------------------------------------------------- |
| [M5Bala2-Unified.ino](M5Bala2-Unified.ino)                                              | メイン。初期化、PID タスク（84Hz）、ボタン処理            |
| [src/display.cpp](src/display.cpp) / [src/display.h](src/display.h)                     | M5Canvas による描画タスク（コア 0、20 FPS）               |
| [src/bala.cpp](src/bala.cpp) / [src/bala.h](src/bala.h)                                 | BALA2 ベースとの I2C 通信（速度指令、エンコーダ、サーボ） |
| [src/imu_filter.cpp](src/imu_filter.cpp) / [src/imu_filter.h](src/imu_filter.h)         | IMU 読み出しタスクと姿勢角推定                            |
| [src/MadgwickAHRS.cpp](src/MadgwickAHRS.cpp) / [src/MadgwickAHRS.h](src/MadgwickAHRS.h) | Madgwick 姿勢推定フィルタ                                 |
| [src/pid.cpp](src/pid.cpp) / [src/pid.h](src/pid.h)                                     | PID 制御器                                                |
| [src/calibration.cpp](src/calibration.cpp) / [src/calibration.h](src/calibration.h)     | ジャイロオフセットと中心角の NVS 保存                     |
| [sketch.yaml](sketch.yaml)                                                              | Arduino CLI のビルドプロファイル                          |
| [debug_config.h](debug_config.h)                                                        | シリアルテレメトリの有効/無効                             |
| [tools/gen_vscode_config.py](tools/gen_vscode_config.py)                                | 実ビルドから IntelliSense 設定を生成                      |

## タスクとコアの割り当て

ESP32 の 2 コアを、描画と制御で分けています。

| タスク | コア | 優先度 | 周期 | 役割 |
| ------ | ---- | ------ | ---- | ---- |
| `imu_task` | 1 | 5 | 1ms ポーリング | IMU 読み出しと姿勢推定（500Hz のサンプルを取得） |
| `pid_task` | 1 | 4 | 12ms | 角度 PID と速度 PID、BALA2 ベースへの I2C |
| `loop()` | 1 | 1 | 20ms | ボタン処理 |
| `display_task` | 0 | 1 | 50ms | M5Canvas による描画 |

描画をコア 0 に分離しているのは、320×240×16bpp のスプライト転送に約 45ms かかるためです。コア 1 に置くと 12ms の制御周期を確実に破綻させます。優先度を最低にしているのは、フレーム落ちは許容できても制御の遅延は許容できないためです。

フレーム周期を転送時間より長い 50ms にしているのは意図的です。40ms にすると描画タスクが休みなく回って PSRAM と SPI バスを占有し続け、制御周期のばらつきが実測で min 10333 / max 13678µs まで広がりました。50ms では min 11918 / max 12062µs に収まります。

スプライトは 150KB になるため PSRAM に確保しています。確保に失敗した場合は 8bpp にフォールバックします。描画タスクは I2C に触れません（`getAngle()` は専用のミューテックスで保護されており、I2C バスとは無関係です）。

## 調査用テレメトリ

[debug_config.h](debug_config.h) の `BALA_DEBUG_TELEMETRY` を 1 にすると、115200 baud で制御ループの状態を出力します。`BALA_DEBUG_STREAM` を 1 にすると 20Hz で 1 行ずつ、角度・エンコーダ・車輪速度・PID 各出力を流します。

```
angle=   2.18 d=+0.012 enc=   89536 speed=    0.00 pwm_angle=   20 pwm_speed=    0 out=   20
[imu] t=  2001ms rate=499 Hz raw=(  0.92  -0.61   0.67) corrected=(  0.58  -0.59   1.09) acc=(-0.04  0.03  1.00)
[pid] period avg=12000us min=11918us max=12062us n=84
[lcd] 20 fps (core 0)
```

制御周期そのものを調べるときは `BALA_DEBUG_STREAM` を 0 にしてください。20Hz の出力は 1 行約 95 バイトあり、115200 baud では送信待ちが PID タスクをブロックして、測りたい周期自体を歪めます。

## VSCode

`.vscode/` に設定を含めています。上流の `.gitignore` は `.vscode/` を除外していますが、本フォークでは必要な 3 ファイルのみ除外を解除しています。

- **タスク** — `Cmd+Shift+B` でビルド。「Tasks: Run Task」から書き込み、ポート一覧、シリアルモニタを実行できます
- **IntelliSense** — `.vscode/c_cpp_properties.json` が `build/compile_commands.json` を参照します

`c_cpp_properties.json` の `includePath` には ESP32 core の絶対パスが埋め込まれているため、環境が異なる場合やコア・ライブラリを更新した場合は再生成が必要です。

```bash
arduino-cli compile --profile m5stack_fire --build-path build .
python3 tools/gen_vscode_config.py
```

ESP32 core 3.x は大半のフラグを GCC のレスポンスファイル（`@file`）と `-iprefix` / `-iwithprefixbefore` で渡します。C/C++ 拡張はどちらも展開できないため、このスクリプトが実パスに解決したうえで書き出します。

`includePath` にワイルドカード（`**`）を使うのは避けてください。ESP32 core 内の別の `FreeRTOSConfig.h` や無関係なユーザーライブラリまで拾ってしまい、`#include` 行にエラーが出ます。実ビルドのインクルードパスをそのまま列挙するのが確実です。

## 上流からの変更点

- デバイス層を `M5Stack` ライブラリから M5Unified に移行し、ESP32 core 3.x / ESP-IDF 5.x でビルドできるようにした
- Arduino CLI のビルドプロファイル [sketch.yaml](sketch.yaml) を追加（M5Stack Fire 向けに固定）
- VSCode の設定（タスクと IntelliSense）とテレメトリを追加
- 描画を M5Canvas に移行し、コア 0 の専用タスクに分離（ちらつきの排除と、制御周期への干渉の回避）
- 背景画像 `src/bala_img.c` を削除して黒背景に（253KB の未使用データ、フラッシュ使用量が 41KB 減）
- スケッチ名をリポジトリ名に合わせて `M5Bala2.ino` から `M5Bala2-Unified.ino` に変更（Arduino CLI はディレクトリ名と `.ino` 名の一致を要求するため）

### M5Unified 移行に伴う挙動の変更

| 項目                   | 上流                                        | 本フォーク                                                             |
| ---------------------- | ------------------------------------------- | ---------------------------------------------------------------------- |
| IMU 読み出し           | MPU6886 の FIFO を直接バースト読み出し      | `M5.Imu.update()` によるポーリング                                     |
| IMU サンプリングレート | 500Hz（`SMPLRT_DIV=1`）                     | 500Hz を維持（M5Unified の既定 250Hz を明示的に書き換え）              |
| Madgwick の積分幅      | 500Hz 固定                                  | `imu_data.usec` から実測した間隔を毎回反映                             |
| 制御周期               | 5ms 指定・実測 11.9ms                       | **12ms を明示指定**（下記参照）                                        |
| ジャイロオフセット     | 生 ADC 値、NVS キー `gryo_*`                | deg/s、NVS キー `gyro_*`                                               |
| 満充電判定             | `M5.Power.isChargeFull()`                   | `M5.Power.getBatteryLevel() >= 100`（M5Unified に同等 API がないため） |
| BALA2 ベースとの I2C   | `M5.I2C.writeBytes` / `readBytes`（100kHz） | `M5.In_I2C.writeRegister` / `readRegister`（100kHz を維持）            |

### 制御周期を 12ms に固定している理由

上流のコードは PID タスクに 5ms を指定していますが、実機で計測すると**実際の周期は平均 11.9ms（84Hz）**でした。IMU タスクが MPU6886 の FIFO を 100kHz の I2C で一括読み出しする間 I2C ミューテックスを保持し続けるため、PID タスクが毎周期ブロックされていたためです。

M5Unified はポーリング方式で FIFO の一括読み出しがないため、この待ちが消えて制御ループが指定どおり 200Hz で回るようになります。これが 2 つの問題を生みました。

1. **ゲインの実効値が変わる** — [src/pid.cpp](src/pid.cpp) の D 項・I 項は周期で正規化されていません。`dinput` は周期に比例するため、周期が半分以下になると減衰を担う `kd=90` の効きが 2.4 分の 1 に低下します
2. **BALA2 ベースへの通信頻度が 2.4 倍になる** — 読み書き合わせて毎秒 168 回から 400 回に増えます。ベース側の STM32F030C8T6 が追従できず、速度指令を取りこぼします

実測では、同じ `out` に対する車輪の加速が上流の 7〜8 分の 1 に落ちていました。ゲインだけを周期比で換算しても改善せず、周期そのものを 12ms に戻して初めて上流と同等の安定性が得られました。したがって支配的な要因は 2 の通信頻度です。

制御周期を上げたい場合は、BALA2 ベースが追従できる I2C 頻度を先に確認してください。

> **キャリブレーションのやり直しが必要です**
> ジャイロオフセットの単位と保存キーが変わったため、上流で保存した値は読み込まれません。ボタン B を押しながら起動して、キャリブレーションを実行し直してください。中心角（`angle`）のキーは変更していないため、そのまま引き継がれます。

MPU6886 の加速度・ジャイロについて、M5Unified は M5Stack Fire で軸の入れ替えを行いません（反転するのは地磁気のみ）。単位も g / deg/s で上流の生値変換と一致するため、Madgwick に渡す値の意味は変わっていません。

## ライセンス

上流と同じ MIT ライセンスです。著作権表示は上流の M5Stack Technology CO LTD と、本フォークでの変更分に対する Yuya Iwata の連名です。詳細は [LICENSE](LICENSE) を参照してください。

上流のコードは相当部分がそのまま残っています。制御アルゴリズムの本体は上流のもので、[src/pid.cpp](src/pid.cpp) は変更していません。[src/MadgwickAHRS.cpp](src/MadgwickAHRS.cpp) は積分幅を可変にした変更のみです。BALA2 ベースの I2C プロトコル（レジスタ 0x00/0x10/0x20/0x30 の割り当てとバイト並び）も上流の実装です。

なお [src/MadgwickAHRS.cpp](src/MadgwickAHRS.cpp) は Sebastian Madgwick 氏（x-io Technologies）の実装が元になっており、ファイル冒頭にその旨が記載されています。上流がこれを含めて MIT で配布しているため、本フォークもその条件を引き継いでいます。
