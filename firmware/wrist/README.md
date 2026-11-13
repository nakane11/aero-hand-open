# 手首 磁気センサファームウェア

M5Stack AtomS3 と MLX90393 磁気センサ用のファームウェアです。

- 磁場の X,Y,Z を `x,y,z` の CSV で、約 50Hz で USB シリアル（115200 baud）に出力します。
- 本体ボタン（画面）を押すたびにオン/オフが切り替わり、`BTN,1`（ON）/`BTN,0`（OFF）を同じシリアルに出力します。状態が変わったときと、1 秒ごとに送ります。画面は ON で緑、OFF で灰色になります。
- `BTN` 行はカンマ区切りが 2 要素なので、`x,y,z` だけを読む既存ノード（`aero_hand_magnetic_sensor.py` など）は無視します。
- センサは Grove ポートにつなぎます（SDA=2, SCL=1、I2C アドレス 0x18）。
- 出力は `ros1/scripts/aero_hand_magnetic_*.py`（`aero_hand_magnetic_cmd_vel_node.py` など）が読みます。

## 書き込み

```bash
cd firmware/wrist
pio run -t upload
pio device monitor   # 動作確認
```
