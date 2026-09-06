# np2 TAB5

![np2 TAB5](cover.png)

本プログラムは、AZO234 氏が開発された PC-9801 エミュレータ **np2kai** をベースに
開発した ESP32-S3 用 PC-9801 エミュレータ **np2 espresso** をさらにフォークした、
**M5Stack Tab5 用 PC-9801 エミュレータ**です。

English: [README.en.md](README.en.md)

---

## 1. 特徴

### 互換 ROM 内蔵

新規に開発した簡易 BIOS とパブリックドメインのフォントを内蔵しています。
**実機の ROM が無くても、空の microSD カードだけで起動します。**

### USB キーボード / マウス対応

ただし、ESP-IDF が未対応のため、**USB キーボードと USB マウスを同時に使用することはできません。**

### Bluetooth キーボード / マウス対応

BLE のみ対応。パスコード入力を要するタイプの機器には対応しません。

### マウスエミュレートタッチパネル

タッチパネルがマウスとして動作します。

### スクリーンショット機能

画面のハードコピーを microSD に PNG 形式で保存します。

### SD Card Reader モード

Tab5 に挿入された microSD を PC のフォルダとしてマウントできます。
転送速度は 0.6MB/s 程度と非常に遅いです。

### Disk Image Reader モード

マウント中の HDD ディスクイメージを PC のフォルダとしてマウントできます。
転送速度は 0.6MB/s 程度と非常に遅いです。

### GreaseWeazle モード

Tab5 の USB-A ポートに接続した GreaseWeazle 経由で、ドライブ実機に挿入したディスクを
マウントします。詳細は [3. 起動方法の GreaseWeazle モード](#greaseweazle-モード) を
参照してください。

---

## 2. ビルド方法

### (1) ビルド

素の ESP-IDF プロジェクトです（Arduino コア・PlatformIO は使いません）。

**ESP-IDF v5.5 が必要です。v6.x ではビルドできません。**

```sh
git clone https://github.com/<you>/np2_TAB5.git
cd np2_TAB5
. $HOME/esp/esp-idf/export.sh     # v5.5 を入れた場所
idf.py build
```

`idf.py set-target` は不要です（`sdkconfig.defaults` でターゲット指定済み）。
初回ビルドは `dependencies.lock` で固定されたコンポーネントを取得するため
数分かかります。

`./build.sh` でも同じことができます（ESP-IDF を自動で探します）。

### (2) 書き込み

```sh
idf.py -p <PORT> flash monitor
```

`<PORT>` は Linux なら `/dev/ttyACM0`、macOS なら `/dev/cu.usbmodem*`、
Windows なら `COMn` です。Tab5 の USB-C ポートは USB-Serial-JTAG として見えます。
モニタは `Ctrl-]` で抜けます。`./flash.sh [PORT]` でも同じです。

#### ツールチェーン無しで書き込む場合

ビルド済みバイナリが [`bin/`](bin/) に、結合イメージが [`firmware/`](firmware/) に
あります（結合イメージの妙なファイル名は M5Burner の命名規則です）。

結合イメージ 1 つを書く場合（ブートローダ・パーティションテーブル・アプリを
すべて含みます）:

```sh
esptool --chip esp32p4 -p <PORT> write-flash 0x0 firmware/np2_TAB5_0x0.bin
```

4 分割を書く場合:

```sh
esptool --chip esp32p4 -p <PORT> -b 921600 write-flash \
    --flash-mode qio --flash-freq 80m --flash-size 16MB \
    0x2000  bootloader.bin \
    0x8000  partition-table.bin \
    0xe000  ota_data_initial.bin \
    0x10000 np2_espresso.bin
```

**ブートローダも `qio` モードで書いてください。** `dio` だとエミュレータが
目に見えて遅くなります。

#### 設定は再書き込みでは消えません

ディスク選択・CPU クロック・バックライト・音量・Bluetooth のペアリング情報は
NVS にあり、書き込みでは消えません。完全に初期状態から始めるには:

```sh
esptool --chip esp32p4 -p <PORT> erase-flash    # そのあと上記の書き込み
```

ファームウェアはそのままで設定だけ消すには:

```sh
esptool --chip esp32p4 -p <PORT> erase-region 0x9000 0x5000
```

---

## 3. 起動方法

ディスクイメージ（`.NFD` または `.NHD` など）を FAT32 でフォーマットした microSD
カードのルートに入れ、Tab5 に挿入して電源を入れると、内蔵の互換簡易 BIOS が
起動します。

**F11 / F12 / Pause** のいずれかで内蔵メニューが開きます。

FDD1 / FDD2 / HDD に任意のディスクイメージを指定し、**RESET** でメニューを抜けると、
指定したディスクイメージをマウントして再起動します。

実機から吸い出した `BIOS.ROM` や `FONT.ROM` がある場合は、microSD のルートに置いて
メニューで指定してください。内蔵 ROM より優先されます。

### Bluetooth 機器の接続

キーボードとマウスの両方が接続されるまで、**待ち受けは止まりません。時間制限は
ありません。**

- 電源投入後でも、しばらく経ってからでも、機器の電源を入れれば約 1 秒で繋がります。
- 休止している機器は、キーを押す・マウスを動かすなどして起こしてください。
- キーボードとマウスが揃った時点で待ち受けを停止します。

### GreaseWeazle モード

Tab5 の USB-A ポートに接続した GreaseWeazle 経由で、ドライブ実機に挿入したディスクを
直接マウントします。使用するときは、メニューの FDD1 / FDD2 から GreaseWeazle を
マウントしてください。

- GreaseWeazle は 2 台まで接続できます。その場合は**セルフパワーの USB ハブ**を
  使用してください。
- **ドライブの電源は別途確保してください。** 5 インチドライブは外部電源が必要です。
- ディスクを入れ替えた場合は、メニューで一度イジェクトしてから再マウントする
  必要があります。
- **コピープロテクトには対応していません。**

GreaseWeazle に接続するドライブのジャンパピンは、以下のように設定してください。
複数あっても同一に設定します。

```
DX:1
MON:1
USE:2
RD:1
HDE:1
DEN:1
```

---

## 4. 留意事項

- M5Stack Tab5 は、最新ビルド現在、液晶に
  **ILI9881C (ver.1) / ST7123 (ver.2) / ST7121 (ver.3)** のいずれかが使用されています。
  ILI9881C (ver.1) は、実機で確認できていないため、未対応です。
- MIDI 関係は、RAM に収まらなかった都合上、ベースとなった np2kai からすべて
  オミットしています。

---

## 5. ライセンス

内蔵の互換簡易 BIOS を含む本プログラム固有のライセンスは **MIT ライセンス**です。

ただし、採用しているライブラリやソースごとに固有のライセンスがあります。
詳細は **[LICENSE.jp.md](LICENSE.jp.md)**（日本語）または
**[LICENSE.md](LICENSE.md)**（英語・正式版）を参照してください。

---

Copyright 2026 mochimochi-man / うっ — X: [@calorie0](https://x.com/calorie0)
