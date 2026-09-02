# np2_TAB5 — ライセンスと第三者著作物の表示

np2_TAB5 は PC-9801 エミュレータ **NP2kai** を **M5Stack Tab5**（ESP32-P4）へ
移植したものです。

- 作者: **mochimochi-man / うっ**
- 連絡先: **X / Twitter [@calorie0](https://x.com/calorie0)**

**本書は `LICENSE.md`（英語）の参考訳です。両者に相違がある場合は英語版が優先します。**

本書で言及するライセンスの全文は [`licenses/`](licenses/) に収録しています。

---

## 1. 要約

| | |
|---|---|
| 本プロジェクトが書いたコード | **MIT** — §2 |
| それ以外 | 各々のライセンス。**すべて寛容型（permissive）** — §4 |
| コピーレフト（GPL/LGPL）の混入 | **無し**。イメージにもこのアーカイブにも — §5 |
| NEC の PC-9801 ROM / BIOS / フォントデータ | **無し** — §3 |

ファームウェアイメージは結合著作物です。作者自身のコードは MIT ですが、それ以外は
各著作者のライセンスのままです。イメージやアーカイブを再配布するときは、それら全部を
同時に満たす必要があります。すべて寛容型なので負担は軽いものの、**このプロジェクト
全体が単に「MIT」なのではありません。** 実務上必要なことは §6 にまとめました。

---

## 2. 本プロジェクトが書いたコード

**MIT License / Copyright (c) 2026 mochimochi-man / うっ**（全文は英語版 §2 および
`LICENSE.md` を参照）

**対象**: `main/` と `rom_src/` の全体、ルートの `CMakeLists.txt`、
`sdkconfig.defaults`、`partitions.csv`、ビルドスクリプト。すなわち Tab5 移植の
本体 — MIPI-DSI パネルの立ち上げ、PPA によるブリット、メニュー、タッチをマウスに
する処理、USB マスストレージの各モード、BLE HID ホスト、GreaseWeazle Mode、
スクリーンショット出力、RTC 対応です。

**対象外 — `main/` の中の 2 ファイル:**

- `main/st7123_init_data.h`
- `main/ili9881c_init_data.h`

これらは M5Stack Tab5 BSP のパネル初期化データを**逐語複製**したもので、
**Apache-2.0** のままです（§4.5）。各ファイルの冒頭に出典を明記しています。

---

## 3. 意図的に含めていないもの

- **NEC の PC-9801 BIOS / ITF / CGROM データは一切含みません。**
  - イメージに内蔵した BIOS（`main/rom/BIOS_Compatible_O.ROM`）は独立実装で、
    本アーカイブの `rom_src/bios_compat_o.asm` からビルドできます。NEC のコードは
    含みません。
  - 内蔵フォント（`main/rom/FONT_ESP.ROM`）はパブリックドメインの東雲ビットマップ
    フォントから `rom_src/mkfont_esp.py` で生成したものです（§4.6）。実機の
    キャラクタジェネレータのダンプではありません。
  - 両方を内蔵しているのは、**空の SD カードでも起動できるようにするため**です。
    メニューから SD 上の実機吸い出し ROM を指定すればそちらが優先されますが、
    その入手は利用者の責任です。
- **エミュレート対象機のディスクイメージ・ゲーム・その他ソフトウェアは含みません。**

---

## 4. 第三者著作物

### 4.1 NP2kai / Neko Project II — エミュレータ本体

`components/np2kai/`

2 つのライセンスが適用され、どちらもツリーに同梱しています。

- **MIT**, Copyright (c) 2017 AZO — `components/np2kai/np2kai/LICENSE`
- **BSD 3-Clause**, Copyright (c) 1999-2025 NP2 developer team —
  `components/np2kai/np2kai/LICENSES/LICENSE.TXT`
  （同ディレクトリの他のライセンス文書に該当しないソースへ適用、と明記されています）

イメージにコンパイルされるもの: i286c（V30/286）CPU コア、`io`, `mem`, `vram`,
`fdd`, `font`, `generic`, `lio`, `bios`, `cbus`, `codecnv`, `common`, `trap`,
`diskimage`、および `sound/` 配下のネイティブ音源（OPNA/opngen, PSG, beep,
ADPCM, PCM86, TMS3631, リズム, CS4231, CT1741, OPL3）。

`font/fontdata` の内蔵 ANK 代替フォントは NP2 作者のオリジナルであり、
NEC / EPSON 実機からのダンプではありません。

上流: <https://github.com/AZO234/NP2kai>

未使用デバイスのヘッダが数点だけ include パス上に残っています。コア側が無効な
機能スイッチの内側で `#include` しているためです。いずれも QEMU 由来の **MIT** です。

- `wab/cirrus_vga_extern.h` — QEMU Cirrus CLGD 54xx VGA,
  Copyright (c) 2004 Fabrice Bellard, Copyright (c) 2004 Makoto Suzuki (suzu)
- `network/lgy98.h`, `network/lgy98dev.h` — QEMU NE2000,
  Copyright (c) 2003-2004 Fabrice Bellard

### 4.2 ESP-IDF v5.5 — Espressif Systems

**Apache-2.0。** ESP-IDF に `NOTICE` ファイルはありません。同ライセンス §4(a) が
再配布者に求めるライセンス全文の頒布のため、`licenses/Apache-2.0.txt` を同梱して
います。

ESP-IDF の中で独自条項を持ち、かつ本イメージに入るもの:

- **FatFs** — Copyright (C) 2022, ChaN。条件は 1 つだけで、
  **ソース再配布時に著作権表示を保持すること**。`licenses/FatFs.txt`
- **TinyUSB** — **MIT**, Copyright (c) Ha Thach (tinyusb.org)
- **miniz** — ESP32-P4 のブート ROM に入っており、スクリーンショット出力で使用。
  上流 miniz は **MIT**（Copyright 2013-2014 RAD Game Tools and Valve Software /
  Copyright 2010-2014 Rich Geldreich and Tenacious Software LLC）。
  宣言に使う ESP-IDF の `miniz.h` は Apache-2.0（Espressif）
- **Bluedroid** — ESP-IDF 同梱のもの、Apache-2.0
- **Newlib** および RISC-V ツールチェーンのランタイム — BSD 系

### 4.3 Espressif マネージドコンポーネント

ビルド時に ESP Component Registry から取得されます。版は `dependencies.lock` で
固定。**本アーカイブには含まれません**（ビルド時にダウンロードされます）。

**Apache-2.0**: `bmi270`, `cmake_utilities`, `eppp_link`, `esp_cam_sensor`,
`esp_codec_dev`, `esp_h264`, `esp_hosted`, `esp_io_expander`,
`esp_io_expander_pi4ioe5v6408`, `esp_lcd_ili9881c`, `esp_lcd_st7121`,
`esp_lcd_st7123`, `esp_lcd_touch`, `esp_lcd_touch_gt911`,
`esp_lcd_touch_st7123`, `esp_lvgl_port`, `esp_sccb_intf`,
`esp_serial_slave_link`, `esp_wifi_remote`, `i2c_bus`, `m5stack_tab5`,
`sensor_hub`, `usb`, `usb_host_cdc_acm`, `usb_host_hid`, `usb_host_uvc`,
`wifi_remote_over_eppp`

**MIT**: `tinyusb`, `esp_ipa`, `esp_video`

### 4.4 LVGL

**MIT**, Copyright (c) 2025 LVGL Kft。本プロジェクトが直接使うのではなく、
Tab5 BSP 経由で入ります。同梱の VGLite カーネルドライバ（Vivante Corporation,
2014-2022）は MIT/GPL-2.0 のデュアルライセンスで、ここでは **MIT** を選択します。

### 4.5 M5Stack Tab5 BSP — Espressif Systems

**Apache-2.0。** リンクされるほか、2 ファイルを**逐語複製**しており、出典表示を
保持する必要があります。

| 本プロジェクト内 | 複製元 |
|---|---|
| `main/st7123_init_data.h` | `espressif/m5stack_tab5` → `priv_include/disp_init_data_1.h` |
| `main/ili9881c_init_data.h` | `espressif/m5stack_tab5` → `priv_include/disp_init_data.h` |

### 4.6 東雲フォント — パブリックドメイン

内蔵 `FONT_ESP.ROM` の生成に `rom_src/mkfont_esp.py` が使用します。

- 作者: Yasuyuki Furukawa / 2001-2004 /efont/ Project
- 上流: <http://openlab.ring.gr.jp/efont/shinonome/>
- ライセンス: **パブリックドメイン。** 配布物には「すべてのフォントデータ、
  ドキュメント、スクリプト類はすべて Public Domain で提供」、ただし
  「日本に於いては現時点で著作権を放棄することは法律上不可能」であるため
  「AUTHORS に列挙されている作者がその権利を行使しないと宣言することで実質的な
  Public Domain とする」と明記されています。自由な改造・フォーマット変換・組込み・
  再配布が可能で、完全に無保証です。`licenses/Shinonome.txt`

同フォントの Debian パッケージング部分は GPL-2+ ですが、それはパッケージング
スクリプトのみに掛かるものです。本プロジェクトでは一切使用も配布もしていません。

### 4.7 Greaseweazle — パブリックドメイン（The Unlicense）

`main/gw_mode.cpp` は Greaseweazle プロトコルのホスト側を、参照実装をもとに
実装したものです。

- 作者: Keir Fraser
- 上流: <https://github.com/keirf/greaseweazle>
- ライセンス: **The Unlicense** —
  "This is free and unencumbered software released into the public domain."
  `licenses/Unlicense.txt`

表示義務はありません。敬意として記載しています。

### 4.8 libretro-common ヘッダ

`components/np2kai/np2kai/sdl/libretro/libretro-common/include/`

**MIT**, Copyright (C) 2010-2020 The RetroArch team。ヘッダのみで、libretro の
コードは一切コンパイルしていません。NP2kai の `compiler.h` がこのディレクトリに
あり、全コアソースがそれを include するため残しています。

---

## 5. 何を削除したか、なぜか

コピーレフトのコードは、**ファームウェアにも、このアーカイブにも一切ありません。**
それは自然にそうなったのではなく削除の結果なので、検証できるよう一覧にします。

NP2kai ツリーから削除したもの（いずれも本ビルドでは未使用）:

| 削除 | 理由 |
|---|---|
| `sound/fmgen/` | cisc 氏の FM 音源コア。独自ライセンス |
| `sound/mame/` | MAME OPL — **GPL** |
| `sound/mamebsd/`, `sound/mamebsdsub/` | ymfm（3条項BSD）、未使用 |
| `sound/vermouth/` | GM/MIDI ソフトシンセ、未使用 |
| `i386c/` | IA-32 コア。DOSBox 由来の FPU コード（**GPLv2**）を含む |
| `sdl/cmmidi.c` | MIDI 出力、未使用 |
| `sdl/` 配下の `.c` すべて | SDL/libretro フロントエンド、未使用。これにより `sdl/libretro/rsemaphore.c`（**GPL-2.0-or-later**）が除去された |
| `sdl/libretro/libretro-common/include/uwp/` | Windows UWP 用 — **GPL** |
| `wab/tgui9680*.h` | Trident TGUI9680 — **GPL**。`wab/` の残りは MIT でヘッダのみ保持 |
| `wab/*.c`, `network/*.c` | 未使用のデバイスモデル（MIT）。ヘッダは §4.1 の理由で保持 |
| `i286x/`, `i386hax/*.c`, `misc/`, `textnorm/`, `accessories/`, `jni/`, `sample/`, `tests/`, `vst3sdk/` | 未使用 |
| `LICENSES/LICENSE-C86CTL.TXT`, `-SCCI.TXT`, `-TGUI9680.TXT`, `-ZLIB.TXT` | 対象コードがアーカイブに存在しないライセンス文書。特に `-TGUI9680.TXT` は GNU GPL v2 であり、存在しないファイルに条件を主張することになるため |

保持: `LICENSES/LICENSE.TXT`（コア本体の 3 条項 BSD）、および
`LICENSE-GD54XX.TXT` / `LICENSE-LGY98.TXT`（MIT。§4.1 のヘッダに今も掛かるため）。

本アーカイブに対する `grep -ril "GNU General Public"` は何も返しません。

---

## 6. 再配布するとき

関係するライセンスはすべて寛容型なので、必要なことは多くありません。

1. **本書と `licenses/` を一緒に渡してください。**
2. **Apache-2.0 のライセンス全文を同梱してください。** ESP-IDF・Tab5 BSP・
   Espressif コンポーネントを含むバイナリ、つまり本プロジェクトのあらゆるビルドが
   対象です。`licenses/Apache-2.0.txt` がそのために置いてあります。
3. **著作権表示を保持してください。** §2 に挙げた複製 2 ファイル、および NP2kai の
   `LICENSE` と `LICENSES/` を含みます。
4. Apache-2.0 の成果物を改変した場合は**変更した旨を明示**してください。
5. 派生物の推奨・宣伝に**作者やプロジェクトの名前を使わない**でください
   （BSD 3-Clause、§4.1）。

**独自の追加部分を公開する義務はありません。** ソース公開を強制する条項は
どこにもありません。

---

## 7. 商標

PC-9801、PC-98、NEC は日本電気株式会社の商標です。M5Stack、Tab5 は
M5Stack Technology Co., Ltd. の商標です。ESP32、ESP32-P4、ESP-IDF は
Espressif Systems の商標です。Windows は Microsoft Corporation の商標です。
本書ではいずれも、このソフトウェアが何の上で動き何をエミュレートするかを
述べるためだけに使用しています。本プロジェクトはこれらのいずれとも提携・
承認・後援の関係にありません。

## 8. 無保証

上記のすべてのライセンスが一切の保証を否認しており、本プロジェクトも同様です。
これはハードウェアを直接制御する趣味のエミュレータです。**自己責任でご利用ください。**
