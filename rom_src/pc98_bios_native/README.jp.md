# PC98N 独立互換BIOS

PC98Nは、PC-9801系およびEPSON PC-486互換機向けのクリーンルーム実装ROM BIOSです。ROM内の16ビットコードが直接ハードウェアを初期化し、割り込みサービスとブート処理を実行します。np2kai、ESP-IDF、ホスト側フック、実機BIOSのコードやデータには依存しません。

## 対応範囲

- V30、80186、80286、i386、i486以降のリアルモードCPU
- 8259A PIC、8237A DMA、8253 PIT、8251キーボード、uPD7220 GDC
- INT 09hキーボード、INT 18hの基本キーボード／画面機能、INT 1Bhディスク、INT 1Chタイマー／RTC
- uPD765Aによる1.25 MB、1.44 MB、720 KBフロッピー。最大4台を走査
- PC-98標準SASI（I/O 80h/82h）の256バイトセクタ読み書き。2台を走査してブート
- PC-98 IDEのATA LBA28読み書き。マスターとスレーブを走査し、ATA IDENTIFYから媒体固有のCHSジオメトリを取得
- PC-9801-55/L/U互換SCSI（WD33C93、I/O CC0h/CD0h/CE0h/CF0h）の非同期PIO読み書きとブート。EPSONスタートアップが使うINT 1Bh/AH=B0hのINQUIRY、READ CAPACITY、PREVENT/ALLOWにも対応
- D2000hにPC-9801-55互換の拡張SCSI ROMがある場合は、その標準INT 1Bh入口を初期化して通常のSCSIディスク操作に使用。ない場合は内蔵WD33C93ドライバを使用
- IDE／SASI／SCSI INT 1BhのCHS指定と線形ブロック指定。SCSIのDA/UAは20h/A0h

PC-98互換I/Oとメモリ配置を持つ環境なら、特定エミュレータ用のコールバックなしで実行できます。IBM PC/ATはハードウェア仕様が異なるため対象外です。エミュレータが独自のROMサイズや配置を要求する場合に備え、同一コードの96 KiB、64 KiB、32 KiB版を生成します。

## ビルド

必要なのはNASMとPython 3だけです。

Windows PowerShell:

```powershell
./build.ps1
```

Linux、macOS、WSL、BSD系シェル:

```sh
./build.sh
```

生成物:

| ファイル | ROMの物理配置 | 用途 |
|---|---:|---|
| `PC98N.ROM` | E8000h-FFFFFh | 標準の96 KiB BIOS.ROM |
| `PC98N_F000.ROM` | F0000h-FFFFFh | 64 KiBを要求するローダー |
| `PC98N_F800.ROM` | F8000h-FFFFFh | 32 KiBをF8000hへ配置するローダー |

通常は`PC98N.ROM`を`BIOS.ROM`として使用します。短縮版は、ローダーが表の開始アドレスへ明示的にマップする場合だけ使用してください。

BIOS自身による起動時の識別文字列表示と、そのための1秒待機は行いません。代わりに、EPSON PC-486NAV実機BIOSでメーカー文字列が始まる物理アドレス`F3274h`へ、ASCII識別子`Compatible BIOS by Codex`を格納します。`F3274h`を含まない32 KiB版でも確認できるよう、`FFFC0h`にも同じ識別子をミラーしています。通常BIOSの入口はF800:0000、F800:0004、F800:0008のいずれにも対応します。

### DOS上で実行中のBIOSを確認する

ビルド時に16ビットDOS実行ファイル`BIOSCHK.COM`も生成されます。DOSから次のように実行してください。

```dos
BIOSCHK
```

新版PC98Nを検出すると、次のように表示します。

```text
BIOSCHK 1.0 - active PC-98 ROM check
Active BIOS: Compatible BIOS by Codex
Marker address: F3274h (PC-486NAV vendor-text slot)
```

32 KiB版ではアドレス行が`FFFC0h (32 KiB ROM mirror)`になります。旧版の`PC98NATIVE01`を検出した場合は旧PC98N、どちらの識別子もなければ`Active BIOS: another BIOS`と表示します。終了コードは新版が0、未検出が1、旧版が2です。プログラムは現在DOSから見えているROMを読むだけで、ROMバンクやBIOSメモリを書き換えません。

`verify_rom.py`はサイズ、F3274hとFFFC0hの識別子、識別子以外の未使用領域、リセットベクタ、8ビットチェックサムを検査します。NASMは全警告をエラー扱いにしているため、16ビットI/O命令でポート番号が切り詰められるようなROMは生成されません。

## 現在の制限

これはDOS起動に必要な範囲を優先した互換BIOSで、NEC／EPSON純正BIOSの完全代替ではありません。

- N88-BASIC、IDE CD-ROM、ネットワークブートは未実装
- SASI/SCSIのフォーマット、SCSIの同期転送・切断再選択・DMA、INT 1Bh C0h系低レベルSCSIコマンドは未実装
- PC-9801-92/100などWD33C93を使わないSCSIボードは未対応。PC-9801-55系SCSIのBIOS CHSは8ヘッド×17セクタ、論理セクタ長は512バイト
- INT 1Fh拡張BIOS、LIO、全グラフィックBIOS機能は未実装
- 機種固有の電源管理、PCカード、ノート固有LCD制御は未実装
- 日本語表示には環境側のFONT.ROMまたはCG実装が必要

EPSONのHDD IPLが参照するHDD装備ビット（SASI／IDEは0000:055Dh、SCSIは0000:0482hと0460h以降の記述子）は、IPLを正常に読み込んだ時点で設定します。これにより、EPSONスタートアップが未押下のSHIFTによるメニュー表示と同じ状態へ落ちることなく、自動起動対象を検出できます。

したがって、どのエミュレータ／実機でも無条件に動作すると断言するものではありません。移植性の意味は、PC-98互換ハードウェアだけを利用し、np2kai固有機能を一切呼ばないことです。実機へROMを書き込む前に、対象機のROM電圧、バンク構成、配線を必ず確認してください。

## ソースの由来

実機BIOS ROMを逆アセンブルして作ったものではありません。各デバイスの公開レジスタ仕様と、公開されているPC-98のI/Oポート、BIOSワークエリア、割り込み呼出し規約に基づく独自実装です。SCSI制御はWD33C93の公開レジスタ仕様と公開Linux/98ドライバのハードウェア初期化手順を資料として、ROM用のポーリング状態機械を新規実装しています。

JISキーボード変換値は機能上の互換性を保ちつつ、ROM内では走査コード優先の独自マトリクスとして保持し、起動時にRAM上のBIOS形式へ転置します。実機BIOSとの比較では、単調なFF／00領域を除く32バイト以上の完全一致はありません。
