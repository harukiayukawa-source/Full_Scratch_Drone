# Full_Scratch_Drone
## 概要
キットを購入するのではなく<br>
- KiCADで設計し表面実装したESP32搭載制御基板
- 3DCADで設計し3Dプリンタで造形したフレーム
- AmazonやRCヘリドローン専門ショップで購入したモータ・プロペラ・LiPoバッテリ

を用いてフルスクラッチのドローンを製作しました。

### 制御基板
- 構成 → MCU(マイコン)＋IMU(9軸慣性センサ)＋ESC(MOSFET×4個)＋コネクタなどを1枚の基板に搭載しました。
- 部品 → ESP32-MINI-1-N4(MCU)、ICM-20948(IMU)、AO3400A(Nch MOSFET)などなど。
- 購入 → 秋月電子通商　様やcorestaff　様で購入しました。一部のパーツはAmazonで購入しました。
- 基板 → SnapMagicなどでパーツのデータを収集し、KiCADで回路図及びPCB基板を設計後、JLCPCBに発注しました。
- 実装 → ホットプレートやペーストはんだを用いて表面実装しました。

### フレーム
- 設計 → Fusionで設計しました。
- 製造 → 3Dプリンタ(RAISE3D E2　ノズル径0.4mm　フィラメントPLA黒色)で造形しました。

### モータ
- 部品 → 「[直径8.5mm　長さ20mm　3~5V](https://amzn.asia/d/07lUeh3T)
- 購入 → Amazonで購入しました。

### プロペラ
- 部品 → [マイクロクワッド用55mm Blade Propeller Prop 4pcs](https://wda-jp.com/shop/products/detail/863)
- 購入 → 若狭小浜ドローン協会(WDA) 様の公式オンラインショップで購入しました。

### LiPoバッテリ
- 部品 → [GNB POWER LIPO 550mAh 1s 3.8V 100C コネクタ種：A30](https://helimonster.jp/?pid=176671622)
- 購入 → ヘリモンスター　様で購入しました。

### 制御コード(フライトコントローラ：ドローン側)
- VSCodeでコーディングし、VSCodeの拡張機能であるPlatformIOでコンパイルとビルドを行いました。

### 制御コード(プロポーショナルコントロール送信機：スマホアプリ側)
- MIT App Inventorというブラウザタイプのアプリ開発ツールを用いてスマホをドローンのプロポにしました。
- 詳細は別のリポジトリ[Example_usingMITAppInventor_Drone_Controller]()をご覧ください。

