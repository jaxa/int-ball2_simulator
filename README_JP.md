<div align="center"><img src="https://github.com/jaxa/int-ball2_simulator/blob/main/docs/image/ib2_mission_emblem.png" width="230"/></div>

# Int-Ball2 シミュレータ

<p style="display: inline">

  <img src="https://img.shields.io/badge/-Ubuntu_18.04_LTS-555555.svg?style=flat&logo=ubuntu">
  <img src="https://img.shields.io/badge/-ROS1--Melodic-%2322314E?style=flat&logo=ROS&logoColor=white">
  <img src="https://img.shields.io/badge/-Python-F2C63C.svg?logo=python&style=flat">
  <img src="https://img.shields.io/badge/-C++-00599C.svg?logo=cplusplus&style=flat">
  <img src="https://img.shields.io/badge/-Docker-1488C6.svg?logo=docker&style=flat">
  <img src="https://img.shields.io/badge/License-Apache--2.0-60C060.svg?style=flat">
</p>

---
## 概要
**Int-Ball2**（JEM自律移動型船内カメラ実証２号機）は、国際宇宙ステーション（ISS）の日本実験棟「きぼう」内で動作する船内ドローンです。
宇宙飛行士は地上の管制官と協力して様々な実験やISSのメンテナンス作業を実施しています。Int-Ball2は地上の管制官の操作によりISS内を飛び回り、写真や動画の撮影を宇宙飛行士の代わりに行います。現在、「きぼう」日本実験棟内での写真・動画撮影は、宇宙飛行士がカメラを自身で準備して実施していますが、地上からの遠隔操作でInt-Ball2が行うことで、その準備や撮影に要する宇宙飛行士の作業時間を軽減できます。

加えてInt-Ball2はその拡張機能としてユーザが開発した任意のソフトウェアを実行し、宇宙空間でロボット技術の実証のプラットフォームとして使用できます。

**このリポジトリ**は、**Int-Ball2**を活用したソフトウェア開発をするためのROS/Gazeboシミュレータを提供します。

## Int-Ball2シミュレータの主な機能
- **シミュレーション環境**：Gazebo シミュレータ上での Int-Ball2 と ISS 環境（エアフロー含む）の模擬。
- **ユーザー実証環境**：Int-Ball2用のセンサーデータ取得およびアクチュエーター制御インターフェースを個別のROSノードとして提供し、既存の機能（例：ビジュアルSLAM、センサーフュージョン）の選択的な有効化または無効化を可能にします。
- **ユーザー実証環境の地上支援装置（GSE）**：ユーザー実証プログラムを軌道上のInt-Ball2に適用し、地上から運用する際に使われる地上支援装置（GSE）の模擬。GUI でのプログラム選択・実行の運用模擬が可能。

## 必要条件
- **オペレーティングシステム**：Ubuntu 18.04 Bionic 
- **ROSバージョン**：ROS 1 Melody (Python3)
- **Gazeboバージョン**：Gazebo 9 
追加ライブラリ：

| Name | Version |
| ---- | ---- |
|NumPy|1.18.2|
|EmPy|3.3.4|
|NASM|2.15.05|
|FFmpeg|4.1.3|
|VLC|3.0.7.1|
|Qt|5.12.3|


## インストール
[INSTALL.md](https://github.com/jaxa/int-ball2_simulator/blob/main/INSTALL.md)

日本語版は「Int-Ball2技術実証プラットフォームユーザーマニュアル」の「4. 環境構築手順」を参照してください。

## プロジェクト構造
```.
├── Int-Ball2_platform_gse/ # 地上支援装置 S/W
│   └── ...
├── Int-Ball2_platform_simulator/  # 3D Simulator
│   └── ...
├── docs/ 
│   ├── manual/ 
│   │   ├── Int-Ball2 Technology Demonstration Platform User's Manual.pdf  # マニュアル（EN）
│   │   └── Int-Ball2ユーザプラットフォームマニュアル.pdf                     # マニュアル(JP)
│   └── ...
├── INSTALL.md
├── README.md
└── README_JP.md
```

## ライセンス
本プロジェクトは Apache 2.0 ライセンスで配布されています。詳細は LICENSE ファイルをご覧ください。
公序良俗に反する利用など社会通念上の不適切な利用を禁止します。
また、利用条件の変更、予告なく配布終了する可能性があります。




