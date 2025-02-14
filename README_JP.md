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
**Int-Ball2**（JEM Internal Ball Camera 2 System）は、国際宇宙ステーション（ISS）の日本実験棟（JEM）内で動作するカメラロボットです。地上からの遠隔操作が可能で、また、ユーザー開発ソフトウェアをサポートすることで拡張機能を実現しています。このプラットフォームは、宇宙空間でのロボット技術の実証と開発を目的とした[きぼうロボットプログラミングチャレンジ（K-RPC）](https://jaxa.krpc.jp/)で頻繁に使用されています。**このリポジトリ**は、**Int-Ball2**用のROS + Gazeboベースのシミュレータを提供します。
- 無人機のようなロボットの動作とアルゴリズムのテストと検証が可能。
- カスタム機能とプラグインを統合するためのユーザープラットフォームを提供。
- セットアップの合理化を図るDockerサポートを提供（計画中または一部実装済み）。





## 主な機能
- **現実的なROSベースのフライトソフトウェア**：Int-Ball2用のセンサーデータ取得およびアクチュエーター制御インターフェースを個別のROSノードとして提供し、既存の機能（例：ビジュアルSLAM、センサーフュージョン）の選択的な有効化または無効化を可能にします。
- **Dockerによる柔軟なユーザープログラム統合**：コアのフライトソフトウェアを変更することなく、Dockerコンテナ内で新しい機能（ナビゲーションや制御アルゴリズムなど）を追加できます。既存のモジュールは、ユーザー開発の機能と組み合わせたり、置き換えたりすることができます。
- **地上支援装置（GSE）とのシームレスな統合**：現実的な運用ワークフローのためのテレメトリ受信とコマンド送信を可能にします。ユーザープログラムはGSEから起動または停止でき、既存のフライトソフトウェアと安全に共存できます。
- **Dockerによる開発と実行**：ユーザープログラムをホストシステムから分離し、nasm、ffmpeg、x264、VLC、Qtなどのソフトウェアの依存関係管理を簡素化します。
- **包括的なシミュレーション環境**：Gazebo を使用して、ほぼ現実的な Int-Ball2 モデルとオプションの ISS 環境を提供します。GUI により、シミュレーションの開始と停止が容易になります。
- **詳細なセットアップ手順と例**：OS/ROS/Python のインストールから GSE およびシミュレータの構築までをカバーします。ユーザープログラム用の ROS パッケージ構造と roslaunch テンプレートのサンプルを含みます。



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
「Int-Ball2技術実証プラットフォームユーザーマニュアル」の「4. 環境構築手順」を参照してください。

## プロジェクト構造
```.
├── Int-Ball2_platform_gse/ # 地上支援装置 S/W
│   └── ...
├── Int-Ball2_platform_simulator/  # 3D Simulator
│   └── ...
├── docs/ 
│   ├── manual/ 
│   │   ├── Int-Ball2 Technology Demonstration Platform User's Manual.pdf  # マニュアル
│   │   └── Int-Ball2ユーザプラットフォームマニュアル.pdf                     # マニュアル(JP)
│   └── ...
├── README.md
└── README_JP.md
```


## トラブルシューティング
* よくある問題 
  * 未定
* その他のサポート
  その他の問題が発生した場合は、GitHubにIssueを作成してください。


## 貢献
貢献は大歓迎です。
* Issue: バグの報告、機能のリクエスト、質問など。
* Pull Requests: 修正や新機能の提出。大きな変更の場合は、まず Issue を開いてください。
貢献に関するガイドラインやコーディング規約については、CONTRIBUTING.md (TBD) をご覧ください。


## ライセンス
本プロジェクトは Apache 2.0 ライセンスで配布されています。詳細は LICENSE ファイルをご覧ください。
このリポジトリは宇宙航空研究開発機構が提供しています。


## 参考文献
* [Wikipedia] [Int-Ball](https://en.wikipedia.org/wiki/Int-Ball)
* [NewsRelease] [Int-Ball2が宇宙に旅立ちました！, 2023](https://humans-in-space.jaxa.jp/news/detail/003155.html)
* [論文] [Int-Ball2: 自律型船内飛行およびドッキングによる画像撮影と充電の軌道上実証、2024年](https://ieeexplore.ieee.org/document/10813456)
* [論文] [Int-Ball2: ISS JEM Internal Camera Robot with Increased Degree of Autonomy – Design and Initial Checkout, 2024](https://ieeexplore.ieee.org/document/10688008)
* [論文] [ISS搭載型自律型フリーフライヤーロボットInt-Ball2のGNC設計および軌道性能評価、2024年](https://ieeexplore.ieee.org/document/10802183)
* [論文] [宇宙用フリーフライロボットの自律飛行システムのための補完的な地上試験方法、2024年](https://ieeexplore.ieee.org/document/10521401)
* [論文] [低推力アクチュエータによるフリーフライングロボットの全方位制御用ハードウェア・イン・ザ・ループ・シミュレータ、2023年](https://ieeexplore.ieee.org/document/10161499)
* [論文] [Int-Ball2: ISS JEM Internal Camera Robot with Increased Degree of Autonomy – Design and Initial Checkout, 2024](https://ieeexplore.ieee.org/document/10688008)
* [論文] [JEM船内可搬型ビデオカメラシステム実証2号機(Int-Ball2)による撮影作業の自動化, 2022](https://www.jstage.jst.go.jp/article/jsmermd/2022/0/2022_1P1-H07/_article/-char/ja/)
* [ウェブ] [ROS Melodic](https://wiki.ros.org/melodic)
* [ウェブ] [Gazebo 9.0.0 リリース](https://classic.gazebosim.org/blog/gazebo9)


## 今後の計画
未定
最新情報にご注目ください！




