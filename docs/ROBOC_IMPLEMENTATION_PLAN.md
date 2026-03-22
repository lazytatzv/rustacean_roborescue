# RoboCup Rescue Implementation Plan

概要
---
このドキュメントは `docs/RoboCupRescue-Rules-2026B.txt` の要件を受け、
リポジトリ (`main_ws`) に対して優先度付けされた実装計画と見積りを示します。

優先順位（推奨）
1. Autonomy / Mapping（高）
2. Mobility 安定化（高）
3. Dexterity（中）
4. Sensing / Detection パイプライン（中）
5. CI・テスト・運用（高）

マイルストーン & 見積り（概算、エンジニア日）
- M0: 準備・CI 整備（dev-shell 確認、colcon/CIジョブ） — 1-2d
- M1: Mapping PoC — SLAM→PLY 出力 + CSV 検出出力（perception/ sensor_gateway） — 4-6d
- M2: Mobility 安定化 & 最小 URDF テスト（arm_controller 統合テスト） — 3-5d
- M3: Dexterity PoC（線形/omniタスクの自動化、挿入/押しボタン） — 4-6d
- M4: Sensing パイプライン（QR/hazmat/thermal/motion） — 4-6d
- M5: 統合テスト・評価ツール（GE/CV/LE/DS スコア算出器） — 3-4d
- M6: 最終 CI 統合 + ドキュメント・提出フォーマット生成（GeoTIFF/PLY/CSV） — 2-3d

合計見積り（ラフ）: 18–30 エンジニア日（優先順で段階的に進行）

短期アクション（私が着手する項目）
1. `docs/` に本ファイルを追加（完了）
2. ブランチ作成: `feature/robocup-mapping-poc`
3. `main_ws/src/perception/robo_map_exporter` スケルトンを追加（ROS2ノード: SLAM入力→PLY出力）
4. CSV 検出出力のインターフェース仕様を作成（`docs/templates/pois-template.csv`）
5. CI に短時間で回る smoke-test ジョブを追加（`colcon build --packages-select perception`）

長期タスク（概要）
- 完全な SLAM パイプライン評価（PCL/ICP 整列、スケール検証）
- 検出アルゴリズムの精度向上（hazmat, thermal, qr）
- 自律ナビゲーション（経路計画、復旧ロジック、通信劣化対策）
- 実機試験とスコアリング自動化

次の私の作業（即時）
1. ブランチ `feature/robocup-mapping-poc` を作成します。
2. `main_ws/src/perception/robo_map_exporter` のベースパッケージ（CMake/Rust/py）スケルトンを追加します。
3. `docs/templates/pois-template.csv` を作成します。

---
注: 見積りは現状のリポジトリの実装度合いに依存します。詳細見積りは各マイルストーンで再評価します。

連絡: まず `M1` の PoC（SLAM→PLY + CSV 出力）を開始します。進捗は小刻みに報告します。
