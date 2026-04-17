# armの制御用パッケージ

* オペレーター・アームの操作・rvizの表示のプログラムのセット

## node

* demo_arm_ctl

  アームの制御ノード、dynamixel workbenchを扱ってる
* dxl_test

  テスト用ノード、使わない
* demo_arm_state

  アームのモデルを表示させるためのtopicを吐き出すノード
* demo_arm_operator

  joyスティックの情報を制御ノードに渡すノード、_廃止済み_

## launch

* demo_arm_from_joy.launch

  使わない、operatorが廃止済みのものを使用
* demo_joy.launch

  joyスティックのみのノードを起動、使用可能

* display.launch

  demo_arm_stateやrvizを起動してくれる、使用可能

* test_arm.launch

  使わない

## アームの操作方法

* 順運動モード(6)
    * SELECT押しながら右スティックを上下に動かして、modeを「6」にする
    * 左スティック-左右： 根本の左右
    * 左スティック-上下： 根本の上下
    * 右スティック-上下： 肘部分の上下
    * 左矢印ボタン-上下： 手首の上下
    * 左矢印ボタン-左右： 手首の左右調整
    * L1,R1： 手首回転
    * L2,R2： 指の開閉
* 逆運動モード(2,3,4,5)
    * SELECT押しながら右スティックを上下に動かして、modeを「6」にする
    * mode2から(指先自由、指先上向き、指先下向き、指先水平)モード
  > 指先の上、下への移動の際、自分の機体や周辺環境にかかわらず移動するため 注意が必要
    * 左スティック-左右： 指先目標y座標変更(横)
    * 左スティック-上下： 指先目標x座標変更(奥行き方向)
    * 右スティック-上下： 指先目標z座標変更(高さ)
    * 左矢印ボタン-上下： 手首の上下
    * 左矢印ボタン-左右： 手首の左右調整
    * L1,R1： 手首回転
    * L2,R2： 指の開閉

## ~~requirements~~

必要なくなりました

```bash 
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux64
make & sudo make install
```