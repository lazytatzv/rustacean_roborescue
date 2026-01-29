#!/bin/bash
set -e # エラーが出たら即座に止まる設定

echo "🧹 (Option) クリーンビルドしたい場合は 'clean' を引数につけてね"
if [ "$1" == "clean" ]; then
    echo "🗑️  以前のビルドを削除中..."
    rm -rf build/ install/ log/
fi

echo "🚀 [Stage 1] ジェネレータ（道具）を先にビルドします..."
# まず道具だけ作る
colcon build --merge-install --packages-up-to rosidl_generator_rs

echo "⚡ [Source] 新しい道具を装備中..."
# これをしないと古いジェネレータが使われてしまう
source install/setup.bash

echo "🚀 [Stage 2] メッセージとドライバーを一気にビルドします..."
# 最後に残りをビルド
export ROSIDL_GENERATOR_RUST=ON
# パスしていしなくてもなんか探してくれるっぽい
colcon build --merge-install --symlink-install --packages-up-to flipper_driver

echo "✅ ビルド完了！お疲れ様でした！"
