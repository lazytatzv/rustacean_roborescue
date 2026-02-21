##!/bin/bash
set -e # エラーが出たら即座に止まる設定

if [ "$1" == "clean" ]; then
    echo "Clean build..."
    rm -rf build/ install/ log/
fi

# まず道具だけ作る
colcon build --packages-up-to rosidl_generator_rs

# これをしないと古いジェネレータが使われてしまう
source install/setup.bash

# 最後に残りをビルド
# パス指定しなくてもなんか探してくれるっぽい
colcon build 

echo "ビルド完了！"
