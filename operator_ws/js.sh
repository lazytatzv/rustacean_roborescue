#!/bin/bash

# /dev/js* デバイスが存在するか確認するスクリプト

if ls /dev/js* >/dev/null 2>&1; then
  echo "ジョイスティックデバイスが見つかりました:"
  ls /dev/js*
else
  echo "ジョイスティックデバイスは見つかりませんでした (/dev/js*)"
fi
