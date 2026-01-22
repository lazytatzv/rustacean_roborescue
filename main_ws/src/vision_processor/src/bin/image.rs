use opencv::{Result, core::Vector, imgcodecs, prelude::*, wechat_qrcode::WeChatQRCode};
use std::env;

fn main() -> Result<()> {
    // 画像PATH
    // いまはproject root(cargo.tomlあるとこ)
    let img_path = "qrcode.png";

    println!("読み込み中: {}", img_path);

    // 2. 画像のロード
    let img = imgcodecs::imread(img_path, imgcodecs::IMREAD_COLOR)?;
    if img.empty() {
        eprintln!("エラー: 画像が見つかりません");
        return Ok(());
    }

    // 3. WeChat QRモデルの初期化
    // ダウンロードした models フォルダの中身を指定します
    let mut detector = WeChatQRCode::new(
        "models/detect.prototxt",
        "models/detect.caffemodel",
        "models/sr.prototxt",
        "models/sr.caffemodel",
    )?;

    // 4. QRコード検出の実行
    // pointsにはQRコードの四隅の座標が入ります
    let mut points = Vector::<opencv::core::Mat>::new();
    // 戻り値は検出された文字列のリストです
    let results = detector.detect_and_decode(&img, &mut points)?;

    // 5. 結果の表示
    if results.len() == 0 {
        println!("QRコードは見つかりませんでした。");
    } else {
        println!("=== 検出成功！ ({}個) ===", results.len());
        for (i, res) in results.iter().enumerate() {
            println!("QR[{}] の中身: {}", i + 1, res);
        }
    }

    Ok(())
}
