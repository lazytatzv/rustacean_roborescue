use opencv::{
    core::Vector,
    imgcodecs,
    prelude::*,
    wechat_qrcode::WeChatQRCode,
    Result,
};
use std::path::PathBuf; // パス操作用

fn main() -> Result<()> {
    // ★魔法の呪文: パッケージのルートディレクトリを取得
    let root_dir = env!("CARGO_MANIFEST_DIR");
    let root = PathBuf::from(root_dir);

    // パスを結合して絶対パスを作る
    let img_path = root.join("qrcode.png");
    let model_dir = root.join("models");

    println!("読み込み中: {:?}", img_path);

    // 2. 画像のロード (パスを文字列に変換して渡す)
    let img = imgcodecs::imread(
        img_path.to_str().unwrap(),
        imgcodecs::IMREAD_COLOR
    )?;

    if img.empty() {
        eprintln!("エラー: 画像が見つかりません: {:?}", img_path);
        return Ok(());
    }

    // 3. WeChat QRモデルの初期化
    let mut detector = WeChatQRCode::new(
        model_dir.join("detect.prototxt").to_str().unwrap(),
        model_dir.join("detect.caffemodel").to_str().unwrap(),
        model_dir.join("sr.prototxt").to_str().unwrap(),
        model_dir.join("sr.caffemodel").to_str().unwrap(),
    )?;

    // 4. QRコード検出の実行
    let mut points = Vector::<opencv::core::Mat>::new();
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
