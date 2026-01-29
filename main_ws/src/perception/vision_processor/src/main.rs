use opencv::{
    Result,
    core::{Mat, Point, Scalar, Vector}, // Matもここでuseに追加しておきました
    highgui,
    imgproc,
    prelude::*,
    videoio,
    wechat_qrcode::WeChatQRCode,
};

fn main() -> Result<()> {
    // 1. WeChat QRモデルの初期化
    let mut detector = WeChatQRCode::new(
        "models/detect.prototxt",
        "models/detect.caffemodel",
        "models/sr.prototxt",
        "models/sr.caffemodel",
    )?;

    // 2. カメラの起動
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;

    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("カメラを開けませんでした。");
    }

    highgui::named_window("Real-time WeChat QR", highgui::WINDOW_AUTOSIZE)?;

    println!("カメラ起動中... 'q' キーで終了します。");

    let mut frame = Mat::default();

    loop {
        // 3. フレーム読み込み
        cam.read(&mut frame)?;
        if frame.empty() {
            continue;
        }

        // 4. 検出実行
        let mut points = Vector::<Mat>::new();
        let results = detector.detect_and_decode(&frame, &mut points)?;

        // 5. 結果の描画
        for (i, content) in results.iter().enumerate() {
            let pt_mat = points.get(i)?;

            // --- 修正箇所: * をつけてデリファレンスしてからキャスト ---
            let p0 = Point::new(*pt_mat.at::<f32>(0)? as i32, *pt_mat.at::<f32>(1)? as i32);
            let p1 = Point::new(*pt_mat.at::<f32>(2)? as i32, *pt_mat.at::<f32>(3)? as i32);
            let p2 = Point::new(*pt_mat.at::<f32>(4)? as i32, *pt_mat.at::<f32>(5)? as i32);
            let p3 = Point::new(*pt_mat.at::<f32>(6)? as i32, *pt_mat.at::<f32>(7)? as i32);
            // ----------------------------------------------------

            // 緑色の線 (BGR: 0, 255, 0)
            let color = Scalar::new(0.0, 255.0, 0.0, 0.0);
            let thickness = 2;

            imgproc::line(&mut frame, p0, p1, color, thickness, imgproc::LINE_8, 0)?;
            imgproc::line(&mut frame, p1, p2, color, thickness, imgproc::LINE_8, 0)?;
            imgproc::line(&mut frame, p2, p3, color, thickness, imgproc::LINE_8, 0)?;
            imgproc::line(&mut frame, p3, p0, color, thickness, imgproc::LINE_8, 0)?;

            // テキスト表示
            let text_pos = Point::new(p0.x, p0.y - 10);
            imgproc::put_text(
                &mut frame,
                &content,
                text_pos,
                imgproc::FONT_HERSHEY_SIMPLEX,
                0.8,
                Scalar::new(0.0, 0.0, 255.0, 0.0), // 赤色
                2,
                imgproc::LINE_AA,
                false,
            )?;

            println!("検出: {}", content);
        }

        // 6. 表示
        highgui::imshow("Real-time WeChat QR", &frame)?;

        if highgui::wait_key(1)? == 113 {
            // 'q' key
            break;
        }
    }

    Ok(())
}
