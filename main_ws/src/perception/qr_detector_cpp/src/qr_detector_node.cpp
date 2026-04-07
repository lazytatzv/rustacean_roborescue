#include "qr_detector_cpp/qr_detector_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>

namespace qr_detector_cpp
{

QrDetectorNode::QrDetectorNode(const rclcpp::NodeOptions &options) : Node("qr_detector", options)
{
  // ---- Parameters ----------------------------------------------------
  this->declare_parameter("model_dir", "");
  this->declare_parameter("publish_compressed", true);
  this->declare_parameter("jpeg_quality", 60);
  this->declare_parameter("detection_interval", 1);

  model_dir_ = this->get_parameter("model_dir").as_string();
  publish_compressed_ = this->get_parameter("publish_compressed").as_bool();
  jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
  detection_interval_ = this->get_parameter("detection_interval").as_int();

  // Resolve model directory
  if (model_dir_.empty())
  {
    try
    {
      std::string python_pkg_share = ament_index_cpp::get_package_share_directory("qr_detector");
      model_dir_ = (std::filesystem::path(python_pkg_share) / "models").string();
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "Could not find 'qr_detector' share directory: %s", e.what());
    }
  }

  // ---- WeChatQRCode detector -----------------------------------------
  std::string detect_proto = (std::filesystem::path(model_dir_) / "detect.prototxt").string();
  std::string detect_model = (std::filesystem::path(model_dir_) / "detect.caffemodel").string();
  std::string sr_proto = (std::filesystem::path(model_dir_) / "sr.prototxt").string();
  std::string sr_model = (std::filesystem::path(model_dir_) / "sr.caffemodel").string();

  bool all_files_exist = true;
  for (const auto &f : {detect_proto, detect_model, sr_proto, sr_model})
  {
    if (!std::filesystem::exists(f))
    {
      RCLCPP_WARN(this->get_logger(), "Model file not found: %s", f.c_str());
      all_files_exist = false;
    }
  }

  if (all_files_exist)
  {
    try
    {
      detector_ = std::make_unique<cv::wechat_qrcode::WeChatQRCode>(detect_proto, detect_model,
                                                                    sr_proto, sr_model);
      RCLCPP_INFO(this->get_logger(), "WeChatQRCode detector initialized");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize WeChatQRCode: %s", e.what());
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "WeChatQRCode models not found — using zbar fallback");
  }

  // zbar: QRコードのみ有効化
  zbar_scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  zbar_scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

  // ---- QoS: sensor data = BEST_EFFORT --------------------------------
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5))
                        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // ---- Subscriber ----------------------------------------------------
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", sensor_qos,
      std::bind(&QrDetectorNode::image_callback, this, std::placeholders::_1));

  // ---- Publishers ----------------------------------------------------
  qr_pub_ = this->create_publisher<std_msgs::msg::String>("qr_codes", 10);

  if (publish_compressed_)
  {
    compressed_pub_ =
        this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", sensor_qos);
  }

  RCLCPP_INFO(this->get_logger(), "qr_detector started (interval=%d, jpeg_q=%d, compressed=%d)",
              detection_interval_, jpeg_quality_, publish_compressed_);
}

void QrDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  frame_count_++;

  if (frame_count_ % detection_interval_ != 0)
  {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;
  std::vector<cv::Mat> points;
  std::vector<std::string> results;

  bool use_zbar = !detector_;

  if (detector_)
  {
    try
    {
      results = detector_->detectAndDecode(frame, points);
    }
    catch (const cv::Exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "WeChatQRCode failed, falling back to zbar: %s", e.what());
      use_zbar = true;
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "WeChatQRCode failed, falling back to zbar: %s", e.what());
      use_zbar = true;
    }
    catch (...)
    {
      RCLCPP_WARN(this->get_logger(),
                  "WeChatQRCode failed with unknown error, falling back to zbar");
      use_zbar = true;
    }
  }

  if (use_zbar)
  {
    // zbar フォールバック (MONO8 で直接変換)
    cv_bridge::CvImagePtr gray_ptr;
    try
    {
      gray_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge mono8 exception: %s", e.what());
      return;
    }
    cv::Mat gray = gray_ptr->image.isContinuous() ? gray_ptr->image : gray_ptr->image.clone();
    zbar::Image zbar_img(gray.cols, gray.rows, "Y800", gray.data,
                         static_cast<unsigned long>(gray.cols * gray.rows));
    zbar_scanner_.scan(zbar_img);
    for (auto sym = zbar_img.symbol_begin(); sym != zbar_img.symbol_end(); ++sym)
    {
      if (sym->get_type() == zbar::ZBAR_QRCODE)
      {
        results.push_back(sym->get_data());
      }
    }
  }

  for (size_t i = 0; i < results.size(); ++i)
  {
    if (results[i].empty())
    {
      continue;
    }

    auto qr_msg = std_msgs::msg::String();
    qr_msg.data = results[i];
    qr_pub_->publish(qr_msg);
    RCLCPP_INFO(this->get_logger(), "QR detected: %s", results[i].c_str());

    if (publish_compressed_ && i < points.size())
    {
      cv::Mat pts_mat = points[i];
      std::vector<cv::Point> pts;
      for (int row = 0; row < pts_mat.rows; ++row)
      {
        pts.push_back(cv::Point(static_cast<int>(pts_mat.at<float>(row, 0)),
                                static_cast<int>(pts_mat.at<float>(row, 1))));
      }
      cv::polylines(frame, pts, true, cv::Scalar(0, 255, 0), 2);
      cv::putText(frame, results[i].substr(0, 40), cv::Point(pts[0].x, pts[0].y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }
  }

  if (publish_compressed_)
  {
    auto comp_msg = sensor_msgs::msg::CompressedImage();
    comp_msg.header = msg->header;
    comp_msg.format = "jpeg";
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    cv::imencode(".jpg", frame, comp_msg.data, params);
    compressed_pub_->publish(comp_msg);
  }
}

}  // namespace qr_detector_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(qr_detector_cpp::QrDetectorNode)
