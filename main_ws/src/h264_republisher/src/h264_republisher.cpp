#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <memory>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

using std::placeholders::_1;

class H264Republisher : public rclcpp::Node {
public:
  H264Republisher()
  : Node("h264_republisher") {
    avcodec_register_all();
    avformat_network_init();

    pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/ffmpeg", 10, std::bind(&H264Republisher::cb, this, _1));

    // Setup decoder
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
      RCLCPP_FATAL(this->get_logger(), "H.264 codec not found");
      throw std::runtime_error("codec not found");
    }
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) throw std::runtime_error("failed to alloc codec ctx");
    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open codec");
      throw std::runtime_error("failed to open codec");
    }
    frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();
  }

  ~H264Republisher() override {
    if (pkt_) av_packet_free(&pkt_);
    if (frame_) av_frame_free(&frame_);
    if (codec_ctx_) avcodec_free_context(&codec_ctx_);
    avformat_network_deinit();
  }

private:
  void cb(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    // Feed packet
    av_packet_unref(pkt_);
    pkt_->data = const_cast<uint8_t*>(msg->data.data());
    pkt_->size = static_cast<int>(msg->data.size());

    int ret = avcodec_send_packet(codec_ctx_, pkt_);
    if (ret < 0) {
      RCLCPP_WARN(this->get_logger(), "avcodec_send_packet failed: %d", ret);
      return;
    }
    while (ret >= 0) {
      ret = avcodec_receive_frame(codec_ctx_, frame_);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return;
      if (ret < 0) {
        RCLCPP_WARN(this->get_logger(), "Error during decoding: %d", ret);
        return;
      }

      // Convert to BGR24 using sws
      SwsContext *sws_ctx = sws_getContext(
        frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
        frame_->width, frame_->height, AV_PIX_FMT_BGR24,
        SWS_BILINEAR, nullptr, nullptr, nullptr);
      if (!sws_ctx) {
        RCLCPP_WARN(this->get_logger(), "sws_getContext failed");
        return;
      }

      cv::Mat bgr(frame_->height, frame_->width, CV_8UC3);
      uint8_t *dst[4];
      int dst_linesize[4];
      dst[0] = bgr.data;
      dst_linesize[0] = static_cast<int>(bgr.step);

      sws_scale(sws_ctx, frame_->data, frame_->linesize, 0, frame_->height, dst, dst_linesize);
      sws_freeContext(sws_ctx);

      // Publish
      std_msgs::msg::Header h = msg->header;
      auto out = cv_bridge::CvImage(h, "bgr8", bgr).toImageMsg();
      pub_->publish(*out);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  AVCodecContext *codec_ctx_ = nullptr;
  AVPacket *pkt_ = nullptr;
  AVFrame *frame_ = nullptr;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<H264Republisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
