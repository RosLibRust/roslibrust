#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/UInt32.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <functional>
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <boost/function.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using Ns = std::chrono::nanoseconds;

struct Options {
  std::string mode = "same_process";
  std::string topic = "/image_bench";
  std::string ack_topic = "/image_bench_ack";
  int iterations = 1000;
  int warmup = 25;
  int queue_size = 1;
  int width = 1920;
  int height = 1080;
  int channels = 3;
  std::string encoding = "rgb8";
  bool tcp_no_delay = false;
  double connect_timeout_s = 10.0;
  double message_timeout_s = 5.0;
};

struct Stats {
  double mean_ns = 0.0;
  double median_ns = 0.0;
  double p95_ns = 0.0;
  double min_ns = 0.0;
  double max_ns = 0.0;
  double stddev_ns = 0.0;
};

void usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [--mode same_process|driver|responder] [options]\n"
      << "\n"
      << "Options:\n"
      << "  --iterations N       Measured iterations, default 1000\n"
      << "  --warmup N           Warmup iterations ignored in stats, default 25\n"
      << "  --width N            Image width, default 1920\n"
      << "  --height N           Image height, default 1080\n"
      << "  --channels N         Bytes per pixel, default 3\n"
      << "  --encoding NAME      Image encoding, default rgb8\n"
      << "  --queue-size N       ROS pub/sub queue size, default 1\n"
      << "  --topic NAME         Image topic, default /image_bench\n"
      << "  --ack-topic NAME     Ack topic for driver/responder mode, default /image_bench_ack\n"
      << "  --tcp-no-delay       Use ros::TransportHints().tcpNoDelay()\n"
      << "  --connect-timeout S  Connection wait timeout, default 10\n"
      << "  --message-timeout S  Per-message wait timeout, default 5\n";
}

int parse_int(const std::string& value, const std::string& name) {
  char* end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || parsed > std::numeric_limits<int>::max() ||
      parsed < std::numeric_limits<int>::min()) {
    throw std::runtime_error("Invalid integer for " + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

double parse_double(const std::string& value, const std::string& name) {
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    throw std::runtime_error("Invalid number for " + name + ": " + value);
  }
  return parsed;
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else if (arg == "--mode") {
      options.mode = need_value(arg);
    } else if (arg == "--iterations") {
      options.iterations = parse_int(need_value(arg), arg);
    } else if (arg == "--warmup") {
      options.warmup = parse_int(need_value(arg), arg);
    } else if (arg == "--width") {
      options.width = parse_int(need_value(arg), arg);
    } else if (arg == "--height") {
      options.height = parse_int(need_value(arg), arg);
    } else if (arg == "--channels") {
      options.channels = parse_int(need_value(arg), arg);
    } else if (arg == "--encoding") {
      options.encoding = need_value(arg);
    } else if (arg == "--queue-size") {
      options.queue_size = parse_int(need_value(arg), arg);
    } else if (arg == "--topic") {
      options.topic = need_value(arg);
    } else if (arg == "--ack-topic") {
      options.ack_topic = need_value(arg);
    } else if (arg == "--tcp-no-delay") {
      options.tcp_no_delay = true;
    } else if (arg == "--connect-timeout") {
      options.connect_timeout_s = parse_double(need_value(arg), arg);
    } else if (arg == "--message-timeout") {
      options.message_timeout_s = parse_double(need_value(arg), arg);
    } else {
      throw std::runtime_error("Unknown option: " + arg);
    }
  }

  if (options.mode != "same_process" && options.mode != "driver" && options.mode != "responder") {
    throw std::runtime_error("Unsupported mode: " + options.mode);
  }
  if (options.iterations <= 0 || options.warmup < 0 || options.queue_size <= 0 ||
      options.width <= 0 || options.height <= 0 || options.channels <= 0) {
    throw std::runtime_error("iterations, queue-size, width, height, and channels must be positive");
  }
  return options;
}

sensor_msgs::Image make_image(const Options& options) {
  sensor_msgs::Image image;
  image.header.seq = 0;
  image.height = static_cast<uint32_t>(options.height);
  image.width = static_cast<uint32_t>(options.width);
  image.encoding = options.encoding;
  image.is_bigendian = 0;
  image.step = static_cast<uint32_t>(options.width * options.channels);
  image.data.assign(static_cast<size_t>(image.step) * image.height, 0);
  return image;
}

Stats compute_stats(std::vector<double> values) {
  if (values.empty()) {
    throw std::runtime_error("No samples collected");
  }

  Stats stats;
  stats.min_ns = *std::min_element(values.begin(), values.end());
  stats.max_ns = *std::max_element(values.begin(), values.end());
  stats.mean_ns = std::accumulate(values.begin(), values.end(), 0.0) / values.size();

  double variance = 0.0;
  for (const double value : values) {
    const double delta = value - stats.mean_ns;
    variance += delta * delta;
  }
  stats.stddev_ns = std::sqrt(variance / values.size());

  std::sort(values.begin(), values.end());
  const auto percentile = [&](double p) {
    const double idx = p * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(idx));
    const auto hi = static_cast<size_t>(std::ceil(idx));
    const double frac = idx - static_cast<double>(lo);
    return values[lo] * (1.0 - frac) + values[hi] * frac;
  };

  stats.median_ns = percentile(0.50);
  stats.p95_ns = percentile(0.95);
  return stats;
}

ros::TransportHints transport_hints(const Options& options) {
  ros::TransportHints hints;
  if (options.tcp_no_delay) {
    hints.tcpNoDelay();
  }
  return hints;
}

bool wait_for_connections(const std::function<bool()>& ready, double timeout_s) {
  const ros::Time deadline = ros::Time::now() + ros::Duration(timeout_s);
  ros::Rate rate(100.0);
  while (ros::ok() && ros::Time::now() < deadline) {
    if (ready()) {
      return true;
    }
    rate.sleep();
  }
  return ready();
}

void settle_connections() {
  ros::Duration(0.1).sleep();
}

void print_results(const Options& options, const Stats& stats, size_t payload_bytes) {
  const double mean_s = stats.mean_ns / 1.0e9;
  const double mib_per_s = (static_cast<double>(payload_bytes) / (1024.0 * 1024.0)) / mean_s;
  const double gb_per_s = (static_cast<double>(payload_bytes) / 1.0e9) / mean_s;

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "mode: " << options.mode << "\n";
  std::cout << "samples: " << options.iterations << " measured, " << options.warmup << " warmup\n";
  std::cout << "image: " << options.width << "x" << options.height << " " << options.encoding
            << ", payload_bytes: " << payload_bytes << "\n";
  std::cout << "mean: " << stats.mean_ns / 1000000.0 << " ms\n";
  std::cout << "median: " << stats.median_ns / 1000000.0 << " ms\n";
  std::cout << "p95: " << stats.p95_ns / 1000000.0 << " ms\n";
  std::cout << "min: " << stats.min_ns / 1000000.0 << " ms\n";
  std::cout << "max: " << stats.max_ns / 1000000.0 << " ms\n";
  std::cout << "stddev: " << stats.stddev_ns / 1000000.0 << " ms\n";
  std::cout << "throughput_payload: " << mib_per_s << " MiB/s (" << gb_per_s << " GB/s)\n";

  std::cout << std::setprecision(6);
  std::cout << "json: {"
            << "\"mode\":\"" << options.mode << "\","
            << "\"iterations\":" << options.iterations << ","
            << "\"warmup\":" << options.warmup << ","
            << "\"width\":" << options.width << ","
            << "\"height\":" << options.height << ","
            << "\"channels\":" << options.channels << ","
            << "\"encoding\":\"" << options.encoding << "\","
            << "\"queue_size\":" << options.queue_size << ","
            << "\"tcp_no_delay\":" << (options.tcp_no_delay ? "true" : "false") << ","
            << "\"payload_bytes\":" << payload_bytes << ","
            << "\"mean_ns\":" << stats.mean_ns << ","
            << "\"median_ns\":" << stats.median_ns << ","
            << "\"p95_ns\":" << stats.p95_ns << ","
            << "\"min_ns\":" << stats.min_ns << ","
            << "\"max_ns\":" << stats.max_ns << ","
            << "\"stddev_ns\":" << stats.stddev_ns << ","
            << "\"throughput_mib_s\":" << mib_per_s << ","
            << "\"throughput_gb_s\":" << gb_per_s << "}\n";
}

class SequenceWaiter {
 public:
  void mark(uint32_t seq) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_seq_ = seq;
    cv_.notify_all();
  }

  bool wait(uint32_t seq, double timeout_s) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, std::chrono::duration<double>(timeout_s),
                        [&] { return last_seq_ >= seq; });
  }

  bool wait_spinning(uint32_t seq, double timeout_s) {
    const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
    while (ros::ok() && Clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_seq_ >= seq) {
          return true;
        }
      }
      ros::spinOnce();
      std::this_thread::yield();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    return last_seq_ >= seq;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  uint32_t last_seq_ = 0;
};

int run_same_process(const Options& options) {
  ros::NodeHandle nh;
  SequenceWaiter waiter;
  uint64_t checksum = 0;

  ros::Publisher publisher =
      nh.advertise<sensor_msgs::Image>(options.topic, options.queue_size, false);
  boost::function<void(const sensor_msgs::ImageConstPtr&)> image_cb =
      [&](const sensor_msgs::ImageConstPtr& msg) {
        checksum += msg->data.empty() ? 0 : msg->data.back();
        waiter.mark(msg->header.seq);
      };
  ros::Subscriber subscriber = nh.subscribe<sensor_msgs::Image>(
      options.topic, options.queue_size, image_cb, ros::VoidConstPtr(), transport_hints(options));

  if (!wait_for_connections([&] { return publisher.getNumSubscribers() > 0; },
                            options.connect_timeout_s)) {
    throw std::runtime_error("Timed out waiting for same-process subscriber connection");
  }
  settle_connections();

  sensor_msgs::Image image = make_image(options);
  image.header.seq = 1;
  const auto prime_deadline = Clock::now() + std::chrono::duration<double>(options.message_timeout_s);
  while (ros::ok() && Clock::now() < prime_deadline) {
    publisher.publish(image);
    if (waiter.wait_spinning(image.header.seq, 0.01)) {
      break;
    }
    ros::Duration(0.01).sleep();
  }
  if (!waiter.wait_spinning(image.header.seq, 0.0)) {
    throw std::runtime_error("Timed out priming same-process image delivery");
  }

  std::vector<double> timings;
  timings.reserve(static_cast<size_t>(options.iterations));

  const int total = options.warmup + options.iterations;
  for (int i = 0; ros::ok() && i < total; ++i) {
    image.header.seq = static_cast<uint32_t>(i + 2);
    const auto start = Clock::now();
    publisher.publish(image);
    if (!waiter.wait_spinning(image.header.seq, options.message_timeout_s)) {
      throw std::runtime_error("Timed out waiting for image callback");
    }
    const auto end = Clock::now();
    if (i >= options.warmup) {
      timings.push_back(static_cast<double>(std::chrono::duration_cast<Ns>(end - start).count()));
    }
  }

  const Stats stats = compute_stats(std::move(timings));
  print_results(options, stats, image.data.size());
  std::cerr << "checksum: " << checksum << "\n";
  return 0;
}

int run_driver(const Options& options) {
  ros::NodeHandle nh;
  SequenceWaiter ack_waiter;

  ros::Publisher image_pub =
      nh.advertise<sensor_msgs::Image>(options.topic, options.queue_size, false);
  boost::function<void(const std_msgs::UInt32ConstPtr&)> ack_cb =
      [&](const std_msgs::UInt32ConstPtr& msg) { ack_waiter.mark(msg->data); };
  ros::Subscriber ack_sub = nh.subscribe<std_msgs::UInt32>(
      options.ack_topic, options.queue_size, ack_cb, ros::VoidConstPtr(), transport_hints(options));

  if (!wait_for_connections(
          [&] { return image_pub.getNumSubscribers() > 0 && ack_sub.getNumPublishers() > 0; },
          options.connect_timeout_s)) {
    throw std::runtime_error("Timed out waiting for responder connections");
  }
  settle_connections();

  sensor_msgs::Image image = make_image(options);
  image.header.seq = 1;
  const auto prime_deadline = Clock::now() + std::chrono::duration<double>(options.message_timeout_s);
  while (ros::ok() && Clock::now() < prime_deadline) {
    image_pub.publish(image);
    if (ack_waiter.wait_spinning(image.header.seq, 0.01)) {
      break;
    }
    ros::Duration(0.01).sleep();
  }
  if (!ack_waiter.wait_spinning(image.header.seq, 0.0)) {
    throw std::runtime_error("Timed out priming responder image delivery");
  }

  std::vector<double> timings;
  timings.reserve(static_cast<size_t>(options.iterations));

  const int total = options.warmup + options.iterations;
  for (int i = 0; ros::ok() && i < total; ++i) {
    image.header.seq = static_cast<uint32_t>(i + 2);
    const auto start = Clock::now();
    image_pub.publish(image);
    if (!ack_waiter.wait_spinning(image.header.seq, options.message_timeout_s)) {
      throw std::runtime_error("Timed out waiting for ack");
    }
    const auto end = Clock::now();
    if (i >= options.warmup) {
      timings.push_back(static_cast<double>(std::chrono::duration_cast<Ns>(end - start).count()));
    }
  }

  const Stats stats = compute_stats(std::move(timings));
  print_results(options, stats, image.data.size());
  return 0;
}

int run_responder(const Options& options) {
  ros::NodeHandle nh;
  uint64_t received = 0;
  uint64_t checksum = 0;

  ros::Publisher ack_pub =
      nh.advertise<std_msgs::UInt32>(options.ack_topic, options.queue_size, false);
  boost::function<void(const sensor_msgs::ImageConstPtr&)> image_cb =
      [&](const sensor_msgs::ImageConstPtr& msg) {
        checksum += msg->data.empty() ? 0 : msg->data.back();
        ++received;
        std_msgs::UInt32 ack;
        ack.data = msg->header.seq;
        ack_pub.publish(ack);
      };
  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>(
      options.topic, options.queue_size, image_cb, ros::VoidConstPtr(), transport_hints(options));

  ROS_INFO_STREAM("Responder ready on " << options.topic << ", publishing acks on "
                                        << options.ack_topic);
  ros::spin();
  std::cerr << "received: " << received << ", checksum: " << checksum << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::string node_name = "cpp_image_bench_" + options.mode;
    ros::init(argc, argv, node_name, ros::init_options::AnonymousName);

    if (options.mode == "same_process") {
      return run_same_process(options);
    }
    if (options.mode == "driver") {
      return run_driver(options);
    }
    return run_responder(options);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    usage(argv[0]);
    return 1;
  }
}
