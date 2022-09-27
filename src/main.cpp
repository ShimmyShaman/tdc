#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <time.h>
#include <atomic>
#include <dirent.h>
#include <sys/stat.h>

// #include "image_converter.h"
#include <jetson-utils/gstCamera.h>
#include <jetson-utils/videoOptions.h>
#include <JetsonGPIO.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

#define GPIO_LENC 7           /*216*/
#define GPIO_RENC 11          /*50*/
#define GPIO_IR_SENSOR 12     /*79*/
#define GPIO_LOWBATT 13       /*14*/

#define CAPTURE_BASE_DIR "/home/boo/proj/tdc/cap"

const char *session_directory;

const int video_width = 640; // 1920; // video_options.width;
const int video_height = 480; // 1080; // video_options.height;

std::atomic_uint left_enc_register;
std::atomic_uint right_enc_register;

videoOptions vo;

int kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

char getch(void) {
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if(tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if(read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    printf("%c\n", buf);
    return buf;
}

bool createAndOpenCamera(gstCamera **pCamera) {
  
  vo.resource = "csi://0";
  vo.width = video_width;
  vo.height = video_height;
  // vo.frameRate = 30;
  // vo.bitRate = 4Mbps
  vo.numBuffers = 4;
  vo.zeroCopy = true;
  vo.loop = 0;
  vo.rtspLatency = 2000;
  vo.deviceType = videoOptions::DeviceType::DEVICE_CSI;
  vo.ioType = videoOptions::IoType::INPUT;
  vo.flipMethod = videoOptions::FlipMethod::FLIP_NONE;
  vo.codec = videoOptions::Codec::CODEC_MJPEG;

  printf("[tbs] opening video source: %s", (const char *)vo.resource);

  /*
   * open video source
   */
  gstCamera *camera = gstCamera::Create(vo);

  if (!camera) {
    puts("[tbs] failed to open video source");
    return false;
  }
  puts("\n[tbs] Video Source Opened!");

  /*
   * start the camera streaming
  image_transport
   */
  if (!camera->Open()) {
    puts("[tbs] failed to start streaming video source");
    delete camera;
    return false;
  }
  
  puts("\n[tbs] Camera Opened!");
  *pCamera = camera;
  return true;
}

bool captureImage(gstCamera *camera, std::string filename/*, imageConverter *image_cvt/*, sensor_msgs::Image *img*/) {
  uchar3 *capture = NULL;
  if (!camera->Capture(&capture, 1000)) {
    puts("\n[tbs] Failed to capture image");
    return false;
  }

  // puts("\n[tbs] Image Captured!");
  // printf("[tbs] %zu\n", sizeof(capture));

  cv::Mat img = cv::Mat(cv::Size(video_width, video_height), CV_8UC3, capture).clone();
  cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_BGR2RGB);
  // cv::imwrite(filename + std::string(".png"), img);

  cv::Mat sca;
  cv::Rect crop_a(0, 0, 300, 300);
  cv::Rect crop_b(220, 0, 300, 300);
  
  cv::resize(img, sca, cv::Size(520, 300));
  cv::Mat left = sca(crop_a);
  cv::Mat right = sca(crop_b);

  cv::imwrite(filename + std::string("a.png"), left);
  cv::imwrite(filename + std::string("b.png"), right);


  // // Convert the image to ROS output
  // if (!image_cvt->Convert(*img, imageConverter::ROSOutputFormat, capture)) {
  //   puts("\n[tbs] Failed to convert image");
  //   return false;
  // }

  return true;
}

void closeCamera(gstCamera *camera) {
  puts("\n[tbs] Closing Camera");
  camera->Close();
  delete camera;
}

void sensor_signal_callback(const std::string& channel) {
  int ich = atoi(channel.c_str());
  switch (ich) {
    case GPIO_LENC:
      // puts("Left");
      ++left_enc_register;
      break;
    case GPIO_RENC:
      // puts("Right");
      ++right_enc_register;
      break;
  }
}

void log(char *str) {
  puts(str);
}

float get_diff_secsf(struct timespec *time, struct timespec *from) {
  return (float)(time->tv_sec - from->tv_sec) + 1e-9f * (time->tv_nsec - from->tv_nsec);
}

int main(int argc, char *argv[]) {
  // Create session directory

  char cap_dir_buffer[512];

  {
    int session_idx = 1;
    struct stat sb;

    for (;; ++session_idx) {
      sprintf(cap_dir_buffer, "%s/s%i", CAPTURE_BASE_DIR, session_idx);
      if (stat(cap_dir_buffer, &sb) != 0 || !S_ISDIR(sb.st_mode))
        break;
    }
    session_directory = (const char *)&cap_dir_buffer;
    if (mkdir(session_directory, 0777)) {
      puts("Could not create directory for session captures, aborting");
      return -10;
    }

    printf("Created directory '%s' for session captures", session_directory);
  }

  // GPIOs
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(GPIO_LENC, GPIO::Directions::IN);
  GPIO::setup(GPIO_RENC, GPIO::Directions::IN);
  GPIO::setup(GPIO_LOWBATT, GPIO::Directions::IN);
  GPIO::setup(GPIO_IR_SENSOR, GPIO::Directions::IN);

  gstCamera *pCamera;
  createAndOpenCamera(&pCamera);

  GPIO::add_event_detect(GPIO_LENC, GPIO::Edge::RISING, sensor_signal_callback);
  GPIO::add_event_detect(GPIO_RENC, GPIO::Edge::RISING, sensor_signal_callback);
  // GPIO::add_event_detect(GPIO_LOWBATT, GPIO::Edge::RISING, sensor_signal_callback);
  // GPIO::add_event_detect(GPIO_IR_SENSOR, GPIO::Edge::RISING, sensor_signal_callback);

  puts("Initialized! Press 'q' to quit:");
	struct timespec loop_begin, r, t;
	clock_gettime(CLOCK_REALTIME, &loop_begin);
  r = loop_begin;

  const float FramePeriod = 0.20f;

  int frame_index = 0, iter = 0;
  char buf[512];
  while (true) {
    // if(kbhit() && getch() == 'q')
    //   break;
	  clock_gettime(CLOCK_REALTIME, &t);
    float since = get_diff_secsf(&t, &r);
    if (since < FramePeriod) {
      usleep(500);
      continue;
    }
    r = t;

    // Next Frame
      // if (r % 10 == 0)
      //   puts(".");
    float elapsed_secs = get_diff_secsf(&r, &loop_begin);
    // printf("frame %i (%.2fs)\n", frame_index, elapsed_secs);

    uint lenc = left_enc_register.exchange(0);
    uint renc = right_enc_register.exchange(0);
    bool irdetect = !GPIO::input(GPIO_IR_SENSOR);
    bool lbsignal = !GPIO::input(GPIO_LOWBATT);

    // Capture the image and save to file
    std::string capture_path = session_directory + std::string("/f") + std::to_string(frame_index);
    captureImage(pCamera, capture_path);

    // Save the sensor info
    sprintf(buf, "%s/d%i.txt", session_directory, frame_index);
    FILE *fp = fopen(buf, "w");
    if (fp != NULL) {
      fprintf(fp, "frame_index:%i\nsession_time_secs:%f\nleft:%u\nright:%u\nirs:%i\nlbs:%i\n", frame_index, elapsed_secs,
        lenc, renc, irdetect ? 1 : 0, lbsignal ? 1 : 0);
      fclose(fp);
    } else {
      printf("Error opening file '%s': %d (%s)\n", buf, errno, strerror(errno));
    }

	  clock_gettime(CLOCK_REALTIME, &t);
    since = get_diff_secsf(&t, &r);

    printf("-- %i L:%u R:%u IR:%s LB:%s\n (%ims)\n", frame_index, lenc, renc, irdetect ? "detect" : "none", lbsignal ? "ok" : "low",
      (int)(since * 1000.f));
    ++frame_index;

    if (elapsed_secs > 116.f) {
      break;
    }
  }

  GPIO::cleanup(GPIO_LENC);
  GPIO::cleanup(GPIO_RENC);
  GPIO::cleanup(GPIO_LOWBATT);
  GPIO::cleanup(GPIO_IR_SENSOR);
  GPIO::cleanup();

  closeCamera(pCamera);
  
  puts("Successfully exited!");
  
  system("shutdown -P now");

	return 0;
}
