#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <time.h>
#include <atomic>

// #include "image_converter.h"
#include <jetson-utils/gstCamera.h>
#include <JetsonGPIO.h>
#include <opencv2/opencv.hpp>

#define GPIO_LENC 7           /*216*/
#define GPIO_RENC 11          /*50*/
#define GPIO_IR_SENSOR 12     /*79*/
#define GPIO_LOWBATT 13       /*14*/

const int video_width = 1920; // 640;  // video_options.width;
const int video_height = 1080; // 480; // video_options.height;

std::atomic_uint left_enc_register;
std::atomic_uint right_enc_register;

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
  
  std::string resource_str = "csi://0";
  std::string codec_str = "";

  if (resource_str.size() == 0) {
    puts("[tbs] resource param wasn't set - please set the node's resource parameter to the input device"
      "/filename/URL");
    return false;
  }

  printf("[tbs] opening video source: %s", resource_str.c_str());

  /*
   * open video source
   */
  gstCamera *camera = gstCamera::Create(video_width, video_height);

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
  if (!camera->Capture(&capture, 2000)) {
    puts("\n[tbs] Failed to capture image");
    return false;
  }

  // puts("\n[tbs] Image Captured!");
  // printf("[tbs] %zu\n", sizeof(capture));

  cv::Mat img = cv::Mat(cv::Size(video_width, video_height), CV_8UC3, capture);
  // cv::cvtColor(img, img, cv::ColorConversionCodes::COLOR_BGR2GRAY);
  

  cv::imwrite(filename, img);


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

float get_diff_secsf(struct timespec *time, struct timespec *from) {
  return (float)(time->tv_sec - from->tv_sec) + 1e-9f * (time->tv_nsec - from->tv_nsec);
}

int main(int argc, char *argv[]) {
	// // # Start Recording Reminder #
	// // ** Comment this if you have no clue what it is **
	// int days_since_this_code_fragment_was_written = 0;



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

  const float FramePeriod = 2.25f;

  int frame_index = 0, iter = 0;
  while (true) {
    if(kbhit() && getch() == 'q')
      break;
	  clock_gettime(CLOCK_REALTIME, &t);
    float since = get_diff_secsf(&t, &r);
    if (since < FramePeriod) {
      usleep(1000);
      continue;
    }
    r = t;

    // Next Frame
      // if (r % 10 == 0)
      //   puts(".");
    printf("frame %i (%.2fs)\n", frame_index, get_diff_secsf(&r, &loop_begin));

    uint lenc = left_enc_register.exchange(0);
    uint renc = right_enc_register.exchange(0);
    bool irdetect = !GPIO::input(GPIO_IR_SENSOR);
    bool lbsignal = !GPIO::input(GPIO_LOWBATT);

    std::string session_dir = std::string("/home/boo/proj/tdc/cap/1/");
    std::string capture_path = session_dir + std::string("capture_") + std::to_string(frame_index) + std::string(".jpg");
    captureImage(pCamera, capture_path);

	  clock_gettime(CLOCK_REALTIME, &t);
    since = get_diff_secsf(&t, &r);

    printf("-- L:%u R:%u IR:%s LB:%s\n (%.2fs)\n", lenc, renc, irdetect ? "detect" : "none", lbsignal ? "ok" : "low",
      since);
    ++frame_index;
  }

  GPIO::cleanup(GPIO_LENC);
  GPIO::cleanup(GPIO_RENC);
  GPIO::cleanup(GPIO_LOWBATT);
  GPIO::cleanup(GPIO_IR_SENSOR);
  GPIO::cleanup();

  closeCamera(pCamera);
  
	return 0;
}
