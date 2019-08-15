/*
Capture application and VideoCapture object for displaying and writing frames
from Leopard OV-580 stereo camera. Commands can be entered via the command
line after the application starts. Commands are:
    start - Starts writing frames continuously to disk.
    n     - Where 'n' is an integer; writes 'n' frames to disk.
    stop  - Stops writing. Can be used after one of the previous two commands
            are called.
    q     - Quits application.

Capture Application initializes video capture device with address /dev/video0.

To initialize a VideoCapture object:
    vc = VideoCapture()
To grab a frame, call vc.read(). To release capture, vc.release().

    - David Henry 2018
*/
#ifndef VIDEO_CAP_H
#define VIDEO_CAP_H

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <ctime>
#include <cctype>

extern "C" {
#include <getopt.h>
// Low level i/o
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
// V4L2 - video4linux
#include <linux/videodev2.h>
}

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include "../dlib-master/dlib/optimization.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/fast_math.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/call_traits.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp> 

#define CLEAR(x) memset(&(x), 0, sizeof(x))

typedef dlib::matrix<double, 4, 1> parameter_vector;

class Frame
{
public:
    cv::Mat image;
    struct timeval timestamp;

    Frame() : image(cv::Mat(480, 640, CV_8U)) {};
    void clear();
};

class buffer {
public:
    void *start;
    size_t length;
};

enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

class VideoCapture{
private:
    const char *dev_name; // /dev/videoX
    const enum io_method io = IO_METHOD_MMAP; // Memory mapping.
    int fd = -1;
    buffer *buffers;
    unsigned int n_buffers;
    int out_buf;
    int force_format = 1; // If set != 0, img format specified in init_device()
    int fps = 60; // Defaults at 100 fps.

    void errno_exit(const char *s);
    int xioctl(int fh, int request, void *arg);

    void open_device(); // 'open()' call on file descriptor.
    void init_device(); // Sets video capture format, fps, calls init_mmap().
    void init_mmap(); // Initiates memory mapping.
    void start_capturing(); // Starts capture, queues buffers.
    int process_frame(cv::Mat *frame); // Reads buffer into frame.
    int process_frame(Frame &frame);
    void uninit_device(); // Unitiates memory map.
    void close_device(); // Closes device.
    void switch_fps(); // Fps value switch, called by capture() if needed.

public:
    VideoCapture();
    /*
    On initialization, dev_name is set to "/dev/video0",
        open_device();
        init_device();
        start_capturing();
    */
    int read(cv::Mat *frame);
    int read(Frame &frame);
    /*
    Reads buffer into input frame.
    */
    void release();
    /*
    Releases the video capture.
        cv::destroyAllWindows;
        uninit_device();
        close_device();
    */
    void capture(bool fpsSwitch = false);
    /*
    Run this after release() has been called to re-initiate capture. If
    fpsSwitch is true, then the fps is also switched (between 100 and 60).
        open_device();
        init_device();
        start_capturing();
    */
    int get_fps(); // Returns fps value.
    int set_exposure(int exposure); // Sets exposure to desired value.
};

class bounded_buffer
/*
Thread-safe implementation of circular buffer, ensuring mutual exclusion
between capture and write threads during buffer access. This is achieved by
wrapping the operations of the underlying boost::circular_buffer object with
lock acquisition and release.

Implementation is based on this example:
http://www.boost.org/doc/libs/master/libs/circular_buffer/example/circular_buffer_bound_example.cpp
*/
{
public:
    typedef boost::circular_buffer<Frame> container_type;
    typedef typename container_type::size_type size_type;
    typedef typename container_type::value_type value_type;
    typedef typename boost::call_traits<value_type>::param_type param_type;

    explicit bounded_buffer(size_type capacity)
    : m_unread(0), m_container(capacity) {}

    void push_front(typename boost::call_traits<value_type>::param_type item);
    void pop_back(value_type* frameCopy);
    void pop_back(value_type &frameCopy);

    void clear_buffer() { m_container.clear(); m_unread = 0;}
    void clear_consumer();
    void clear_producer();

private:
    // Disabled copy constructor.
    bounded_buffer(const bounded_buffer&);
    // Disabled assign operator.
    bounded_buffer& operator = (const bounded_buffer&);

    bool is_not_empty() const { return m_unread > 0; }
    bool is_not_full() const { return m_unread < m_container.capacity(); }

    size_type m_unread;
    container_type m_container;
    boost::mutex m_mutex;
    boost::condition m_not_empty;
    boost::condition m_not_full;
};

class CaptureApplication
{
private:
    VideoCapture vc;
    std::thread readThread; // Thread for reading frames from VideoCapture.
    std::thread writeThread; // Thread for writing frames from VideoCapture.
    std::thread improThread; // Thread for image processing.
    bounded_buffer *CapAppBuffer; // Circular buffer for frame R/W.
    std::atomic_bool writeContinuous; // Switch for writing frames to disk.
    std::atomic_bool writeSingles; // Switch for writing given amount of frames.
    std::atomic_bool writing; // Write status.
    std::atomic_bool captureOn; // Video capture switch.
    std::atomic_ulong writeCount; // Number of frames written to disk.
    std::atomic_uint additionalFrames; // User specified number of frames.
    std::atomic_uint exposure; // User defined exposure.
    //cv::Mat frame; // OpenCV Mat object which camera buffer is read to.
    std::atomic_bool focusOn; // Focus calculation switch.
    std::atomic_bool laplOn; // Laplacian variance calculation switch.
    std::mutex framelock;
    Frame frame;
    const unsigned int cap_app_size = 500; // Frame capacity of circular buffer.

    void run_capture(); // Loops through Videocapture.read() calls.
    void parse_command();
    void print_timestamp();
    void write_image(Frame &frame);
    void write_image(cv::Mat *image); // Write current frame to disk.
    void write_image_raw(cv::Mat *image);
    void update_write_status(); // Updates the write status.
    void get_write_status();
    bool numeric_command(const std::string *command); // Checks if input is num.
    unsigned int str2int(const std::string *command); // Converts str to int.
    void read_frames(); // Reads frames to CapAppBuffer, and displays them.
    void write_frames(); // Writes frames from CapAppBuffer.
    void image_processing();
    void calculate_fwhm(); // Calculates fwhm of detected circle.
    void calculate_lapvar(); // Calculates laplacian variance.
public:
    CaptureApplication();
    ~CaptureApplication();
};

class GaussianFit
{

private:
    static double model(const double& x, const parameter_vector& params);
    static double residual(const std::pair<double, double>& data, 
        const parameter_vector& params);
    static parameter_vector residual_derivative(const std::pair<double, double>& data,
        const parameter_vector& params);

    parameter_vector params; // mu, sigma, amplitude, base
    std::vector<std::pair<double, double>> data_samples;
    
public:
    GaussianFit(std::vector<std::pair<double, double>>& data, 
        double mu, double sigma, double amplitude, double base);
    void fit();
    double get_mu() {return params(0);};
    double get_sigma() {return params(1);};
    double get_amp() {return params(2);};
    double get_base() {return params(3);};
    void print_params() {std::cout << dlib::trans(params) <<std::endl;};
};

#endif // VIDEO_CAP_H
