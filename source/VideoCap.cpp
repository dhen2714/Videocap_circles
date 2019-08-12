#include <VideoCap.hpp>


VideoCapture::VideoCapture()
{
    dev_name = "/dev/video0";
    open_device();
    init_device();
    start_capturing();
}

void VideoCapture::capture(bool fpsSwitch)
{
    open_device();
    if (fpsSwitch)
        switch_fps();
    init_device();
    start_capturing();
}

void VideoCapture::release()
{
    cv::destroyAllWindows();
    uninit_device();
    close_device();
}

void VideoCapture::errno_exit(const char *s)
{
    //errno is number of last error.
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

int VideoCapture::xioctl(int fh, int request, void *arg)
{
    int r;
    do {
        r = ioctl(fh, request, arg);
    } while(-1 == r && EINTR == errno);

    return r;
}

void VideoCapture::open_device()
{
    struct stat st;

    if (-1 == stat(dev_name, &st)) {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name , O_RDWR | O_NONBLOCK, 0);

    if (fd == -1) {
        fprintf(stderr, "Cannot open '%s' : %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

void VideoCapture::close_device()
{
    if (-1 == close(fd))
        errno_exit("close");
    fd = -1;
}

void VideoCapture::init_device()
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;

    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s is no video capture device \n",
                dev_name);
        exit(EXIT_FAILURE);
    }

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (xioctl(fd, VIDIOC_CROPCAP, &cropcap) == 0) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; // reset to default

        if (xioctl(fd, VIDIOC_S_CROP, &crop) == -1) {
            switch (errno) {
            case EINVAL:
                break;
            default:
                break;
            }
        }
    }

    CLEAR(fmt);
    // The settings below are for a LI OV-580 OV7251 stereo camera.
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //std::cout << "FORCE FORMAT: " << force_format << std::endl;
    if (force_format) {
        // fprintf(stderr, "SET PARAMETERS FOR OV580\r\n");
        fmt.fmt.pix.width = 640;
        fmt.fmt.pix.height = 480;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_ANY;

        if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
            errno_exit("VIDIOC_S_FMT");
    } else {
        if (xioctl(fd, VIDIOC_G_FMT, &fmt) == -1)
            errno_exit("VIDIOC_G_FMT");
    }

    // Set exposure
    struct v4l2_control control;
    control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control.value = 510;

    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &control))
        errno_exit("VIDIOC_S_CTRL");

    struct v4l2_streamparm parm;

    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;

    if (-1 == xioctl(fd, VIDIOC_S_PARM, &parm)) {
        errno_exit("VIDIOC_S_PARM");
    }
    /*
    std::cout << fmt.fmt.pix.width << std::endl;
    std::cout << fmt.fmt.pix.height << std::endl;
    std::cout << fmt.fmt.pix.pixelformat << std::endl;
    */
    init_mmap();
}

void VideoCapture::init_mmap()
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 500; // Originally 4.
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        if (EINVAL == errno) {
            fprintf(stderr, "%s does not support memory mapping\n", dev_name);
            exit(EXIT_FAILURE);
        } else {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2) {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    buffers = static_cast<buffer*>(calloc(req.count, sizeof(*buffers)));

    if (!buffers) {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(NULL, buf.length,
                                        PROT_READ | PROT_WRITE,
                                        MAP_SHARED,
                                        fd, buf.m.offset);
        if (buffers[n_buffers].start == MAP_FAILED)
            errno_exit("mmap");
    }
}

void VideoCapture::uninit_device()
{
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
            errno_exit("VIDIOC_STREAMOFF");

    unsigned int i;

    for (i = 0; i < n_buffers; ++i)
        if (munmap(buffers[i].start, buffers[i].length) == -1)
            errno_exit("munmap");

    free(buffers);
}

void VideoCapture::start_capturing()
{
    unsigned int i;
    enum v4l2_buf_type type;

    for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            errno_exit("VIDIOC_QBUF");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
}

int VideoCapture::read(cv::Mat *frame)
{
    for (;;) {
        fd_set fds; // Bit string of file descriptors.
        struct timeval tv;
        int r;

        FD_ZERO(&fds); // Initializes file descriptor set &fds to be zero.
        FD_SET(fd, &fds); // Sets bit for file descriptor fd in &fds.

        tv.tv_sec = 2;
        tv.tv_usec = 0;

        /* The select() function indicates which of the specified file
        descriptors is ready for reading, ready for writing, or  has an error
        condition pending. */
        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (r == -1) {
            if (errno == EINTR) // Interrupted system call.
                continue;
            errno_exit("select");
        }
        if (r == 0) {
            fprintf(stderr, "select timeout \n");
            exit(EXIT_FAILURE);
        }
        if (process_frame(frame))
            break;
    }
    return !(frame->data == NULL);
}

int VideoCapture::process_frame(cv::Mat *frame)
{
    struct v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
        switch (errno) {
        case EAGAIN: //Resource temporarily unavailable.
            return 0;
        default:
            errno_exit("VIDIOC_DQBUF");
        }
    }
    assert(buf.index < n_buffers);

    // if (buf.flags & V4L2_BUF_FLAG_ERROR)
    //     std::cout << "BUFF FLAG ERROR" << std::endl;

    // Output timestamps, when first data byte was captured.
    // struct timeval tv = buf.timestamp;
    // std::cout << tv.tv_sec << "." << tv.tv_usec << std::endl;

    // Write buffer data into opencv mat.
    unsigned char* buffer_data;
    unsigned char* processed_buffer;

    buffer_data = static_cast<uchar*>(buffers[buf.index].start);
    processed_buffer = static_cast<uchar*>(malloc(sizeof(uchar) * 640 * 480));

    uint16_t raw_pix_val;
    uint8_t processed_pix_val;
    unsigned char word1, word2;

    for (int i = 0; i < 640 * 480; i++) {
        word1 = buffer_data[2 * i];
        word2 = buffer_data[2 * i + 1];

        raw_pix_val = (word2 << 8) | word1;
        processed_pix_val = static_cast<uint8_t>((255.0/1023.0) * static_cast<float>(raw_pix_val));

        processed_buffer[i] = processed_pix_val;
    }

    // frame->data = static_cast<uchar*>(buffers[buf.index].start);
    frame->data = processed_buffer;

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

    return 1;
}

int VideoCapture::read(Frame &frame)
{
    for (;;) {
        fd_set fds; // Bit string of file descriptors.
        struct timeval tv;
        int r;

        FD_ZERO(&fds); // Initializes file descriptor set &fds to be zero.
        FD_SET(fd, &fds); // Sets bit for file descriptor fd in &fds.

        tv.tv_sec = 2;
        tv.tv_usec = 0;

        /* The select() function indicates which of the specified file
        descriptors is ready for reading, ready for writing, or  has an error
        condition pending. */
        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (r == -1) {
            if (errno == EINTR) // Interrupted system call.
                continue;
            errno_exit("select");
        }
        if (r == 0) {
            fprintf(stderr, "select timeout \n");
            exit(EXIT_FAILURE);
        }
        if (process_frame(frame))
            break;
    }
    return 1;
}

int VideoCapture::process_frame(Frame &frame)
{
    struct v4l2_buffer buf;
    frame.clear();
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
        switch (errno) {
        case EAGAIN: //Resource temporarily unavailable.
            return 0;
        default:
            errno_exit("VIDIOC_DQBUF");
        }
    }
    assert(buf.index < n_buffers);

    // if (buf.flags & V4L2_BUF_FLAG_ERROR)
    //     std::cout << "BUFF FLAG ERROR" << std::endl;

    // Output timestamps, when first data byte was captured.
    // struct timeval tv = buf.timestamp;
    // std::cout << tv.tv_sec << "." << tv.tv_usec << std::endl;

    // Write buffer data into opencv mat.
    // Also resamples 10 bit raw data into an 8 bit image for the MIPI tester.
    unsigned char* buffer_data;
    unsigned char* processed_buffer;

    buffer_data = static_cast<uchar*>(buffers[buf.index].start);
    processed_buffer = static_cast<uchar*>(malloc(sizeof(uchar) * 640 * 480));

    uint16_t raw_pix_val;
    uint8_t processed_pix_val;
    unsigned char word1, word2;

    for (int i = 0; i < 640 * 480; i++) {
        word1 = buffer_data[2 * i];
        word2 = buffer_data[2 * i + 1];

        raw_pix_val = (word2 << 8) | word1;
        processed_pix_val = static_cast<uint8_t>((255.0/1023.0) * static_cast<float>(raw_pix_val));

        processed_buffer[i] = processed_pix_val;
    }

    // frame.image.data = static_cast<uchar*>(buffers[buf.index].start);
    frame.image.data = processed_buffer;
    frame.timestamp = buf.timestamp;

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

    return 1;
}

void VideoCapture::switch_fps()
{
    if (fps == 60) {
        fps = 100;
    } else if (fps == 100) {
        fps = 60;
    }
    std::cout << "FPS set to " << fps << std::endl;
}

int VideoCapture::get_fps() {return fps;}

int VideoCapture::set_exposure(int exposure)
{
    // Sets exposure level to desired value.
    struct v4l2_control control;
    control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control.value = exposure;

    if (-1 == xioctl(fd, VIDIOC_S_CTRL, &control))
        errno_exit("VIDIOC_S_CTRL");

    return 1;
}
