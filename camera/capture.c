/*
 * Filename   : capture.c
 *
 * Description: C270 web camera driver with V4L2 API
 *            : Code Flow:
 *            : 1) Open device
 *            : 2) Initialize device
 *            : 3) Start capturing
 *            : 4) Main Loop
 *            : 5) Stop Capturing
 *            : 6) Unitialize device
 *            : 7) Close Device
 *    
 * Author     : Swathi Venkatachalam
 *
 * Reference  : http://linuxtv.org/docs.php
 *            : https://github.com/siewertsmooc/RTES-ECEE-5623/blob/main/simple-capture/capture.c
 * Starter    : Code from Prof. Sam Siewert's RTES Course
 */


/*************************************************************************
 *                            Header Files                               *
 *************************************************************************/
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */ // open system call
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>           // for stat struct
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <time.h>

/*************************************************************************
 *                            Macros                                     *
 *************************************************************************/

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 320
#define VRES 240
#define HRES_STR "320"
#define VRES_STR "240"

/*************************************************************************
 *                        Structures                                     *
 *************************************************************************/
 
 // Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

struct buffer 
{
        void   *start;
        size_t  length;
};

/*************************************************************************
 *                  Global Variables                                     *
 *************************************************************************/

static char            *dev_name;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 30;

/*************************************************************************
 *                         Functions                                     *
 *************************************************************************/

/*************************************************************************
 *                       Error Handling Function                         *
 *************************************************************************/

/**
 * @name   errno_exit
 * @brief  Handles error, exits program
 * @param  const char *s - ptr to constant character string
 * 
 * @descr  Static: function scope limited to current file
 *         Prints input str with error code and str describing error to std error output using fprintf
 *
 * @return none
 */

static void errno_exit(const char *s)
{
	    // printf outputs to standard output stream; fprintf goes to a file handle FILE*
		// printf = fprintf(stout,)
		// fprintf(FILE *stream, const char *format)
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno)); 
		// stderr =  standard error output
		// errno = error code of recent failure
		// strerror(errno) = human-readable string describing the error taking input errno as error code
        exit(EXIT_FAILURE); //EXIT_FAILURE from stdlib.h
}

/*************************************************************************
 *                                IOCTL Function                         *
 *************************************************************************/

/**
 * @name   xioctl
 * @brief  Wrapper function for ioctl sys call, handles interruptions caused by signals
 * @param  fh - fd on which ioctl op will be performed
 *         request - request code indicating specific op to be performed
 *         arg - ptr to arg/ds associated with ioctl function
 * 
 * @descr  Checks if file exists using stat
 *         Checks if it's a device file by accessing st_mode and S_ISCHR
 *         Open file with read and write permissions in non-blocking mode
 *         Exit on failure of any mentioned above
 *
 * @return int r - return val of ioctl function
 */
 
static int xioctl(int fh, int request, void *arg)
{
        int r; // Stores ret val of ioctl sys call
        // do wile to ensure ioctl is called at least once, runs until while condition false
        do 
        {
            r = ioctl(fh, request, arg); // device-specific input/output operations

        } while (r == -1 && errno == EINTR); // checks ioctl failure and if it was interrupted by signal EINTR

        return r;
}

/*************************************************************************
 *                          Open Device Function                         *
 *************************************************************************/

/**
 * @name   open_device
 * @brief  Open camera device named dev_name
 * @param  none
 * 
 * @descr  Checks if file exists using stat
 *         Checks if it's a device file by accessing st_mode and S_ISCHR
 *         Open file with read and write permissions in non-blocking mode
 *         Exit on failure of any mentioned above
 *
 * @return none
 */
 
static void open_device()
{
        struct stat st; // To store file information; st variable of type struct stat
				
        // return val -1 on error
        if (stat(dev_name, &st) == -1) //stat function retrieves info about file pointed to by dev_name and stores it in st structure
		{
                fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

		// st_mode is member of struct stat which contains info about file type and permissions.
        if (!S_ISCHR(st.st_mode)) //checks if mode is a char device; error if not char special file
		{
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }
		
        // open system call to open device name with reading and writing in non-blocking mode
        fd = open(dev_name, O_RDWR | O_NONBLOCK, 0); //file name, flags, mode
		// Access modes: O_NONBLOCK - file is opened in nonblocking mode (Doesn't wait for resource to be available)
		// O_RDWR - file opened for both reading and writing
		// mode = 0; no special permissions for file opened, use default, do not modify

        if (fd == -1) // exit on opening failure 
		{
                fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

/*************************************************************************
 *       Init MMAP Function called in init_device                        *
 *************************************************************************/

/**
 * @name   init_mmap
 * @brief  Initializes mem mapping for video capture buffers
 * @param  none
 * 
 * @descr  Requests buffers from the device driver and checks sufficiency
 *         Queries buffer info
 *         Allocates buffer structs
 *         For each buffer, sets type, memory index and maps buffer memory into process's address space
 *
 * @return none
 */

static void init_mmap()
{
        struct v4l2_requestbuffers req; //struct to request memory buffers from device driver

        CLEAR(req); //clears memory associated with req struct

        req.count = 6; // buffers requested from device
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // type requested video capture buffers
        req.memory = V4L2_MEMORY_MMAP; // memory mapping used for buffers

        if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1) //request memory buffers from the device drive check
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) //checks if requested number of buffers is sufficient
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = calloc(req.count, sizeof(*buffers)); //dynamically allocates memory for arr of buffer structs (each memory-mapped buffer)

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) //iterates over each buffer requested and obtained from device driver
		{ // for each buffer
                struct v4l2_buffer buf; //initializes a v4l2_buffer structure buf 

                CLEAR(buf); //clears its memory
				
				// set buffer type, memory and index
                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) //request to query buffer info from device driver
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length; //Stores buffer length obtained from driver in corresponding buffer struct in buffers arr
				//Uses mmap to map the buffer's memory into process's address space; mapped memory stored in start member of corresponding buffer struct
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start) // checks mmap failure
                        errno_exit("mmap");
        }
}

/*************************************************************************
 *                          Init Device Function                         *
 *************************************************************************/

/**
 * @name   init_device
 * @brief  Initializes video capture device
 * @param  none
 * 
 * @descr  Queries device's capabilties
 *         Checks if device supports video capture and streaming I/O
 *         Queries and sets cropping parameters
 *         Configures video format
 *         Ensures proper buffer size
 *         Initializes memory mapping
 *
 * @return none
 */

static void init_device()
{
    struct v4l2_capability cap; // struct holds device capabilities
    struct v4l2_cropcap cropcap; // struct holds cropping capabilities
    struct v4l2_crop crop; // struct holds cropping settings
    unsigned int min; // min buffer size

    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) // queries the device's capabilities 
    {
        if (errno == EINVAL) 
		{
            fprintf(stderr, "%s is no V4L2 device\n", dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }
	
	// checks if device supports video capture and streaming I/O by examining capabilities returned by query
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n", dev_name);
        exit(EXIT_FAILURE);
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
        exit(EXIT_FAILURE);
    }
  

    // Select video input, video standard and tune here.

    CLEAR(cropcap); // clears memory associated with cropcap struct

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //set type of cropping operation for video capture

    if (xioctl(fd, VIDIOC_CROPCAP, &cropcap) == 0) //Queries cropping capabilities of device and stores them in cropcap
    {   // on success enters
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (xioctl(fd, VIDIOC_S_CROP, &crop) == -1)
        {
			if(errno == EINVAL)
				fprintf(stderr, "Cropping not supported\n");
        }
    }

    CLEAR(fmt); //clears v4l2 format struct var

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; //set type of video capture

    if (force_format)
    {
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // This one work for Logitech C200
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
            errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (xioctl(fd, VIDIOC_G_FMT, &fmt) == -1)
            errno_exit("VIDIOC_G_FMT");
    }

    // Buggy driver paranoia.
	//precautionary measure for buggy drivers
	//ensures buffer size (sizeimage) is large enough to hold the captured image data
	// calculates min req buffer size based on the width, height, and bytes per line, and adjusts bytesperline and sizeimage if necessary.
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
		
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    init_mmap(); //initialize memory mapping for video capture (efficient data transfer between user space and device)
}

/*************************************************************************
 *                      Start capturing Function                         *
 *************************************************************************/

/**
 * @name   start_capturing
 * @brief  Prepares device for video capture bu queueing buffers for capturing and starting video stream
 * @param  none
 * 
 * @descr  Queues buffers
 *         Starts video stream
 *
 * @return none
 */

static void start_capturing()
{
        unsigned int i;
        enum v4l2_buf_type type; // buffer type for streaming

        for (i = 0; i < n_buffers; ++i) //iterates over each buffer allocated during initialization
        {
            //printf("allocated buffer %d\n", i);
            struct v4l2_buffer buf; //init struct

            CLEAR(buf); //clear it
			//sets buffer type, memory type, and buffer index
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (xioctl(fd, VIDIOC_QBUF, &buf) == -1) //request to enqueue the buffer for video capture
                errno_exit("VIDIOC_QBUF");
        }
		
		//After queuing all buffers, sets the type variable to V4L2_BUF_TYPE_VIDEO_CAPTURE to specify the buffer type for streaming
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) == -1) //request to start streaming video
            errno_exit("VIDIOC_STREAMON");
}

/*************************************************************************
 *         Dump PPM Function called in Image Processing Function         *
 *************************************************************************/
 
 /**
 * @name   start_capturing
 * @brief  Prepares device for video capture bu queueing buffers for capturing and starting video stream
 * @param  none
 * 
 * @descr  Queues buffers
 *         Starts video stream
 *
 * @return none
 */
 
char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="frames/test00000000.ppm";

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    // subtract 1 because sizeof for string includes null terminator
    written=write(dumpfd, ppm_header, sizeof(ppm_header)-1);

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}

/*************************************************************************
 * YUV to RGB Conversion Function called in Image Processing Function    *
 *************************************************************************/
 
 /**
 * @name   start_capturing
 * @brief  Prepares device for video capture bu queueing buffers for capturing and starting video stream
 * @param  none
 * 
 * @descr  Queues buffers
 *         Starts video stream
 *
 * @return none
 */

// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}

/*************************************************************************
 *          Image Processing Function called in Read_frame               *
 *************************************************************************/
 
 /**
 * @name   start_capturing
 * @brief  Prepares device for video capture bu queueing buffers for capturing and starting video stream
 * @param  none
 * 
 * @descr  Queues buffers
 *         Starts video stream
 *
 * @return none
 */

unsigned int framecnt=0;
unsigned char bigbuffer[(1280*960)];

static void process_image(const void *p, int size)
{
    int i, newi, newsize=0;
    struct timespec frame_time;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    printf("frame %d: ", framecnt);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

#if defined(COLOR_CONVERT)
        //printf("Dump YUYV converted to RGB size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        {
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }

        dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
#endif

    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("Dump RGB as-is size %d\n", size);
        dump_ppm(p, size, framecnt, &frame_time);
    }

    fflush(stderr);
    //fprintf(stderr, ".");
    fflush(stdout);
}

/*************************************************************************
 *                 Read frame Function called in Main loop               *
 *************************************************************************/
 
 /**
 * @name   start_capturing
 * @brief  Prepares device for video capture bu queueing buffers for capturing and starting video stream
 * @param  none
 * 
 * @descr  Queues buffers
 *         Starts video stream
 *
 * @return none
 */
 
static int read_frame()
{
    struct v4l2_buffer buf;
    unsigned int i;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for non-fatal errors too.*/
                return 0;


             default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);

    process_image(buffers[buf.index].start, buf.bytesused);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    return 1;
}

/*************************************************************************
 *                           Main loop function                          *
 *************************************************************************/
 
 /**
 * @name   mainloop
 * @brief  Prepares device for video capture bu queueing buffers for capturing and starting video stream
 * @param  none
 * 
 * @descr  Queues buffers
 *         Starts video stream
 *
 * @return none
 */

static void mainloop()
{
    unsigned int count;
    struct timespec read_delay;
    struct timespec time_error;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;

    count = frame_count;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }

        if(count <= 0) break;
    }
}


/*************************************************************************
 *                       Stop capturing Function                         *
 *************************************************************************/

/**
 * @name   stop_capturing
 * @brief  Initializes video capture device
 * @param  none
 * 
 * @descr  Stops video stream
 *
 * @return none
 */
 
static void stop_capturing()
{
        enum v4l2_buf_type type; // buffer type for streaming

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // sets type as video capture
        if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1) // request to stop video stream
            errno_exit("VIDIOC_STREAMOFF");
}

/*************************************************************************
 *                        UnInit Device Function                         *
 *************************************************************************/

/**
 * @name   uninit_device
 * @brief  Uninitializes video capture device
 * @param  none
 * 
 * @descr  Avoids mem leak by releasing allocated memory
 *
 * @return none
 */
 
static void uninit_device()
{
        unsigned int i;
		//munmap function deallocates memory region mapped by mmap, which was used for memory mapping the device buffers

        for (i = 0; i < n_buffers; ++i) // iterates over each buffer allocated during init phase
            if (munmap(buffers[i].start, buffers[i].length) == -1) // munmap function to unmap the memory associated with each buffer
                errno_exit("munmap");
			
        free(buffers); //After unmapping all buffers, frees memory allocated for array of buffer structs
}

/*************************************************************************
 *                          Close Device Function                         *
 *************************************************************************/

/**
 * @name   close_device
 * @brief  Closes camera device named dev_name
 * @param  none
 * 
 * @descr  Close camera device pointed to by fd
 *         Set fd as -1 to not accidentally use file after closure
 *
 * @return none
 */
 
static void close_device()
{
        if (close(fd) == -1) // check status of closing fd = camera device
		{
            errno_exit("close");
		}
        fd = -1; //set fd to -1, so fd is not accidentally used after closure
}


/*************************************************************************
 *                                 Main Function                         *
 *************************************************************************/
 
int main(int argc, char **argv)
{
	//argv[0] = name of program itself
    if(argc > 1)
        dev_name = argv[1]; // video device name provided in CLI
    else
        dev_name = "/dev/video0";
	
	printf("Starting camera driver...\n");
    open_device();
	printf("Camera device opened...\n");
    init_device();
	printf("Initialized device...\n");
	
    start_capturing();
    mainloop();
    stop_capturing();
	
    uninit_device();
	printf("Uninitialized device...\n");
    close_device();
	printf("Closed device...\n");
    fprintf(stderr, "\n");
	printf("Exiting program!\n");
    return 0;
}
