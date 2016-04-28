#include "led_matrix.h"
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

#define FB_FILENAME "/dev/fb1"

LedMatrix::LedMatrix() {
    fd_ = open(FB_FILENAME, O_RDWR);
    if (fd_ == -1) {
        perror("Error (call to 'open')");
        exit(EXIT_FAILURE);
    }

    fb_fix_screeninfo fix_info;
    if (ioctl(fd_, FBIOGET_FSCREENINFO, &fix_info) == -1) {
        perror("Error (call to 'ioctl')");
        close(fd_);
        exit(EXIT_FAILURE);
    }

    if (std::strcmp(fix_info.id, "RPi-Sense FB") != 0) {
        printf("%s\n", "Error: RPi-Sense FB not found");
        close(fd_);
        exit(EXIT_FAILURE);
    }

    buffer_ = (uint16_t*)(mmap(NULL, Width * Height * sizeof(uint16_t),
                            PROT_READ|PROT_WRITE, MAP_SHARED, fd_, 0));
    if (buffer_ == MAP_FAILED) {
        close(fd_);
        perror("Error mmapping the file");
        exit(EXIT_FAILURE);
    }
}

LedMatrix::~LedMatrix() {
    munmap(buffer_, Width * Height * sizeof(uint16_t));
    close(fd_);
}
