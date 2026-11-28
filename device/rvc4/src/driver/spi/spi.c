#include "spi.h"

#ifndef SPI_IOC_WR_BUS_LOCKED
    #define SPI_IOC_RD_BUS_LOCKED _IOR(SPI_IOC_MAGIC, 6, __u32)
    #define SPI_IOC_WR_BUS_LOCKED _IOW(SPI_IOC_MAGIC, 6, __u32)
#endif

#include <fcntl.h>      // open()
#include <string.h>     // memset()
#include <sys/ioctl.h>  // ioctl()
#include <unistd.h>     // close()

static long is_bus_lock_supported;
int spi_init(spi_t* self,         // mode: mask of SPI_LOOP, SPI_CPHA, SPI_CPOL, SPI_LSB_FIRST, SPI_CS_HIGH, SPI_3WIRE, SPI_NO_CS, SPI_READY or 0
             const char* device,  // filename like "/dev/spidev0.0"
             int mode,            // SPI_* (look "linux/spi/spidev.h")
             int bits,            // bits per word (usually 8)
             int speed)           // max speed [Hz]
{
    int retVal = 0;
    long is_bus_locked;
    self->fd = open(device, O_RDWR);
    if(self->fd < 0) {
        SPI_DBG("error in spi_init(): failed to open the bus");
        return SPI_ERR_OPEN;
    }

    self->mode = (__u8)mode;
    if(ioctl(self->fd, SPI_IOC_WR_MODE, &self->mode) < 0) {
        SPI_DBG("error in spi_init(): can't set bus mode");
        return SPI_ERR_SET_MODE;
    }

    if(ioctl(self->fd, SPI_IOC_RD_MODE, &self->mode) < 0) {
        SPI_DBG("error in spi_init(): can't get bus mode");
        return SPI_ERR_GET_MODE;
    }

    if(ioctl(self->fd, SPI_IOC_RD_LSB_FIRST, &self->lsb) < 0) {
        SPI_DBG("error in spi_init(): can't get 'LSB first'");
        return SPI_ERR_GET_LSB;
    }

    if(bits) {
        self->bits = (__u8)bits;
        if(ioctl(self->fd, SPI_IOC_WR_BITS_PER_WORD, &self->bits) < 0) {
            SPI_DBG("error in spi_init(): can't set bits per word");
            return SPI_ERR_SET_BITS;
        }
    }

    if(ioctl(self->fd, SPI_IOC_RD_BITS_PER_WORD, &self->bits) < 0) {
        SPI_DBG("error in spi_init(): can't get bits per word");
        return SPI_ERR_GET_BITS;
    }

    if(speed) {
        self->speed = (__u32)speed;
        if(ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &self->speed) < 0) {
            SPI_DBG("error in spi_init(): can't set max speed [Hz]");
            return SPI_ERR_SET_SPEED;
        }
    }

    if(ioctl(self->fd, SPI_IOC_RD_MAX_SPEED_HZ, &self->speed) < 0) {
        SPI_DBG("error in spi_init(): can't get max speed [Hz]");
        return SPI_ERR_SET_SPEED;
    }

    retVal = ioctl(self->fd, SPI_IOC_RD_BUS_LOCKED, &is_bus_locked);
    if(retVal) {
        is_bus_lock_supported = 0;
        SPI_DBG("error in spi_init(): can't get bus locked status, assuming it's not supported");
    } else {
        is_bus_lock_supported = 1;
    }
    SPI_DBG("open device='%s' mode=%d bits=%d lsb=%d max_speed=%d [Hz]", device, (int)self->mode, (int)self->bits, (int)self->lsb, (int)self->speed);

    return SPI_ERR_NONE;
}

int spi_set_mode(spi_t* self, int mode) {
    self->mode = (__u8)mode;
    if(ioctl(self->fd, SPI_IOC_WR_MODE, &self->mode) < 0) {
        SPI_DBG("error in spi_init(): can't set bus mode");
        return SPI_ERR_SET_MODE;
    }

    if(ioctl(self->fd, SPI_IOC_RD_MODE, &self->mode) < 0) {
        SPI_DBG("error in spi_init(): can't get bus mode");
        return SPI_ERR_GET_MODE;
    }

    SPI_DBG("set mode to=%d, actual=%d", mode, (int)self->mode);

    return SPI_ERR_NONE;
}

int spi_set_speed(spi_t* self, int speed) {
    self->speed = (__u32)speed;
    if(ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &self->speed) < 0) {
        SPI_DBG("error in spi_init(): can't set max speed [Hz]");
        return SPI_ERR_SET_SPEED;
    }

    if(ioctl(self->fd, SPI_IOC_RD_MAX_SPEED_HZ, &self->speed) < 0) {
        SPI_DBG("error in spi_init(): can't get max speed [Hz]");
        return SPI_ERR_SET_SPEED;
    }

    SPI_DBG("set speed max_speed=%d [Hz], actual max_speed=%d [Hz]", speed, (int)self->speed);

    return SPI_ERR_NONE;
}

void spi_free(spi_t* self) {
    close(self->fd);
}
int spi_read(spi_t* self, void* rx_buf, int len) {
    int retv;

    struct spi_ioc_transfer xfer[1] = {0};

    xfer[0].tx_buf = (__u64)0;       // output buffer
    xfer[0].rx_buf = (__u64)rx_buf;  // input buffer
    xfer[0].len = (__u32)len;        // length of data to read

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), xfer);
    if(retv < 0) {
        SPI_DBG("error in spi_read(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_READ;
    }

    return retv;
}
int spi_write(spi_t* self, const void* tx_buf, int len) {
    int retv;

    struct spi_ioc_transfer xfer[1] = {0};

    xfer[0].tx_buf = (__u64)tx_buf;  // output buffer
    xfer[0].rx_buf = (__u64)0;       // input buffer
    xfer[0].len = (__u32)len;        // length of data to write

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), xfer);
    if(retv < 0) {
        SPI_DBG("error in spi_write(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_WRITE;
    }

    return retv;
}
int spi_exchange(spi_t* self, void* rx_buf, const void* tx_buf, int len) {
    int retv;

    struct spi_ioc_transfer xfer[1] = {0};

    xfer[0].tx_buf = (__u64)tx_buf;  // output buffer
    xfer[0].rx_buf = (__u64)rx_buf;  // input buffer
    xfer[0].len = (__u32)len;        // length of data to write

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(1), xfer);
    if(retv < 0) {
        SPI_DBG("error in spi_exchange(): ioctl(SPI_IOC_MESSAGE(1)) return %d", retv);
        return SPI_ERR_EXCHANGE;
    }

    return retv;
}
int spi_read_reg8(spi_t* self, uint8_t reg_addr, void* rx_buf, int len) {
    int retv;

    struct spi_ioc_transfer xfer[2] = {0};

    xfer[0].tx_buf = (__u64)&reg_addr;      // output buffer
    xfer[0].rx_buf = (__u64)0;              // input buffer
    xfer[0].len = (__u32)sizeof(reg_addr);  // length of data to read

    xfer[1].tx_buf = (__u64)0;       // output buffer
    xfer[1].rx_buf = (__u64)rx_buf;  // input buffer
    xfer[1].len = (__u32)len;        // length of data to read

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(2), xfer);
    if(retv < 0) {
        SPI_DBG("error in spi_read_reg8(): ioctl(SPI_IOC_MESSAGE(2)) return %d", retv);
        return SPI_ERR_READ;
    }

    return retv;
}
int spi_write_reg8(spi_t* self, uint8_t reg_addr, const void* tx_buf, int len) {
    int retv;

    struct spi_ioc_transfer xfer[2] = {0};

    xfer[0].tx_buf = (__u64)&reg_addr;      // output buffer
    xfer[0].rx_buf = (__u64)0;              // input buffer
    xfer[0].len = (__u32)sizeof(reg_addr);  // length of data to write

    xfer[1].tx_buf = (__u64)tx_buf;  // output buffer
    xfer[1].rx_buf = (__u64)0;       // input buffer
    xfer[1].len = (__u32)len;        // length of data to write

    retv = ioctl(self->fd, SPI_IOC_MESSAGE(2), xfer);
    if(retv < 0) {
        SPI_DBG("error in spi_write_reg8(): ioctl(SPI_IOC_MESSAGE(2)) return %d", retv);
        return SPI_ERR_WRITE;
    }

    return retv;
}

int spi_lock(spi_t* self) {
    int retv = 0;
    uint64_t lock_mode = 1;
    if(is_bus_lock_supported && ioctl(self->fd, SPI_IOC_WR_BUS_LOCKED, &lock_mode) < 0) {
        SPI_DBG("error in spi_init(): can't lock bus");
        retv = SPI_ERR_WRITE;
    }
    return retv;
}

int spi_unlock(spi_t* self) {
    int retv = 0;
    uint64_t lock_mode = 0;
    if(is_bus_lock_supported && ioctl(self->fd, SPI_IOC_WR_BUS_LOCKED, &lock_mode) < 0) {
        SPI_DBG("error in spi_init(): can't unlock bus");
        retv = SPI_ERR_WRITE;
    }

    return retv;
}
