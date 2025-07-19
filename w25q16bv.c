#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/slab.h>    
#include <linux/delay.h>


// Parameters
#define DEVICE_NAME "spiflash"
#define CLASS_NAME  "w25q16"

#define W25Q16BV_TOTAL_SIZE      2097152 // 2 M-Bytes
#define W25Q16BV_SECTOR_SIZE     4096    // 4 K-Bytes
#define W25Q16BV_PAGE_SIZE       256

// SPI Commands
#define W25Q16BV_CMD_WRITE_ENABLE     0x06
#define W25Q16BV_CMD_SECTOR_ERASE     0x20
#define W25Q16BV_CMD_PAGE_PROGRAM     0x02
#define W25Q16BV_CMD_READ_STATUS1     0x05
#define W25Q16BV_CMD_READ_DATA        0x03
#define W25Q16BV_STATUS_BUSY_MASK     0x01


static int W25Q16BV_open(struct inode *inode, struct file *file);
static int W25Q16BV_release(struct inode *inode, struct file *file);
static ssize_t W25Q16BV_read(struct file *filp, char __user *buf, size_t len, loff_t *off);
static ssize_t W25Q16BV_write(struct file *filp, const char *buf, size_t len, loff_t *off);
static int w25q16bv_write_enable(struct spi_device *spi);
static int w25q16bv_read_status(struct spi_device *spi, u8 *status);
static int w25q16bv_wait_busy(struct spi_device *spi);

static dev_t dev_num;
static struct class* dev_class = NULL;
static struct cdev w25q16bv_cdev;
static struct spi_device *w25q16bv_spi_dev = NULL;

struct file_operations w25q16bv_fops = {
	.owner = THIS_MODULE,
	.read = W25Q16BV_read,
	.write = W25Q16BV_write,
	.open = W25Q16BV_open,
	.release = W25Q16BV_release
};


static int W25Q16BV_open(struct inode *inode, struct file *file){
	return 0;
}

static int W25Q16BV_release(struct inode *inode, struct file *file){
    return 0;
}

static ssize_t W25Q16BV_read(struct file *filp, char __user *buff, size_t len, loff_t *off){
    uint8_t *kernel_buf;
    uint8_t command[4];
    struct spi_transfer task[2];
    struct spi_message message;
    int ret;

	if (*off >= W25Q16BV_TOTAL_SIZE) {
    return 0;
	}
	if (*off + len > W25Q16BV_TOTAL_SIZE) {
    	len = W25Q16BV_TOTAL_SIZE - *off;
	}

	kernel_buf = kmalloc(len, GFP_KERNEL);
    if (!kernel_buf) {
        return -ENOMEM;
    }

    // Prepare for read
    command[0] = W25Q16BV_CMD_READ_DATA;
    command[1] = (*off >> 16) & 0xFF;
    command[2] = (*off >> 8) & 0xFF;
    command[3] = *off & 0xFF;

	spi_message_init(&message);
	memset(task, 0, sizeof(task));

	// Add the task to send command and address for read
	task[0].tx_buf = command;
	task[0].len = sizeof(command);
	spi_message_add_tail(&task[0], &message);

	// Add the task to read the data
    task[1].rx_buf = kernel_buf;
    task[1].len = len;
    spi_message_add_tail(&task[1], &message);

    ret = spi_sync(w25q16bv_spi_dev, &message);
    if (ret) {
        pr_err("SPI read failed\n");
        kfree(kernel_buf);
        return -EIO;
    }

	if (copy_to_user(buff, kernel_buf, len)) {
    	return -EFAULT;
    }

	kfree(kernel_buf);
    *off += len;
    return len;
}

static ssize_t W25Q16BV_write(struct file *filp, const char *buf, size_t len, loff_t *off){
    return 1;
}

static int W25Q16BV_probe(struct spi_device *spi){
	struct device *dev;
	int ret;

	w25q16bv_spi_dev = spi; // Define the spi device
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret){
		pr_err("Fail configuring spi device");
		return ret;
	}

	cdev_init(&w25q16bv_cdev, &w25q16bv_fops);
	ret = cdev_add(&w25q16bv_cdev, dev_num, 1);
    if (ret) {
        pr_err("Fail adding the cdev\n");
        return ret;
    }

    // Create the device in /dev
    dev = device_create(dev_class, NULL, dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(dev)) {
        pr_err("Fail creating the device\n");
        cdev_del(&w25q16bv_cdev);
        return PTR_ERR(dev);
    }
    
    pr_info("The driven was load, /dev/%s is created.\n", DEVICE_NAME);
    return 0;
}

static void W25Q16BV_remove(struct spi_device *spi) {
	pr_info("Remove function called\n");
    device_destroy(dev_class, dev_num);
    cdev_del(&w25q16bv_cdev);
    w25q16bv_spi_dev = NULL;
}

static const struct spi_device_id w25q16bv_spi_id[] = {
    { "w25q16bv", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, w25q16bv_spi_id);

static struct spi_driver w25q16bv_driver = {
    .driver = {
        .name = "w25q16bv",
        .owner = THIS_MODULE,
    },
    .id_table = w25q16bv_spi_id,
    .probe = W25Q16BV_probe,
    .remove = W25Q16BV_remove,
};




static int w25q16bv_write_enable(struct spi_device *spi){
    char cmd = W25Q16BV_CMD_WRITE_ENABLE;
    return spi_write(spi, &cmd, 1);
}

static int w25q16bv_read_status(struct spi_device *spi, u8 *status){
    return spi_write_then_read(spi, "\x05", 1, status, 1);
}

static int w25q16bv_wait_busy(struct spi_device *spi){
    uint8_t status;
    do {
        if (w25q16bv_read_status(spi, &status) < 0) {
            pr_err("Error reading the status\n");
            return -EIO;
        }
        if (status & W25Q16BV_STATUS_BUSY_MASK) {
            msleep(1);
        }
    } while (status & W25Q16BV_STATUS_BUSY_MASK);
    return 0;
}

static int __init w25q16bv_init(void) {
	int ret;
    pr_info("Initializing the driver\n");

    // Choose the major and minor number
    ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Fail choosing major and minor number\n");
        return ret;
    }
    pr_info("Major = %d, Minor = %d\n", MAJOR(dev_num), MINOR(dev_num));

    // Create dispositive class
    dev_class = class_create(CLASS_NAME);
    if (IS_ERR(dev_class)) {
        pr_err("Fail creating the dispositive class\n");
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(dev_class);
    }

    return spi_register_driver(&w25q16bv_driver);
}

static void __exit w25q16bv_exit(void) {
    spi_unregister_driver(&w25q16bv_driver);
    class_destroy(dev_class);
    unregister_chrdev_region(dev_num, 1);
	pr_info("W25Q16BV Driver removed\n");
}

module_init(w25q16bv_init);
module_exit(w25q16bv_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Caio Felipe");
MODULE_DESCRIPTION("Driver SPI  for W25Q16BV flash memory");























