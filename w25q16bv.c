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

#define DEVICE_NAME "spiflash"
#define CLASS_NAME  "w25q16"

#define W25Q16BV_TOTAL_SIZE   0x200000 /* 2 MiB */
#define W25Q16BV_SECTOR_SIZE  0x1000   /* 4 KiB */
#define W25Q16BV_PAGE_SIZE    0x100    /* 256 B */

// SPI Commands
#define W25Q16BV_CMD_WRITE_ENABLE 0x06
#define W25Q16BV_CMD_SECTOR_ERASE 0x20
#define W25Q16BV_CMD_PAGE_PROGRAM 0x02
#define W25Q16BV_CMD_READ_STATUS1 0x05
#define W25Q16BV_CMD_READ_DATA    0x03
#define W25Q16BV_STATUS_BUSY_MASK 0x01

// Declaration of principal functions
static int  W25Q16BV_open(struct inode *, struct file *);
static int  W25Q16BV_release(struct inode *, struct file *);
static ssize_t W25Q16BV_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t W25Q16BV_write(struct file *, const char __user *, size_t, loff_t *);
static loff_t W25Q16BV_llseek(struct file *, loff_t, int);

static int  w25q16bv_write_enable(struct spi_device *);
static int  w25q16bv_read_status(struct spi_device *, uint8_t *);
static int  w25q16bv_wait_busy(struct spi_device *);
static int  w25q16bv_page_program(loff_t, const uint8_t *, size_t);
static int  w25q16bv_sector_erase(loff_t);

// Global variables
static dev_t dev_num;
static struct class *dev_class;
static struct cdev   w25q16bv_cdev;
static struct spi_device *w25q16bv_spi_dev;

// Definition of fops functions
static const struct file_operations w25q16bv_fops = {
        .owner   = THIS_MODULE,
        .read    = W25Q16BV_read,
        .write   = W25Q16BV_write,
        .llseek  = W25Q16BV_llseek,
        .open    = W25Q16BV_open,
        .release = W25Q16BV_release,
};

// Helper functions
static int w25q16bv_write_enable(struct spi_device *spi){
    uint8_t cmd = W25Q16BV_CMD_WRITE_ENABLE; 
    return spi_write(spi, &cmd, 1); 
}

static int w25q16bv_read_status(struct spi_device *spi, uint8_t *st){
    return spi_write_then_read(spi, (uint8_t[]){W25Q16BV_CMD_READ_STATUS1}, 1, st, 1);
}

static int w25q16bv_wait_busy(struct spi_device *spi){
    uint8_t st; 
    do { 
        if (w25q16bv_read_status(spi, &st)) return -EIO; 
        if (st & 1) msleep(1); 
    } while (st & 1); 
    return 0; 
}

static int w25q16bv_sector_erase(loff_t sector){
   int ret; 
   uint8_t cmd[4];
   cmd[0] = W25Q16BV_CMD_SECTOR_ERASE;
   cmd[1] = (sector>>16)&0xFF;
   cmd[2] = (sector>>8)&0xFF;
   cmd[3] = sector&0xFF; 

   ret = w25q16bv_write_enable(w25q16bv_spi_dev); 
   if(ret) return ret; 
   
   ret = spi_write(w25q16bv_spi_dev, cmd, 4); 
   if(ret) return ret; 
   return w25q16bv_wait_busy(w25q16bv_spi_dev);
} 

static int w25q16bv_page_program(loff_t addr, const uint8_t *src, size_t len){
   int ret; 
   uint8_t hdr[4];
   hdr[0] = W25Q16BV_CMD_PAGE_PROGRAM;
   hdr[1] = (addr>>16)&0xFF;
   hdr[2] = (addr>>8)&0xFF;
   hdr[3] = addr&0xFF; 
   struct spi_transfer task[2] = {0}; 
   struct spi_message message; 

   ret=w25q16bv_write_enable(w25q16bv_spi_dev); 
   if(ret) return ret; 
   
   spi_message_init(&message); 
   task[0].tx_buf=hdr; 
   task[0].len=4; 
   spi_message_add_tail(&task[0], &message); 
   
   task[1].tx_buf=src; 
   task[1].len=len; 
   spi_message_add_tail(&task[1], &message); 
   
   ret=spi_sync(w25q16bv_spi_dev, &message); 
   if(ret) return ret; 

   return w25q16bv_wait_busy(w25q16bv_spi_dev);
} 

// Definition of principal functions for the driver
static int W25Q16BV_open(struct inode *inode, struct file *file){
    return 0; 
}
static int W25Q16BV_release(struct inode *inode, struct file *file){
    return 0; 
}

static loff_t W25Q16BV_llseek(struct file *f, loff_t off, int whence){
   return fixed_size_llseek(f, off, whence, W25Q16BV_TOTAL_SIZE); 
}

static ssize_t W25Q16BV_read(struct file *filp, char __user *buff, size_t len, loff_t *off){
    uint8_t *kbuf; 
    struct spi_transfer task[2]={0}; 
    struct spi_message message; 
    uint8_t cmd[4]; 
    int ret;
    
    if(*off >= W25Q16BV_TOTAL_SIZE) return 0; 

    if(*off+len > W25Q16BV_TOTAL_SIZE) len = W25Q16BV_TOTAL_SIZE-*off;

    kbuf = kmalloc(len, GFP_KERNEL);
    if(!kbuf) return -ENOMEM;

    // Configure command to read data
    cmd[0] = W25Q16BV_CMD_READ_DATA; 
    cmd[1] = (*off>>16)&0xFF; 
    cmd[2] = (*off>>8)&0xFF; 
    cmd[3] = *off&0xFF;
    spi_message_init(&message); 
    task[0].tx_buf = cmd; 
    task[0].len = 4; 
    spi_message_add_tail(&task[0], &message); 

    task[1].rx_buf = kbuf; 
    task[1].len = len; 
    spi_message_add_tail(&task[1], &message);

    // Send command to read data
    ret = spi_sync(w25q16bv_spi_dev, &message); 
    if(ret){
        kfree(kbuf); 
        return -EIO;
    }

    // Copy the readed data to the buffer
    if(copy_to_user(buff, kbuf, len)){
        kfree(kbuf); 
        return -EFAULT;
    }
    kfree(kbuf); 
    *off+=len; 
    return len;
}

static ssize_t W25Q16BV_write(struct file *filp, const char __user *ubuf, size_t len, loff_t *off){
    uint8_t *page; 
    ssize_t written = 0; 
    int ret = 0; 
    if(*off >= W25Q16BV_TOTAL_SIZE) return -ENOSPC; 
    if(*off + len > W25Q16BV_TOTAL_SIZE) len = W25Q16BV_TOTAL_SIZE - *off;
    page = kmalloc(W25Q16BV_PAGE_SIZE, GFP_KERNEL); 
    if(!page) return -ENOMEM;
    
    while(written < len){ 
        loff_t addr = *off + written; 
        loff_t pbase = addr & ~(loff_t)(W25Q16BV_PAGE_SIZE-1); 
        size_t poff = addr & (W25Q16BV_PAGE_SIZE-1); 
        size_t chunk = min(len-written, W25Q16BV_PAGE_SIZE-poff); 
        loff_t sbase = (addr/W25Q16BV_SECTOR_SIZE) * W25Q16BV_SECTOR_SIZE;
         
        struct spi_transfer task[2]={0}; 
        struct spi_message message; 
        uint8_t hdr[4];

        // Read the sector
        hdr[0] = W25Q16BV_CMD_READ_DATA;
        hdr[1] = (pbase>>16)&0xFF;
        hdr[2] = (pbase>>8)&0xFF;
        hdr[3] = pbase&0xFF; 
        
        spi_message_init(&message); 
        task[0].tx_buf = hdr; 
        task[0].len = 4; 
        spi_message_add_tail(&task[0], &message);

        task[1].rx_buf = page; 
        task[1].len = W25Q16BV_PAGE_SIZE; 
        spi_message_add_tail(&task[1], &message); 
        
        ret = spi_sync(w25q16bv_spi_dev, &message); 
        if(ret) goto out; 
        
        // Copy readed data to user
        if(copy_from_user(page+poff, ubuf+written, chunk)){ 
            ret=-EFAULT; 
            goto out; 
        }
        for(size_t i=0;i<W25Q16BV_PAGE_SIZE;i++){ 
            if(~page[i]){ 
                ret = w25q16bv_sector_erase(sbase); 
                if(ret) goto out; 
                break;
            } 
        }
        ret = w25q16bv_page_program(pbase, page, W25Q16BV_PAGE_SIZE); 
        if(ret) goto out; 
        written += chunk; 
    }
    *off += written; 
    ret = written;
    out:   
        kfree(page); 
        return ret;
}

static int W25Q16BV_probe(struct spi_device *spi){ 
    struct device *dev; 
    int ret; 

    // Configure SPI
    w25q16bv_spi_dev = spi; 
    spi->mode = SPI_MODE_0; 
    spi->bits_per_word = 8; 
    spi->max_speed_hz = 1000000; 

    ret = spi_setup(spi); 
    if(ret) {
        pr_err("Fail configuring spi device");
        return ret;
    } 

    // Configure the character device 
    cdev_init(&w25q16bv_cdev, &w25q16bv_fops); 
    ret = cdev_add(&w25q16bv_cdev, dev_num, 1); 
    if(ret) {
        pr_err("Fail adding the cdev\n");
        return ret;
    } 
    
    // Create the device
    dev = device_create(dev_class, NULL, dev_num, NULL, DEVICE_NAME); 
    if(IS_ERR(dev)){ 
        cdev_del(&w25q16bv_cdev); 
        return PTR_ERR(dev);
    } 
    pr_info("The driven was load, /dev/%s is created.\n", DEVICE_NAME);
    return 0; 
}

static void W25Q16BV_remove(struct spi_device *spi){ 
    device_destroy(dev_class, dev_num); 
    cdev_del(&w25q16bv_cdev); 
    w25q16bv_spi_dev=NULL; 
    pr_info("W25Q16BV driver removed\n"); 
}

static const struct spi_device_id w25q16bv_id[] = {
    {"w25q16bv", 0},
    {}
}; 
MODULE_DEVICE_TABLE(spi, w25q16bv_id);

static struct spi_driver w25q16bv_driver = { 
    .driver={
        .name="w25q16bv",
        .owner=THIS_MODULE,
    }, 
    .probe=W25Q16BV_probe, 
    .remove=W25Q16BV_remove, 
    .id_table=w25q16bv_id 
};

static int __init w25q16bv_init(void){ 
    pr_info("Initializing the driver\n");

    // Choose the major and minor number
    int ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME); 
    if(ret) {
        pr_err("Fail choosing major and minor number\n");
        return ret;
    } 
    pr_info("Major = %d, Minor = %d\n", MAJOR(dev_num), MINOR(dev_num));

    // Create dispositive class
    dev_class = class_create(CLASS_NAME); 
    if(IS_ERR(dev_class)){ 
        pr_err("Fail creating the dispositive class\n");
        unregister_chrdev_region(dev_num, 1); 
        return PTR_ERR(dev_class);
    } 
    return spi_register_driver(&w25q16bv_driver);
} 

static void __exit w25q16bv_exit(void){ 
    spi_unregister_driver(&w25q16bv_driver); 
    class_destroy(dev_class); 
    unregister_chrdev_region(dev_num, 1);
    pr_info("W25Q16BV Exit Driver\n");
} 

module_init(w25q16bv_init); 
module_exit(w25q16bv_exit);

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("Caio Felipe"); 
MODULE_DESCRIPTION("Driver SPI for W25Q16BV flash memory");

// echo -n "testando o driver para a memoria flash" | sudo dd of=/dev/spiflash bs=1

// sudo dd if=/dev/spiflash bs=1 count=64 |hexdump -C