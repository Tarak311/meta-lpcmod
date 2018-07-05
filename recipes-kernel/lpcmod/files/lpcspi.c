#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/uaccess.h>
#include <linux/socket.h>
#include <linux/netlink.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/ioctl.h>

//------------------------------------------------------------------------------------------------------------------------------//


#define SPI_MODE_MASK (SPI_CPHA|SPI_CPOL|SPI_CS_HIGH|SPI_LSB_FIRST|SPI_3WIRE|SPI_LOOP|SPI_NO_CS|SPI_READY|SPI_TX_DUAL|SPI_TX_QUAD|SPI_RX_DUAL \
		|SPI_RX_QUAD)

#define SPIDEV_MAJOR		156
#define N_SPI_MINORS		3


int sleep_condition;


static DECLARE_BITMAP(minors,N_SPI_MINORS);


struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct mutex 		buf_lock;
	unsigned		users;
	u8			*tx_buffer;

	u8			*rx_buffer;
	u32			speed_hz;
};

static ssize_t spidev_sync(struct spidev_data*, struct spi_message*);

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static unsigned bufsiz = 1024;
static struct class *spidev_class;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz,"lpc-parmeter");




//-----------------------------------------------------------------------------------------------------------------------------//
static struct spi_ioc_transfer * spidev_get_ioc_message(unsigned int cmd,struct spi_ioc_transfer __user *u_ioc, unsigned *n_ioc)
{
	struct spi_ioc_transfer *ioc;
	u32 tmp; // note that it is of 32 bit value
	// next line check if it is write and SPI_IOC_MESSAGE type
	if(_IOC_TYPE(cmd)!=SPI_IOC_MAGIC||_IOC_NR(cmd)!=_IOC_NR(SPI_IOC_MESSAGE(0))||_IOC_DIR(cmd)!= _IOC_WRITE)
		return ERR_PTR(-ENOTTY);
	tmp =_IOC_SIZE(cmd); //it copies size from command
	/*
	 *	IOCTL_WR_EXAMPLE_IOC
	 *	31-30 bits are for read/write
	 *	29-16 size
	 *	7-0 function
	 **/
	if((tmp % sizeof(struct spi_ioc_transfer))!=0) //check that our transfer(argument from cmd) is  is integral multiple of spi_ioc_transfer
		return ERR_PTR(-EINVAL); // as u_ioc is passed as pointer and someone needs to know number of transfers.
	*n_ioc=tmp/sizeof(struct spi_ioc_transfer);
	if(*n_ioc==0)
		return NULL;
	ioc=kmalloc(tmp,GFP_KERNEL);
	if(!ioc)
		return ERR_PTR(-ENOMEM);
	if(__copy_from_user(ioc,u_ioc,tmp)){
		kfree(ioc);
		return ERR_PTR(-EFAULT);
	}
	return ioc;
}





static int spidev_message(struct spidev_data *spidev,struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total, tx_total, rx_total;
	u8			*tx_buf, *rx_buf;
	int			status = -EFAULT;
printk("In spi msg init \n ");
	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	tx_buf = spidev->tx_buffer;
	rx_buf = spidev->rx_buffer;
	total = 0;
	tx_total = 0;
	rx_total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		/* Since the function returns the total length of transfers
		 * on success, restrict the total to positive int values to
		 * avoid the return value looking like an error.  Also check
		 * each transfer length to avoid arithmetic overflow.
		 */
		if (total > INT_MAX || k_tmp->len > INT_MAX) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			/* this transfer needs space in RX bounce buffer */
			rx_total += k_tmp->len;
			if (rx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->rx_buf = rx_buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
			rx_buf += k_tmp->len;
		}
		if (u_tmp->tx_buf) {
			/* this transfer needs space in TX bounce buffer */
			tx_total += k_tmp->len;
			if (tx_total > bufsiz) {
				status = -EMSGSIZE;
				goto done;
			}
			k_tmp->tx_buf = tx_buf;
			if (copy_from_user(tx_buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
			tx_buf += k_tmp->len;
		}

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->tx_nbits = u_tmp->tx_nbits;
		k_tmp->rx_nbits = u_tmp->rx_nbits;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
		if (!k_tmp->speed_hz)
			k_tmp->speed_hz = spidev->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}
 printk("In spi msg fun \n ");
	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	rx_buf = spidev->rx_buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, rx_buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			rx_buf += u_tmp->len;
		}
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}









//-----------------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------//
static ssize_t spidev_sync(struct spidev_data *spidat, struct spi_message *message)
{	DECLARE_COMPLETION_ONSTACK(done);
	int status;
	struct spi_device *spi;
	spin_lock_irq(&spidat->spi_lock);
	spi = spidat->spi;
	spin_unlock_irq(&spidat->spi_lock);

	if (spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_sync(spi,message);
	if (status==0)
		status = message->actual_length;
	return status;
}




 static inline ssize_t spidev_sync_write(struct spidev_data *spidat, size_t len)
{
		struct spi_transfer t = {
					.tx_buf 	= spidat->tx_buffer,
					.len		= len,
					.speed_hz	= spidat->speed_hz,
	};
	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&t,&m);
	return spidev_sync(spidat,&m);
}
 static inline ssize_t spidev_sync_read(struct spidev_data *spidat, size_t len)
{
		struct spi_transfer t = {
					 .rx_buf	= spidat->rx_buffer,
					 .len		= len,
					 .speed_hz	= spidat->speed_hz,
		};
		struct spi_message	 m;
		spi_message_init(&m);
		spi_message_add_tail(&t, &m);
		return spidev_sync(spidat, &m);
}
//-------------------------------------------------------------------------------//:wq
static uint8_t tx1[]={0x81, 0x73, 0x44, 0x55};
static uint8_t rx1[]={0x00, 0x00, 0x00, 0x00};
struct work_data {
			struct work_struct msg_work;
			int print_data;
			__u64 *tx;
			__u64 *rx;
			__u32 size;
			struct spidev_data* spidata;
			struct spi_device* spidev;
			struct spi_ioc_transfer *tr;
		};
static void msg_thread_handler(struct work_struct *my_work)
{
 int retval;
 struct work_data *any_data = container_of(my_work, struct work_data,msg_work);
 msleep(2000);
 //any_data->tx=tx1;
 //any_data->rx=rx1;
 printk("work started :%d\n",any_data->print_data);
 struct spi_ioc_transfer *ioc1;
 u32 tmp;
 unsigned n_ioc=1;
 ioc1 = spidev_get_ioc_message(SPI_IOC_MESSAGE(1), (struct spi_ioc_transfer __user *)any_data->tr, &n_ioc);
 if (IS_ERR(ioc1)) {
	 printk("something wrong");
 }
 if (!ioc1){
	 printk("can't allocate ioc");
 }
 retval = spidev_message(any_data->spidata,ioc1,n_ioc);
 msleep(2000);
 printk("Value of TX is:%08x\n", any_data->tr->tx_buf);
 printk("Value of RX is:%08x\n", any_data->tr->rx_buf);
 printk("work handled :%d\n",any_data->print_data);
 kfree(any_data);
 }
static int LPC_probe(struct spi_device *spi)
{

	struct spidev_data *spidat;
	struct work_data* my_msg_work;
  my_msg_work = kmalloc(sizeof(struct work_data), GFP_KERNEL);
	spidat=kzalloc(sizeof(*spidat),GFP_KERNEL);
	if(!spidat)
		return -ENOMEM;
	spidat->spi=spi;
	struct spi_ioc_transfer ioc_tr={
  	.rx_buf=rx1,
  	.tx_buf=tx1,
  	.len= ARRAY_SIZE(tx1),
  	.delay_usecs = 0,
  	.speed_hz = spi->max_speed_hz,
  	.bits_per_word =spi->bits_per_word
  };
	spin_lock_init(&spidat->spi_lock);
	mutex_init(&spidat->buf_lock);
	INIT_LIST_HEAD(&spidat->device_entry);
	int status;
	unsigned long minor;
	mutex_lock(&device_list_lock);
	minor=find_first_zero_bit(minors,N_SPI_MINORS);
	if (minor< N_SPI_MINORS) {
		struct device *dev;
		spidat->devt=MKDEV(SPIDEV_MAJOR,minor); //it create dev number
		dev =device_create(spidev_class,&spi->dev,spidat->devt,spidat,"lpc1769");
		status=PTR_ERR_OR_ZERO(dev);
	}
	else {
		dev_dbg(&spi->dev,"no minor available\n");
		status=-ENODEV;
	}
	if (status==0){
		set_bit(minor,minors); // this is to set bit in an array of minors number
		list_add(&spidat->device_entry,&device_list);
	}
	mutex_unlock(&device_list_lock);
	spidat->speed_hz=spi->max_speed_hz;
	if(status==0)
		spi_set_drvdata(spi,spidat);
	else
		kfree(spidat);

	my_msg_work->print_data = 1;
	my_msg_work->spidev = spi;
  my_msg_work->tr=&ioc_tr;
	my_msg_work->spidata=spidat;
	INIT_WORK(&my_msg_work->msg_work,msg_thread_handler);
	schedule_work(&my_msg_work->msg_work);
	printk(KERN_EMERG "CS: %d\n",spi->chip_select);
	printk(KERN_EMERG "LPC Module probed\n");
	printk("spi->mode: %04x\n",spi->mode);
	return status;

}
static const struct of_device_id lpc1769_of_ids[]={
	{.compatible="linux,lpcspi"},
	{},
};
MODULE_DEVICE_TABLE(of,lpc1769_of_ids);


//--------------------------------------------------------------------------------//
static struct spi_driver lpc1769_spi_driver = {

	.driver = {
		.name="lpc1769_spi",
		.of_match_table = of_match_ptr(lpc1769_of_ids),
	},
	.probe=LPC_probe,
};
//--------------------------------------------------------------------------------//
//*****************************Init-exit****************************************//
//--------------------------------------------------------------------------------//
static int __init Init_LPC(void)
	{
		int status;
		printk(KERN_EMERG "LPC moudle initialized \n");
		spidev_class=class_create(THIS_MODULE,"soc_spi_dev");
		if (IS_ERR(spidev_class)){
			return PTR_ERR(spidev_class);
		}
		status=spi_register_driver(&lpc1769_spi_driver);

		if(status<0)
		{
			class_destroy(spidev_class);
			printk("You are screwed\n");
		}

		return 0;

	}


static void __exit Exit_LPC(void)
	{
		spi_unregister_driver(&lpc1769_spi_driver);
		class_destroy(spidev_class);
		printk(KERN_EMERG " LPC	moudle unloaded \n");
	}

module_init(Init_LPC);
module_exit(Exit_LPC);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tarak Patel <tarak_1@live.com>");
MODULE_DESCRIPTION("lpc driver");
