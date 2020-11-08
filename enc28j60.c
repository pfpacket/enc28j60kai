#include <linux/module.h>
#include <linux/spi/spi.h>

struct enc_device {
    struct spi_device *spi;
};

static int __init enc_init(void)
{
	pr_info("registering enc28j60");
	return 0;
}

static void __exit enc_exit(void)
{
	pr_info("unregistering enc28j60");
}

module_init(enc_init);
module_exit(enc_exit);

MODULE_AUTHOR("Ryo Munakata");
MODULE_LICENSE("GPL v2");
