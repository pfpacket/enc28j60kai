#include <linux/module.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "enc28j60kai"

struct enc_device {
    struct spi_device *spi;
};

static int enc_probe(struct spi_device *spi)
{
	dev_warn(&spi->dev, "%s: probe", DRIVER_NAME);
	return 0;
}

static int enc_remove(struct spi_device *spi)
{
	dev_warn(&spi->dev, "%s: remove", DRIVER_NAME);
	return 0;
}

static const struct of_device_id enc28j60kai_dev_ids[] = {
	{ .compatible = "microchip,enc28j60kai" },
	{}
};
MODULE_DEVICE_TABLE(of ,enc28j60kai_dev_ids);

static struct spi_driver enc_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = enc28j60kai_dev_ids,
	},
	.probe = enc_probe,
	.remove = enc_remove,
};

module_spi_driver(enc_driver);

MODULE_AUTHOR("Ryo Munakata");
MODULE_LICENSE("GPL v2");
