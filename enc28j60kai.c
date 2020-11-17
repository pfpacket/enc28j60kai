#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>

#define DRIVER_NAME "enc28j60kai"

enum bank_number {
	BANK_0 = 0,
	BANK_1 = 1,
	BANK_2 = 2,
	BANK_3 = 3,
	BANK_COMMON = 255,
};

struct control_register {
	enum bank_number bank;
	uint8_t address;
};

#define CONTROL_REGISTER(b, a) \
	{ \
		.bank = (b), \
		.address = (a) \
	}

/* Common registers */
const struct control_register EIE	= CONTROL_REGISTER(BANK_COMMON, 0x1b);
const struct control_register EIR	= CONTROL_REGISTER(BANK_COMMON, 0x1c);
const struct control_register ESTAT	= CONTROL_REGISTER(BANK_COMMON, 0x1d);
const struct control_register ECON2	= CONTROL_REGISTER(BANK_COMMON, 0x1e);
const struct control_register ECON1	= CONTROL_REGISTER(BANK_COMMON, 0x1f);

/* Bank 3 */
const struct control_register EREVID = CONTROL_REGISTER(BANK_3, 0x12);

#define ECON1_BSEL1 0x02
#define ECON1_BSEL0 0x01

enum spi_command {
	SPI_COM_RCR = 0x0,
	SPI_COM_RBM = 0x1,
	SPI_COM_WCR = 0x2,
	SPI_COM_WBM = 0x3,
	SPI_COM_BFS = 0x4,
	SPI_COM_BFC = 0x5,
	SPI_COM_SRC = 0xff,
};

union spi_operation {
	uint8_t raw8;
	struct __attribute__ ((packed)) {
		uint8_t arg:5;
		uint8_t opcode:3;
	} op;
};

struct enc_driver_data {
	struct net_device *netdev;
	struct spi_device *spi;
};

static void enc_soft_reset(struct spi_device *spi)
{
	spi_w8r8(spi, SPI_COM_SRC);
	udelay(2000);
}

static void enc_set_bank(struct spi_device *spi, enum bank_number bank);

static uint8_t spi_enc_read(struct spi_device *spi, enum spi_command com, struct control_register reg)
{
	union spi_operation spi_op = {
		.op = {
			.arg = reg.address,
			.opcode = com,
		}
	};

	enc_set_bank(spi, reg.bank);

	return (uint8_t)spi_w8r8(spi, spi_op.raw8);
}

static void spi_enc_write(struct spi_device *spi, enum spi_command com, struct control_register reg, uint8_t data)
{
	union spi_operation spi_op = {
		.op = {
			.arg = reg.address,
			.opcode = com,
		}
	};
	uint8_t txbuf[2] = {spi_op.raw8, data};
	uint8_t rxbuf;

	enc_set_bank(spi, reg.bank);

	spi_write_then_read(spi, txbuf, 2, &rxbuf, 1);
}

static void enc_set_bank(struct spi_device *spi, enum bank_number bank)
{
	if (bank != BANK_COMMON) {
		spi_enc_write(spi, SPI_COM_BFC, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
		spi_enc_write(spi, SPI_COM_BFS, ECON1, bank);
	}
}


static int enc_netdev_open(struct net_device *dev)
{
	return 0;
}

static int enc_netdev_stop(struct net_device *dev)
{
	return 0;
}

static netdev_tx_t enc_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct enc_driver_data *priv = netdev_priv(dev);

	dev_info(&priv->spi->dev, "%s: called", __func__);

	return NETDEV_TX_OK;
}

static const struct net_device_ops enc_netdev_ops = {
	.ndo_open = enc_netdev_open,
	.ndo_stop = enc_netdev_stop,
	.ndo_start_xmit = enc_start_xmit,
	.ndo_validate_addr = eth_validate_addr
};

static int enc_probe(struct spi_device *spi)
{
	int ret, revision;
	struct net_device *netdev;
	struct enc_driver_data *priv;

	netdev = alloc_etherdev(sizeof (struct enc_driver_data));
	if (!netdev)
		goto err_alloc;

	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->spi = spi;
	spi_set_drvdata(spi, priv);

	SET_NETDEV_DEV(netdev, &spi->dev);

	enc_soft_reset(spi);

	revision = spi_enc_read(spi, SPI_COM_RCR, EREVID);
	dev_info(&spi->dev, "%s: revision=%d", __func__, revision);

	if (revision == 0 || revision == 0xff) {
		return -EINVAL;
	}

	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = &enc_netdev_ops;

	ret = register_netdev(netdev);
	if (ret) {
		return ret;
	}

	return 0;

err_alloc:
	return -ENOMEM;
}

static int enc_remove(struct spi_device *spi)
{
	struct enc_driver_data *priv = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s", __func__);

	unregister_netdev(priv->netdev);
	free_netdev(priv->netdev);

	return 0;
}

static const struct of_device_id enc28j60kai_dev_ids[] = {
	{ .compatible = "microchip,enc28j60" },
	{ }
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
