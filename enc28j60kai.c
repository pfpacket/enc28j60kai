#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include "enc28j60kai.h"

#define DRIVER_NAME "enc28j60kai"

#define MAC_MAX_FRAME_LEN 1518

static const uint8_t DEFAULT_MAC_ADDR[6] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};

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
	struct enc_driver_data *priv = netdev_priv(dev);

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

static int enc_set_mac_addr(struct enc_driver_data *priv, const char *address)
{
	const char *addr = address;
	spi_enc_write(priv->spi, SPI_COM_WCR, MAADR6, addr[0]);
	spi_enc_write(priv->spi, SPI_COM_WCR, MAADR5, addr[1]);
	spi_enc_write(priv->spi, SPI_COM_WCR, MAADR4, addr[2]);
	spi_enc_write(priv->spi, SPI_COM_WCR, MAADR3, addr[3]);
	spi_enc_write(priv->spi, SPI_COM_WCR, MAADR2, addr[4]);
	spi_enc_write(priv->spi, SPI_COM_WCR, MAADR1, addr[5]);
	return 0;
}

static int enc_set_mac_address(struct net_device *dev, void *addr)
{
	const char *a = addr;
	struct enc_driver_data *priv = netdev_priv(dev);

	dev_info(&priv->spi->dev, "%s: %x:%x:%x:%x:%x:%x",
			__func__, a[5], a[4], a[3], a[2], a[1], a[0]);

	return enc_set_mac_addr(priv, addr);
}

static const struct net_device_ops enc_netdev_ops = {
	.ndo_open = enc_netdev_open,
	.ndo_stop = enc_netdev_stop,
	.ndo_start_xmit = enc_start_xmit,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = enc_set_mac_address
};

static int enc_init_rx(struct enc_driver_data *priv)
{
	struct spi_device *spi = priv->spi;
	uint8_t enable_rx, macon3, enable_defer;

	/*
	 * Ether initialization
	 */
	spi_enc_write(spi, SPI_COM_WCR, ETXSTL, 0x00);
	spi_enc_write(spi, SPI_COM_WCR, ETXSTH, 0x00);

	spi_enc_write(spi, SPI_COM_WCR, ETXNDL, INT16_L(ENC_TX_BUF_SIZE));
	spi_enc_write(spi, SPI_COM_WCR, ETXNDH, INT16_H(ENC_TX_BUF_SIZE));

	spi_enc_write(spi, SPI_COM_WCR, ERXSTL, INT16_L(ENC_RX_START_ADDR));
	spi_enc_write(spi, SPI_COM_WCR, ERXSTH, INT16_H(ENC_RX_START_ADDR));

	spi_enc_write(spi, SPI_COM_WCR, ERXNDL, INT16_L(ENC_RX_END_ADDR));
	spi_enc_write(spi, SPI_COM_WCR, ERXNDH, INT16_H(ENC_RX_END_ADDR));

	/* Discard ethernet frames with invalid CRCs */
	spi_enc_write(spi, SPI_COM_BFS, ERXFCON, ERXFCON_CRCEN);

	/*
	 * MAC initialization
	 */
	while (spi_enc_read(spi, SPI_COM_RCR, ESTAT) & ESTAT_CLKRDY) {
		dev_info(&spi->dev, "%s: polling ESTAT_CLKRDY", __func__);
	}

	enable_rx = spi_enc_read(spi, SPI_COM_WCR, MACON1) |
		MACON1_MARXEN | MACON1_RXPAUS | MACON1_TXPAUS;
	spi_enc_write(spi, SPI_COM_WCR, MACON1, enable_rx);

	macon3 = spi_enc_read(spi, SPI_COM_WCR, MACON3) |
		MACON3_FULLDPX | MACON3_FRMLNEN | MACON3_TXCRCEN | PADCFG_60;
	spi_enc_write(spi, SPI_COM_WCR, MACON3, macon3);

	enable_defer = spi_enc_read(spi, SPI_COM_WCR, MACON4) | MACON4_DEFER;
	spi_enc_write(spi, SPI_COM_WCR, MACON4, enable_defer);

	spi_enc_write(spi, SPI_COM_WCR, MAMXFLL, INT16_L(MAC_MAX_FRAME_LEN));
	spi_enc_write(spi, SPI_COM_WCR, MAMXFLH, INT16_H(MAC_MAX_FRAME_LEN));

	spi_enc_write(spi, SPI_COM_WCR,	MABBIPG, 0x15);

	spi_enc_write(spi, SPI_COM_WCR,	MAIPGL, 0x12);

	enc_set_mac_addr(priv, DEFAULT_MAC_ADDR);

	dev_info(&spi->dev, "%s: ETXNDH:ETXNDL=%x:%x H:L=%x:%x ERXFCON=%x",
			__func__,
			spi_enc_read(spi, SPI_COM_RCR, ETXNDH),
			spi_enc_read(spi, SPI_COM_RCR, ETXNDL),
			INT16_H(ENC_TX_BUF_SIZE),
			INT16_L(ENC_TX_BUF_SIZE),
			spi_enc_read(spi, SPI_COM_RCR, ERXFCON)
			);

	return 0;
}

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

	enc_init_rx(priv);

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
