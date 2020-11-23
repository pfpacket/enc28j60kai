#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include "enc28j60kai.h"

#define DRIVER_NAME "enc28j60kai"

#define INT16_L(n) (uint8_t)((n) & 0xff)
#define INT16_H(n) (uint8_t)((n) >> 8)

const uint16_t ENC_RX_BUF_SIZE = 1024 * 5;
const uint16_t ENC_RX_START_ADDR = 0;
const uint16_t ENC_RX_END_ADDR = ENC_RX_START_ADDR + ENC_RX_BUF_SIZE - 1;

const uint16_t ENC_TX_START_ADDR = ENC_RX_END_ADDR + 1;
const uint16_t ENC_TX_END_ADDR = 0x1fff;

#define MAC_MAX_FRAME_LEN 1518

static struct sockaddr default_mac_addr = {
	.sa_family = 0,
	.sa_data = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}
};

struct enc_adapter {
	struct net_device *netdev;
	struct spi_device *spi;
	struct task_struct *poller;
	enum bank_number current_bank;
};

static void enc_soft_reset(struct enc_adapter *adapter)
{
	spi_w8r8(adapter->spi, SPI_COM_SRC);
	udelay(2000);
}

static void enc_set_bank(struct enc_adapter *adapter, enum bank_number bank);

static uint8_t spi_enc_read(struct enc_adapter *adapter, enum spi_command_read com, struct control_register reg)
{
	union spi_operation spi_op = {
		.op = {
			.arg = reg.address,
			.opcode = com,
		}
	};

	enc_set_bank(adapter, reg.bank);

	return (uint8_t)spi_w8r8(adapter->spi, spi_op.raw8);
}

static void spi_enc_write(struct enc_adapter *adapter, enum spi_command_write com, struct control_register reg, uint8_t data)
{
	union spi_operation spi_op = {
		.op = {
			.arg = reg.address,
			.opcode = com,
		}
	};
	uint8_t txbuf[2] = {spi_op.raw8, data};

	enc_set_bank(adapter, reg.bank);

	spi_write(adapter->spi, txbuf, 2);
}

static void spi_enc_wait_for_phy_busy(struct enc_adapter *adapter)
{
	while (spi_enc_read(adapter, SPI_COM_RCR, MISTAT) & MISTAT_BUSY) {
		dev_warn(&adapter->spi->dev, "%s: pollnig MISTAT_BUSY", __func__);
		udelay(10);
	}
}

static uint16_t spi_enc_read_phy(struct enc_adapter *adapter, struct phy_register reg)
{
	uint8_t micmd;

	spi_enc_write(adapter, SPI_COM_WCR, MIREGADR, reg.address);

	micmd = spi_enc_read(adapter, SPI_COM_RCR, MICMD) | MICMD_MIIRD;
	spi_enc_write(adapter, SPI_COM_WCR, MICMD, micmd);

	udelay(20);
	spi_enc_wait_for_phy_busy(adapter);

	micmd &= ~MICMD_MIIRD;
	spi_enc_write(adapter, SPI_COM_WCR, MICMD, micmd);

	return spi_enc_read(adapter, SPI_COM_RCR, MIRDL) |
		(spi_enc_read(adapter, SPI_COM_RCR, MIRDH) << 8);
}


static void spi_enc_write_phy(struct enc_adapter *adapter, struct phy_register reg, uint16_t data)
{
	spi_enc_write(adapter, SPI_COM_WCR, MIREGADR, reg.address);

	dev_info(&adapter->spi->dev, "%s: INT_H=%x INT_L=%x", __func__, INT16_H(data), INT16_L(data));
	spi_enc_write(adapter, SPI_COM_WCR, MIWRL, INT16_L(data));
	spi_enc_write(adapter, SPI_COM_WCR, MIWRH, INT16_H(data));

	spi_enc_wait_for_phy_busy(adapter);
}

static void enc_set_bank(struct enc_adapter *adapter, enum bank_number bank)
{
	if (bank != BANK_COMMON && bank != adapter->current_bank) {
		spi_enc_write(adapter, SPI_COM_BFC, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
		spi_enc_write(adapter, SPI_COM_BFS, ECON1, bank);
		adapter->current_bank = bank;
	}
}

static int poll_pktcnt_thread(void *arg)
{
	struct enc_adapter *adapter = arg;

	dev_info(&adapter->spi->dev, "%s: polling", __func__);

	while (!kthread_should_stop()) {
		int pktcnt = spi_enc_read(adapter, SPI_COM_RCR, EPKTCNT);
		if (pktcnt > 0)
			dev_warn(&adapter->spi->dev, "%s: pktcnt=%d", __func__, pktcnt);

		ssleep(1);
	}

	dev_info(&adapter->spi->dev, "%s: stopped polling", __func__);

	return 0;
}

static int enc_netdev_open(struct net_device *dev)
{
	struct enc_adapter *adapter = netdev_priv(dev);

	adapter->poller = kthread_run(poll_pktcnt_thread, adapter, "pktcnt-poller");
	if (IS_ERR(adapter->poller)) {
		return PTR_ERR(adapter->poller);
	}

	netif_start_queue(adapter->netdev);

	return 0;
}

static int enc_netdev_stop(struct net_device *dev)
{
	struct enc_adapter *adapter = netdev_priv(dev);

	netif_stop_queue(dev);

	if (adapter->poller)
		kthread_stop(adapter->poller);

	return 0;
}

static netdev_tx_t enc_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	//struct enc_adapter *adapter = netdev_priv(dev);

	//dev_info(&adapter->spi->dev, "%s: called", __func__);

	kfree_skb(skb);

	return NETDEV_TX_OK;
}

static int enc_set_mac_addr(struct enc_adapter *adapter, const char *address)
{
	const char *addr = address;

	spi_enc_write(adapter, SPI_COM_WCR, MAADR6, addr[0]);
	spi_enc_write(adapter, SPI_COM_WCR, MAADR5, addr[1]);
	spi_enc_write(adapter, SPI_COM_WCR, MAADR4, addr[2]);
	spi_enc_write(adapter, SPI_COM_WCR, MAADR3, addr[3]);
	spi_enc_write(adapter, SPI_COM_WCR, MAADR2, addr[4]);
	spi_enc_write(adapter, SPI_COM_WCR, MAADR1, addr[5]);

	return 0;
}

static int enc_set_mac_address(struct net_device *dev, void *addr)
{
	int ret;
	struct enc_adapter *adapter = netdev_priv(dev);
	struct sockaddr *sa = addr;

	if ((ret = eth_mac_addr(dev, addr)))
		return ret;

	dev_info(&adapter->spi->dev, "%s: %x:%x:%x:%x:%x:%x",
			__func__, sa->sa_data[0], sa->sa_data[1], sa->sa_data[2],
			sa->sa_data[3], sa->sa_data[4], sa->sa_data[5]);

	return enc_set_mac_addr(adapter, addr);
}

static const struct net_device_ops enc_netdev_ops = {
	.ndo_open = enc_netdev_open,
	.ndo_stop = enc_netdev_stop,
	.ndo_start_xmit = enc_start_xmit,
	.ndo_set_mac_address = enc_set_mac_address,
	.ndo_validate_addr = eth_validate_addr
};

static int enc_init_rx(struct enc_adapter *adapter)
{
	/*
	 * Ether initialization
	 */
	/* RX buffer start pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ERXSTL, INT16_L(ENC_RX_START_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ERXSTH, INT16_H(ENC_RX_START_ADDR));

	/* RX buffer end pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ERXNDL, INT16_L(ENC_RX_END_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ERXNDH, INT16_H(ENC_RX_END_ADDR));

	spi_enc_write(adapter, SPI_COM_WCR, ERXRDPTL, INT16_L(ENC_RX_START_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ERXRDPTH, INT16_H(ENC_RX_START_ADDR));

	dev_info(&adapter->spi->dev, "%s: ENC_RX_START_ADDR=%x ENC_RX_END_ADDR=%x", __func__,
			spi_enc_read(adapter, SPI_COM_RCR, ERXSTL),
			(spi_enc_read(adapter, SPI_COM_RCR, ERXNDH) << 8) | spi_enc_read(adapter, SPI_COM_RCR, ERXNDL));

	/* TX buffer start pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ETXSTL, INT16_L(ENC_TX_START_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ETXSTH, INT16_H(ENC_TX_START_ADDR));

	/* TX buffer end pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ETXNDL, INT16_L(ENC_TX_END_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ETXNDH, INT16_H(ENC_TX_END_ADDR));

	/* Discard ethernet frames with invalid CRCs */
	spi_enc_write(adapter, SPI_COM_WCR, ERXFCON, ERXFCON_CRCEN);
	//spi_enc_write(adapter, SPI_COM_BFS, ERXFCON, ERXFCON_CRCEN);

	/*
	 * MAC initialization
	 */
	/*while (!(spi_enc_read(adapter, SPI_COM_RCR, ESTAT) & ESTAT_CLKRDY)) {
		dev_info(&adapter->dev, "%s: polling ESTAT_CLKRDY", __func__);
	}*/

	/*
	 * Here we can't use BFC/BFS for registers other than ETH registers
	 *
	 * Enable RX set up
	 * Enable full duplex mode
	 * */
	spi_enc_write(adapter, SPI_COM_WCR, MACON1, MACON1_MARXEN | MACON1_RXPAUS | MACON1_TXPAUS);

	/*
	 * Enable frane length report
	 * Enable full duplex mode
	 * Enable TX CRC auto calculation
	 * Padding configuration
	 * */
	spi_enc_write(adapter, SPI_COM_WCR, MACON3, /*MACON3_FULLDPX |*/ MACON3_FRMLNEN | MACON3_TXCRCEN | PADCFG_60);
	dev_info(&adapter->spi->dev, "%s: expected=%x MACON3=%x",
			__func__,
			/*MACON3_FULLDPX |*/ MACON3_FRMLNEN | MACON3_TXCRCEN | PADCFG_60,
			spi_enc_read(adapter, SPI_COM_RCR, MACON3));

	spi_enc_write(adapter, SPI_COM_WCR, MACON4, MACON4_DEFER);

	spi_enc_write(adapter, SPI_COM_WCR, MAMXFLL, INT16_L(MAC_MAX_FRAME_LEN));
	spi_enc_write(adapter, SPI_COM_WCR, MAMXFLH, INT16_H(MAC_MAX_FRAME_LEN));

	spi_enc_write(adapter, SPI_COM_WCR, MAIPGL, 0x12);
	spi_enc_write(adapter, SPI_COM_WCR, MAIPGH, 0x0c);
	spi_enc_write(adapter, SPI_COM_WCR, MABBIPG, 0x12);

	enc_set_mac_address(adapter->netdev, &default_mac_addr);

	/*
	 * PHY initialization
	 */
	dev_info(&adapter->spi->dev, "%s: PHLCON=%x", __func__, spi_enc_read_phy(adapter, PHLCON));
	dev_info(&adapter->spi->dev, "%s: PHCON2=%x", __func__, spi_enc_read_phy(adapter, PHCON2));
	spi_enc_write_phy(adapter, PHCON2, PHCON2_HDLDIS);
	dev_info(&adapter->spi->dev, "%s: PHCON2=%x", __func__, spi_enc_read_phy(adapter, PHCON2));

	/* enable IRQ */
	spi_enc_write(adapter, SPI_COM_BFS, EIE, EIE_INTIE | EIE_PKTIE);

	/* Start RX */
	spi_enc_write(adapter, SPI_COM_BFS, ECON1, ECON1_RXEN);

	return 0;
}

static int enc_probe(struct spi_device *spi)
{
	int ret, revision;
	struct net_device *netdev;
	struct enc_adapter *adapter;

	netdev = alloc_etherdev(sizeof (struct enc_adapter));
	if (!netdev)
		goto err_alloc;

	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->spi = spi;
	adapter->current_bank = BANK_COMMON;	/* to let enc_set_bank(BANK_0) work */
	spi_set_drvdata(spi, adapter);
	SET_NETDEV_DEV(netdev, &spi->dev);

	enc_soft_reset(adapter);
	enc_set_bank(adapter, BANK_0);

	spi_enc_write(adapter, SPI_COM_WCR, ECON1, 0x00);
	spi_enc_write(adapter, SPI_COM_WCR, ECON2, ECON2_AUTOINC);

	revision = spi_enc_read(adapter, SPI_COM_RCR, EREVID);
	dev_info(&spi->dev, "%s: revision=%d", __func__, revision);

	if (revision == 0 || revision == 0xff) {
		return -EINVAL;
	}

	enc_init_rx(adapter);

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
	struct enc_adapter *adapter = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s", __func__);

	unregister_netdev(adapter->netdev);
	free_netdev(adapter->netdev);

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
