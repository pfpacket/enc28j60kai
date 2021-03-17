// SPDX-License-Identifier: GPL-2.0
/*
 * ENC28J60 ethernet driver
 */

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

#define NETDEV_LOG(level, fmt, args...) \
	netdev_##level(adapter->netdev, "%s: " fmt "\n", __func__, ##args)

#define INT16_L(n) ((uint8_t)((n) & 0xff))
#define INT16_H(n) ((uint8_t)((n) >> 8))

const uint16_t ENC_RX_START_ADDR;
const uint16_t ENC_RX_END_ADDR = 0x19ff;
const uint16_t ENC_TX_START_ADDR = 0x1a00;
const uint16_t ENC_TX_END_ADDR = 0x1fff;

#define ETH_MAX_FRAME_LEN 1518
#define DMA_BUFFER_SIZE (PAGE_ALIGN(ETH_MAX_FRAME_LEN + 4))

static struct sockaddr default_mac_addr = {
	.sa_family = 0,
	.sa_data = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}
};

struct enc_adapter {
	struct net_device *netdev;
	struct spi_device *spi;
	struct mutex lock;
	enum bank_number current_bank;
	struct work_struct irq_work;
	struct work_struct tx_work;
	struct sk_buff *pending_skb;
	uint16_t next_packet_ptr;
	dma_addr_t dma_handle;
	uint8_t *dma_buf;
};

static void enc_soft_reset(struct enc_adapter *adapter)
{
	spi_w8r8(adapter->spi, SPI_COM_SRC);
	udelay(2000);
}

static void enc_set_bank(struct enc_adapter *adapter, enum bank_number bank);

static uint8_t spi_enc_read(struct enc_adapter *adapter, enum spi_command com, struct control_register reg)
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

static void spi_enc_write(struct enc_adapter *adapter, enum spi_command com, struct control_register reg, uint8_t data)
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

static uint8_t spi_enc_read_buffer_memory(struct enc_adapter *adapter)
{
	return (uint8_t)spi_w8r8(adapter->spi, SPI_COM_RBM);
}

static int enc_read_buffer(struct enc_adapter *adapter, uint16_t addr, void *buf, size_t size)
{
	int ret;
	void *rxbuf = adapter->dma_buf + 4;
	struct spi_transfer tx_trans = {
		.tx_buf = adapter->dma_buf,
		.len = 1
	};
	struct spi_transfer rx_trans = {
		.rx_buf = rxbuf,
		.len = size
	};
	struct spi_message msg;

	spi_message_init(&msg);
	spi_message_add_tail(&tx_trans, &msg);
	spi_message_add_tail(&rx_trans, &msg);

	adapter->dma_buf[0] = SPI_COM_RBM;
	spi_enc_write(adapter, SPI_COM_WCR, ERDPTL, INT16_L(addr));
	spi_enc_write(adapter, SPI_COM_WCR, ERDPTH, INT16_H(addr));

	ret = spi_sync(adapter->spi, &msg);
	if (ret)
		return ret;

	memcpy(buf, rxbuf, size);

	return 0;
}

static void spi_enc_write_buffer_memory(struct enc_adapter *adapter, uint8_t data)
{
	uint8_t txbuf[2] = {SPI_COM_WBM, data};

	spi_write(adapter->spi, txbuf, 2);
}

static int enc_write_buffer(struct enc_adapter *adapter, uint16_t addr, const void *buf, size_t size)
{
	if (size > ETH_MAX_FRAME_LEN)
		return -EINVAL;

	spi_enc_write(adapter, SPI_COM_WCR, EWRPTL, INT16_L(addr));
	spi_enc_write(adapter, SPI_COM_WCR, EWRPTH, INT16_H(addr));

	adapter->dma_buf[0] = SPI_COM_WBM;
	memcpy(adapter->dma_buf + 1, buf, size);

	return spi_write(adapter->spi, adapter->dma_buf, size + 1);
}

static void enc_wait_for_phy_ready(struct enc_adapter *adapter)
{
	udelay(20);

	while (spi_enc_read(adapter, SPI_COM_RCR, MISTAT) & MISTAT_BUSY) {
		dev_warn(&adapter->spi->dev, "%s: pollnig MISTAT_BUSY\n", __func__);
		udelay(10);
	}
}

static uint16_t spi_enc_read_phy(struct enc_adapter *adapter, struct phy_register reg)
{
	uint8_t micmd;

	spi_enc_write(adapter, SPI_COM_WCR, MIREGADR, reg.address);

	micmd = spi_enc_read(adapter, SPI_COM_RCR, MICMD) | MICMD_MIIRD;
	spi_enc_write(adapter, SPI_COM_WCR, MICMD, micmd);

	enc_wait_for_phy_ready(adapter);

	micmd &= ~MICMD_MIIRD;
	spi_enc_write(adapter, SPI_COM_WCR, MICMD, micmd);

	return spi_enc_read(adapter, SPI_COM_RCR, MIRDL) |
		(spi_enc_read(adapter, SPI_COM_RCR, MIRDH) << 8);
}

static void spi_enc_write_phy(struct enc_adapter *adapter, struct phy_register reg, uint16_t data)
{
	spi_enc_write(adapter, SPI_COM_WCR, MIREGADR, reg.address);

	spi_enc_write(adapter, SPI_COM_WCR, MIWRL, INT16_L(data));
	spi_enc_write(adapter, SPI_COM_WCR, MIWRH, INT16_H(data));

	enc_wait_for_phy_ready(adapter);
}

static void enc_set_bank(struct enc_adapter *adapter, enum bank_number bank)
{
	if (bank != BANK_COMMON && bank != adapter->current_bank) {
		spi_enc_write(adapter, SPI_COM_BFC, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
		spi_enc_write(adapter, SPI_COM_BFS, ECON1, bank);
		adapter->current_bank = bank;
	}
}

static void enc_start_rx(struct enc_adapter *adapter)
{
	/* Enable IRQ */
	spi_enc_write(adapter, SPI_COM_BFS, EIE, EIE_INTIE | EIE_PKTIE);

	/* Start RX */
	spi_enc_write(adapter, SPI_COM_BFS, ECON1, ECON1_RXEN);
}

static void enc_stop_rx(struct enc_adapter *adapter)
{
	/* Disable IRQ */
	spi_enc_write(adapter, SPI_COM_BFC, EIE, EIE_INTIE | EIE_PKTIE);

	/* Stop RX */
	spi_enc_write(adapter, SPI_COM_BFC, ECON1, ECON1_RXEN);
}

static int enc_netdev_open(struct net_device *dev)
{
	struct enc_adapter *adapter = netdev_priv(dev);

	enc_start_rx(adapter);

	netif_start_queue(adapter->netdev);

	return 0;
}

static int enc_netdev_stop(struct net_device *dev)
{
	struct enc_adapter *adapter = netdev_priv(dev);

	enc_stop_rx(adapter);

	netif_stop_queue(dev);

	return 0;
}

static void enc_tx_handler(struct work_struct *work)
{
	struct enc_adapter *adapter =
		container_of(work, struct enc_adapter, tx_work);
	struct sk_buff *skb;
	uint16_t tx_end_addr;

	mutex_lock(&adapter->lock);

	NETDEV_LOG(info, "trace: pending_skb=%p", adapter->pending_skb);
	skb = adapter->pending_skb;
	if (!skb) {
		NETDEV_LOG(err, "BUG: invalid TX request");
		netif_wake_queue(adapter->netdev);
		goto err;
	}

	/* Containing control bit (1 byte), but the address itself isn't included */
	tx_end_addr = ENC_TX_START_ADDR + skb->len;

	/* TX buffer start pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ETXSTL, INT16_L(ENC_TX_START_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ETXSTH, INT16_H(ENC_TX_START_ADDR));

	/* Control bit */
	spi_enc_write(adapter, SPI_COM_WCR, EWRPTL, INT16_L(ENC_TX_START_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, EWRPTH, INT16_H(ENC_TX_START_ADDR));
	spi_enc_write_buffer_memory(adapter, /*PHUGEEN |*/ PPADEN | PCRCEN | POVERRIDE);

	/* Write the entire packet */
	enc_write_buffer(adapter, ENC_TX_START_ADDR + 1, skb_mac_header(skb), skb->len);

	/* TX buffer end pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ETXNDL, INT16_L(tx_end_addr));
	spi_enc_write(adapter, SPI_COM_WCR, ETXNDH, INT16_H(tx_end_addr));

	/* Set up interrupts */
	spi_enc_write(adapter, SPI_COM_BFC, EIR, EIR_TXIF);
	spi_enc_write(adapter, SPI_COM_BFS, EIE, EIE_TXIE | EIE_INTIE);

	/* Start TX */
	spi_enc_write(adapter, SPI_COM_BFS, ECON1, ECON1_TXRTS);

err:
	mutex_unlock(&adapter->lock);
}

static netdev_tx_t enc_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct enc_adapter *adapter = netdev_priv(dev);

	/* Prevent further xmit's */
	netif_stop_queue(dev);

	NETDEV_LOG(info, "trace: pending_skb=%p", adapter->pending_skb);
	if (adapter->pending_skb)
		NETDEV_LOG(err, "the previous buffer isn't still processed!");

	adapter->pending_skb = skb;
	NETDEV_LOG(info, "set: pending_skb=%p", adapter->pending_skb);
	schedule_work(&adapter->tx_work);

	return NETDEV_TX_OK;
}

static int enc_write_mac_address(struct enc_adapter *adapter, const void *address)
{
	const uint8_t *addr = address;

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

	ret = eth_mac_addr(dev, addr);
	if (ret)
		return ret;

	dev_info(&adapter->spi->dev, "%s: %x:%x:%x:%x:%x:%x\n",
			__func__, sa->sa_data[0], sa->sa_data[1], sa->sa_data[2],
			sa->sa_data[3], sa->sa_data[4], sa->sa_data[5]);

	return enc_write_mac_address(adapter, addr);
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
	adapter->next_packet_ptr = ENC_RX_START_ADDR;

	/* RX buffer end pointer */
	spi_enc_write(adapter, SPI_COM_WCR, ERXNDL, INT16_L(ENC_RX_END_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ERXNDH, INT16_H(ENC_RX_END_ADDR));

	spi_enc_write(adapter, SPI_COM_WCR, ERXRDPTL, INT16_L(ENC_RX_START_ADDR));
	spi_enc_write(adapter, SPI_COM_WCR, ERXRDPTH, INT16_H(ENC_RX_START_ADDR));

	/* Discard ethernet frames with invalid CRCs */
	spi_enc_write(adapter, SPI_COM_WCR, ERXFCON, ERXFCON_CRCEN);

	/*
	 * MAC initialization
	 */
	/*
	 * We are supposed to check ESTAT_CLKRDY here,
	 * but it turns out that it is something we cannnot rely on
	 * while (!(spi_enc_read(adapter, SPI_COM_RCR, ESTAT) & ESTAT_CLKRDY)) {
	 *	netdev_info(adapter->netdev, "%s: polling ESTAT_CLKRDY\n", __func__);
	 *}
	 */

	/*
	 * Here we can't use BFC/BFS for registers other than ETH registers
	 *
	 * Enable RX set up
	 * Enable full duplex mode
	 */
	spi_enc_write(adapter, SPI_COM_WCR, MACON1, MACON1_MARXEN | MACON1_RXPAUS | MACON1_TXPAUS);

	/*
	 * Enable frame length report
	 * Enable full duplex mode
	 * Enable TX CRC auto calculation
	 * Padding configuration
	 */
	spi_enc_write(adapter, SPI_COM_WCR, MACON3, /*MACON3_FULLDPX |*/ MACON3_FRMLNEN | MACON3_TXCRCEN | PADCFG_60);

	spi_enc_write(adapter, SPI_COM_WCR, MACON4, MACON4_DEFER);

	spi_enc_write(adapter, SPI_COM_WCR, MAMXFLL, INT16_L(ETH_MAX_FRAME_LEN));
	spi_enc_write(adapter, SPI_COM_WCR, MAMXFLH, INT16_H(ETH_MAX_FRAME_LEN));

	spi_enc_write(adapter, SPI_COM_WCR, MAIPGL, 0x12);
	spi_enc_write(adapter, SPI_COM_WCR, MAIPGH, 0x0c);
	spi_enc_write(adapter, SPI_COM_WCR, MABBIPG, 0x12);

	enc_set_mac_address(adapter->netdev, &default_mac_addr);

	/*
	 * PHY initialization
	 */
	spi_enc_write_phy(adapter, PHCON2, PHCON2_HDLDIS);

	return 0;
}

static irqreturn_t enc_irq_handler(int irq, void *dev)
{
	struct enc_adapter *adapter = dev;

	schedule_work(&adapter->irq_work);

	return IRQ_HANDLED;
}

static void enc_prepare_for_next_packet(struct enc_adapter *adapter, uint16_t next_packet_ptr)
{
	adapter->next_packet_ptr = next_packet_ptr;

	spi_enc_write(adapter, SPI_COM_WCR, ERXRDPTL, INT16_L(next_packet_ptr));
	spi_enc_write(adapter, SPI_COM_WCR, ERXRDPTH, INT16_H(next_packet_ptr));

	spi_enc_write(adapter, SPI_COM_BFS, ECON2, ECON2_PKTDEC);
}

static struct sk_buff *enc_read_packet(struct enc_adapter *adapter, size_t frame_len)
{
	uint8_t *buf;
	struct sk_buff *skb;

	skb = netdev_alloc_skb_ip_align(adapter->netdev, frame_len);
	if (!skb)
		return NULL;

	/* Read an ethernet frame and write it to the skb */
	buf = skb_put(skb, frame_len);
	enc_read_buffer(adapter, adapter->next_packet_ptr + sizeof(struct receive_status_vector), buf, frame_len);
	skb->protocol = eth_type_trans(skb, adapter->netdev);

	return skb;
}

static void enc_irq_work_handler(struct work_struct *work)
{
	struct enc_adapter *adapter =
		container_of(work, struct enc_adapter, irq_work);
	uint8_t eir, pktcnt;
	struct receive_status_vector rsv;

	mutex_lock(&adapter->lock);

	/* Disable interrupts */
	spi_enc_write(adapter, SPI_COM_BFC, EIE, EIE_INTIE);

	/* Check what kind of interrupts occurred */
	eir = spi_enc_read(adapter, SPI_COM_RCR, EIR);
	pktcnt = spi_enc_read(adapter, SPI_COM_RCR, EPKTCNT);

	for (; pktcnt-- > 0; enc_prepare_for_next_packet(adapter, rsv.next_packet_ptr)) {
		struct sk_buff *skb;

		enc_read_buffer(adapter, adapter->next_packet_ptr, &rsv, sizeof(rsv));
		if (!rsv.rx_ok || rsv.zero) {
			NETDEV_LOG(err, "rx failed rx_ok=%d zero=%d", rsv.rx_ok, rsv.zero);
			continue;
		}

		skb = enc_read_packet(adapter, rsv.rx_byte_count - ETH_CRC_SIZE);
		if (!skb)
			continue;

		NETDEV_LOG(info, "rx: count=%d", rsv.rx_byte_count - ETH_CRC_SIZE);

		netif_rx_ni(skb);
	}

	if (eir & EIR_TXIF) {
		uint16_t tsv_addr = 1 +
			(((uint16_t)spi_enc_read(adapter, SPI_COM_RCR, ETXNDH) << 8) |
			 (uint16_t)spi_enc_read(adapter, SPI_COM_RCR, ETXNDL));
		struct transmit_status_vector tsv;

		//enc_read_buffer(adapter, tsv_addr, &tsv, sizeof(tsv));
		NETDEV_LOG(info, "trace: pending_skb=%p", adapter->pending_skb);
		if (adapter->pending_skb) {
			dev_kfree_skb(adapter->pending_skb);
			adapter->pending_skb = NULL;
			NETDEV_LOG(info, "set: pending_skb=%p", adapter->pending_skb);
			netif_wake_queue(adapter->netdev);
		}
	}

	/* Clear interrupt flags */
	spi_enc_write(adapter, SPI_COM_WCR, EIR, 0x0);

	/* Re-enable interrupts */
	spi_enc_write(adapter, SPI_COM_BFS, EIE, EIE_INTIE);

	mutex_unlock(&adapter->lock);
}

static int enc_probe(struct spi_device *spi)
{
	int ret, revision;
	struct net_device *netdev;
	struct enc_adapter *adapter;

	netdev = alloc_etherdev(sizeof(struct enc_adapter));
	if (!netdev)
		return -ENOMEM;

	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->spi = spi;
	adapter->pending_skb = NULL;
	mutex_init(&adapter->lock);
	adapter->current_bank = BANK_0;
	INIT_WORK(&adapter->irq_work, enc_irq_work_handler);
	INIT_WORK(&adapter->tx_work, enc_tx_handler);
	spi_set_drvdata(spi, adapter);
	SET_NETDEV_DEV(netdev, &spi->dev);

	ret = dma_set_coherent_mask(&spi->dev, DMA_BIT_MASK(64));
	if (ret) {
		ret = -ENOMEM;
		goto err_free_netdev;
	}

	adapter->dma_buf = dma_alloc_coherent(&spi->dev, DMA_BUFFER_SIZE,
			&adapter->dma_handle, GFP_KERNEL | GFP_DMA);
	if (!adapter->dma_buf) {
		ret = -ENOMEM;
		goto err_free_netdev;
	}

	enc_soft_reset(adapter);
	spi_enc_write(adapter, SPI_COM_WCR, ECON1, 0x00);
	spi_enc_write(adapter, SPI_COM_WCR, ECON2, ECON2_AUTOINC);

	revision = spi_enc_read(adapter, SPI_COM_RCR, EREVID);
	dev_info(&spi->dev, "%s: revision=%d\n", __func__, revision);

	if (revision == 0 || revision == 0xff) {
		ret = -EINVAL;
		goto err_free_buf;
	}

	enc_init_rx(adapter);

	ret = request_irq(adapter->spi->irq, enc_irq_handler, 0, DRIVER_NAME, adapter);
	if (ret < 0)
		goto err_free_buf;

	netdev->if_port = IF_PORT_10BASET;
	netdev->irq = spi->irq;
	netdev->netdev_ops = &enc_netdev_ops;

	ret = register_netdev(netdev);
	if (ret)
		goto err_irq;

	return 0;

err_irq:
	free_irq(adapter->spi->irq, adapter);
err_free_buf:
	dma_free_coherent(&spi->dev, DMA_BUFFER_SIZE, adapter->dma_buf, adapter->dma_handle);
err_free_netdev:
	free_netdev(netdev);
	return ret;
}

static int enc_remove(struct spi_device *spi)
{
	struct enc_adapter *adapter = spi_get_drvdata(spi);

	NETDEV_LOG(info, "called");

	unregister_netdev(adapter->netdev);
	free_irq(adapter->spi->irq, adapter);
	dma_free_coherent(&spi->dev, DMA_BUFFER_SIZE, adapter->dma_buf, adapter->dma_handle);
	free_netdev(adapter->netdev);

	return 0;
}

static const struct of_device_id enc28j60kai_dev_ids[] = {
	{ .compatible = "microchip,enc28j60" },
	{}
};
MODULE_DEVICE_TABLE(of, enc28j60kai_dev_ids);

static struct spi_driver enc_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = enc28j60kai_dev_ids,
	},
	.probe = enc_probe,
	.remove = enc_remove,
};

module_spi_driver(enc_driver);

MODULE_DESCRIPTION("enc28j60 ethernet driver");
MODULE_AUTHOR("Ryo Munakata");
MODULE_LICENSE("GPL");
