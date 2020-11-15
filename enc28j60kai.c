#include <linux/module.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "enc28j60kai"

struct enc_device {
    struct spi_device *spi;
};

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

/* Common Register Set */
const struct control_register EIE	= CONTROL_REGISTER(BANK_COMMON, 0x1b);
const struct control_register EIR	= CONTROL_REGISTER(BANK_COMMON, 0x1c);
const struct control_register ESTAT	= CONTROL_REGISTER(BANK_COMMON, 0x1d);
const struct control_register ECON2	= CONTROL_REGISTER(BANK_COMMON, 0x1e);
const struct control_register ECON1	= CONTROL_REGISTER(BANK_COMMON, 0x1f);

/* Bank 3 */
const static struct control_register EREVID = CONTROL_REGISTER(BANK_3, 0x12);

enum spi_command {
	SPI_COM_RCR = 0x0,
	SPI_COM_RBM = 0x1,
	SPI_COM_WCR = 0x2,
	SPI_COM_WBM = 0x3,
	SPI_COM_BFS = 0x4,
	SPI_COM_BFC = 0x5,
	SPI_COM_SRC = 0x7,
};

union spi_operation {
	uint8_t raw8;
	struct __attribute__ ((packed)) {
		uint8_t opcode:3;
		uint8_t arg:5;
	} op;
};

static void enc_set_bank(struct spi_device *spi, enum bank_number bank);

static uint8_t spi_enc_rcr(struct spi_device *spi, struct control_register reg)
{
	union spi_operation spi_op = {
		.op = {
			.arg = reg.address,
			.opcode = SPI_COM_RCR,
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

	enc_set_bank(spi, reg.bank);

	spi_w8r8(spi, spi_op.raw8);
	spi_w8r8(spi, data);
}

#define ECON1_BSEL1 0x02
#define ECON1_BSEL0 0x01

static void enc_set_bank(struct spi_device *spi, enum bank_number bank)
{
	if (bank != BANK_COMMON) {
		uint8_t econ1 = spi_enc_rcr(spi, ECON1);
		spi_enc_write(spi, SPI_COM_BFC, ECON1, ECON1_BSEL1 | ECON1_BSEL0);
		spi_enc_write(spi, SPI_COM_BFS, ECON1, (uint8_t)bank);
		dev_info(&spi->dev, "enc_set_bank: econ1=%x set=%x", econ1, (econ1 & 0b11111100) | (uint8_t)bank);
	}
}

static int enc_probe(struct spi_device *spi)
{
	dev_info(&spi->dev, "%s: probe: revision=%d", DRIVER_NAME, spi_enc_rcr(spi, EREVID));
	return 0;
}

static int enc_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "%s: remove", DRIVER_NAME);
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
