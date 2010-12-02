/*
 * Serial Peripheral Interface (SPI) driver for the Freescale i.MX
 *
 * (c) SAN People (Pty) Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef IMX_SPI_H
#define IMX_SPI_H

/* Maximum number of buffers in a single SPI transfer.
 *  DataFlash uses maximum of 2
 *  spidev interface supports up to 8.
 */
#define MAX_SPI_TRANSFERS	8

#define NR_SPI_DEVICES  	3

#define SPI_MAJOR		153	/* registered device number */

/*
 * Describes the buffers for a SPI transfer.
 * A transmit & receive buffer must be specified for each transfer
 */
struct spi_transfer_list {
    void* tx[MAX_SPI_TRANSFERS];	/* transmit */
    int txlen[MAX_SPI_TRANSFERS];
    void* rx[MAX_SPI_TRANSFERS];	/* receive */
    int rxlen[MAX_SPI_TRANSFERS];
    int nr_transfers;		/* number of transfers */
    int curr;			/* current transfer */
    int txpos;                      /* Position in current buffer */
    int rxpos;                      /* Position in current buffer */
};

struct spi_local {
    struct semaphore spi_lock;
    int spi_enabled;
    struct spi_transfer_list *xfers;	/* current transfer list */
    struct completion transfer_complete;
};


/* Exported functions */
extern void spi_access_bus(short device);
extern void spi_release_bus(short device);
extern int spi_transfer(short dev, struct spi_transfer_list* list);

#endif
