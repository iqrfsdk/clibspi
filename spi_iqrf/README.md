# SPI-IQRF

SPI master driver for Raspberry Pi support of IQRF DCTR-5xDx and 7xDx transceiver modules with `OS v3.08D and higher`.

### Device's mode 
Set according to document: SPI. Implementation in IQRF TR modules (http://www.iqrf.org/weben/downloads.php?id=85)
- Idle clock polarity: low 
- Clock edge: output data on SCK rising edge
- Bitrate: 250 kHz
- Timing intervals T1 and T2 set according to IQRF SPI documentation

Implemented SPI Commands are:
```
spi_iqrf_init 					Initialization of the SPI interface for communication with IQRF module.
spi_iqrf_initDefault 			Initialization of the SPI for IQRF with default settings.
spi_iqrf_getCommunicationMode 	Gets current communication mode
spi_iqrf_setCommunicationMode 	Sets communication mode
spi_iqrf_getSPIStatus 			Gets SPI status
spi_iqrf_write					Writes data to SPI
spi_iqrf_read 					Reads data from SPI
spi_iqrf_destroy 				Destroys IQRF SPI library object and releases SPI port
```

See [wiki](https://github.com/MICRORISC/iqrfsdk/wiki) for more information.