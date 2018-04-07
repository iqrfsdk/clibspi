# SPI-IQRF

[![Build Status](https://travis-ci.org/iqrfsdk/clibspi.svg?branch=master)](https://travis-ci.org/iqrfsdk/clibspi)
[![Build Status](https://img.shields.io/appveyor/ci/spinarr/clibspi/master.svg)](https://ci.appveyor.com/project/spinarr/clibspi)

SPI master driver for Raspberry Pi support of IQRF DCTR-5xDx and 7xDx transceiver modules with `OS v3.08D and higher`.
IQRF module is connected to Raspberry Pi platform via [KON-RASP-01](http://www.iqrf.org/products/kon-rasp-01).
SPI-IQRF lib combines source codes for SPI interface management and GPIO settings.
To demonstrate usage of the SPI-IQRF library is prepared example with DPA communication on IQRF via SPI.

Features
*   compatible with TR-72(52) modules
*   supported programming languages: C, Java
*   supported operating systems: Linux


Library contais following folders
```
├── examples                      Examples of SPI-IQRF lib usage
│   ├── io
│   │   └── io_example_led
│   │   
│   └── spi
│       ├── spi_example_dpa         Example of DPA communication via SPI
│       ├── spi_example_idf	    Example of TR module identification reading via SPI
│       ├── spi_example_pgm_iqrf    Example of TR module programming with *.iqrf file 
│       ├── spi_example_pgm_hex     Example of TR module programming with *.hex file 
│       └── spi_example_pgm_trcnfg  Example of TR module programming with *.trcnfg file 
│
├── include                       Folder contains header files for SPI-IQRF src
├── spi_iqrf                      Main part of the IQRF SPI lib source codes
├── spi_javastub                  Source codes for IQRF SPI Java stub
└── sysfs_gpio                    Source codes for necessary GPIO settings
```

Quick command line compilation on the target
```bash
git clone https://github.com/iqrfsdk/clibspi.git
cd clibspi
mkdir -p .build; cd .build; cmake ..; make -j4
```

See [wiki](https://github.com/MICRORISC/iqrfsdk/wiki) for more information.
