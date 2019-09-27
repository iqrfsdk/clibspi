# SPI-IQRF

[![Build Status](https://travis-ci.org/iqrfsdk/clibspi.svg?branch=master)](https://travis-ci.org/iqrfsdk/clibspi)
[![Build Status](https://img.shields.io/appveyor/ci/spinarr/clibspi/master.svg)](https://ci.appveyor.com/project/spinarr/clibspi)

SPI master driver for Raspberry Pi support of IQRF DCTR-5xDx and 7xDx transceiver modules with `OS v3.08D and higher`.
IQRF module is connected to Raspberry Pi platform via [KON-RASP-01](http://www.iqrf.org/products/kon-rasp-01).
SPI-IQRF lib combines source codes for SPI interface management and GPIO settings.
To demonstrate usage of the SPI-IQRF library, an example has been prepared with DPA communication on IQRF via SPI.

Features
*   compatible with TR-72(52) modules
*   supported programming languages: C, Java
*   supported operating systems: Linux


Library contains following folders
```
├── examples                      Examples of SPI-IQRF lib usage
│   │  
│   └── spi
│       ├── spi_example_dpa         Example of DPA communication via SPI
│       ├── spi_example_idf	    Example of TR module identification reading via SPI
│       ├── spi_example_pgm_iqrf    Example of TR module programming with *.iqrf file
│       ├── spi_example_pgm_hex     Example of TR module programming with *.hex file
│       ├── spi_example_pgm_trcnfg  Example of TR module programming with *.trcnfg file
│       └── spi_example_readmem     Example of TR module memory reading
│
├── include                       Folder contains header files for SPI-IQRF src
├── spi_iqrf                      Main part of the IQRF SPI lib source codes
└── spi_javastub                  Source codes for IQRF SPI Java stub
```

Quick command line compilation on the target
```bash
git clone https://github.com/iqrfsdk/clibspi.git
cd clibspi
mkdir -p .build; cd .build; cmake ..; make -j4
```

