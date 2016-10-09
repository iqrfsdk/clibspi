# SPI-IQRF

SPI master driver for Raspberry Pi support of IQRF DCTR-5xDx and 7xDx transceiver modules with `OS v3.08D and higher`.
IQRF module is connected to Raspberry Pi platform via KON-RASP-01 (http://www.iqrf.org/products/kon-rasp-01).
SPI-IQRF lib combines source codes for SPI interface management and GPIO settings.
To demonstrate usage of the SPI-IQRF library is prepared example with DPA communication on IQRF via SPI.

Features
* compatible with TR-72(52) modules
* supported programming languages: C, Java
* supported operating systems: Linux


Library contais following folders
```
examples 					Examples of SPI-IQRF lib usage
	spi 					
		spi_example_dpa	 	Example of DPA communication via SPI
include 					Folder contains header files for SPI-IQRF src
src 						Source codes of the SPI-IQRF lib
	lib 					
		spi_iqrf 			Main part of the SPI-IQRF lib source codes
		sysfs_gpio 			Source codes for necessary GPIO settings
```

Quick command line compilation on the target
```
git clone https://github.com/iqrfsdk/clibspi-linux.git
cd clibspi-linux
mkdir -p .build; cd .build; cmake ..; make -j4
```

See [wiki](https://github.com/MICRORISC/iqrfsdk/wiki) for more information.
