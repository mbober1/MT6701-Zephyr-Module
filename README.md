# MT6701 Zephyr Module

A Zephyr RTOS driver module for the MT6701 magnetic rotation sensor.

## Features

- Support for SPI communication interfaces
- Read angle
- Read velocity
- LAG filter support

## TODO
- Implement I2C communication support

## Usage

Add this module to your Zephyr project.
Enable the MT6701 sensor in your `prj.conf`:

```
CONFIG_SENSOR=y
CONFIG_MT6701=y
```

Add MT6701 to device tree:

```
&spi1 {
	pinctrl-0 = <&spi1_sck_pa5 &spi1_miso_pa6>;
	pinctrl-names = "default";
	cs-gpios = <&gpioa 4 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
	status = "okay";

	mt6701: mt6701@0 {
		compatible = "novosense,mt6701";
		reg = <0>;
		spi-max-frequency = <DT_FREQ_M(15)>;
		status = "okay";
	};
};
```