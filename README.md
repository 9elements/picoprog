# Picoprog

Picoprog is a firmware for the Raspberry Pi Pico that provides a USB-to-serial and USB-to-SPI bridge. It allows you to communicate with UART and SPI peripherals via USB.

## Prerequisites

Before you can compile and use Picoprog, you need to install the following dependencies:

- Rust and Cargo: Follow the instructions on the [official Rust website](https://www.rust-lang.org/tools/install) to install Rust and Cargo.
- Install flip-link and elf2uf2

```sh
sudo apt install libudev-dev
cargo install flip-link elf2uf2-rs
```

## Compiling the Firmware

To compile the firmware, follow these steps:

1. Clone the repository:

```sh
git clone https://github.com/9elements/picoprog.git
cd picoprog
```

2. Build the firmware:

```sh
cargo run --release
```

3. The compiled binary will be located in the `target/thumbv6m-none-eabi/release` directory.

## Flashing the Firmware

To flash the firmware onto the Raspberry Pi Pico, follow these steps:

1. Connect the Raspberry Pi Pico to your computer while holding the BOOTSEL button. This will put the Pico into USB mass storage mode.

2. Copy the UF2 file to the Pico:

```sh
cp target/thumbv6m-none-eabi/release/picoprog.uf2 /media/$USER/RPI-RP2/
```

3. The Pico will automatically reboot and start running the new firmware.

## Usage

Once the firmware is running on the Raspberry Pi Pico, you can use any terminal program to communicate with the UART and SPI peripherals via USB. The device will appear as a USB CDC (Communications Device Class) device. Currently `/dev/ttyACM0` is a debug console that prints information about the picos current operation.

### UART Communication

To communicate with the UART peripheral, open the corresponding serial port (e.g., `/dev/ttyACM1` on Linux) with your terminal program. For now the Baud is fixed at 115200 but can be changed in code. Dynamic reconfiguration is still planned.

### Using Flashrom or Flashprog

To interact with the Raspberry Pi Pico for reading and writing SPI flash chips, you can use tools like `flashrom` or `flashprog`. These tools support the `serprog` protocol, which allows communication over a serial interface.

1. Install `flashrom` or `flsahprog` e.g.:

```sh
sudo apt-get install flashrom
```

2. Use the following command to read from the SPI flash chip:

```sh
flashrom -p serprog:dev=/dev/ttyACM2 -r backup.bin
```

This command reads the contents of the SPI flash chip and saves it to `backup.bin`.

3. Use the following command to write to the SPI flash chip:

```sh
flashrom -p serprog:dev=/dev/ttyACM2 -w firmware.bin
```

This command writes the contents of `firmware.bin` to the SPI flash chip.

Make sure to replace `/dev/ttyACM2` with the correct serial port if your device is connected to a different port.


## License

This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
