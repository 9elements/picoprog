use embassy_futures::join::join;
use embassy_rp::peripherals::USB;
use embassy_rp::pio::{Instance as PioInstance, Pio};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::usb::Driver;
use embassy_rp::usb::Instance as UsbInstance;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pipe::{Pipe, Reader, Writer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use embassy_usb::driver::EndpointError;

use crate::UartResources;

pub struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("USB buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

#[embassy_executor::task]
pub async fn uart_task(class: CdcAcmClass<'static, Driver<'static, USB>>, r: UartResources) {
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(r.peripheral, crate::Irqs);

    let tx_prog = PioUartTxProgram::new(&mut common);
    let mut uart_tx = PioUartTx::new(115200, &mut common, sm0, r.tx, &tx_prog);

    let rx_prog = PioUartRxProgram::new(&mut common);
    let mut uart_rx = PioUartRx::new(115200, &mut common, sm1, r.rx, &rx_prog);

    let mut usb_pipe: Pipe<NoopRawMutex, 64> = Pipe::new();
    let (mut usb_pipe_reader, mut usb_pipe_writer) = usb_pipe.split();

    let mut uart_pipe: Pipe<NoopRawMutex, 64> = Pipe::new();
    let (mut uart_pipe_reader, mut uart_pipe_writer) = uart_pipe.split();

    let (mut usb_tx, mut usb_rx) = class.split();

    // Read + write from USB
    let usb_future = async {
        loop {
            log::debug!("[UART]: Wait for USB connection");
            usb_rx.wait_connection().await;
            log::debug!("[UART]: USB Connected");
            let _baud = usb_rx.line_coding().data_rate(); // TODO: Make use of this in the PIO program
            let _ = join(
                usb_read(&mut usb_rx, &mut uart_pipe_writer),
                usb_write(&mut usb_tx, &mut usb_pipe_reader),
            )
            .await;
            log::debug!("[UART]: USB Disconnected");
        }
    };

    // Read + write from UART
    let uart_future = join(
        uart_read(&mut uart_rx, &mut usb_pipe_writer),
        uart_write(&mut uart_tx, &mut uart_pipe_reader),
    );

    join(usb_future, uart_future).await;
}

/// Read from the USB and write it to the UART TX pipe
async fn usb_read<'d, T: UsbInstance + 'd>(
    usb_rx: &mut embassy_usb::class::cdc_acm::Receiver<'d, Driver<'d, T>>,
    uart_pipe_writer: &mut embassy_sync::pipe::Writer<'_, NoopRawMutex, 64>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = usb_rx.read_packet(&mut buf).await?;
        let data = &buf[..n];
        log::debug!("[UART]: USB IN: {:?}", data);
        (*uart_pipe_writer).write(data).await;
    }
}

/// Read from the USB TX pipe and write it to the USB
async fn usb_write<'d, T: UsbInstance + 'd>(
    usb_tx: &mut embassy_usb::class::cdc_acm::Sender<'d, Driver<'d, T>>,
    usb_pipe_reader: &mut Reader<'_, NoopRawMutex, 64>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = (*usb_pipe_reader).read(&mut buf).await;
        let data = &buf[..n];
        log::debug!("[UART]: USB OUT: {:?}", data);
        usb_tx.write_packet(data).await?;
    }
}

/// Read from the UART and write it to the USB TX pipe
async fn uart_read<PIO: PioInstance, const SM: usize>(
    uart_rx: &mut PioUartRx<'_, PIO, SM>,
    usb_pipe_writer: &mut Writer<'_, NoopRawMutex, 64>,
) -> ! {
    loop {
        let byte = uart_rx.read_u8().await;
        let data = &[byte];
        log::debug!("[UART]: UART IN: {:?}", data);
        (*usb_pipe_writer).write(data).await;
    }
}

/// Read from the UART TX pipe and write it to the UART
async fn uart_write<PIO: PioInstance, const SM: usize>(
    uart_tx: &mut PioUartTx<'_, PIO, SM>,
    uart_pipe_reader: &mut Reader<'_, NoopRawMutex, 64>,
) -> ! {
    let mut buf = [0; 64];
    loop {
        let n = (*uart_pipe_reader).read(&mut buf).await;
        let data = &buf[..n];
        log::debug!("[UART]: UART OUT: {:?}", data);
        for &byte in data {
            uart_tx.write_u8(byte).await;
        }
    }
}
