use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Receiver, Sender};
use embassy_usb::class::cdc_acm::CdcAcmClass;

#[derive(Clone)]
pub enum UsbCommand {
    Read { size: usize },
    Write { data: heapless::Vec<u8, 64> },
    WriteAck,
}

pub async fn usb_task<'d, D: embassy_usb::driver::Driver<'d>>(
    mut class: CdcAcmClass<'d, D>,
    mut cmd_receiver: Receiver<'_, NoopRawMutex, UsbCommand>,
    mut data_to_usb: Sender<'_, NoopRawMutex, Result<heapless::Vec<u8, 64>, ()>>,
) -> ! {
    loop {
        let cmd = cmd_receiver.receive().await;
        match cmd {
            UsbCommand::Read { size } => {
                let received = data_to_usb.send().await;
                assert!(*size <= 64);
                *received = Ok(heapless::Vec::new());
                let result = if let Ok(ref mut buffer) = received {
                    unsafe {
                        buffer.set_len(*size);
                    }

                    match class.read_packet(buffer).await {
                        Ok(bytes_read) => {
                            buffer.truncate(bytes_read);
                            Ok(())
                        }
                        Err(_) => Err(()),
                    }
                } else {
                    Ok(())
                };

                match result {
                    Ok(_) => (),
                    Err(_) => *received = Err(()),
                }

                data_to_usb.send_done();
            }
            UsbCommand::Write { data } => {
                let result = data_to_usb.send().await;
                // Write directly to USB CDC-ACM class
                let mut write_success = true;
                for chunk in data.chunks(class.max_packet_size().into()) {
                    if class.write_packet(chunk).await.is_err() {
                        write_success = false;
                        break;
                    }
                }
                if write_success {
                    *result = Ok(heapless::Vec::new()); // Empty vec indicates write success
                } else {
                    *result = Err(());
                }
                data_to_usb.send_done();
            }
            UsbCommand::WriteAck => {
                let result = data_to_usb.send().await;
                // Write ACK directly to USB CDC-ACM class
                if class.write_packet(&[crate::S_ACK]).await.is_ok() {
                    *result = Ok(heapless::Vec::new()); // Empty vec indicates write success
                } else {
                    *result = Err(());
                }
                data_to_usb.send_done();
            }
        }
        cmd_receiver.receive_done();
    }
}
