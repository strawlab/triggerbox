use crate::{NameType, DEVICE_NAME_LEN};
use anyhow::Result;

use crc::{Crc, CRC_8_MAXIM_DOW};

use tokio_serial::{SerialPort, SerialPortBuilderExt};

pub const CRC_MAXIM: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);

async fn reset_device(device: &mut tokio_serial::SerialStream) -> Result<()> {
    device.write_request_to_send(false)?;
    device.write_data_terminal_ready(false)?;
    tokio::time::sleep(std::time::Duration::from_millis(250)).await;
    device.write_request_to_send(true)?;
    device.write_data_terminal_ready(true)?;
    tokio::time::sleep(std::time::Duration::from_millis(250)).await;
    Ok(())
}

async fn flush_device<W: tokio::io::AsyncWriteExt + std::marker::Unpin>(ser: &mut W) -> Result<()> {
    for _ in 0..5 {
        ser.flush().await?;
        tokio::time::sleep(std::time::Duration::from_millis(50)).await;
    }
    Ok(())
}

#[derive(Debug, thiserror::Error)]
enum UdevError {
    #[error("CRC failed")]
    CrcFail,
    #[error("IO error {0}")]
    Io(#[from] std::io::Error),
}

async fn get_device_name(
    device: &mut tokio_serial::SerialStream,
) -> std::result::Result<NameType, UdevError> {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    device.write_all(b"N?").await?;

    let alloc_len = DEVICE_NAME_LEN + 2;

    let mut buf = vec![0; alloc_len];

    // Wait half second for full answer.
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    let len = device.read(&mut buf).await?;
    assert!(len > DEVICE_NAME_LEN, "No CRC returned");

    let name_and_crc = &buf[..len];
    trace!("get_device_name read {} bytes: {:?}", len, name_and_crc);
    let name = &name_and_crc[..DEVICE_NAME_LEN];
    let crc_buf = &name_and_crc[DEVICE_NAME_LEN..];
    let expected_crc = std::str::from_utf8(crc_buf).expect("from utf8");
    trace!("expected CRC: {:?}", expected_crc);

    let computed_crc = format!("{:X}", CRC_MAXIM.checksum(name));
    trace!("computed CRC: {:?}", computed_crc);
    if computed_crc == expected_crc {
        let mut result = [0; DEVICE_NAME_LEN];
        result[..DEVICE_NAME_LEN].copy_from_slice(name);
        Ok(Some(result))
    } else {
        Err(UdevError::CrcFail)
    }
}

pub async fn serial_handshake(
    serial_device: &str,
    baud_rate: u32,
) -> Result<(tokio_serial::SerialStream, NameType)> {
    #[allow(unused_mut)]
    let mut ser = tokio_serial::new(serial_device, baud_rate).open_native_async()?;

    #[cfg(unix)]
    ser.set_exclusive(false)
        .expect("Unable to set serial port exclusive to false");

    debug!("Resetting port {}", serial_device);
    reset_device(&mut ser).await?;
    debug!("Sleeping 10 seconds. (This is required for Arduino Nano to reset.)");
    tokio::time::sleep(std::time::Duration::from_millis(10_000)).await;
    debug!("Flushing serial port");
    flush_device(&mut ser).await?;
    debug!("Getting device name");
    let name = get_device_name(&mut ser)
        .await
        .map_err(anyhow::Error::from)?;
    Ok((ser, name))
}
