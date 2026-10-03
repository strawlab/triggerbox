// SPDX-License-Identifier: MIT OR Apache-2.0

use crate::{DEVICE_NAME_LEN, NameType};
use anyhow::Result;
use log::{debug, trace};

use crc::{CRC_8_MAXIM_DOW, Crc};

use tokio_serial::{SerialPort, SerialPortBuilderExt};

pub(crate) const CRC_MAXIM: Crc<u8> = Crc::<u8>::new(&CRC_8_MAXIM_DOW);

async fn reset_device(device: &mut tokio_serial::SerialStream) -> Result<()> {
    device.write_data_terminal_ready(false)?;
    tokio::time::sleep(std::time::Duration::from_millis(250)).await;
    device.write_data_terminal_ready(true)?;
    tokio::time::sleep(std::time::Duration::from_millis(250)).await;
    Ok(())
}

#[derive(Debug, thiserror::Error)]
enum UdevError {
    #[error("CRC failed")]
    CrcFail,
    #[error("no CRC returned")]
    NoCrc,
    #[error("CRC is not valid UTF-8")]
    CrcNotUtf8(#[from] std::str::Utf8Error),
    #[error("IO error {0}")]
    Io(#[from] std::io::Error),
}

async fn get_device_name(
    device: &mut tokio_serial::SerialStream,
) -> std::result::Result<NameType, UdevError> {
    use tokio::io::{AsyncReadExt, AsyncWriteExt};

    device.write_all(b"N?").await?;

    let mut buf = [0; DEVICE_NAME_LEN + 2];

    // Wait half second for full answer.
    tokio::time::sleep(std::time::Duration::from_millis(500)).await;

    let len = device.read(&mut buf).await?;
    let name_and_crc = buf.get(..len).unwrap_or_default();
    trace!("get_device_name read {len} bytes: {name_and_crc:?}");
    let Some((name, crc_buf)) = name_and_crc.split_first_chunk::<DEVICE_NAME_LEN>() else {
        return Err(UdevError::NoCrc);
    };
    if crc_buf.is_empty() {
        return Err(UdevError::NoCrc);
    }
    let expected_crc = std::str::from_utf8(crc_buf)?;
    trace!("expected CRC: {expected_crc:?}");

    let computed_crc = format!("{:X}", CRC_MAXIM.checksum(name));
    trace!("computed CRC: {computed_crc:?}");
    if computed_crc == expected_crc {
        Ok(Some(*name))
    } else {
        Err(UdevError::CrcFail)
    }
}

pub(crate) async fn serial_handshake(
    serial_device: &str,
    baud_rate: u32,
    sleep_dur: std::time::Duration,
) -> Result<(tokio_serial::SerialStream, NameType)> {
    let mut ser = tokio_serial::new(serial_device, baud_rate).open_native_async()?;

    #[cfg(unix)]
    ser.set_exclusive(false)?;

    debug!("Resetting port {serial_device}");
    reset_device(&mut ser).await?;
    debug!(
        "Sleeping {:.1} seconds. (This is required for Arduino Nano to reset.)",
        sleep_dur.as_secs_f32()
    );
    tokio::time::sleep(sleep_dur).await;
    debug!("Getting device name");
    let name = get_device_name(&mut ser)
        .await
        .map_err(anyhow::Error::from)?;
    Ok((ser, name))
}
