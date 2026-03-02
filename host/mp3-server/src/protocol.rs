pub use mp3_protocol::*;

use serde::{Deserialize, Serialize};
use tokio::io::{self, AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

/// Write a postcard-serialized message with a 4-byte big-endian length prefix.
pub async fn write_message<T: Serialize, W: AsyncWrite + Unpin>(
    stream: &mut W,
    msg: &T,
) -> io::Result<usize> {
    let payload =
        postcard::to_stdvec(msg).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    let len = (payload.len() as u32).to_be_bytes();

    // prepend the length bytes to the serialized message and write it all at once
    let mut buf = Vec::with_capacity(4 + payload.len());
    buf.extend_from_slice(&len);
    buf.extend_from_slice(&payload);
    stream.write_all(&buf).await?;
    stream.flush().await?;
    Ok(payload.len())
}

/// Read a length-prefixed postcard message.
pub async fn read_message<T: for<'de> Deserialize<'de>, R: AsyncRead + Unpin>(
    stream: &mut R,
) -> io::Result<T> {
    let mut len_buf = [0u8; 4];
    stream.read_exact(&mut len_buf).await?;
    let len = u32::from_be_bytes(len_buf) as usize;

    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf).await?;

    postcard::from_bytes(&buf).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}
