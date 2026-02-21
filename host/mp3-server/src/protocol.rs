pub use mp3_protocol::*;

use serde::{Deserialize, Serialize};
use tokio::io::{self, AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

/// Write a postcard-serialized message with a 4-byte big-endian length prefix.
pub async fn write_message<T: Serialize>(stream: &mut TcpStream, msg: &T) -> io::Result<()> {
    let payload =
        postcard::to_stdvec(msg).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    let len = (payload.len() as u32).to_be_bytes();
    stream.write_all(&len).await?;
    stream.write_all(&payload).await?;
    stream.flush().await
}

/// Read a length-prefixed postcard message.
pub async fn read_message<T: for<'de> Deserialize<'de>>(stream: &mut TcpStream) -> io::Result<T> {
    let mut len_buf = [0u8; 4];
    stream.read_exact(&mut len_buf).await?;
    let len = u32::from_be_bytes(len_buf) as usize;

    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf).await?;

    postcard::from_bytes(&buf).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))
}
