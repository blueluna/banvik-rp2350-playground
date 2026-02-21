mod library;
mod protocol;
mod transcode;

use std::path::PathBuf;

use clap::Parser;
use tokio::net::{TcpListener, TcpStream};

use library::Library;
use protocol::{Request, Response, Status};

#[derive(Parser)]
#[command(name = "mp3-server", about = "MP3 streaming server for RP2350")]
struct Args {
    /// Directory containing MP3 files
    directory: PathBuf,

    /// TCP port to listen on
    #[arg(short, long, default_value_t = 6564)]
    port: u16,
}

const CHUNK_SIZE: usize = 4096;

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let args = Args::parse();

    println!("Scanning {:?} for MP3 files...", args.directory);
    let mut library = Library::scan_directory(&args.directory)?;

    let addr = format!("0.0.0.0:{}", args.port);
    let listener = TcpListener::bind(&addr).await?;
    println!("Listening on {}", addr);

    loop {
        let (mut stream, peer) = listener.accept().await?;
        println!("Connection from {}", peer);

        if let Err(e) = handle_connection(&mut stream, &mut library).await {
            eprintln!("Error handling {}: {}", peer, e);
        }

        println!("Disconnected {}", peer);
    }
}

async fn handle_connection(
    stream: &mut TcpStream,
    library: &mut Library,
) -> std::io::Result<()> {
    loop {
        let request: Request = match protocol::read_message(stream).await {
            Ok(req) => req,
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(e) => return Err(e),
        };

        match request {
            Request::List => {
                println!("  -> List request");
                // Rescan directory before responding
                if let Err(e) = library.rescan() {
                    eprintln!("  -> Rescan error: {}", e);
                }
                let songs: heapless::Vec<_, 128> =
                    library.list_songs().into_iter().collect();
                protocol::write_message(
                    stream,
                    &Response::SongList {
                        status: Status::Ok,
                        songs,
                    },
                )
                .await?;
            }
            Request::Play { hash } => {
                println!("  -> Play request for {:08x}", hash);
                match library.get_transcoded(hash) {
                    Ok(data) => send_chunks(stream, hash, &data).await?,
                    Err(()) => {
                        eprintln!("  -> Title not found: {:08x}", hash);
                        let response = Response::Chunk {
                            status: Status::TitleNotFound,
                            hash,
                            chunk_index: 0,
                            total_chunks: 0,
                            data: heapless::Vec::new(),
                        };
                        protocol::write_message(stream, &response).await?;
                    }
                }
            }
            Request::Stop => {
                println!("  -> Stop request");
                protocol::write_message(stream, &Response::Stop { status: Status::Ok }).await?;
            }
        }
    }
}

async fn send_chunks(
    stream: &mut TcpStream,
    hash: u32,
    data: &[u8],
) -> std::io::Result<()> {
    let total_chunks = data.len().div_ceil(CHUNK_SIZE) as u32;

    for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
        // Check if client sent a new request (Play or Stop) to interrupt streaming.
        // Use a non-blocking peek to see if data is available.
        let mut peek_buf = [0u8; 1];
        match stream.try_read(&mut peek_buf) {
            Ok(0) => {
                // Connection closed
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "client disconnected",
                ));
            }
            Ok(_) => {
                // Client sent data - they want to interrupt.
                // We can't un-read the byte, so just stop streaming.
                println!("  -> Stream interrupted at chunk {}/{}", i, total_chunks);
                return Ok(());
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                // No data available, continue streaming
            }
            Err(e) => return Err(e),
        }

        let mut heapless_data = heapless::Vec::new();
        heapless_data
            .extend_from_slice(chunk)
            .expect("chunk <= 4096");

        protocol::write_message(
            stream,
            &Response::Chunk {
                status: Status::Ok,
                hash,
                chunk_index: i as u32,
                total_chunks,
                data: heapless_data,
            },
        )
        .await?;
    }

    println!("  -> Sent {} chunks ({} bytes)", total_chunks, data.len());
    Ok(())
}
