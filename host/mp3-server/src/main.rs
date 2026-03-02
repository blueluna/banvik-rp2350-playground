mod library;
mod protocol;
mod transcode;

use std::path::PathBuf;
use std::sync::Arc;

use clap::Parser;
use tokio::net::TcpListener;
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::sync::{Mutex, mpsc};

use library::Library;
use protocol::{Request, Response, Status, StreamChunk};

use crate::library::Sha256Hash;

#[derive(Parser)]
#[command(name = "mp3-server", about = "MP3 streaming server for RP2350")]
struct Args {
    /// Directory containing MP3 files
    directory: PathBuf,

    /// TCP port for the control interface (stream uses port+1)
    #[arg(short, long, default_value_t = 6564)]
    port: u16,
}

const CHUNK_SIZE: usize = 4096;

enum WriteItem {
    Message(Response),
    /// Begin streaming `data` over the stream TCP socket; send Play ack over control TCP first.
    StartStream {
        data: Vec<u8>,
        hash: [u8; 32],
    },
    StopStream,
}

#[tokio::main]
async fn main() -> std::io::Result<()> {
    let args = Args::parse();

    println!("Scanning {:?} for MP3 files...", args.directory);
    let library = Arc::new(Mutex::new(Library::from_path(&args.directory)?));

    let ctrl_addr = format!("0.0.0.0:{}", args.port);
    let stream_addr = format!("0.0.0.0:{}", args.port + 1);
    let ctrl_listener = TcpListener::bind(&ctrl_addr).await?;
    let stream_listener = TcpListener::bind(&stream_addr).await?;
    println!(
        "Listening on {} (TCP control), {} (TCP stream)",
        ctrl_addr, stream_addr
    );

    loop {
        let (ctrl_conn, ctrl_peer) = ctrl_listener.accept().await?;
        println!("Control connection from {}", ctrl_peer);
        let (stream_conn, stream_peer) = stream_listener.accept().await?;
        println!("Stream connection from {}", stream_peer);
        let lib = library.clone();
        tokio::spawn(async move {
            if let Err(e) = handle_connection(ctrl_conn, stream_conn, lib).await {
                eprintln!("Error handling {}: {}", ctrl_peer, e);
            }
            println!("Disconnected {}", ctrl_peer);
        });
    }
}

async fn handle_connection(
    ctrl_conn: tokio::net::TcpStream,
    stream_conn: tokio::net::TcpStream,
    library: Arc<Mutex<Library>>,
) -> std::io::Result<()> {
    let (reader, writer) = ctrl_conn.into_split();
    let (tx, rx) = mpsc::channel::<WriteItem>(32);
    let (_, stream_writer) = stream_conn.into_split();
    let stream_writer = Arc::new(Mutex::new(stream_writer));

    tokio::select! {
        r = reader_task(reader, library, tx) => r,
        r = writer_task(writer, rx, stream_writer) => r,
    }
}

// ── reader task (TCP control) ─────────────────────────────────────────────────

async fn reader_task(
    mut reader: OwnedReadHalf,
    library: Arc<Mutex<Library>>,
    tx: mpsc::Sender<WriteItem>,
) -> std::io::Result<()> {
    loop {
        let request: Request = match protocol::read_message(&mut reader).await {
            Ok(r) => r,
            Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(e) => return Err(e),
        };

        match request {
            Request::List => {
                println!("  -> List");
                if let Err(e) = library.lock().await.rescan() {
                    eprintln!("  -> Rescan error: {e}");
                }
                let songs: heapless::Vec<_, 128> =
                    library.lock().await.list_songs().into_iter().collect();
                tx.send(WriteItem::Message(Response::SongList {
                    status: Status::Ok,
                    songs,
                }))
                .await
                .ok();
            }

            Request::QueryUid { uid } => {
                println!("  -> QueryUid {uid:016x}");
                let mut lib = library.lock().await;
                let msg = match lib.lookup_hash_from_uid(uid).and_then(|h| lib.get_song(h)) {
                    Some(song) => {
                        println!("     found: {}", String::from_utf8_lossy(&song.title));
                        Response::Song {
                            status: Status::Ok,
                            song,
                        }
                    }
                    None => {
                        let _ = lib.rescan();
                        match lib.lookup_hash_from_uid(uid).and_then(|h| lib.get_song(h)) {
                            Some(song) => {
                                println!("     found: {}", String::from_utf8_lossy(&song.title));
                                Response::Song {
                                    status: Status::Ok,
                                    song,
                                }
                            }
                            None => {
                                Response::Song {
                                    status: Status::UidNotFound,
                                    song: protocol::SongEntry::default(),
                                }
                            }
                        }
                    }
                };
                tx.send(WriteItem::Message(msg)).await.ok();
            }

            Request::PlayUid { uid } => {
                println!("  -> PlayUid {uid:016x}");
                let result = library.lock().await.get_transcoded_by_uid(uid);
                let item = match result {
                    Ok((data, hash)) => WriteItem::StartStream { data, hash: hash.0 },
                    Err(()) => WriteItem::Message(Response::Play {
                        status: Status::UidNotFound,
                        hash: [0u8; 32],
                        total_chunks: 0,
                    }),
                };
                tx.send(item).await.ok();
            }

            Request::PlayHash { hash } => {
                let sha = Sha256Hash::from(hash);
                println!("  -> PlayHash {}", sha.to_hex());
                let result = library.lock().await.get_transcoded_by_hash(sha);
                let item = match result {
                    Ok((data, hash)) => WriteItem::StartStream { data, hash: hash.0 },
                    Err(()) => WriteItem::Message(Response::Play {
                        status: Status::HashNotFound,
                        hash: [0u8; 32],
                        total_chunks: 0,
                    }),
                };
                tx.send(item).await.ok();
            }

            Request::Stop => {
                println!("  -> Stop");
                tx.send(WriteItem::StopStream).await.ok();
            }
        }

        tokio::task::yield_now().await;
    }
}

// ── writer task (TCP control + TCP streaming) ─────────────────────────────────

async fn writer_task(
    mut writer: OwnedWriteHalf,
    mut rx: mpsc::Receiver<WriteItem>,
    stream_writer: Arc<Mutex<OwnedWriteHalf>>,
) -> std::io::Result<()> {
    // If a stream is active this holds its cancel sender and join handle.
    let mut active_stream: Option<(
        tokio::sync::oneshot::Sender<()>,
        tokio::task::JoinHandle<std::io::Result<()>>,
    )> = None;

    loop {
        let Some(item) = rx.recv().await else {
            return Ok(());
        };

        match item {
            WriteItem::Message(msg) => {
                println!("  -> Sending response: {:?}", msg);
                protocol::write_message(&mut writer, &msg).await?;
            }

            WriteItem::StopStream => {
                if let Some((cancel_tx, handle)) = active_stream.take() {
                    let _ = cancel_tx.send(());
                    if let Err(e) = handle.await {
                        eprintln!("  -> Stream task panicked: {e:?}");
                    }
                    println!("  -> Stream stopped");
                }
            }

            WriteItem::StartStream { data, hash } => {
                // Cancel any in-progress stream first.
                if let Some((cancel_tx, handle)) = active_stream.take() {
                    let _ = cancel_tx.send(());
                    let _ = handle.await;
                }

                let total_chunks = data.len().div_ceil(CHUNK_SIZE) as u32;
                println!(
                    "  -> Starting stream of {} bytes in {total_chunks} chunks (hash {})",
                    data.len(),
                    Sha256Hash(hash).to_hex()
                );

                // Acknowledge over control TCP so the client knows what's coming.
                protocol::write_message(
                    &mut writer,
                    &Response::Play {
                        status: Status::Ok,
                        hash,
                        total_chunks,
                    },
                )
                .await?;

                let (cancel_tx, cancel_rx) = tokio::sync::oneshot::channel();
                let sw = stream_writer.clone();
                let handle = tokio::spawn(async move {
                    let mut guard = sw.lock().await;
                    stream_tcp(&data, hash, total_chunks, &mut *guard, cancel_rx).await
                });
                active_stream = Some((cancel_tx, handle));
            }
        }
    }
}

/// Stream `data` as length-prefixed postcard-encoded [`StreamChunk`]s over TCP.
/// Stops early when `cancel` is triggered.
async fn stream_tcp(
    data: &[u8],
    hash: [u8; 32],
    total_chunks: u32,
    stream_writer: &mut OwnedWriteHalf,
    mut cancel: tokio::sync::oneshot::Receiver<()>,
) -> std::io::Result<()> {
    for chunk_index in 0..total_chunks {
        // Poll cancel before each chunk without blocking.
        if cancel.try_recv().is_ok() {
            println!("  -> Stream cancelled at chunk {chunk_index}/{total_chunks}");
            return Ok(());
        }

        let start = chunk_index as usize * CHUNK_SIZE;
        let end = ((chunk_index as usize + 1) * CHUNK_SIZE).min(data.len());
        let slice = &data[start..end];

        let mut chunk_data: heapless::Vec<u8, 4096> = heapless::Vec::new();
        chunk_data
            .extend_from_slice(slice)
            .expect("slice <= CHUNK_SIZE");

        let chunk = StreamChunk {
            status: Status::Ok,
            hash,
            chunk_index,
            total_chunks,
            data: chunk_data,
        };
        protocol::write_message(stream_writer, &chunk).await?;
    }

    println!(
        "  -> Streamed {total_chunks} chunks ({} bytes) over TCP",
        data.len()
    );
    Ok(())
}
