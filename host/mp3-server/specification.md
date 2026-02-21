# MP3 streaming server

Create a mp3 streaming server in Rust.

Use the clap crate for command argument parsing.
Use the id3 crate for ID3 reading.
Use the mp3lame-encoder crate to transcode MP3 files.
Use the postcard crate to implement network packages.

## Network protocol

 - Use TCP/IP as transport on port 6564.
 - Use Tokio async.

Postcard packages shall be `no_std` compatible without `alloc`. Use `heapless` for collections like `Vec`, `String`.

The server shall be able to responde to List, Play, Stop event when streaming.

There is a 8-bit status value, with following status codes.
 - OK
 - Unknown error
 - Title not found

Following chapters describe commands shall exist.

### List

The client requests a list of files to play.

Update the content of the cache from the music file directory before sending a response.

The response starts with the 8-bit status OK.
The response contains a list of files with their 32-bit hash and the music title from IDv3 in the music file. The title is a UTF-8 encoded string, [u8; 64].

### Play request

The client requests a MP3 file using the 32-bit hash.

The response is 4Kib chunks of the transcoded MP3 file sent in separate packages.

The response package shall contain,

 - The 8-bit status, OK, Title not found, ...
 - The 32-bit hash corresponding to the MP3 file
 - The chunk index
 - The end chunk index, corresponding to the MP3 file size.
 - The 4Kib chunk corresponding to the data from the transcoded MP3 file

The server shall start a new stream if a new Play request is received.

### Stop request

The client requests stop of streaming.

The response is the 8-bit status OK.

## Music storage

 - Support mp3, aac and m4a input files.
 - Scan a supplied directory for music files. Read the song title from IDv3 and create a 32-bit hash sum from the file content. Transcode the file and put in cache.
 - Create a cache directory for transcoded MP3 files.
 - When requested by the client, transcode the mp3 file like the ffmpeg command,
  `ffmpeg -i in.mp3 -ac 1 -ar 44100 -q:a 6 -map 0:a -map_metadata -1 out.mp3`
