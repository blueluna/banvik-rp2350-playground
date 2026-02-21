use std::path::Path;

use symphonia::core::audio::SampleBuffer;
use symphonia::core::codecs::DecoderOptions;
use symphonia::core::errors::Error as SymphoniaError;
use symphonia::core::formats::FormatOptions;
use symphonia::core::io::MediaSourceStream;
use symphonia::core::meta::MetadataOptions;
use symphonia::core::probe::Hint;

use mp3lame_encoder::{Builder, FlushNoGap, MonoPcm, Quality, VbrMode};

/// Transcode an MP3 file to mono 44100 Hz VBR quality 6.
///
/// Equivalent to: ffmpeg -i in.mp3 -ac 1 -ar 44100 -q:a 6 -map 0:a -map_metadata -1 out.mp3
pub fn transcode(source_path: &Path) -> Result<Vec<u8>, String> {
    let pcm = decode_to_mono_pcm(source_path).map_err(|e| e.to_string())?;
    let mp3 = encode_mp3(&pcm)?;
    Ok(mp3)
}

/// Decode an MP3 file to mono i16 PCM at 44100 Hz.
fn decode_to_mono_pcm(source_path: &Path) -> Result<Vec<i16>, Box<dyn std::error::Error>> {
    let file = std::fs::File::open(source_path)?;
    let mss = MediaSourceStream::new(Box::new(file), Default::default());

    println!("Probe: {}", source_path.display());

    let hint = Hint::new();
    // hint.with_extension("mp3");

    let probed = symphonia::default::get_probe().format(
        &hint,
        mss,
        &FormatOptions::default(),
        &MetadataOptions::default(),
    )?;


    println!("Probed: {}", source_path.display());

    let mut format = probed.format;
    let track = format.default_track().ok_or("no audio track found")?;
    let track_id = track.id;
    let channels = track.codec_params.channels.map(|c| c.count()).unwrap_or(2);
    let sample_rate = track.codec_params.sample_rate.unwrap_or(44100);

    let mut decoder =
        symphonia::default::get_codecs().make(&track.codec_params, &DecoderOptions::default())?;
    
    let track_codec = track.codec_params.codec;
    let codec = symphonia::default::get_codecs().get_codec(track_codec)
        .ok_or_else(|| format!("unsupported codec: {:?}", track_codec))?;
    
    println!("Codec: {}", codec.short_name);

    let mut all_samples: Vec<i16> = Vec::new();
    let mut sample_buf: Option<SampleBuffer<i16>> = None;

    loop {
        let packet = match format.next_packet() {
            Ok(p) => p,
            Err(SymphoniaError::IoError(ref e))
                if e.kind() == std::io::ErrorKind::UnexpectedEof =>
            {
                break;
            }
            Err(e) => return Err(e.into()),
        };

        if packet.track_id() != track_id {
            continue;
        }

        let decoded = match decoder.decode(&packet) {
            Ok(d) => d,
            Err(SymphoniaError::DecodeError(_)) => continue,
            Err(e) => return Err(e.into()),
        };

        let spec = *decoded.spec();
        let duration = decoded.capacity() as u64;

        let sbuf = sample_buf.get_or_insert_with(|| SampleBuffer::new(duration, spec));
        sbuf.copy_interleaved_ref(decoded);
        let samples = sbuf.samples();

        // Mix to mono
        if channels >= 2 {
            for chunk in samples.chunks(channels) {
                let mono = chunk.iter().map(|&s| s as i32).sum::<i32>() / channels as i32;
                all_samples.push(mono as i16);
            }
        } else {
            all_samples.extend_from_slice(samples);
        }
    }

    // Resample to 44100 Hz if needed
    if sample_rate != 44100 {
        all_samples = resample(&all_samples, sample_rate, 44100);
    }

    Ok(all_samples)
}

/// Encode mono i16 PCM at 44100 Hz to MP3 with VBR quality 6.
fn encode_mp3(pcm: &[i16]) -> Result<Vec<u8>, String> {
    println!("Encoding MP3, {} samples", pcm.len());
    let mut builder = Builder::new().ok_or("failed to create LAME builder")?;
    builder
        .set_num_channels(1)
        .map_err(|e| format!("set_num_channels: {e}"))?;
    builder
        .set_sample_rate(44100)
        .map_err(|e| format!("set_sample_rate: {e}"))?;
    builder
        .set_quality(Quality::NearBest)
        .map_err(|e| format!("set_quality: {e}"))?;
    builder
        .set_vbr_mode(VbrMode::Mtrh)
        .map_err(|e| format!("set_vbr_mode: {e}"))?;
    builder
        .set_vbr_quality(Quality::NearBest)
        .map_err(|e| format!("set_vbr_quality: {e}"))?;
    let mut encoder = builder.build().map_err(|e| format!("build: {e}"))?;

    let buffer_size = mp3lame_encoder::max_required_buffer_size(pcm.len());
    let mut mp3_out = Vec::with_capacity(buffer_size);

    // Encode in 1152-sample chunks (one MP3 frame)
    for chunk in pcm.chunks(1152) {
        let input = MonoPcm(chunk);
        encoder
            .encode_to_vec(input, &mut mp3_out)
            .map_err(|e| format!("encode: {e}"))?;
    }

    // Flush encoder
    encoder
        .flush_to_vec::<FlushNoGap>(&mut mp3_out)
        .map_err(|e| format!("flush: {e}"))?;

    Ok(mp3_out)
}

/// Simple linear interpolation resampler.
fn resample(samples: &[i16], from_rate: u32, to_rate: u32) -> Vec<i16> {
    let ratio = from_rate as f64 / to_rate as f64;
    let output_len = (samples.len() as f64 / ratio) as usize;
    let mut output = Vec::with_capacity(output_len);

    for i in 0..output_len {
        let src_pos = i as f64 * ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;
        let s0 = samples[idx] as f64;
        let s1 = samples.get(idx + 1).copied().unwrap_or(samples[idx]) as f64;
        output.push((s0 + frac * (s1 - s0)) as i16);
    }

    output
}
