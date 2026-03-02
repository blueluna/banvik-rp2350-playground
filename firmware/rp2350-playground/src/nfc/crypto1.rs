use crate::parity;
/// Non-linear filter function for Crypto1.
/// Based on the implementation in proxmark3/common/crapto1/crapto1.h
#[inline]
pub fn filter(x: u32) -> bool {
    let mut f: u32;

    f = (0xf22c0 >> (x & 0xf)) & 16;
    f |= (0x6c9c0 >> ((x >> 4) & 0xf)) & 8;
    f |= (0x3c8b0 >> ((x >> 8) & 0xf)) & 4;
    f |= (0x1e458 >> ((x >> 12) & 0xf)) & 2;
    f |= (0x0d938 >> ((x >> 16) & 0xf)) & 1;
    
    bit(0xec57e80a, f as usize)
}

/// Parity helper for u32.
/// Returns 1 if the number of set bits is odd, otherwise returns 0.
#[inline]
fn even_parity32(x: u32) -> u32 {
    if x.count_ones() % 2 == 0 { 0 } else { 1 }
}

/// Bit extraction helper.
#[inline]
fn bit(x: u64, n: usize) -> bool {
    ((x >> n) & 1) == 1
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Crypto1State {
    pub odd: u32,
    pub even: u32,
}

const LF_POLY_ODD: u32 = 0x29CE5C;
const LF_POLY_EVEN: u32 = 0x870804;

impl Crypto1State {
    /// Creates a new Crypto1State and initializes it with a 48-bit key.
    pub fn new(key: u64) -> Self {
        let mut state = Self::default();
        state.init(key);
        state
    }

    /// Initializes the state with a 48-bit key.
    pub fn init(&mut self, key: u64) {
        self.odd = 0;
        self.even = 0;
        for i in (1..=47).step_by(2) {
            self.odd = (self.odd << 1) | (bit(key, (i - 1) ^ 7) as u32);
            self.even = (self.even << 1) | (bit(key, i ^ 7) as u32);
        }
    }

    /// Steps the LFSR by one bit.
    /// Returns the keystream bit.
    pub fn step_bit(&mut self, input: Option<bool>, is_encrypted: bool) -> bool {
        let ks_bit = filter(self.odd);
        
        let mut feedin = if is_encrypted { ks_bit as u32 } else { 0u32 };
        if let Some(input) = input {
            feedin ^= if input { 1 } else { 0 };
        }
        feedin ^= LF_POLY_ODD & self.odd;
        feedin ^= LF_POLY_EVEN & self.even;
        
        // Step and swap
        let next_even = (self.even << 1) | even_parity32(feedin);
        self.even = self.odd;
        self.odd = next_even;

        ks_bit
    }

    /// Encrypts or decrypts a byte.
    pub fn step_byte(&mut self) -> u8 {
        let mut ret = 0u8;
        for i in 0..8 {
            ret |= (self.step_bit(None, false) as u8) << i;
        }
        ret
    }

    /// Encrypts or decrypts a 32-bit word.
    pub fn step_word(&mut self) -> u32 {
        let mut ret = 0u32;
        for i in 0..32 {
            let shifter = i ^ 24;
            let ks_bit = self.step_bit(None, false);
            ret |= (ks_bit as u32) << shifter;
        }
        ret
    }

    /// Encrypts or decrypts a 32-bit word.
    pub fn step_word_fill(&mut self, input: u32, is_encrypted: bool) -> u32 {
        let src = input;
        let mut ret = 0u32;
        for i in 0..32 {
            let shifter = i ^ 24;
            let ks_bit = self.step_bit(Some(bit(src as u64, shifter)), is_encrypted);
            ret |= (ks_bit as u32) << shifter;
        }
        ret
    }

    /// Encrypts or decrypts a byte.
    pub fn step_byte_parity(&mut self, input: Option<u8>) -> (u8, bool) {
        let mut ret = 0u8;
        if let Some(input) = input {
            for i in 0..8 {
                ret |= (self.step_bit(Some(bit(input as u64, i)), false) as u8) << i;
            }
        }
        else {
            for i in 0..8 {
                ret |= (self.step_bit(None, false) as u8) << i;
            }
        }
        let parity_bit = filter(self.odd) ^ parity::odd_parity(ret);
        (ret, parity_bit)
    }

    /// Encrypts a slice of bytes with parity bits packed into output.
    /// Each input byte produces 9 bits of output (8 data bits + 1 parity bit).
    /// Output is packed as: [byte0, parity0|byte1_bits..., ...]
    /// Returns the number of bytes written to output, or None if output is too small.
    pub fn encrypt_with_parity(&mut self, input: &[u8], fill: &[u8], output: &mut [u8]) -> Option<usize> {
        let output_bits_needed = input.len() * 9;
        let output_bytes_needed = (output_bits_needed + 7) / 8;
        if output.len() < output_bytes_needed {
            return None;
        }
        output.fill(0); // Clear output buffer

        let mut bit_pos = 0usize;
        for (n, &byte) in input.iter().enumerate() {
            let (keystream, parity_bit) = if n < fill.len() {
                // Feed fill bytes into the state as encrypted input
                self.step_byte_parity(Some(fill[n]))
            }
            else {
                // Step the state for the input byte without feeding it in (as if it were encrypted)
                self.step_byte_parity(None)
            };
            let xored = keystream ^ byte;

            for b in 0..8 {
                let byte_idx = bit_pos / 8;
                let bit_idx = bit_pos % 8;
                if (xored >> b) & 1 == 1 {
                    output[byte_idx] |= 1 << bit_idx;
                } else {
                    output[byte_idx] &= !(1 << bit_idx);
                }
                bit_pos += 1;
            }

            let byte_idx = bit_pos / 8;
            let bit_idx = bit_pos % 8;
            if parity_bit {
                output[byte_idx] |= 1 << bit_idx;
            } else {
                output[byte_idx] &= !(1 << bit_idx);
            }
            bit_pos += 1;
        }

        Some(output_bytes_needed)
    }

    /// Recovers the 48-bit LFSR state.
    pub fn get_lfsr(&self) -> u64 {
        let mut lfsr = 0u64;
        for i in (0..24).rev() {
            lfsr = (lfsr << 1) | (bit(self.odd as u64, i ^ 3) as u64);
            lfsr = (lfsr << 1) | (bit(self.even as u64, i ^ 3) as u64);
        }
        lfsr
    }
}


/// MIFARE Classic PRNG successor function.
/// Evolves the nonce `x` by `n` steps.
pub fn prng_successor(input: u32, steps: u32) -> u32 {
    // MIFARE Classic nonces are often handled in a specific endianness
    let mut x = input.swap_bytes();
    for _n in 0..steps {
        // Feedback polynomial: x^16 + x^18 + x^19 + x^21
        let bit = (x >> 16) ^ (x >> 18) ^ (x >> 19) ^ (x >> 21);
        x = (x >> 1) | ((bit & 1) << 31);
    }
    x.swap_bytes()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crypto1() {
        // Mifare authentication trace test using Crypto1State
        // Decrypted random nonces (plaintext)
        // Nt:  b0 0b 77 90
        // Nt': cc 02 30 82
        // Nt'': eb 4e fb 2e
        // Nr:  ae 99 ca 99
        //
        // Trace (encrypted bytes are in brackets):
        // Auth(00): 60 00 f5 7b
        //       Nt: b0 0b 77 90
        // [Nr,Nt']: ca fe ba be 48 76 af fa
        //   [Nt'']: 69 1e 8d 21
//
//   [Read(00)]: 3c  ce  19  28
//  [Block(00)]: c7! 84! b5! 92! a4! 16! 82  fe  eb! 0b! ac  22! ed! 89  e1! 6d  1e  6e!
//   [Read(01)]: c5! 0a  84! c6
//  [Block(01)]: a7  2b! e7! 1b  12! 75! 3a! 13  ef  d1! a3! 5f  28! a5! 69  1a! 4d! 0b
//   [Read(02)]: b0! 7c  a2  29!
//  [Block(02)]: 6a! 15  87! 4b! 25  99! ed! 0f! 6b! f4  85! 3e  77! 6f  4c! c0! 8c! 9c
//       [Halt]: f5  d0  de! 59!
// 

        // Use the common default key (48-bit all ones) as in many Mifare traces.
        let mut s = Crypto1State::new(0xFFFF_FFFF_FFFF);

        let uid = [0xde, 0xad, 0xbe, 0xaf];
        let uid_u32 = u32::from_be_bytes(uid);

        let nt = [0xb0, 0x0b, 0x77, 0x90];
        let nt_prime = [0xcc, 0x02, 0x30, 0x82];

        let nr = [0xae, 0x99, 0xca, 0x99];
        let nt_prime_prime = [0xeb, 0x4e, 0xfb, 0x2e];

        let nt_u32 = u32::from_be_bytes(nt);
        let nt_prime_u32 = u32::from_be_bytes(nt_prime);

        let nt_prime_u32_calculated = prng_successor(nt_u32, 64);
        let nt_prime_calculated = nt_prime_u32_calculated.to_be_bytes();

        assert_eq!(nt_prime_calculated, nt_prime, "Nt' mismatch");

        let nr_u32 = u32::from_be_bytes(nr);

        let nt_prime_prime_u32 = u32::from_be_bytes(nt_prime_prime);
        let nt_prime_prime_u32_calculated = prng_successor(nt_u32, 96);
        let nt_prime_prime_calculated = nt_prime_prime_u32_calculated.to_be_bytes();

        assert_eq!(nt_prime_prime_calculated, nt_prime_prime, "Nt'' mismatch");

        let _ = s.step_word_fill(uid_u32 ^ nt_u32, false);

        let challenge_response_0 = nr_u32 ^ s.step_word_fill(nr_u32, false);
        let challenge_response_1 = nt_prime_u32_calculated ^ s.step_word();

        assert_eq!(challenge_response_0, 0xca_fe_ba_be, "Challenge response 0 mismatch");
        assert_eq!(challenge_response_1, 0x48_76_af_fa, "Challenge response 1 mismatch");

        let challenge_response_2 = s.step_word();
        let challenge_response_2_xor = 0x691e8d21 ^ challenge_response_2;
        assert_eq!(challenge_response_2_xor, nt_prime_prime_u32_calculated, "Challenge response 2 mismatch");

        let read_sequence = [0x30, 0x00, 0x02, 0xa8];
        let mut read_sequence_encoded = [0u8; 4];
        let read_sequence_encoded_expected = [0x3c, 0xce, 0x19, 0x28];

        for (i, byte) in read_sequence.iter().enumerate() {
            let response = s.step_byte();
            read_sequence_encoded[i] = response ^ byte;
        }
        assert_eq!(read_sequence_encoded, read_sequence_encoded_expected, "Read sequence encoding mismatch");

        let read_response_sequence = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x49];
        let mut read_response_sequence_encoded = [0u8; 18];
        let read_response_sequence_encoded_expected = [0xc7, 0x84, 0xb5, 0x92, 0xa4, 0x16, 0x82, 0xfe, 0xeb, 0x0b, 0xac, 0x22, 0xed, 0x89, 0xe1, 0x6d, 0x1e, 0x6e];

        for (i, byte) in read_response_sequence.iter().enumerate() {
            let response = s.step_byte();
            read_response_sequence_encoded[i] = response ^ byte;
        }
        assert_eq!(read_response_sequence_encoded, read_response_sequence_encoded_expected, "Read response sequence encoding mismatch");
    }

    #[test]
    fn test_crypto1_2() {
        // Use the common default key (48-bit all ones) as in many Mifare traces.
        let mut s = Crypto1State::new(0xFFFF_FFFF_FFFF);

        let uid = [0xa9, 0xd1, 0x10, 0x06];
        let uid_u32 = u32::from_be_bytes(uid);
        let nt = [0x00, 0x90, 0x80, 0xa2];
        let nr = [0x12, 0x34, 0x56, 0x78];

        let nt_u32 = u32::from_be_bytes(nt);

        let nt_prime_u32 = prng_successor(nt_u32, 64);
        let nt_prime = nt_prime_u32.to_be_bytes();

        let nr_u32 = u32::from_be_bytes(nr);
        let nt_prime_prime_u32 = prng_successor(nt_u32, 96);
        let nt_prime_prime = nt_prime_prime_u32.to_be_bytes();

        let _ = s.step_word_fill(uid_u32 ^ nt_u32, false);

        let challenge_response_0 = nr_u32 ^ s.step_word_fill(nr_u32, false);
        let challenge_response_1 = nt_prime_u32 ^ s.step_word();

        println!("T {:08x} T' {:08x} R {:08x} UID {:08x} UID + T {:08x}", nt_u32, nt_prime_u32, nr_u32, uid_u32, uid_u32 ^ nt_u32);
        println!("Challenge response: {:08x} {:08x}", challenge_response_0, challenge_response_1);

        assert_eq!(challenge_response_0, 0x8c0deec6, "Challenge response 0 mismatch");
        assert_eq!(challenge_response_1, 0x9ef386fb, "Challenge response 1 mismatch");
    }
}
