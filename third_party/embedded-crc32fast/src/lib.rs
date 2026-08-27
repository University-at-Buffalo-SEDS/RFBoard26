#![no_std]

/// Incremental IEEE CRC-32 with the subset of crc32fast's API used by SEDSNet.
///
/// This bitwise implementation trades throughput for roughly 16 KiB less flash
/// than crc32fast's slicing-by-16 lookup table. CAN-FD frames are small enough
/// that the bounded CPU cost is preferable on 512 KiB STM32 targets.
#[derive(Clone, Copy, Debug)]
pub struct Hasher {
    state: u32,
}

impl Hasher {
    #[inline]
    pub const fn new() -> Self {
        Self { state: u32::MAX }
    }

    pub fn update(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.state ^= u32::from(byte);
            for _ in 0..8 {
                let mask = 0u32.wrapping_sub(self.state & 1);
                self.state = (self.state >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
    }

    #[inline]
    pub const fn finalize(self) -> u32 {
        !self.state
    }
}

impl Default for Hasher {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::Hasher;

    #[test]
    fn ieee_check_vector_and_incremental_updates() {
        let mut one = Hasher::new();
        one.update(b"123456789");
        assert_eq!(one.finalize(), 0xCBF4_3926);

        let mut split = Hasher::new();
        split.update(b"1234");
        split.update(b"56789");
        assert_eq!(split.finalize(), 0xCBF4_3926);
    }
}
