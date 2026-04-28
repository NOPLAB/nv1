/// Bounds-checked, byte-by-byte assembler for COBS frames terminated by 0x00.
///
/// Each call to [`FrameReader::push`] consumes one byte and returns a
/// [`PushOutcome`]:
///
/// - `Pending`  — more bytes needed
/// - `Frame(&mut [u8])` — a complete frame (including the trailing 0x00 sentinel),
///   ready to be passed to `postcard::from_bytes_cobs`
/// - `Overflow` — current frame exceeded the buffer; the reader silently drops
///   the rest of the corrupt frame and resumes on the next 0x00 terminator
///
/// Empty frames (a single lone 0x00) are absorbed and not surfaced to the caller.
pub struct FrameReader<const N: usize> {
    buf: [u8; N],
    len: usize,
    overflowed: bool,
}

#[derive(Debug)]
pub enum PushOutcome<'a> {
    Pending,
    Frame(&'a mut [u8]),
    Overflow,
}

impl<const N: usize> FrameReader<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
            overflowed: false,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.overflowed = false;
    }

    pub fn push(&mut self, byte: u8) -> PushOutcome<'_> {
        if self.overflowed {
            if byte == 0 {
                self.overflowed = false;
            }
            return PushOutcome::Pending;
        }

        if self.len >= N {
            self.len = 0;
            self.overflowed = true;
            if byte == 0 {
                self.overflowed = false;
            }
            return PushOutcome::Overflow;
        }

        self.buf[self.len] = byte;
        self.len += 1;

        if byte == 0 {
            let frame_len = self.len;
            self.len = 0;
            if frame_len == 1 {
                return PushOutcome::Pending;
            }
            return PushOutcome::Frame(&mut self.buf[..frame_len]);
        }

        PushOutcome::Pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect_frames(input: &[u8]) -> (Vec<Vec<u8>>, usize) {
        let mut reader: FrameReader<16> = FrameReader::new();
        let mut out: Vec<Vec<u8>> = Vec::new();
        let mut overflows = 0;
        for &b in input {
            match reader.push(b) {
                PushOutcome::Pending => {}
                PushOutcome::Frame(f) => out.push(f.to_vec()),
                PushOutcome::Overflow => overflows += 1,
            }
        }
        (out, overflows)
    }

    #[test]
    fn single_complete_frame() {
        let (frames, overflows) = collect_frames(&[1, 2, 3, 0]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], &[1, 2, 3, 0]);
        assert_eq!(overflows, 0);
    }

    #[test]
    fn pending_until_terminator() {
        let mut reader: FrameReader<8> = FrameReader::new();
        for &b in &[1u8, 2, 3] {
            assert!(matches!(reader.push(b), PushOutcome::Pending));
        }
        assert!(matches!(reader.push(0), PushOutcome::Frame(_)));
    }

    #[test]
    fn lone_zero_is_absorbed() {
        let (frames, overflows) = collect_frames(&[0]);
        assert!(frames.is_empty());
        assert_eq!(overflows, 0);
    }

    #[test]
    fn back_to_back_frames() {
        let (frames, _) = collect_frames(&[1, 2, 0, 3, 4, 5, 0]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], &[1, 2, 0]);
        assert_eq!(frames[1], &[3, 4, 5, 0]);
    }

    #[test]
    fn multiple_zero_terminators_in_a_row() {
        let (frames, _) = collect_frames(&[1, 0, 0, 0, 2, 0]);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], &[1, 0]);
        assert_eq!(frames[1], &[2, 0]);
    }

    #[test]
    fn oversized_frame_signals_overflow_and_recovers() {
        let mut reader: FrameReader<4> = FrameReader::new();
        let mut overflowed = false;
        // 5 non-zero bytes with no terminator — exceeds capacity
        for &b in &[1u8, 2, 3, 4, 5] {
            if let PushOutcome::Overflow = reader.push(b) {
                overflowed = true;
            }
        }
        assert!(overflowed, "should signal overflow at least once");
        // Continued garbage bytes are silently dropped until next terminator
        for &b in &[6u8, 7, 8] {
            assert!(matches!(reader.push(b), PushOutcome::Pending));
        }
        // Terminator clears the overflow latch but yields no frame
        assert!(matches!(reader.push(0), PushOutcome::Pending));
        // Next legit frame is accepted
        match reader.push(9) {
            PushOutcome::Pending => {}
            other => panic!("expected pending, got {:?}", other),
        }
        match reader.push(0) {
            PushOutcome::Frame(f) => assert_eq!(f, &[9, 0]),
            other => panic!("expected frame, got {:?}", other),
        }
    }

    #[test]
    fn reset_clears_state() {
        let mut reader: FrameReader<8> = FrameReader::new();
        let _ = reader.push(1);
        let _ = reader.push(2);
        reader.reset();
        match reader.push(3) {
            PushOutcome::Pending => {}
            other => panic!("expected pending, got {:?}", other),
        }
        match reader.push(0) {
            PushOutcome::Frame(f) => assert_eq!(f, &[3, 0]),
            other => panic!("expected frame, got {:?}", other),
        }
    }

    #[test]
    fn roundtrip_with_postcard() {
        use nv1_msg::md::ToMD;
        let msg = ToMD {
            enable: true,
            m1: 1.5,
            m2: -2.0,
            m3: 0.25,
            m4: 100.0,
        };
        let mut scratch = [0u8; 64];
        let encoded = postcard::to_slice_cobs(&msg, &mut scratch).unwrap();

        let mut reader: FrameReader<64> = FrameReader::new();
        let mut got = None;
        for &b in encoded.iter() {
            if let PushOutcome::Frame(frame) = reader.push(b) {
                got = Some(postcard::from_bytes_cobs::<ToMD>(frame).unwrap());
            }
        }
        assert_eq!(got, Some(msg));
    }

    #[test]
    fn fragmented_postcard_stream_recovers_message() {
        use nv1_msg::md::ToMD;
        let msg1 = ToMD {
            enable: true,
            m1: 1.0,
            m2: 2.0,
            m3: 3.0,
            m4: 4.0,
        };
        let msg2 = ToMD {
            enable: false,
            m1: -1.0,
            m2: 0.0,
            m3: 0.5,
            m4: -0.5,
        };
        let mut buf = [0u8; 64];
        let e1 = postcard::to_slice_cobs(&msg1, &mut buf).unwrap();
        let e1_owned = e1.to_vec();
        let e2 = postcard::to_slice_cobs(&msg2, &mut buf).unwrap();
        let mut stream = e1_owned.clone();
        stream.extend_from_slice(e2);

        let mut reader: FrameReader<64> = FrameReader::new();
        let mut decoded: Vec<ToMD> = Vec::new();
        for &b in &stream {
            if let PushOutcome::Frame(frame) = reader.push(b) {
                decoded.push(postcard::from_bytes_cobs::<ToMD>(frame).unwrap());
            }
        }
        assert_eq!(decoded, vec![msg1, msg2]);
    }
}
