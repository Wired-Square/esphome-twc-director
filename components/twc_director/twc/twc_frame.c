// twc_frame.c
// Minimal TWC frame helper (SLIP + checksum utilities, C99).
// Provides a small decoder/encoder for raw Gen2 TWC frames and checksum
// validation on top of an underlying byte stream.

#include "twc_frame.h"

// SLIP control bytes used by both decoder and encoder.
#define TWC_SLIP_END      0xC0
#define TWC_SLIP_ESC      0xDB
#define TWC_SLIP_ESC_END  0xDC
#define TWC_SLIP_ESC_ESC  0xDD

// Many real Gen2 TWC installations append an extra "end-type" marker
// byte immediately after the trailing END (0xC0). The most commonly
// observed values are 0xFC, 0xFA, 0xF8 and 0xFE, and other values may
// exist. The decoder in this file does not treat this byte specially:
// it simply ignores any bytes that occur between END delimiters (so
// callers will never see a standalone 0xFC / 0xFA / 0xF8 / 0xFE frame).
// This constant only controls what we *emit* when
// encoding a frame; receivers should ignore the extra byte.
#define TWC_SLIP_END_DEFAULT_TYPE 0xFC

void twc_frame_decoder_init(twc_frame_decoder_t *dec) {
  if (dec == NULL) {
    return;
  }
  dec->len = 0;
  dec->in_frame = false;
  dec->esc_pending = false;
  dec->frames_decoded = 0;
  dec->frames_dropped_overflow = 0;
  dec->frames_dropped_invalid_esc = 0;
  // Buffer contents are undefined until bytes are received.
}

// Decoder behaviour note:
//
// The decoder only emits frames that are delimited by END (0xC0) bytes
// and ignores any raw bytes that occur between frames when `in_frame`
// is false. In particular, real TWCs (and this library's encoder) may
// place an extra "end-type" marker byte (commonly 0xFC, 0xFA, 0xF8 or
// 0xFE) immediately after the trailing END. Those bytes are considered
// framing noise and are never surfaced to the caller as standalone
// frames.

bool twc_frame_decoder_push(twc_frame_decoder_t *dec,
                            uint8_t byte,
                            const uint8_t **frame_out,
                            size_t *len_out) {
  if (frame_out != NULL) {
    *frame_out = NULL;
  }
  if (len_out != NULL) {
    *len_out = 0;
  }

  if (dec == NULL) {
    return false;
  }

  // END marks frame boundaries.
  if (byte == TWC_SLIP_END) {
    if (dec->in_frame && dec->len > 0) {
      // End of current frame: hand it to caller.
      // Keep in_frame = true so the next C0 doesn't toggle state.
      if (frame_out != NULL) {
        *frame_out = dec->buf;
      }
      if (len_out != NULL) {
        *len_out = dec->len;
      }
      dec->frames_decoded++;
      dec->len = 0;
      // Stay in frame mode - next C0 will start capturing the next frame
      dec->esc_pending = false;
      return true;
    } else {
      // Start of a new frame (or empty frame between consecutive C0s).
      // Either way, we're now ready to capture the next frame.
      dec->in_frame = true;
      dec->esc_pending = false;
      dec->len = 0;
    }
    return false;
  }

  // Ignore any bytes until we've seen a starting END.
  if (!dec->in_frame) {
    return false;
  }

  // Handle ESC sequences.
  if (dec->esc_pending) {
    uint8_t decoded;

    if (byte == TWC_SLIP_ESC_END) {
      decoded = TWC_SLIP_END;
    } else if (byte == TWC_SLIP_ESC_ESC) {
      decoded = TWC_SLIP_ESC;
    } else {
      // Invalid escape sequence; drop the frame to recover cleanly.
      dec->frames_dropped_invalid_esc++;
      dec->len = 0;
      dec->in_frame = false;
      dec->esc_pending = false;
      return false;
    }

    dec->esc_pending = false;

    if (dec->len < TWC_FRAME_MAX_LEN) {
      dec->buf[dec->len++] = decoded;
    } else {
      // Frame too long, drop it.
      dec->frames_dropped_overflow++;
      dec->len = 0;
      dec->in_frame = false;
      dec->esc_pending = false;
    }
    return false;
  }

  if (byte == TWC_SLIP_ESC) {
    dec->esc_pending = true;
    return false;
  }

  // Normal in-frame byte.
  if (dec->len < TWC_FRAME_MAX_LEN) {
    dec->buf[dec->len++] = byte;
  } else {
    // Frame too long, drop it.
    dec->frames_dropped_overflow++;
    dec->len = 0;
    dec->in_frame = false;
    dec->esc_pending = false;
  }

  return false;
}

uint8_t twc_compute_checksum(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0) {
    return 0u;
  }

  uint32_t sum = 0u;
  for (size_t i = 0; i < len; ++i) {
    sum += (uint32_t)data[i];
  }
  return (uint8_t)(sum & 0xFFu);
}

bool twc_frame_checksum_valid(const uint8_t *frame, size_t len) {
  if (frame == NULL || len < 3u) {
    // Need at least: type, at least one data byte, and checksum.
    return false;
  }

  // The checksum is defined as the sum (mod 256) of all bytes from
  // the command byte onwards, i.e. frame[1..len-2]. The type at
  // frame[0] is not included in the checksum.
  size_t data_len = len - 2u;          // bytes 1..(len-2)
  const uint8_t *data = frame + 1;     // skip type at index 0

  uint8_t expected = frame[len - 1u];  // last byte
  uint8_t computed = twc_compute_checksum(data, data_len);

  return computed == expected;
}

twc_broadcast_kind_t twc_frame_classify_broadcast(const uint8_t *frame,
                                                  size_t len,
                                                  uint8_t *code_hi,
                                                  uint8_t *code_lo) {
  if (frame == NULL || len != 2u) {
    return TWC_BCAST_NONE;
  }

  uint8_t hi = frame[0];
  uint8_t lo = frame[1];

  if (code_hi) *code_hi = hi;
  if (code_lo) *code_lo = lo;

  if (hi == 0x7C && lo == 0x00) {
    return TWC_BCAST_MSG_TOO_FAST;
  }

  return TWC_BCAST_UNKNOWN;
}

bool twc_frame_encode_slip(const uint8_t *frame,
                           size_t len,
                           uint8_t *out,
                           size_t out_max,
                           size_t *encoded_len) {
  if (encoded_len != NULL) {
    *encoded_len = 0u;
  }

  if (frame == NULL || len == 0u || out == NULL || out_max == 0u) {
    return false;
  }

  size_t pos = 0u;

  // Leading END marks the start of the frame on the wire.
  if (pos >= out_max) {
    return false;
  }
  out[pos++] = TWC_SLIP_END;

  for (size_t i = 0; i < len; ++i) {
    uint8_t b = frame[i];
    if (b == TWC_SLIP_END) {
      if (pos + 2u > out_max) {
        return false;
      }
      out[pos++] = TWC_SLIP_ESC;
      out[pos++] = TWC_SLIP_ESC_END;
    } else if (b == TWC_SLIP_ESC) {
      if (pos + 2u > out_max) {
        return false;
      }
      out[pos++] = TWC_SLIP_ESC;
      out[pos++] = TWC_SLIP_ESC_ESC;
    } else {
      if (pos >= out_max) {
        return false;
      }
      out[pos++] = b;
    }
  }

  // Trailing END marks the end of the frame on the wire. Many real
  // TWCs then append an extra marker byte after the END. The most
  // commonly observed values are 0xFC, 0xFA, 0xF8 and 0xFE, and other
  // values may exist. We always emit 0xFC here as a
  // reasonable default; decoders are expected to ignore this byte
  // and treat it as framing, not part of the TWC payload.
  if (pos >= out_max) {
    return false;
  }
  out[pos++] = TWC_SLIP_END;

  if (pos >= out_max) {
    return false;
  }
  out[pos++] = TWC_SLIP_END_DEFAULT_TYPE;

  if (encoded_len != NULL) {
    *encoded_len = pos;
  }

  return true;
}