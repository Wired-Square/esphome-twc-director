// twc_frame.h
// Minimal TWC frame helper (SLIP + checksum utilities, C99).
// Provides a small decoder/encoder for raw Gen2 TWC frames and checksum
// validation on top of an underlying byte stream.

#pragma once

#include <stddef.h>   // size_t
#include <stdint.h>   // uint8_t
#include <stdbool.h>  // bool

#define TWC_FRAME_MAX_LEN 256

// Simple SLIP frame decoder state.
typedef struct {
  uint8_t buf[TWC_FRAME_MAX_LEN];
  size_t len;
  bool in_frame;
  bool esc_pending;

  // Diagnostic counters
  uint32_t frames_decoded;
  uint32_t frames_dropped_overflow;
  uint32_t frames_dropped_invalid_esc;
} twc_frame_decoder_t;

// twc_frame.h
typedef enum {
  TWC_BCAST_NONE = 0,        // not a broadcast (len != 2)
  TWC_BCAST_UNKNOWN = 1,     // 2-byte payload but we don't know it
  TWC_BCAST_MSG_TOO_FAST = 2,   // 0x7C 0x00 (suspected overrun / too many requests)
  // future: TWC_BCAST_XXX, TWC_BCAST_YYY ...
} twc_broadcast_kind_t;

// Initialise a decoder state.
void twc_frame_decoder_init(twc_frame_decoder_t *dec);

// Feed a single SLIP-encoded byte into the decoder.
//
// If a complete frame has just been decoded, this function returns true,
// and *frame_out / *len_out point to the frame bytes and length stored
// inside the decoder. The pointer remains valid until the next call to
// twc_frame_decoder_init or twc_frame_decoder_push.
//
// On partial frames or non-frame bytes, returns false.
bool twc_frame_decoder_push(twc_frame_decoder_t *dec,
                            uint8_t byte,
                            const uint8_t **frame_out,
                            size_t *len_out);

// Compute checksum over `data[0..len-1]` as sum(data) & 0xFF.
// Returns 0 if data==NULL or len==0.
uint8_t twc_compute_checksum(const uint8_t *data, size_t len);

// Validate a full frame where the last byte is the checksum.
//
// Expects:
//   frame[0]          : frame type (not included in checksum)
//   frame[1 .. len-2] : command, IDs, payload...
//   frame[len-1]      : checksum byte
//
// Returns true if the computed checksum over frame[1..len-2]
// matches frame[len-1], false otherwise (or on invalid input).
bool twc_frame_checksum_valid(const uint8_t *frame, size_t len);

// Classify a decoded frame as a broadcast, if applicable.
//
// If len != 2, returns TWC_BCAST_NONE.
// If len == 2 but not a known code, returns TWC_BCAST_UNKNOWN and
// sets *code_hi/*code_lo if provided.
twc_broadcast_kind_t twc_frame_classify_broadcast(const uint8_t *frame,
                                                  size_t len,
                                                  uint8_t *code_hi,
                                                  uint8_t *code_lo);

// SLIP-encode a raw TWC frame into an output buffer.
//
// The input `frame[0..len-1]` is the unescaped TWC frame (type, command,
// payload, checksum). On success, `out` will contain a SLIP-framed sequence
// starting with END (0xC0), followed by the escaped payload, then another
// END (0xC0) and a final "end-type" marker byte (by default 0xFC). Real
// installations have been observed to use other values (e.g. 0xFA, 0xF8,
// 0xFE),
// so decoders should treat the byte immediately following the trailing
// END as framing, not part of the TWC payload or checksum. The worst-case
// encoded size is 3 + 2*len bytes, so callers should ensure `out_max` is
// large enough.
bool twc_frame_encode_slip(const uint8_t *frame,
                           size_t len,
                           uint8_t *out,
                           size_t out_max,
                           size_t *encoded_len);
