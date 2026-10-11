/* adb transport framing for the bridge. Every adb message is a 24-byte
 * little-endian header (command, arg0, arg1, data_length, data_check,
 * magic = ~command) followed by data_length payload bytes, each sent as its
 * own USB transfer. */
#ifndef ADB_PROTO_H
#define ADB_PROTO_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define ADB_HDR_LEN 24
/* Sanity cap on data_length: adb's MAX_PAYLOAD is 1 MiB. */
#define ADB_MAX_PAYLOAD (1024 * 1024)

static inline uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] |
		((uint32_t)p[1] << 8) |
		((uint32_t)p[2] << 16) |
		((uint32_t)p[3] << 24);
}

static inline void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff;
	p[3] = (v >> 24) & 0xff;
}

/* Command words: the four ASCII bytes, little-endian. */
#define A_CNXN 0x4e584e43u
#define A_OKAY 0x59414b4fu
#define A_WRTE 0x45545257u

/* Parse a 24-byte header. False if the magic does not match the command or
 * the payload length is implausible (the stream is not adb, or not framed
 * where the caller thinks it is). */
static inline bool adb_parse_hdr(const uint8_t *p, uint32_t *cmd, uint32_t *arg0,
				 uint32_t *arg1, uint32_t *len)
{
	*cmd = le32(p);
	*arg0 = le32(p + 4);
	*arg1 = le32(p + 8);
	*len = le32(p + 12);
	uint32_t magic = le32(p + 20);
	return magic == (*cmd ^ 0xffffffffU) && *len <= ADB_MAX_PAYLOAD;
}

/* The header's data_check field: adb's payload checksum, the 32-bit sum of
 * the payload bytes (checked by adbd on every packet until the connection
 * negotiates a version that skips it; a CNXN always carries it). */
static inline uint32_t adb_hdr_data_check(const uint8_t *p)
{
	return le32(p + 16);
}

static inline uint32_t adb_data_check(const uint8_t *p, size_t n)
{
	uint32_t sum = 0;
	for (size_t i = 0; i < n; i++)
		sum += p[i];
	return sum;
}

#endif /* ADB_PROTO_H */
