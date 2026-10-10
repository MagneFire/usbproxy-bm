/* CRC-32 (IEEE 802.3, reflected 0xEDB88320): what mkimage puts into the
 * legacy header's ih_dcrc. Nibble table, 64 bytes of rodata. */
#include <stdint.h>
#include <stddef.h>

static const uint32_t tab[16] = {
	0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
};

uint32_t crc32_buf(uint32_t crc, const void *buf, size_t n)
{
	const uint8_t *p = buf;
	crc = ~crc;
	while (n--) {
		crc ^= *p++;
		crc = tab[crc & 15] ^ (crc >> 4);
		crc = tab[crc & 15] ^ (crc >> 4);
	}
	return ~crc;
}
