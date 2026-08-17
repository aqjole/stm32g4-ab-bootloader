#ifndef G4B_PROTO_H
#define G4B_PROTO_H

#include <stdint.h>

/*
 * Frame:  [SOF][len lo][len hi][type][payload...][crc32 le]
 *
 * len counts payload bytes only. The CRC covers len + type + payload. 
 */

#define G4B_SOF             0x7Eu
#define G4B_MAX_PAYLOAD     256u
#define G4B_CHUNK_DATA      248u    /* CHUNK payload: [seq u16 le][data]. Data must be a multiple of 8 for doubleword programming */
#define G4B_FRAME_OVERHEAD  8u      /* SOF 1 + len 2 + type 1 + crc 4 */

/* host -> device */
#define G4B_MSG_HELLO       0x01u
#define G4B_MSG_BEGIN       0x02u
#define G4B_MSG_CHUNK       0x03u
#define G4B_MSG_END         0x04u
#define G4B_MSG_BOOT        0x05u

/* device -> host */
#define G4B_MSG_ACK         0x80u
#define G4B_MSG_NACK        0x81u

/* NACK payload byte */
#define G4B_NACK_BAD_CRC    0x01u
#define G4B_NACK_BAD_LEN    0x02u
#define G4B_NACK_BAD_TYPE   0x03u
#define G4B_NACK_BAD_SEQ    0x04u
#define G4B_NACK_FLASH      0x05u
#define G4B_NACK_NOT_READY  0x06u
#define G4B_NACK_WRONG_SLOT 0x07u

#endif /* G4B_PROTO_H */