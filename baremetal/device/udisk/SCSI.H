#ifndef _SCSI_H
#define _SCSI_H

#define INQUIRY                0x12
#define READ_FORMAT_CAPACITIES 0x23
#define READ_CAPACITY          0x25
#define READ_10                0x28
#define WRITE_10               0x2A
#define REQUEST_SENSE          0x03
#define TEST_UNIT_READY        0x00
#define ModeSense_6            0x1A
#define PREV_ALLOW_REMOVE      0x1E

void ProcScsiCommand(void);
void ProcScsiOutData(void);
void Ep1SendData(void);

#endif
