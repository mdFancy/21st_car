#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_


extern uint8 save_index[6];









void flash_Nag_Write(void);

void flash_index_write(void);
void flash_index_read(void);

#endif
