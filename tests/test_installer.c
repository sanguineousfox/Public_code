#define _GNU_SOURCE
#define FW_INSTALL_TEST
#include <assert.h>
#include <stdio.h>
#include <sys/mman.h>
#include "../MagStrain/Bootloader/main.c"
static uint8_t eeprom[65536];
static int step, cut;
void HAL_IncTick(void) {}
void HAL_FLASH_Unlock(void) {}
void HAL_FLASH_Lock(void) {}
HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t d,uint16_t a,uint8_t *p,uint16_t n)
{ (void)d;assert((uint32_t)a+n<=65536);memcpy(p,eeprom+a,n);return HAL_OK; }
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e,uint32_t *error)
{
    assert(e->PageAddress>=INSTALL_FLAG && e->PageAddress<FW_APP_BASE+FW_APP_SIZE);
    assert(e->PageAddress%1024==0);*error=0;
    memset((void *)(uintptr_t)e->PageAddress,0xFF,1024);
    return ++step==cut?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type,uint32_t a,uint64_t value)
{
    (void)type;assert(a==INSTALL_FLAG || (a>=FW_APP_BASE && a<FW_APP_BASE+FW_APP_SIZE));
    *(uint16_t *)(uintptr_t)a &= value;
    return ++step==cut?HAL_ERROR:HAL_OK;
}
int main(void)
{
    void *flash=mmap((void *)0x08000000,65536,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
    assert(flash==(void *)0x08000000);
    uint32_t n=FW_APP_SIZE-1;
    for(unsigned i=0;i<n;++i)eeprom[FW_IMAGE_BASE+i]=(uint8_t)(i*23);
    uint32_t vectors[2]={0x20005000,FW_APP_BASE+9};memcpy(eeprom+FW_IMAGE_BASE,vectors,8);
    FW_Header h={.magic=FW_MAGIC,.size=n,.app_base=FW_APP_BASE,.target=FW_TARGET,.version=137};
    h.crc=FW_Crc(0xFFFFFFFFUL,eeprom+FW_IMAGE_BASE,n)^0xFFFFFFFFUL;
    h.header_crc=FW_Crc(0xFFFFFFFFUL,(uint8_t *)&h.size,20)^0xFFFFFFFFUL;
    assert(FW_HeaderValid(&h));assert(image_valid(&h));
    int cuts[]={1,2,20,53,54,55,127,1024,26000,26677};
    for(unsigned t=0;t<sizeof(cuts)/sizeof(*cuts);++t){
        memset(flash,0xA5,65536);memset((void *)INSTALL_FLAG,0xFF,1024);
        step=0;cut=cuts[t];assert(!install(&h));
        step=0;cut=0;assert(install(&h));
        assert(!memcmp((void *)FW_APP_BASE,eeprom+FW_IMAGE_BASE,n));
        assert(*(uint16_t *)INSTALL_FLAG==0xFFFF);
        for(unsigned i=0;i<0x2C00;++i)assert(((uint8_t *)flash)[i]==0xA5);
    }
    eeprom[FW_IMAGE_BASE+20]^=1;assert(!image_valid(&h));
    munmap(flash,65536);
    puts("PASS: actual installer retries 10 simulated interrupted erase/program stages; boot code preserved");
}
