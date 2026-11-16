#include "fw_update.h"
#include "AT24C64.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t eeprom[65536], image[FW_APP_SIZE];
static int fail_write, fail_read;
static uint32_t capacity = 65536U;
HAL_StatusTypeDef AT24C64_ReadBytes(uint8_t d, uint16_t a, uint8_t *p, uint16_t n)
{ (void)d; assert((uint32_t)a+n<=65536); if(fail_read)return HAL_ERROR; memcpy(p,eeprom+(a % capacity),n);return HAL_OK; }
HAL_StatusTypeDef AT24C64_WriteBytes(uint8_t d, uint16_t a, const uint8_t *p, uint16_t n)
{ (void)d; assert(a>=FW_META_BASE);assert((uint32_t)a+n<=65536);if(fail_write)return HAL_ERROR;memcpy(eeprom+(a % capacity),p,n);return HAL_OK; }
static void word(uint8_t *p, uint16_t v) {p[0]=v>>8;p[1]=v;}
static void dw(uint8_t *p,uint32_t v){word(p,v);word(p+2,v>>16);}
static uint8_t command(uint16_t c){uint8_t p[2];word(p,c);return FW_Write(6200,1,p);}
static uint8_t begin(uint32_t n,uint32_t crc){uint8_t p[14];word(p,1);dw(p+2,n);dw(p+6,crc);word(p+10,137);word(p+12,FW_TARGET);return FW_Write(6200,7,p);}
static uint8_t chunk(uint16_t offset,uint16_t n){uint8_t p[132]={0};word(p,offset);word(p+2,n);memcpy(p+4,image+offset,n);return FW_Write(6210,2+(n+1)/2,p);}
static void reset(void){fail_write=fail_read=0;memset(eeprom,0xA5,sizeof(eeprom));assert(command(4)==0);}
static void upload(uint32_t n){for(uint32_t i=0;i<n;i+=128)assert(chunk(i,n-i>128?128:n-i)==0);}
static void verify(void){assert(command(2)==0);while(FW_Read(6200)==FW_VERIFYING)FW_Process();}
int main(void){
 for(unsigned i=0;i<sizeof(image);++i)image[i]=(uint8_t)i;
 uint32_t vectors[2]={0x20005000,FW_APP_BASE+9};memcpy(image,vectors,8);
 assert((FW_Crc(0xFFFFFFFFUL,(const uint8_t *)"123456789",9)^0xFFFFFFFFUL)==0xCBF43926UL);
 assert(!FW_VectorsValid(0x20005008,FW_APP_BASE+9,256));
 assert(!FW_VectorsValid(0x20005000,FW_APP_BASE+8,256));
 uint32_t n,crc; FW_Header h;
 reset();n=257;crc=FW_Crc(0xFFFFFFFFUL,image,n)^0xFFFFFFFFUL;
 for (capacity=8192U; capacity<65536U; capacity*=2U) {
   assert(begin(n,crc)==4 && FW_Read(6201)==4);
   for (unsigned i=0; i<FW_META_BASE; ++i) assert(eeprom[i]==0xA5);
   assert(eeprom[0xFF80U % capacity]==0xA5);
 }
 reset();
 assert(begin(n,crc)==0);upload(n);verify();assert(FW_Read(6200)==FW_VERIFIED);
 reset();assert(begin(n,crc^1)==0);upload(n);verify();assert(FW_Read(6200)==FW_ERROR);assert(FW_Read(6201)==2);
 reset();assert(begin(n,crc)==0);upload(n);eeprom[FW_IMAGE_BASE]^=8;verify();assert(FW_Read(6200)==FW_ERROR);
 reset();assert(begin(n,crc)==0);fail_write=1;assert(chunk(0,128)==4);assert(FW_Read(6200)==FW_ERROR);
 reset();assert(begin(n,crc)==0);upload(n);assert(command(2)==0);fail_read=1;FW_Process();assert(FW_Read(6200)==FW_ERROR);
 reset();assert(begin(n,crc)==0);upload(n);verify();fail_write=1;assert(command(3)==4);memcpy(&h,eeprom+FW_META_BASE,sizeof(h));assert(!FW_HeaderValid(&h));
 reset();assert(begin(FW_APP_SIZE+1,0)==3);assert(begin(7,0)==3);
 n=FW_APP_SIZE;crc=FW_Crc(0xFFFFFFFFUL,image,n)^0xFFFFFFFFUL;
 assert(begin(n,crc)==0);assert(command(3)==3);assert(command(2)==3);
 assert(chunk(128,128)==3);assert(chunk(0,128)==0);assert(chunk(0,128)==0);
 image[10]^=1;assert(chunk(0,128)==3);image[10]^=1;
 for(uint32_t i=128;i<n;i+=128)assert(chunk(i,128)==0);
 assert(command(2)==0);assert(command(2)==0);while(FW_Read(6200)==FW_VERIFYING)FW_Process();
 assert(FW_Read(6200)==FW_VERIFIED);
 memcpy(&h,eeprom+FW_META_BASE,sizeof(h));assert(!FW_HeaderValid(&h));
 assert(command(3)==0);assert(command(3)==0);
 memcpy(&h,eeprom+FW_META_BASE,sizeof(h));assert(FW_HeaderValid(&h));
 assert(begin(n,crc)==6);assert(command(4)==6);
 for(unsigned i=0;i<FW_META_BASE;++i)assert(eeprom[i]==0xA5);
 puts("PASS: max/odd images, CRC, vectors, duplicates, ordering, IO errors, commit and calibration isolation");
}
