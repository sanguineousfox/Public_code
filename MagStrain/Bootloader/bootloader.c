/*
 * MagStrain Modbus-RTU recovery bootloader, STM32F103C8 (64 KiB Flash).
 *
 * Memory map:
 *   08000000..08002FFF bootloader (12 KiB)
 *   08003000..0800F7FF application (50 KiB)
 *   0800F800..0800FBFF application configuration (1 KiB)
 *   0800FC00..0800FFFF validity manifest (1 KiB)
 *
 * Transport: USART1 PA9/PA10, RS485 DE=PB0, 9600 8N1, slave=1.
 * Register numbers are ZERO-BASED Modbus PDU addresses.
 * Never write outside [APP_BASE, CONFIG_BASE).
 */
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx.h"

#define FLASH_BASE_ADDR 0x08000000u
#define APP_BASE        0x08003000u
#define CONFIG_BASE     0x0800F800u
#define META_BASE       0x0800FC00u
#define APP_CAPACITY    (CONFIG_BASE - APP_BASE)
#define PAGE_SIZE       1024u
#define MANIFEST_MAGIC  0x4D535446u /* "MSTF" */
#define BOOT_REQUEST    0xB007u
#define DEVICE_ADDR     1u
#define BAUD_RATE       9600u
#define PROTOCOL_VER    1u
#define BLOCK_BYTES     64u
#define FRAME_MAX       160u

#define R_STATUS        6200u
#define R_BEGIN         6220u
#define R_DATA          6240u

enum { ST_IDLE=0, ST_WRITING=1, ST_READY=2, ST_ERROR=3 };
enum { ERR_NONE=0, ERR_ARGUMENT=1, ERR_FLASH=2, ERR_SEQUENCE=3,
       ERR_CHECKSUM=4, ERR_VECTOR=5, ERR_INCOMPLETE=6, ERR_STATE=7 };

typedef struct {
    uint32_t magic;
    uint32_t length;
    uint32_t crc32;
    uint32_t crc32_inverse;
} manifest_t;

static uint8_t rx[FRAME_MAX], tx[48];
static uint16_t rx_count;
static uint32_t ms_ticks, last_rx_ms;
static uint8_t state, last_error;
static uint32_t image_length, image_crc, bytes_written;
static uint16_t next_sequence;
static bool reboot_requested;

static uint16_t be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static uint16_t crc16(const uint8_t *p, uint16_t count) {
    uint16_t crc=0xFFFFu;
    while (count--) {
        crc ^= *p++;
        for (unsigned i=0; i<8; ++i)
            crc=(uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0xA001u : 0u));
    }
    return crc;
}
static uint32_t crc32_flash(uint32_t start, uint32_t n) {
    uint32_t crc=0xFFFFFFFFu;
    const uint8_t *p=(const uint8_t *)start;
    for (uint32_t i=0; i<n; ++i) {
        crc ^= p[i];
        for (unsigned b=0; b<8; ++b)
            crc=(crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}
static bool vectors_valid(uint32_t length) {
    uint32_t sp=*(const uint32_t *)APP_BASE;
    uint32_t entry=*(const uint32_t *)(APP_BASE+4u);
    uint32_t pc=entry & ~1u;
    return length >= 8u &&
           sp > 0x20000000u && sp <= 0x20005000u && !(sp & 7u) &&
           (entry & 1u) && pc >= APP_BASE && pc < APP_BASE+length;
}
static bool manifest_valid(void) {
    const manifest_t *m=(const manifest_t *)META_BASE;
    if (m->magic != MANIFEST_MAGIC || m->length < 8u ||
        m->length > APP_CAPACITY || m->crc32_inverse != ~m->crc32 ||
        !vectors_valid(m->length)) return false;
    return crc32_flash(APP_BASE, m->length)==m->crc32;
}
static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR=FLASH_KEY1;
        FLASH->KEYR=FLASH_KEY2;
    }
}
static bool flash_wait(void) {
    uint32_t timeout=20000000u;
    while ((FLASH->SR & FLASH_SR_BSY) && --timeout) { }
    return timeout && !(FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR));
}
static bool flash_erase(uint32_t addr) {
    if ((addr & (PAGE_SIZE-1u)) ||
        (addr < APP_BASE && addr != META_BASE) ||
        (addr >= CONFIG_BASE && addr != META_BASE)) return false;
    if (!flash_wait()) return false;
    FLASH->SR=FLASH_SR_EOP|FLASH_SR_PGERR|FLASH_SR_WRPRTERR;
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR=addr;
    FLASH->CR |= FLASH_CR_STRT;
    bool ok=flash_wait();
    FLASH->CR &= ~FLASH_CR_PER;
    return ok && (*(volatile uint16_t *)addr == 0xFFFFu);
}
static bool flash_halfword(uint32_t addr, uint16_t value) {
    bool in_app=(addr >= APP_BASE && addr+2u <= CONFIG_BASE);
    bool in_meta=(addr >= META_BASE && addr+2u <= META_BASE+16u);
    if ((addr & 1u) || (!in_app && !in_meta) || !flash_wait()) return false;
    FLASH->SR=FLASH_SR_EOP|FLASH_SR_PGERR|FLASH_SR_WRPRTERR;
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t *)addr=value;
    bool ok=flash_wait();
    FLASH->CR &= ~FLASH_CR_PG;
    return ok && (*(volatile uint16_t *)addr == value);
}
static bool flash_word(uint32_t addr, uint32_t value) {
    return flash_halfword(addr,(uint16_t)value) &&
           flash_halfword(addr+2u,(uint16_t)(value>>16));
}
static void set_error(uint8_t code) {
    last_error=code;
    state=ST_ERROR;
}
static void uart_send(const uint8_t *data, uint16_t count) {
    GPIOB->BSRR=1u; /* DE=1 */
    for (volatile unsigned wait=0; wait<100; ++wait) __NOP();
    for (uint16_t i=0; i<count; ++i) {
        while (!(USART1->SR & USART_SR_TXE)) { }
        USART1->DR=data[i];
    }
    while (!(USART1->SR & USART_SR_TC)) { }
    GPIOB->BRR=1u; /* DE=0, receive */
    (void)USART1->SR; (void)USART1->DR; /* discard local echo if any */
}
static void send_pdu(uint16_t n) {
    uint16_t crc=crc16(tx,n);
    tx[n++]=(uint8_t)crc;
    tx[n++]=(uint8_t)(crc>>8);
    uart_send(tx,n);
}
static void exception(uint8_t function, uint8_t code) {
    tx[0]=DEVICE_ADDR; tx[1]=(uint8_t)(function|0x80u); tx[2]=code;
    send_pdu(3);
}
static uint16_t status_reg(uint16_t addr) {
    switch (addr) {
    case 6200: return BOOT_REQUEST;
    case 6201: return PROTOCOL_VER;
    case 6202: return state;
    case 6203: return next_sequence;
    case 6204: return (uint16_t)image_length;
    case 6205: return (uint16_t)(image_length>>16);
    case 6206: return (uint16_t)image_crc;
    case 6207: return (uint16_t)(image_crc>>16);
    case 6208: return last_error;
    case 6209: return (uint16_t)bytes_written;
    case 6210: return (uint16_t)(bytes_written>>16);
    case 6211: return BLOCK_BYTES;
    default: return 0xFFFFu;
    }
}
static bool begin_update(const uint8_t *words, uint16_t count) {
    if (count != 6u || be16(words)!=0xC0DEu ||
        be16(words+2u)!=PROTOCOL_VER) { last_error=ERR_ARGUMENT; return false; }
    uint32_t length=((uint32_t)be16(words+4u)<<16)|be16(words+6u);
    uint32_t crc=((uint32_t)be16(words+8u)<<16)|be16(words+10u);
    if (length < 8u || length > APP_CAPACITY) {
        last_error=ERR_ARGUMENT; return false;
    }
    flash_unlock();
    /* Invalidate old image BEFORE any application page is modified. */
    if (!flash_erase(META_BASE)) { set_error(ERR_FLASH); return false; }
    for (uint32_t addr=APP_BASE; addr<CONFIG_BASE; addr+=PAGE_SIZE) {
        if (!flash_erase(addr)) { set_error(ERR_FLASH); return false; }
    }
    FLASH->CR |= FLASH_CR_LOCK;
    image_length=length; image_crc=crc;
    bytes_written=0; next_sequence=0; last_error=ERR_NONE;
    state=ST_WRITING;
    return true;
}
static bool write_block(const uint8_t *words, uint16_t count) {
    if (state != ST_WRITING || count < 2u || count > 33u) {
        last_error=ERR_STATE; return false;
    }
    uint16_t seq=be16(words);
    uint32_t offset=(uint32_t)seq*BLOCK_BYTES;
    if (offset>=image_length || offset>=APP_CAPACITY) {
        last_error=ERR_SEQUENCE; return false;
    }
    uint32_t actual=image_length-offset;
    if (actual>BLOCK_BYTES) actual=BLOCK_BYTES;
    uint16_t encoded=(uint16_t)((actual+1u)&~1u);
    if ((uint32_t)(count-1u)*2u != encoded) {
        last_error=ERR_ARGUMENT; return false;
    }
    if ((actual & 1u) && words[2u+actual] != 0xFFu) {
        last_error=ERR_ARGUMENT; return false;
    }
    /* A repeated packet after an ACK loss is idempotent. */
    if (seq != next_sequence) {
        if (next_sequence && seq==(uint16_t)(next_sequence-1u) &&
            offset+actual<=bytes_written) {
            for (uint32_t i=0;i<actual;++i)
                if (*(const uint8_t *)(APP_BASE+offset+i)!=words[2u+i]) {
                    last_error=ERR_SEQUENCE; return false;
                }
            last_error=ERR_NONE;
            return true;
        }
        last_error=ERR_SEQUENCE; return false;
    }
    if (offset!=bytes_written) { last_error=ERR_SEQUENCE; return false; }
    flash_unlock();
    for (uint16_t i=0; i<encoded; i+=2u) {
        uint16_t hw=(uint16_t)words[2u+i] |
                   (uint16_t)((uint16_t)words[2u+i+1u]<<8);
        if (!flash_halfword(APP_BASE+offset+i,hw)) {
            FLASH->CR |= FLASH_CR_LOCK;
            set_error(ERR_FLASH);
            return false;
        }
    }
    FLASH->CR |= FLASH_CR_LOCK;
    bytes_written+=actual;
    ++next_sequence;
    last_error=ERR_NONE;
    return true;
}
static bool commit_update(void) {
    if (state!=ST_WRITING || bytes_written!=image_length) {
        last_error=ERR_INCOMPLETE; return false;
    }
    if (!vectors_valid(image_length)) { set_error(ERR_VECTOR); return false; }
    if (crc32_flash(APP_BASE,image_length)!=image_crc) {
        set_error(ERR_CHECKSUM); return false;
    }
    flash_unlock();
    /* Write magic last: incomplete manifest remains invalid on power loss. */
    bool ok=flash_word(META_BASE+4u,image_length) &&
            flash_word(META_BASE+8u,image_crc) &&
            flash_word(META_BASE+12u,~image_crc) &&
            flash_word(META_BASE,MANIFEST_MAGIC);
    FLASH->CR |= FLASH_CR_LOCK;
    if (!ok || !manifest_valid()) { set_error(ERR_FLASH); return false; }
    last_error=ERR_NONE;
    state=ST_READY;
    return true;
}
static void process_frame(const uint8_t *f, uint16_t n) {
    if (n<8u || f[0]!=DEVICE_ADDR) return; /* no broadcast updates */
    uint16_t crc=crc16(f,(uint16_t)(n-2u));
    if (f[n-2u]!=(uint8_t)crc || f[n-1u]!=(uint8_t)(crc>>8)) return;
    uint8_t fn=f[1];
    uint16_t addr=be16(f+2u), count=be16(f+4u);
    if (fn==0x03u || fn==0x04u) {
        if (n!=8u || addr<R_STATUS || addr>6211u ||
            count==0u || count>12u || (uint32_t)addr+count>6212u) {
            exception(fn,0x02u); return;
        }
        tx[0]=DEVICE_ADDR; tx[1]=fn; tx[2]=(uint8_t)(count*2u);
        for (uint16_t i=0;i<count;++i) {
            uint16_t v=status_reg(addr+i);
            tx[3u+2u*i]=(uint8_t)(v>>8);
            tx[4u+2u*i]=(uint8_t)v;
        }
        send_pdu((uint16_t)(3u+count*2u));
    } else if (fn==0x06u) {
        if (n!=8u || addr!=R_STATUS) { exception(fn,0x02u); return; }
        uint16_t command=count;
        bool ok=false;
        if (command==0xA5A5u) ok=commit_update();
        else if (command==0x5A5Au && state==ST_READY && manifest_valid()) {
            reboot_requested=true; ok=true;
        } else if (command==0xDEADu) {
            flash_unlock();
            ok=flash_erase(META_BASE);
            FLASH->CR|=FLASH_CR_LOCK;
            if (ok) {
                state=ST_IDLE; bytes_written=0; next_sequence=0;
                image_length=0; image_crc=0; last_error=ERR_NONE;
            } else set_error(ERR_FLASH);
        }
        if (!ok) { exception(fn,0x03u); return; }
        for (unsigned i=0; i<6; ++i) tx[i]=f[i];
        send_pdu(6);
    } else if (fn==0x10u) {
        if (n<11u || count==0u || count>67u ||
            f[6]!=(uint8_t)(count*2u) || n!=(uint16_t)(9u+f[6])) {
            exception(fn,0x03u); return;
        }
        bool ok=false;
        if (addr==R_BEGIN) ok=begin_update(f+7u,count);
        else if (addr==R_DATA) ok=write_block(f+7u,count);
        else { exception(fn,0x02u); return; }
        if (!ok) { exception(fn,0x03u); return; }
        tx[0]=DEVICE_ADDR; tx[1]=fn;
        for (unsigned i=2; i<6; ++i) tx[i]=f[i];
        send_pdu(6);
    } else {
        exception(fn,0x01u);
    }
}
static void init_hardware(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    /* PA9 AF push-pull 50MHz, PA10 floating input. */
    GPIOA->CRH=(GPIOA->CRH & ~(0xFFu<<4u)) | (0xBu<<4u) | (0x4u<<8u);
    /* PB0 output push-pull, 2MHz; RS485 receive by default. */
    GPIOB->CRL=(GPIOB->CRL & ~0xFu) | 0x2u;
    GPIOB->BRR=1u;
    USART1->BRR=(8000000u+BAUD_RATE/2u)/BAUD_RATE;
    USART1->CR1=USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    SysTick->LOAD=8000u-1u;
    SysTick->VAL=0;
    SysTick->CTRL=SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_ENABLE_Msk;
}
static void jump_to_app(void) {
    uint32_t sp=*(const uint32_t *)APP_BASE;
    uint32_t pc=*(const uint32_t *)(APP_BASE+4u);
    __disable_irq();
    USART1->CR1=0;
    SysTick->CTRL=0;
    GPIOB->BRR=1u;
    SCB->VTOR=APP_BASE;
    __asm volatile ("msr msp, %0\n bx %1" : : "r"(sp), "r"(pc) : "memory");
    __builtin_unreachable();
}
void boot_main(void) {
    __disable_irq();
    /* Reset default HSI 8 MHz clock is used; no HAL or interrupts. */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    bool forced=(BKP->DR1==BOOT_REQUEST);
    BKP->DR1=0u;
    if (!forced && manifest_valid()) jump_to_app();
    init_hardware();
    if (manifest_valid()) {
        const manifest_t *m=(const manifest_t *)META_BASE;
        image_length=m->length; image_crc=m->crc32;
        bytes_written=image_length;
        state=ST_READY;
    } else state=ST_IDLE;

    for (;;) {
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) ++ms_ticks;
        uint32_t sr=USART1->SR;
        if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
            (void)USART1->DR;
            rx_count=0;
        } else if (sr & USART_SR_RXNE) {
            uint8_t b=(uint8_t)USART1->DR;
            if (rx_count && ms_ticks-last_rx_ms>5u) rx_count=0;
            last_rx_ms=ms_ticks;
            if (rx_count<FRAME_MAX) rx[rx_count++]=b;
            else rx_count=0;
        }
        /* RTU silent gap: 5 ms at 9600 8N1 (>=3.5 chars). */
        if (rx_count && ms_ticks-last_rx_ms>=5u) {
            uint16_t n=rx_count;
            rx_count=0;
            process_frame(rx,n);
        }
        if (reboot_requested) {
            for (volatile uint32_t i=0;i<20000u;++i) __NOP();
            NVIC_SystemReset();
        }
    }
}
