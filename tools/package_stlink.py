"""Combine the bootloader and diagnostic application into a ST-Link ELF."""
from pathlib import Path
from struct import unpack_from
from subprocess import run
import sys

out = Path(sys.argv[1])
cross = sys.argv[2] if len(sys.argv) > 2 else 'arm-none-eabi-'
boot = (out / 'bootloader.bin').read_bytes()
app = (out / 'application.bin').read_bytes()
for image, start, limit in [(boot, 0x08000000, 0x2C00), (app, 0x08003000, 0xD000)]:
    sp, reset = unpack_from('<II', image)
    if len(image) > limit or sp != 0x20005000 or not reset & 1 or not start <= reset < start + len(image):
        raise ValueError(f'Invalid image for slot at {start:#010x}')
(out / 'provision.bin').write_bytes(boot.ljust(0x3000, b'\xff') + app)
(out / 'provision.ld').write_text(
    'OUTPUT_ARCH(arm)\nENTRY(provision_reset)\n'
    f'provision_reset = {unpack_from("<I", boot, 4)[0]:#x};\n'
    'SECTIONS { .flash 0x08000000 : { *(.data) } }\n'
)
run([cross+'objcopy', '-I', 'binary', '-O', 'elf32-littlearm', '-B', 'arm',
     str(out/'provision.bin'), str(out/'provision.o')], check=True)
run([cross+'ld', '-T', str(out/'provision.ld'), str(out/'provision.o'),
     '-o', str(out/'MagStrain_v149_EEPROM_FIX_STLink.elf')], check=True)
