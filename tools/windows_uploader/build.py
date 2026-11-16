"""Build the Windows x64 Modbus update utility with GCC and GNU binutils."""
import argparse
import pathlib
import re
import shutil
import subprocess
import tempfile

src = pathlib.Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("image", type=pathlib.Path)
parser.add_argument("output", type=pathlib.Path)
parser.add_argument("--install", action="store_true")
parser.add_argument("--port", type=int, default=3)
parser.add_argument("--major", type=int, default=149)
parser.add_argument("--minor", type=int, default=1)
args = parser.parse_args()
if not 1 <= args.port <= 256 or not 0 <= args.minor <= 9:
    parser.error("port must be 1–256 and minor version must be 0–9")
if args.major < 1 or 10 * args.major + args.minor > 65535:
    parser.error("firmware version must fit in the 16-bit update header")
image, output = args.image.resolve(), args.output.resolve()
defines = [f"-DFW_WINDOWS_PORT={args.port}", f"-DFW_WINDOWS_MAJOR={args.major}",
           f"-DFW_WINDOWS_MINOR={args.minor}"]
if args.install:
    defines.append("-DFW_WINDOWS_INSTALL")
imports = re.findall(r'^extern .*\(\*(\w+)\)\(', (src / 'windows.c').read_text(), re.M)

def bytes_directive(text):
    return ' '.join(f'BYTE({value})' for value in text.encode('ascii') + b'\0')

def thunks(declare_iat):
    return '\n'.join(
        (f'{name} = .; ' if declare_iat else '') + f'QUAD(hint_{name} - base)'
        for name in imports
    ) + '\nQUAD(0)'

with tempfile.TemporaryDirectory() as directory:
    work = pathlib.Path(directory)
    def run(*args):
        subprocess.run(args, cwd=work, check=True)

    shutil.copyfile(image, work / 'application_v149_1.bin')
    run('ld', '-r', '-b', 'binary', 'application_v149_1.bin', '-o', 'image.o')
    run('objcopy', '--rename-section', '.data=.rodata.image,alloc,load,readonly,data,contents',
        'image.o')
    for name in ('stage', 'windows'):
        run('gcc', '-c', '-Os', '-Wall', '-Wextra', '-Werror', '-ffreestanding', *defines,
            '-mabi=ms', '-mno-red-zone', '-fpie', '-fno-asynchronous-unwind-tables',
            '-fno-stack-protector', '-fcf-protection=none', str(src / f'{name}.c'),
            '-o', f'{name}.o')

    hints = '\n'.join(f'. = ALIGN(2); hint_{name} = .; SHORT(0) {bytes_directive(name)}'
                      for name in imports)
    (work / 'windows.ld').write_text(f'''
ENTRY(entry)
base = 0x140000000;
SECTIONS {{
    . = base + 0x1000;
    .text : {{ *(.text*) }}
    . = ALIGN(0x1000);
    .rdata : {{ *(.rodata*) }}
    . = ALIGN(0x1000);
    .bss : {{ *(.bss*) }}
    . = ALIGN(0x1000);
    .idata : {{
        LONG(ilt - base) LONG(0) LONG(0) LONG(dll - base) LONG(iat - base)
        LONG(0) LONG(0) LONG(0) LONG(0) LONG(0)
        . = ALIGN(8);
        ilt = .;
        {thunks(False)}
        iat = .;
        {thunks(True)}
        {hints}
        dll = .; {bytes_directive('KERNEL32.dll')}
    }}
    /DISCARD/ : {{ *(.comment) *(.note*) *(.eh_frame*) }}
}}
''')
    run('ld', '-mi386pep', '--image-base', '0x140000000', '--subsystem', 'console',
        '-T', 'windows.ld', 'stage.o', 'windows.o', 'image.o', '-o', str(output))
