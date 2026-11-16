#!/usr/bin/env python3
"""Upload application.bin to a device already provisioned with this bootloader.
Requires pyserial. Defaults to the existing PMP-201E link settings (19200 8N1).
"""
import argparse
import pathlib
import struct
import time
import zlib


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


class Device:
    def __init__(self, port, address):
        self.port, self.address = port, address

    def request(self, function, payload):
        packet = bytes([self.address, function]) + payload
        packet += struct.pack('<H', crc16(packet))
        for attempt in range(5):
            time.sleep(0.02)
            self.port.reset_input_buffer()
            self.port.write(packet)
            self.port.flush()
            response = self.port.read(3)
            if len(response) == 3:
                remaining = 2 if response[1] & 0x80 else (response[2] + 2 if function == 3 else 5)
                response += self.port.read(remaining)
                if len(response) >= 5 and crc16(response[:-2]) == int.from_bytes(response[-2:], 'little') and response[0] == self.address:
                    if response[1] == (function | 0x80):
                        if response[2] == 6:
                            time.sleep(0.2)
                            continue
                        raise RuntimeError(f'Modbus exception {response[2]}')
                    if response[1] != function:
                        continue
                    if function in (6, 16) and response[2:6] != payload[:4]:
                        continue
                    return response[2:-2]
            if attempt == 4:
                raise TimeoutError('No valid Modbus response; device may have reset')
        raise TimeoutError('Device remained busy; wait for parameter save and retry')

    def write(self, address, words):
        self.request(16, struct.pack('>HHB', address, len(words), 2 * len(words)) + struct.pack('>' + 'H' * len(words), *words))

    def command(self, command):
        self.request(6, struct.pack('>HH', 6200, command))

    def status(self):
        raw = self.request(3, struct.pack('>HH', 6200, 10))
        if raw[0] != 20 or len(raw) != 21:
            raise RuntimeError('Unexpected update status length')
        return struct.unpack('>10H', raw[1:])


def validate_image(image):
    if not 8 <= len(image) <= 52 * 1024:
        raise ValueError('Image must contain 8..53248 bytes')
    sp, pc = struct.unpack_from('<II', image)
    if not (0x20000000 < sp <= 0x20005000 and sp % 8 == 0 and pc & 1 and 0x08003000 <= (pc & ~1) < 0x08003000 + len(image)):
        raise ValueError('Image must be linked at 0x08003000 for STM32F103C8 (20 KB RAM)')


def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('image', type=pathlib.Path)
    parser.add_argument('--address', type=int, default=1)
    parser.add_argument('--baud', type=int, default=19200)
    parser.add_argument('--parity', choices=['N', 'E', 'O'], default='N')
    parser.add_argument('--stopbits', type=int, choices=[1, 2], default=1)
    parser.add_argument('--version', type=int, required=True)
    parser.add_argument('--stage-only', action='store_true', help='Compatibility option: upload and verify; command 4 aborts')
    args = parser.parse_args()
    if not 1 <= args.address <= 247 or not 0 <= args.version <= 65535:
        parser.error('address=1..247, version=0..65535')
    image = args.image.read_bytes()
    validate_image(image)
    checksum = zlib.crc32(image)
    with serial.Serial(args.port, args.baud, parity=args.parity, stopbits=args.stopbits, timeout=1, write_timeout=2) as port:
        device = Device(port, args.address)
        if device.status()[9] != 1:
            raise RuntimeError('Unsupported update protocol')
        device.write(6200, [1, len(image) & 65535, len(image) >> 16, checksum & 65535, checksum >> 16, args.version, 0x103B])
        for offset in range(0, len(image), 128):
            data = image[offset:offset + 128]
            packed = data + b'\0' * (len(data) % 2)
            device.write(6210, [offset, len(data), *struct.unpack('>' + 'H' * (len(packed) // 2), packed)])
        device.command(2)
        deadline = time.monotonic() + 120
        while True:
            state, error, *_ = device.status()
            if state == 3:
                break
            if state != 2 or time.monotonic() > deadline:
                raise RuntimeError(f'Verification failed or timed out: state={state}, error={error}')
            time.sleep(0.05)
        print(f'Verified {len(image)} bytes, CRC32={checksum:08X}')
        print('Installation requires a separate FC06 write: register 6200 = 3.')


if __name__ == '__main__':
    main()
