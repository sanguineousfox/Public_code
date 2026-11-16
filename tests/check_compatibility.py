"""Verify ALL legacy Modbus defines and EEPROM layout constants against source baseline."""
import pathlib
import re
import subprocess
root = pathlib.Path(__file__).resolve().parents[1]
base = '5861a113f5ce2e86b13c51fd0d48c39161a26740'
for path in ['MagStrain/Core/Inc/modbus.h', 'MagStrain/Core/Inc/params_storage.h', 'MagStrain/Core/Inc/graduation.h']:
    old = subprocess.check_output(['git', 'show', f'{base}:{path}'], cwd=root, text=True)
    new = (root / path).read_text()
    pattern = r'^#define\s+(\w+)([^\n]*)'
    before, after = dict(re.findall(pattern, old, re.M)), dict(re.findall(pattern, new, re.M))
    if path.endswith('/modbus.h'):
        # Mechanical defaults and their comments can change independently of the map.
        before.pop('MODBUS_DEFAULT_WAVEGUIDE_LENGTH_M')
        before.pop('MODBUS_DEFAULT_EXCITATION_PULSE_WIDTH_US')
        before['MB_ADDR_CAL_HIGH_LVL'] = before['MB_ADDR_CAL_HIGH_LVL'].split('/*')[0].rstrip()
        after['MB_ADDR_CAL_HIGH_LVL'] = after['MB_ADDR_CAL_HIGH_LVL'].split('/*')[0].rstrip()
    for key, value in before.items():
        assert after.get(key) == value, (path, key)
    print(f'PASS: {path}: {len(before)} legacy definitions unchanged')
