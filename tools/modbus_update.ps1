# Windows PowerShell 5.1. Transfers and verifies the image in EEPROM.
[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Port,
    [string]$Image = '.\02_Modbus\application_v149_1.bin',
    [ValidateRange(1,247)][int]$Address = 1,
    [ValidateRange(1200,115200)][int]$Baud = 19200,
    [ValidateSet('None','Even','Odd')][string]$Parity = 'None',
    [ValidateSet('One','Two')][string]$StopBits = 'One',
    [ValidateRange(0,65535)][int]$Version = 1491
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Get-ModbusCrc([byte[]]$Bytes, [int]$Count) {
    $crc = 65535
    for ($i = 0; $i -lt $Count; $i++) {
        $crc = $crc -bxor $Bytes[$i]
        for ($bit = 0; $bit -lt 8; $bit++) {
            if ($crc -band 1) { $crc = ($crc -shr 1) -bxor 40961 }
            else { $crc = $crc -shr 1 }
        }
    }
    return $crc
}

function Get-ImageCrc([byte[]]$Bytes) {
    [uint64]$crc = 4294967295
    foreach ($value in $Bytes) {
        $crc = $crc -bxor $value
        for ($bit = 0; $bit -lt 8; $bit++) {
            if ($crc -band 1) { $crc = ($crc -shr 1) -bxor 3988292384L }
            else { $crc = $crc -shr 1 }
        }
    }
    return ($crc -bxor 4294967295L)
}

function Set-Word([byte[]]$Bytes, [int]$Index, [uint64]$Value) {
    $Bytes[$Index] = [byte](($Value -shr 8) -band 255)
    $Bytes[$Index + 1] = [byte]($Value -band 255)
}

function Get-Word([byte[]]$Bytes, [int]$Index) {
    return (([int]$Bytes[$Index] -shl 8) -bor $Bytes[$Index + 1])
}

function Send-Request([byte]$FunctionCode, [byte[]]$Body) {
    [byte[]]$frame = [byte[]]::new($Body.Length + 4)
    $frame[0] = $Address
    $frame[1] = $FunctionCode
    [Array]::Copy($Body, 0, $frame, 2, $Body.Length)
    $crc = Get-ModbusCrc $frame ($frame.Length - 2)
    $frame[$frame.Length - 2] = [byte]($crc -band 255)
    $frame[$frame.Length - 1] = [byte]($crc -shr 8)

    $lastError = 'No response'
    for ($attempt = 0; $attempt -lt 5; $attempt++) {
        Start-Sleep -Milliseconds 20
        $serial.DiscardInBuffer()
        $serial.Write($frame, 0, $frame.Length)
        try {
            [byte[]]$prefix = @($serial.ReadByte(), $serial.ReadByte(), $serial.ReadByte())
            if ($prefix[0] -ne $Address) { throw 'Wrong device address in reply' }
            if ($prefix[1] -eq ($FunctionCode -bor 128)) {
                $length = 5
            } elseif ($prefix[1] -ne $FunctionCode) {
                throw 'Wrong function code in reply'
            } elseif ($FunctionCode -eq 3) {
                if ($prefix[2] -ne (2 * (Get-Word $Body 2))) { throw 'Wrong reply byte count' }
                $length = 5 + $prefix[2]
            } else {
                $length = 8
            }
            [byte[]]$reply = [byte[]]::new($length)
            [Array]::Copy($prefix, $reply, 3)
            for ($i = 3; $i -lt $length; $i++) { $reply[$i] = $serial.ReadByte() }
            $replyCrc = [int]$reply[$length - 2] -bor ([int]$reply[$length - 1] -shl 8)
            if ((Get-ModbusCrc $reply ($length - 2)) -ne $replyCrc) { throw 'Reply CRC16 mismatch' }
            if ($length -eq 8) {
                for ($i = 0; $i -lt 4; $i++) {
                    if ($reply[$i + 2] -ne $Body[$i]) { throw 'Write acknowledgement does not match request' }
                }
            }
        } catch {
            $lastError = $_.Exception.Message
            continue
        }

        if ($reply[1] -eq ($FunctionCode -bor 128)) {
            if ($reply[2] -eq 6) {
                $lastError = 'Device busy (exception 6)'
                Start-Sleep -Milliseconds 200
                continue
            }
            throw ('Modbus exception {0}, FC=0x{1:X2}, register={2}' -f $reply[2], $FunctionCode, (Get-Word $Body 0))
        }
        return ,$reply
    }
    throw ('FC=0x{0:X2}, register={1}: {2}' -f $FunctionCode, (Get-Word $Body 0), $lastError)
}

function Write-Block([int]$Register, [byte[]]$Data) {
    [byte[]]$body = [byte[]]::new(5 + $Data.Length)
    Set-Word $body 0 $Register
    Set-Word $body 2 ($Data.Length / 2)
    $body[4] = $Data.Length
    [Array]::Copy($Data, 0, $body, 5, $Data.Length)
    $null = Send-Request 16 $body
}

function Read-Status {
    [byte[]]$body = [byte[]]::new(4)
    Set-Word $body 0 6200
    Set-Word $body 2 10
    return ,(Send-Request 3 $body)
}

Write-Host ('MagStrain EEPROM upload: {0}, {1} baud, device {2}' -f $Port, $Baud, $Address)
[byte[]]$imageBytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Image).ProviderPath)
if ($imageBytes.Length -lt 8 -or $imageBytes.Length -gt 53248) { throw 'Invalid image length' }
$stack = [BitConverter]::ToUInt32($imageBytes, 0)
$entry = [BitConverter]::ToUInt32($imageBytes, 4)
if ($stack -le 0x20000000 -or $stack -gt 0x20005000 -or ($stack -band 7) -ne 0 -or
    ($entry -band 1) -eq 0 -or ($entry - 1) -lt 0x08003000 -or
    ($entry - 1) -ge (0x08003000 + $imageBytes.Length)) {
    throw 'Use the application BIN linked at 0x08003000 for STM32F103C8'
}
$checksum = Get-ImageCrc $imageBytes

$serial = [System.IO.Ports.SerialPort]::new(
    $Port, $Baud, [System.IO.Ports.Parity]$Parity, 8, [System.IO.Ports.StopBits]$StopBits)
$serial.ReadTimeout = 1000
$serial.WriteTimeout = 2000
try {
    $serial.Open()
    Write-Host 'Port opened. Checking the update protocol...'
    $status = Read-Status
    if ((Get-Word $status 21) -ne 1) { throw 'Unsupported update protocol' }
    [byte[]]$header = [byte[]]::new(14)
    Set-Word $header 0 1
    Set-Word $header 2 $imageBytes.Length
    Set-Word $header 6 ($checksum -band 65535)
    Set-Word $header 8 ($checksum -shr 16)
    Set-Word $header 10 $Version
    Set-Word $header 12 0x103B
    Write-Block 6200 $header
    for ($offset = 0; $offset -lt $imageBytes.Length; $offset += 128) {
        $length = [Math]::Min(128, $imageBytes.Length - $offset)
        [byte[]]$block = [byte[]]::new(4 + $length + ($length % 2))
        Set-Word $block 0 $offset
        Set-Word $block 2 $length
        [Array]::Copy($imageBytes, $offset, $block, 4, $length)
        Write-Block 6210 $block
        Write-Progress -Activity 'Uploading to EEPROM' -PercentComplete (100 * ($offset + $length) / $imageBytes.Length)
    }
    Write-Progress -Activity 'Uploading to EEPROM' -Completed

    [byte[]]$verify = [byte[]]::new(4)
    Set-Word $verify 0 6200
    Set-Word $verify 2 2
    $null = Send-Request 6 $verify
    $timer = [Diagnostics.Stopwatch]::StartNew()
    do {
        $status = Read-Status
        $state = Get-Word $status 3
        if ($state -eq 3) { break }
        if ($state -ne 2 -or $timer.Elapsed.TotalSeconds -ge 120) {
            throw ('Verification failed: state={0}, error={1}' -f $state, (Get-Word $status 5))
        }
        Start-Sleep -Milliseconds 50
    } while ($true)

    Write-Host ('Verified {0} bytes, CRC32={1:X8}' -f $imageBytes.Length, $checksum)
    Write-Host 'Installation requires a separate FC06 write: register 6200 = 3.'
} finally {
    $serial.Dispose()
}
