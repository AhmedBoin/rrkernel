# QEMU-monitor probe used while bringing up bare-metal ports.
#
# It reads interrupt-controller, timer and CPU state from *inside* the running VM,
# which is what distinguishes "the interrupt was never raised" from "it was raised
# and not taken" from "the switch is broken" — far faster than guessing.
#
#   powershell -File scripts/qemu-monitor-probe.ps1 -Arch riscv
#   powershell -File scripts/qemu-monitor-probe.ps1 -Arch arm
param(
    [ValidateSet('riscv', 'arm')]
    [string]$Arch = 'riscv',
    [int]$Port = 45454
)

# NOTE: never name a variable `$args` in PowerShell — it is reserved, so
# assignments are silently ignored and QEMU ends up starting with no arguments.
$qemuArgs = @()

if ($Arch -eq 'riscv') {
    $qemu = 'qemu-system-riscv32'
    $bin = 'target/riscv32imac-unknown-none-elf/release/firmware-riscv'
    $qemuArgs += @('-M', 'virt', '-cpu', 'rv32', '-bios', 'none', '-nographic', '-no-reboot', '-kernel', $bin)
    $probes = @(
        @('mtimecmp (CLINT+0x4000)', 'xp /2xw 0x2004000'),
        @('mtime    (CLINT+0xBFF8)', 'xp /2xw 0x200bff8'),
        @('registers', 'info registers')
    )
} else {
    $qemu = 'qemu-system-arm'
    $bin = 'target/armv7a-none-eabi/release/firmware-arm-a'
    $qemuArgs += @('-M', 'virt', '-cpu', 'cortex-a15', '-nographic', '-no-reboot', '-semihosting', '-kernel', $bin)
    $probes = @(
        @('GICD_ISENABLER0 (timer enabled?)', 'xp /1xw 0x08000100'),
        @('GICD_ISPENDR0  (timer pending?)', 'xp /1xw 0x08000200'),
        @('GICD_IGROUPR0', 'xp /1xw 0x08000080'),
        @('GICC_CTLR / PMR', 'xp /2xw 0x08010000'),
        # Kernel control block, at the address llvm-nm reports for `KERNEL`:
        # current_tcb, ring_head, total_threads, active_threads, ticks(u64),
        # switches(u64), ticks_deferred(u64).
        @('KERNEL.current/ring_head', 'xp /2xw 0x40004de8'),
        @('KERNEL.total/active', 'xp /2xw 0x40004df0'),
        @('KERNEL.ticks', 'xp /2xw 0x40004df8'),
        @('KERNEL.switches', 'xp /2xw 0x40004e00'),
        @('registers', 'info registers')
    )
}

$qemuArgs += @('-monitor', "tcp:127.0.0.1:$Port,server,nowait")
$proc = Start-Process -FilePath $qemu -ArgumentList $qemuArgs -PassThru -NoNewWindow `
    -RedirectStandardOutput qemu_serial.txt -RedirectStandardError qemu_err.txt
Start-Sleep -Seconds 2

$client = New-Object Net.Sockets.TcpClient('127.0.0.1', $Port)
$stream = $client.GetStream()
$writer = New-Object IO.StreamWriter($stream)
$writer.AutoFlush = $true

function Send-Monitor([string]$cmd) {
    $writer.WriteLine($cmd)
    Start-Sleep -Milliseconds 200
    $buf = New-Object byte[] 32768
    $read = $stream.Read($buf, 0, $buf.Length)
    if ($read -gt 0) { [Text.Encoding]::ASCII.GetString($buf, 0, $read) } else { '' }
}

foreach ($probe in $probes) {
    Write-Output "--- $($probe[0]) ---"
    Send-Monitor $probe[1]
}

$client.Close()
if (-not $proc.HasExited) { $proc.Kill() }
Write-Output '--- guest serial output ---'
Get-Content qemu_serial.txt -ErrorAction SilentlyContinue
