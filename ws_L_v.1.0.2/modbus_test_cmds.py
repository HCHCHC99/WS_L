# -*- coding: utf-8 -*-
"""
Modbus command generator - Simple version
Usage:
  1. Modify the config values below
  2. Run: py modbus_test_cmds.py
  3. Copy the output command
=============================================================================
"""

# =============================================================================
# ===== USER CONFIG =====
# =============================================================================

# Device address
NODE_ID = 1

# Operation: "READ" or "WRITE" or "WRITE_MULTI" or "CLEAR_FAULT" or "CTRL_CMD"
OPERATION = "WRITE_MULTI"

# Register address (hex) - only for READ/WRITE
REG_ADDR = 0x2737   # REG_CTRL_CMD

# Value to write (only for WRITE, negative auto-converted)
WRITE_VALUE = 2000

# Control
CTRL_CMD_VALUE = 0x0008   

# ===== Fault clear config (only for OPERATION = "CLEAR_FAULT") =====
# Select fault bits to clear (can combine with |):
#   0x0001 = Overvoltage (FAULT_BIT_OVERVOLTAGE)
#   0x0002 = Overcurrent (FAULT_BIT_OVERCURRENT)
#   0x0004 = Overtemp    (FAULT_BIT_OVERTEMP)
#   0x0008 = Reset       (FAULT_BIT_RESET)
#   0x0010 = Overload    (FAULT_BIT_OVERLOAD)
#   0x0020 = Stall       (FAULT_BIT_STALL)
#   0x0040 = Undervoltage(FAULT_BIT_UNDERVOLTAGE)
#   0x0000 = Clear all faults
CLEAR_FAULT_BITS = 0x0000   # default: clear undervoltage

# ===== Multi-register write config (only for OPERATION = "WRITE_MULTI") =====
# Format: [register_addr, value]
# Uncomment the lines you need and fill in the values, then run the script.
# REG_CTRL_CMD (0x2720) and REG_FAULT_STATUS (0x2740) use single-write (WRITE).
# All values written to RAM first, then saved to Flash once (one erase only).
#
# ===== User parameters (Modbus registers, stored in Flash) =====
#  [0x2710, 1   ],   # REG_NODE_ID              1~247
#  [0x2711, 0   ],   # REG_TARGET_SPEED         r/min (int16)
#  [0x2712, 0   ],   # REG_TARGET_ANGLE         0.1 deg (int16)
#  [0x2714, 270 ],   # REG_VOLTAGE_UPPER_LIMIT  0.1V (uint16)
#  [0x2715, 210 ],   # REG_VOLTAGE_LOWER_LIMIT  0.1V (uint16)
#  [0x2716, 5000],   # REG_CURRENT_UPPER_LIMIT  mA (uint16)
#  [0x271C, -20 ],   # REG_CLOSE_LIMIT_ANGLE    0.1 deg (int16)
#  [0x271D, 880 ],   # REG_OPEN_LIMIT_ANGLE     0.1 deg (int16)
#  [0x271E, 20  ],   # REG_CURRENT_DETECT_MS    ms (uint16)
#
# Note: registers must be contiguous (Modbus 0x10 requirement).
#       The script will check and report an error if they are not.
MULTI_WRITE_REGS = [
      [0x2714, 280],   # REG_VOLTAGE_UPPER_LIMIT  0.1V
      [0x2715, 200],   # REG_VOLTAGE_LOWER_LIMIT  0.1V
      [0x2716, 3000],  # REG_CURRENT_UPPER_LIMIT  mA
]

# ===== REG_CTRL_CMD (0x2720) config (only for OPERATION = "CTRL_CMD") =====
# Bit definition:
#   bit0 = START  (enable RS485 control)
#   bit1 = STOP   (disable RS485 control)
#   bit2 = ESTOP  (cancel rotation, keep RS485 control)
#   bit4 = FWD    (forward, only valid after START)
#   bit5 = REV    (reverse, only valid after START)
#
# Usage steps:
#   1. Send START (0x0001) to enable RS485 control
#   2. Send FWD (0x0011) or REV (0x0021)
#   3. Send STOP (0x0002) to disable RS485 control
#   4. Send ESTOP (0x0004) to cancel rotation without disabling RS485
#
# Common values:
#   0x0001 = START 	(enable RS485 control)	01 06 27 20 00 01 43 74
#   0x0002 = STOP 	(disable RS485 control)	01 06 27 20 00 02 03 75
#   0x0004 = ESTOP	(cancel rotation)			01 06 27 20 00 04 83 77
#   0x0008 = RESET							
#   0x0010 = FWD     	(bit0=1, bit4=1)			01 06 27 20 00 10 83 78
#   0x0020 = REV  	(bit0=1, bit5=1)			01 06 27 20 00 20 83 6C


#===== READ=====
# REG_REAL_SPEED              		(0x2730)    		01 03 27 30 00 01 8E B1
# REG_REAL_ANGLE              		(0x2731)   		01 03 27 31 00 01 DF 71
# REG_REAL_VOLTAGE            		(0x2732)   		01 03 27 32 00 01 2F 71
# REG_REAL_CURRENT            		(0x2733)   		01 03 27 30 00 01 8E B1
# REG_REAL_DIRECTION          		(0x2737)   		01 03 27 37 00 01 3F 70



# REG_FAULT_STATUS           		(0x2740U)  
# ===== END CONFIG =====
# =============================================================================


def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def make_frame(data_bytes):
    crc = modbus_crc16(data_bytes)
    frame = list(data_bytes) + [crc & 0xFF, (crc >> 8) & 0xFF]
    return ' '.join(f'{b:02X}' for b in frame)


def to_u16(val):
    if val < 0:
        return val + 65536
    return val


# Fault bit name mapping
FAULT_NAMES = {
    0x0001: "Overvoltage",
    0x0002: "Overcurrent",
    0x0004: "Overtemp",
    0x0008: "Reset",
    0x0010: "Overload",
    0x0020: "Stall",
    0x0040: "Undervoltage",
}


def describe_fault_bits(bits):
    """Convert fault bit mask to readable description"""
    if bits == 0:
        return "Clear all faults"
    names = []
    for mask, name in FAULT_NAMES.items():
        if bits & mask:
            names.append(name)
    if not names:
        return f"Unknown fault bits(0x{bits:04X})"
    return " + ".join(names)


# =============================================================================
# Generate command
# =============================================================================
print("=" * 50)
print(f"Device addr: {NODE_ID}")
print(f"Operation:   {OPERATION}")
if OPERATION == "READ":
    print(f"Register:    0x{REG_ADDR:04X}")
elif OPERATION == "WRITE":
    print(f"Register:    0x{REG_ADDR:04X}")
    print(f"Write value: {WRITE_VALUE} (0x{to_u16(WRITE_VALUE):04X})")
elif OPERATION == "CLEAR_FAULT":
    print(f"Fault reg:   0x2740 (REG_FAULT_STATUS)")
    print(f"Clear bits:  0x{CLEAR_FAULT_BITS:04X} -> {describe_fault_bits(CLEAR_FAULT_BITS)}")
elif OPERATION == "WRITE_MULTI":
    print(f"Reg count:   {len(MULTI_WRITE_REGS)}")
    for reg, val in MULTI_WRITE_REGS:
        print(f"  0x{reg:04X} <- {val} (0x{to_u16(val):04X})")
print("=" * 50)

if OPERATION == "READ":
    cmd = make_frame([NODE_ID, 0x03, (REG_ADDR >> 8) & 0xFF, REG_ADDR & 0xFF, 0x00, 0x01])
    print(f"\nSend: {cmd}")
    print(f"Resp: {NODE_ID:02X} 03 02 XX XX CRC")
    print(f"  XX XX = value at 0x{REG_ADDR:04X}")

elif OPERATION == "WRITE":
    val = to_u16(WRITE_VALUE)
    cmd = make_frame([NODE_ID, 0x06, (REG_ADDR >> 8) & 0xFF, REG_ADDR & 0xFF,
                      (val >> 8) & 0xFF, val & 0xFF])
    print(f"\nSend: {cmd}")
    print(f"Resp: {NODE_ID:02X} 06 {REG_ADDR >> 8:02X} {REG_ADDR & 0xFF:02X} {val >> 8:02X} {val & 0xFF:02X} CRC (echo)")
    print(f"  Echo back the write command on success")

elif OPERATION == "CLEAR_FAULT":
    # Write REG_FAULT_STATUS (0x2740) with fault bits to clear
    val = CLEAR_FAULT_BITS
    cmd = make_frame([NODE_ID, 0x06, 0x27, 0x40,
                      (val >> 8) & 0xFF, val & 0xFF])
    print(f"\nSend: {cmd}")
    print(f"Resp: {NODE_ID:02X} 06 27 40 {val >> 8:02X} {val & 0xFF:02X} CRC (echo)")
    print(f"  Echo back the write command on success")
    print(f"")
    print(f"  Call chain:")
    print(f"    Modbus write 0x2740 = 0x{val:04X}")
    print(f"    -> FaultHandler_ClearFault()")
    print(f"    -> Voltage_Device_ClearAlarm() + Motor_ClearVoltageBlock()")
    print(f"    -> RealTime_ClearFault(0x{val:04X})")

elif OPERATION == "WRITE_MULTI":
    if not MULTI_WRITE_REGS:
        print("\nError: MULTI_WRITE_REGS is empty, uncomment some [reg, value] entries")
    else:
        start_reg = MULTI_WRITE_REGS[0][0]
        reg_count = len(MULTI_WRITE_REGS)
        byte_count = reg_count * 2

        # Build data bytes in register order
        data_bytes = []
        for i, (reg, val) in enumerate(MULTI_WRITE_REGS):
            # Verify contiguous (Modbus 0x10 requires it)
            if reg != start_reg + i:
                print(f"\nError: registers must be contiguous. "
                      f"Expected 0x{start_reg + i:04X}, got 0x{reg:04X}")
                data_bytes = None
                break
            v = to_u16(val)
            data_bytes.append((v >> 8) & 0xFF)
            data_bytes.append(v & 0xFF)

        if data_bytes is not None:
            cmd = make_frame([NODE_ID, 0x10,
                              (start_reg >> 8) & 0xFF, start_reg & 0xFF,
                              (reg_count >> 8) & 0xFF, reg_count & 0xFF,
                              byte_count] + data_bytes)
            print(f"\nSend: {cmd}")
            print(f"Resp: {NODE_ID:02X} 10 {start_reg >> 8:02X} {start_reg & 0xFF:02X} "
                  f"{reg_count >> 8:02X} {reg_count & 0xFF:02X} CRC")
            print(f"")
            print(f"  Call chain:")
            print(f"    ModbusRTU_HandleWriteMulti({reg_count} regs)")
            print(f"    -> App_Comm_OnWriteMulti()")
            print(f"       -> Param_WriteByReg() x {reg_count} (RAM)")
            print(f"       -> Param_Save() x 1 (Flash, one erase only)")

elif OPERATION == "CTRL_CMD":
    # Write REG_CTRL_CMD (0x2720) with control command value
    val = CTRL_CMD_VALUE

    # Describe the command
    desc_parts = []
    if val & 0x0001:
        desc_parts.append("START")
    if val & 0x0002:
        desc_parts.append("STOP")
    if val & 0x0004:
        desc_parts.append("ESTOP")
    if val & 0x0010:
        desc_parts.append("FWD")
    if val & 0x0020:
        desc_parts.append("REV")
    desc = " + ".join(desc_parts) if desc_parts else "NONE"

    cmd = make_frame([NODE_ID, 0x06, 0x27, 0x20,
                      (val >> 8) & 0xFF, val & 0xFF])
    print(f"\nSend: {cmd}")
    print(f"Resp: {NODE_ID:02X} 06 27 20 {val >> 8:02X} {val & 0xFF:02X} CRC (echo)")
    print(f"  Echo back the write command on success")
    print(f"")
    print(f"  Command: 0x{val:04X} = {desc}")
    print(f"  -> EventBus_Publish(TOPIC_MANUAL_RS485)")
    print(f"  -> Motor_OnManualIO() -> motor arbiter")

else:
    print(f"\nError: unknown OPERATION '{OPERATION}', "
          f"use READ, WRITE, WRITE_MULTI, CLEAR_FAULT or CTRL_CMD")

input()
