"""Gimbal v1 commands plus receive-only IMU telemetry query; little endian."""
from dataclasses import dataclass
import struct

SET, ENABLE, DISABLE, STOP, QUERY = range(1, 6)
IMU_QUERY = 6
INPUT_QUERY = 7
SET_RATE = 8
MIN_PULSE_US, FRAME_PERIOD_US = 500, 20000
# Leave at least one low microsecond so every frame remains a distinct pulse.
MAX_PULSE_US = FRAME_PERIOD_US - 1
MAX_STEP_US = 20
PAN_MIN_US, PAN_MAX_US, PAN_CENTER_US = 1000, 2000, 1500
TILT_MIN_US, TILT_MAX_US, TILT_CENTER_US = 1000, 2000, 1500
ERRORS = {1: 'CRC 校验失败', 2: '脉宽或速度超出范围', 3: '未知命令', 4: 'PWM 尚未启用'}

def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x07 if value & 0x80 else 0)) & 0xFF
    return value

def within_axis_limits(pan, tilt):
    return PAN_MIN_US <= pan <= PAN_MAX_US and TILT_MIN_US <= tilt <= TILT_MAX_US

def request(seq: int, command: int, pan: int = PAN_CENTER_US,
            tilt: int = TILT_CENTER_US, step: int = 5) -> bytes:
    if command not in range(1, 9):
        raise ValueError('Unknown command')
    if not (MIN_PULSE_US <= pan <= MAX_PULSE_US and
            MIN_PULSE_US <= tilt <= MAX_PULSE_US and 1 <= step <= (100 if command == SET_RATE else MAX_STEP_US)):
        raise ValueError('Invalid pulse width / step')
    if command == SET and not within_axis_limits(pan, tilt):
        raise ValueError('水平范围 1000–2000 μs；俯仰范围 1000–2000 μs')
    body = struct.pack('<BBBBHHB', 0xA5, 0x5A, seq & 255, command, pan, tilt, step)
    return body + bytes([crc8(body)])

@dataclass(frozen=True)
class Status:
    seq: int
    result: int
    flags: int
    pan_target: int
    tilt_target: int
    pan_current: int
    tilt_current: int
    @property
    def enabled(self): return bool(self.flags & 1)
    @property
    def moving(self): return bool(self.flags & 2)
    @property
    def timed_out(self): return bool(self.flags & 4)

def response(data: bytes, seq: int) -> Status:
    if len(data) != 14 or data[:2] != b'\x5a\xa5' or crc8(data[:-1]) != data[-1]:
        raise ValueError('Invalid response frame')
    _, _, actual_seq, result, flags, pan, tilt, pan_out, tilt_out = struct.unpack('<BBBBBHHHH', data[:-1])
    if actual_seq != seq or flags & 0xC0 != 0x80:
        raise ValueError('Wrong sequence / protocol version')
    # Accept legacy status for QUERY/DISABLE during a bitstream upgrade.
    # New SET commands and GUI movement enforce the per-axis limits.
    if not all(MIN_PULSE_US <= p <= MAX_PULSE_US for p in (pan, tilt, pan_out, tilt_out)):
        raise ValueError('Invalid response pulse widths')
    return Status(actual_seq, result, flags, pan, tilt, pan_out, tilt_out)

@dataclass(frozen=True)
class ImuStatus:
    seq: int
    result: int
    flags: int
    roll_raw: int
    pitch_raw: int
    yaw_raw: int
    age_ms: int
    sample_count: int
    byte_count: int
    bad_count: int
    baud_code: int
    @property
    def has_sample(self): return bool(self.flags & 1)
    @property
    def fresh(self): return bool(self.flags & 2) and self.has_sample and self.age_ms < 500
    @property
    def angles(self): return tuple(value * 180 / 32768 for value in (self.roll_raw,self.pitch_raw,self.yaw_raw))
    @property
    def baud(self): return {0: None,1: 115200,2: 9600}[self.baud_code]

def imu_response(data: bytes, seq: int) -> ImuStatus:
    if len(data)!=25 or data[:2]!=b'\x5a\xa6' or crc8(data[:-1])!=data[-1]:
        raise ValueError('Invalid IMU response frame')
    _,_,actual_seq,result,flags,roll,pitch,yaw,age,samples,rx_bytes,bad,baud = struct.unpack('<BBBBBhhhHIIHB',data[:-1])
    if actual_seq!=seq or flags & 0xfc != 0x80 or baud not in (0,1,2):
        raise ValueError('Wrong IMU sequence / flags / baud')
    if flags & 2 and (not flags & 1 or age>=500 or baud==0):
        raise ValueError('Inconsistent IMU freshness')
    return ImuStatus(actual_seq,result,flags,roll,pitch,yaw,age,samples,rx_bytes,bad,baud)

@dataclass(frozen=True)
class InputStatus:
    seq: int
    result: int
    flags: int
    edge_count: int
    min_edge_ticks: int
    idle_ms: int
    @property
    def level(self): return bool(self.flags & 1)
    @property
    def min_interval_us(self): return None if self.min_edge_ticks==0xffffffff else self.min_edge_ticks/27

def input_response(data: bytes,seq: int) -> InputStatus:
    if len(data)!=16 or data[:2]!=b'\x5a\xa7' or crc8(data[:-1])!=data[-1]:
        raise ValueError('Invalid input diagnostic frame')
    _,_,actual_seq,result,flags,edges,ticks,idle=struct.unpack('<BBBBBIIH',data[:-1])
    if actual_seq!=seq or flags & 0xfc != 0x80:
        raise ValueError('Wrong diagnostic sequence / flags')
    return InputStatus(actual_seq,result,flags,edges,ticks,idle)

class ReplyStream:
    """Resynchronize without discarding a later valid frame after noise or bad CRC."""
    def __init__(self): self.buffer=bytearray()
    def feed(self,data):
        self.buffer.extend(data)
        replies=[]
        while len(self.buffer)>=2:
            if self.buffer[:2] not in (b'\x5a\xa5',b'\x5a\xa6',b'\x5a\xa7'):
                del self.buffer[0]
                continue
            decoder,size={0xa5:(response,14),0xa6:(imu_response,25),0xa7:(input_response,16)}[self.buffer[1]]
            if len(self.buffer)<size: break
            frame=bytes(self.buffer[:size])
            try:
                decoded=decoder(frame,frame[2])
            except ValueError:
                del self.buffer[0]
                continue
            del self.buffer[:size]
            replies.append(decoded)
        return replies
