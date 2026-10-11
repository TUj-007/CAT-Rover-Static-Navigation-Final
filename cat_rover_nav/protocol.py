import binascii
import math
from .models import Telemetry


def frame(payload):
    raw=payload.encode('ascii')
    return raw+f'*{binascii.crc_hqx(raw,0xffff):04X}\n'.encode()


def unframe(raw):
    if not isinstance(raw,bytes) or len(raw)>180 or not raw.endswith(b'\n'):
        raise ValueError('Invalid frame length/terminator')
    body,crc=raw.rstrip(b'\r\n').rsplit(b'*',1)
    if len(crc)!=4 or any(c not in b'0123456789abcdefABCDEF' for c in crc):
        raise ValueError('Invalid CRC')
    if binascii.crc_hqx(body,0xffff)!=int(crc,16): raise ValueError('CRC mismatch')
    return body.decode('ascii').split(',')


def uint(s):
    if not s.isascii() or not s.isdecimal() or not 0<=int(s)<=0xffffffff:
        raise ValueError('Invalid uint32')
    return int(s)


def parse_telemetry(raw,now):
    p=unframe(raw)
    if len(p)!=13 or p[0]!='T': raise ValueError('Invalid telemetry format')
    n=[uint(v) for v in p[1:]]
    if n[2] not in (0,1,2) or n[3]>31 or any(v>2100 for v in n[8:]):
        raise ValueError('Telemetry out of bounds')
    return Telemetry(n[0],n[1],n[2],n[3],tuple(n[4:8]),now)


def delta32(a,b):
    return ((a-b+0x80000000)&0xffffffff)-0x80000000


def velocity_command(p,seq,left,right):
    if any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>.30 for v in (left,right)):
        raise ValueError('Wheel target outside +/-0.30 m/s')
    return frame(f'C,{p.epoch},{seq&0xffffffff},{p.ms},{round(left*1000)},{round(right*1000)}')
