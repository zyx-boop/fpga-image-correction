"""Windows serial transport, Python standard library only; no driver install."""
import ctypes as C
from ctypes import wintypes as W
import winreg
class DCB(C.Structure):
    _fields_=[('length',W.DWORD),('baud',W.DWORD),('flags',W.DWORD),('reserved',W.WORD),
              ('xonlim',W.WORD),('xofflim',W.WORD),('bytesize',W.BYTE),('parity',W.BYTE),('stop',W.BYTE),
              ('xon',C.c_char),('xoff',C.c_char),('error',C.c_char),('eof',C.c_char),('evt',C.c_char),('reserved1',W.WORD)]
class Timeouts(C.Structure):
    _fields_=[(n,W.DWORD) for n in ('interval','readmul','readconst','writemul','writeconst')]
k=C.WinDLL('kernel32',use_last_error=True)
k.CreateFileW.argtypes=[W.LPCWSTR,W.DWORD,W.DWORD,C.c_void_p,W.DWORD,W.DWORD,W.HANDLE];k.CreateFileW.restype=W.HANDLE
k.GetCommState.argtypes=[W.HANDLE,C.POINTER(DCB)];k.SetCommState.argtypes=[W.HANDLE,C.POINTER(DCB)]
k.SetCommTimeouts.argtypes=[W.HANDLE,C.POINTER(Timeouts)]
k.ReadFile.argtypes=[W.HANDLE,C.c_void_p,W.DWORD,C.POINTER(W.DWORD),C.c_void_p]
k.WriteFile.argtypes=k.ReadFile.argtypes;k.CloseHandle.argtypes=[W.HANDLE]
def ports():
    out=[]
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,r'HARDWARE\DEVICEMAP\SERIALCOMM') as key:
            for i in range(winreg.QueryInfoKey(key)[1]):out.append(winreg.EnumValue(key,i)[1])
    except FileNotFoundError:pass
    return sorted(set(out))
def check(ok):
    if not ok:raise C.WinError(C.get_last_error())
class Serial:
    def __init__(self,port):
        if not port.upper().startswith('COM') or not port[3:].isdigit():raise ValueError('请选择 COM 端口')
        self.h=k.CreateFileW('\\\\.\\'+port,0xC0000000,0,None,3,0,None)
        if self.h==C.c_void_p(-1).value:raise C.WinError(C.get_last_error())
        try:
            d=DCB();d.length=C.sizeof(d);check(k.GetCommState(self.h,C.byref(d)))
            d.baud=115200;d.flags=1;d.bytesize=8;d.parity=0;d.stop=0
            check(k.SetCommState(self.h,C.byref(d)))
            t=Timeouts(0xffffffff,0,20,0,500);check(k.SetCommTimeouts(self.h,C.byref(t)))
        except BaseException:self.close();raise
    def read(self):
        b=C.create_string_buffer(4096);n=W.DWORD();check(k.ReadFile(self.h,b,len(b),C.byref(n),None));return b.raw[:n.value]
    def write(self,data):
        b=C.create_string_buffer(data);n=W.DWORD();check(k.WriteFile(self.h,b,len(data),C.byref(n),None))
        if n.value!=len(data):raise OSError('串口未完整发送')
    def close(self):
        if self.h is not None:k.CloseHandle(self.h);self.h=None
