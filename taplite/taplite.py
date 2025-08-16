import ctypes
import os
import platform


__all__ = ['assignment', 'simulation']


_os = platform.system()
if _os == 'Darwin':
    lib_name = 'libtaplite.dylib'
elif _os == 'Windows':
    lib_name = 'taplite.dll'
elif _os == 'Linux':
    lib_name = 'libtaplite.so'
else:
    raise OSError('Unsupported operating system')


_lib = None
try:
    _lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), lib_name))
except OSError:
    print('failed to load TAPLite library.')


def assignment():
    _lib.DTA_AssignmentAPI()


def simulation():
    _lib.DTA_SimulationAPI()
