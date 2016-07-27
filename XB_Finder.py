'''
@author: Brandon Zoss
MIT         2015-2016
'''
import sys
import glob
import serial
from re import search

startswith = sys.platform.startswith

def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
        
    elif startswith('linux') or startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
        
    elif startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
        
    else:
        raise EnvironmentError('Unsupported platform')
    

    XB = None
    for ser in ports:
        # look for serial devices, which name includes either a D or a A following a hyphen (-)
        # more at https://docs.python.org/2/library/re.html
        if bool(search('(?<=-)[DA]\w+', ser)) or 'USB0' in ser:
            XB = ser
            break

    if not XB: raise EnvironmentError('No Digi-Mesh Radio Found')
    
    return XB


##if __name__ == '__main__':
##    print(serial_ports())

