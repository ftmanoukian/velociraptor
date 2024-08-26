import serial.tools.list_ports
import serial

conversor = serial.Serial(
    port = '/dev/cu.usbserial-A50285BI',
    baudrate = 115200,
    timeout = .1
)

while True:
    comando = input('comando: ')
    if len(comando) < 10:
        comando += ' ' * (10 - len(comando))
    elif len(comando) > 10:
        comando = comando[:10]
    
    for c in comando:
        conversor.write(c.encode())

    r = conversor.read_until()
    if r:
        print(r.decode())