import serial.tools.list_ports
import serial

ports = serial.tools.list_ports.comports(include_links=False)
for n,port in enumerate(ports):
    print(n, ':', port)

try:
    _n = int(input('numero de puerto: '))
    if 0 > _n or _n > len(ports) - 1:
        print("no disponible")
        raise ValueError
except:
    exit()

conversor = serial.Serial(
    port = ports[_n].device,
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
    else:
        print("no response")