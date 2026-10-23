import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pkt = bytes([0x55, 0x04, 1, 200, 2, 127])
for _ in range(60):
    s.sendto(pkt, ("127.0.0.1", 4003))
    time.sleep(0.05)