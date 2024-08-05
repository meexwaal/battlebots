import socket
from cffi import FFI

# Parse protocol.h for packet format definitions
ffi = FFI()
with open('protocol.h', 'r') as protocol_file:
    ffi.cdef(protocol_file.read())

ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host = '0.0.0.0'
port = 2390;

print("Start")
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))
print("bound")

count = 0
while True:
    telem_packet, address = ServerSocket.recvfrom(1000)
    count += 1

    # if count % 100 == 0:
    if count % 1 == 0:
        telem_struct = ffi.from_buffer("telem_packet_t *", telem_packet)
        print(count)
        for i in range(8):
            print(f"Lidar {i}: {telem_struct.lidar_mm[i]}")
        print(f"late_wakeup {telem_struct.late_wakeup_count}")
        print(f"skipped_cycle {telem_struct.skipped_cycle_count}")
