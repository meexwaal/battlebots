import socket
import time
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

next_packet_count = 0
prev_time = time.time()
worst_diff = 0
count = 0
while True:
    telem_packet, address = ServerSocket.recvfrom(4096)
    count += 1
    now = time.time()
    diff = now - prev_time
    prev_time = now
    worst_diff = max(worst_diff, diff)

    # if count % 100 == 0:
    if count % 1 == 0:
        telem_struct = ffi.from_buffer("telem_packet_t *", telem_packet)
        print(count)
        # for i in range(8):
        #     print(f"Lidar {i}: {telem_struct.lidar_mm[i]}")
        # print(f"late_wakeup {telem_struct.late_wakeup_count}")
        # print(f"skipped_cycle {telem_struct.skipped_cycle_count}")
        packet_count = int(telem_struct.packet_count)
        if packet_count != next_packet_count:
            print(f"\n\nGot packet count {packet_count}, expected {next_packet_count}\n")
        next_packet_count = (packet_count + 1) % 256;
        print(f"{diff:.3f}, {worst_diff:.3f}")
