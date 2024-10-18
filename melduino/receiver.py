import socket
import time
import pandas as pd
from cffi import FFI
import datetime

MAX_PACKET_ARRAY_SIZE = 16 # lidar_mm size will usually drive this number

# Parse protocol.h for packet format definitions
ffi = FFI()
with open('protocol.h', 'r') as protocol_file:
    ffi.cdef(protocol_file.read())

ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host = '0.0.0.0'
port = 2390

print(f"Initializing connection to {host}:{port}.")

try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))
    exit

print("Connection successful.")

class PacketHandler:
    def __init__(self):
        self.received_packet_count = 0
        self.processed_packet_count = 0

        self.data = {
            "lidar_mm": [],
            "theta": [],
            "packet_count": [],
            "late_wakeup_count": [],
            "skipped_cycle_count": []
        }

    def handle_packet(self, packet):
        # Packet is in the form of the telem_packet_t struct, and can be accessed
        # via `packet.lidar_mm` etc.
        self.received_packet_count += 1

        print(f"{datetime.datetime.now().strftime("%H:%M:%S")}: {packet.lidar_mm[0]}")

        # lidar.mm is a size X array that houses the last X lidar measurements.
        for lidar_value in packet.lidar_mm:
            if lidar_value == -2:
                # Sentinel value reserved to tell us lidar didn't give us this
                self.data["lidar_mm"].append(lidar_value)

        # All the other values are oversampled in the bot but we only take the last_value in this packet.
        self.data["theta"] = [packet.theta] * len(self.data["lidar_mm"])
        self.data["packet_count"] = [packet.packet_count] * len(self.data["lidar_mm"])
        self.data["late_wakeup_count"] = [packet.late_wakeup_count] * len(self.data["lidar_mm"])
        self.data["skipped_cycle_count"] = [packet.skipped_cycle_count] * len(self.data["lidar_mm"])

        self.processed_packet_count += 1

        # self.export_to_csv()

        self.clear_data()

    def clear_data(self):
        self.data = {
            "lidar_mm": [],
            "theta": [],
            "packet_count": [],
            "late_wakeup_count": [],
            "skipped_cycle_count": []
        }
    
    def export_to_csv(self):
        pd.DataFrame(data=self.data).to_csv("./foo.xlsx", mode="a")

packet_processor = PacketHandler()
longest_duty_cycle = -1.0
last_receive_time = time.time()

while True:
    telem_packet, address = ServerSocket.recvfrom(4096)
    
    # Record metadata about the packets recieve rate
    recieve_time = time.time()
    duty_cycle = recieve_time - last_receive_time
    if duty_cycle > longest_duty_cycle:
        longest_duty_cycle = duty_cycle
        print(f"Packet recieved at {recieve_time} took longer ({duty_cycle} sec) than previous duty cycles ({longest_duty_cycle} sec).")
    last_receive_time = recieve_time

    packet = ffi.from_buffer("telem_packet_t *", telem_packet)

    # Actually handle the packet and export
    packet_processor.handle_packet(packet)


    # print(count)
    # print(f"{min(telem_struct.lidar_mm) / 1000} \t {max(telem_struct.lidar_mm) / 1000}")
    # for i in range(8):
    #     print(f"Lidar {i}: {telem_struct.lidar_mm[i]}")
    # print(f"late_wakeup {telem_struct.late_wakeup_count}")
    # print(f"skipped_cycle {telem_struct.skipped_cycle_count}")
    # packet_count = int(telem_struct.packet_count)
    # if packet_count != next_packet_count:
    #     print(f"\n\nGot packet count {packet_count}, expected {next_packet_count}\n")
    # next_packet_count = (packet_count + 1) % 256;
    # print(f"{diff:.3f}, {worst_diff:.3f}")