"""Ingests packets from the BattleBot using the shared protocol.h file, and dumps to a CSV."""

import datetime
import logging
import socket
from datetime import datetime as dt

from cffi import FFI
from packet_handler import PacketHandler

HOST = '0.0.0.0'
PORT = 2390

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger()

# Parse protocol.h for packet format definitions
ffi = FFI()
with open('../melduino/protocol.h', 'r') as protocol_file:
    ffi.cdef(protocol_file.read())

ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

logger.info(f"Initializing socket on {HOST}:{PORT}...")

try:
    ServerSocket.bind((HOST, PORT))
except socket.error as e:
    logger.error(str(e))
    exit

logger.info(f"Socket created on {HOST}:{PORT} successfully.")

packet_processor = PacketHandler()
longest_duty_cycle = -1.0
last_received_datetime = datetime.datetime.now()

if __name__ == "__main__":
    while True:
        logger.info(f"{dt.now().strftime('%H:%M:%S')}: Listening for next packet...")

        telem_packet, address = ServerSocket.recvfrom(4096)

        # Record metadata about the packets receive rate
        receive_datetime = dt.now()
        logger.info(f"{receive_datetime.strftime('%H:%M:%S')}: Packet received.")

        duty_cycle = (receive_datetime - last_received_datetime).seconds
        if duty_cycle > longest_duty_cycle:
            longest_duty_cycle = duty_cycle
            logger.warning(f"This packet was received longer ({duty_cycle} sec) than previous duty cycles ({longest_duty_cycle} sec).")
        last_received_datetime = receive_datetime

        packet = ffi.from_buffer("telem_packet_t *", telem_packet)

        # Actually handle the packet and export
        packet_processor.handle_packet(packet, receive_time=receive_datetime.strftime('%H:%M:%S.%f'[:-3]))
