"""The PacketHandler class is a generic class that represents one connection."""

import pandas as pd


class PacketHandler:
    def __init__(self, file_name:str):
        self.received_packet_count = 0
        self.processed_packet_count = 0

        self.data = {
            "timestamp": None,
            "lidar_mm": [],
            "theta": [],
            "packet_count": [],
            "late_wakeup_count": [],
            "skipped_cycle_count": []
        }

        self.plotted_data = [] # Arr[{ "timestamp": datetime, "data": packet }], from earliest to latest

        self.file_name = file_name

    def handle_packet(self, packet, receive_time):
        # Packet is in the form of the telem_packet_t struct, and can be accessed
        # via `packet.lidar_mm` etc.
        self.received_packet_count += 1

        self.data["timestamp"] = receive_time

        # lidar.mm is a size X array that houses the last X lidar measurements.
        for lidar_value in packet.lidar_mm:
            if lidar_value != -2:
                # Sentinel value reserved to tell us lidar didn't give us this
                self.data["lidar_mm"].append(lidar_value)

        # All the other values are over-sampled in the bot but we only take the last_value in this packet.
        self.data["theta"] = [packet.theta] * len(self.data["lidar_mm"])
        self.data["packet_count"] = [packet.packet_count] * len(self.data["lidar_mm"])
        self.data["late_wakeup_count"] = [packet.late_wakeup_count] * len(self.data["lidar_mm"])
        self.data["skipped_cycle_count"] = [packet.skipped_cycle_count] * len(self.data["lidar_mm"])

        self.processed_packet_count += 1

        self.export_to_csv()

        self.clear_data()

    def clear_data(self):
        self.data = {
            "timestamp": None,
            "lidar_mm": [],
            "theta": [],
            "packet_count": [],
            "late_wakeup_count": [],
            "skipped_cycle_count": []
        }
    
    def export_to_csv(self):
        pd.DataFrame(data=self.data).to_csv(self.file_name, mode="a", header=self.received_packet_count == 1, index=True)
