"""The PacketHandler class is a generic class that represents one connection."""

import os
from datetime import datetime as dt

import pandas as pd


class PacketHandler:
    def __init__(self):
        self.initialization_time = dt.now()

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

        self._clear_live()

    def _clear_live(self):
        filename: str
        for filename in os.listdir("./umb/live"):
            path = os.path.join("./umb/live", filename)

            if os.path.isfile(path):
                os.remove(path)

    def handle_packet(self, packet, receive_time):
        # Packet is in the form of the telem_packet_t struct, and can be accessed
        # via `packet.lidar_mm` etc.
        self.received_packet_count += 1

        # lidar.mm is a size X array that houses the last X lidar measurements.
        for lidar_value in packet.lidar_mm:
            if lidar_value != -2:
                # Sentinel value reserved to tell us lidar didn't give us this
                self.data["lidar_mm"].append(lidar_value)

        self.data["timestamp"] = [f"{receive_time}.{idx}" for idx in range(len(self.data["lidar_mm"]))]

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
        column: str
        for column in self.data:
            # Each data packet gets the timestamp written.
            if column == "timestamp":
                # Don't export the timestamp, for obvious reasons.
                continue


            if column == "lidar_mm":
                pd.DataFrame(data={ "timestamp": self.data["timestamp"], column: self.data[column]}).to_csv(f"./umb/live/{column}", mode="a", header=self.received_packet_count == 1, index=False)
            else:
                pd.DataFrame(data={column: self.data[column][0]}, index=[self.data["timestamp"][0].rsplit('.', 1)[0]]).to_csv(f"./umb/live/{column}", mode="a", header=self.received_packet_count == 1, index=True)
            
        pd.DataFrame(data=self.data).to_csv(f"./umb/{self.initialization_time.strftime('%Y-%m-%d--%H-%M-%S')}.csv", mode="a", header=self.received_packet_count == 1, index=False)
