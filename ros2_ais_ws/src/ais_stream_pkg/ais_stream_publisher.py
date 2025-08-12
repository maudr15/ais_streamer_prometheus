#!/usr/bin/env python3
import json
import math
import threading
import time
import requests
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path

# =========================
# Configuration
# =========================
REFERENCE_LAT = 63.4305   # Trondheim
REFERENCE_LON = 10.3951
RADIUS_KM = 20.0
STREAM_URL = "https://live.ais.barentswatch.no/v1/combined"
TOKEN_FILE = "../../../token.txt"
TOPIC = "/ais/stream"
RECONNECT_DELAY_SEC = 5.0
READ_TIMEOUT_SEC = 90


def read_token():
    path = os.getenv("AIS_TOKEN_PATH")
    if not path:
        print("AIS_TOKEN_PATH env var not set.")
        return None
    token_path = Path(path)
    try:
        return token_path.read_text().strip()
    except Exception as e:
        print(f"Error reading token: {e}")
        return None


def haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def try_extract_latlon(obj):
    keys = [("latitude", "longitude"), ("lat", "lon"), ("Latitude", "Longitude")]
    for lat_key, lon_key in keys:
        lat, lon = obj.get(lat_key), obj.get(lon_key)
        if isinstance(lat, (float, int)) and isinstance(lon, (float, int)):
            return float(lat), float(lon)

    pos = obj.get("position") or obj.get("Position")
    if isinstance(pos, dict):
        lat = pos.get("latitude") or pos.get("lat")
        lon = pos.get("longitude") or pos.get("lon")
        if isinstance(lat, (float, int)) and isinstance(lon, (float, int)):
            return float(lat), float(lon)

    geom = obj.get("geometry") or obj.get("Geometry")
    if isinstance(geom, dict):
        coords = geom.get("coordinates")
        if isinstance(coords, list) and len(coords) >= 2:
            lon, lat = coords[0], coords[1]
            if isinstance(lat, (float, int)) and isinstance(lon, (float, int)):
                return float(lat), float(lon)

    return None


class AISStreamNode(Node):
    def __init__(self):
        super().__init__("ais_stream_publisher")
        self.publisher_ = self.create_publisher(String, TOPIC, 10)
        self.session = requests.Session()
        self.stop_event = threading.Event()
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        self.get_logger().info(f"Streaming AIS data within {RADIUS_KM} km of ({REFERENCE_LAT}, {REFERENCE_LON})")

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.session.close()
        except Exception:
            pass
        super().destroy_node()
        #try:
            #os.remove(TOKEN_FILE)
        #except Exception:
        #    pass

    def _stream_loop(self):
        while not self.stop_event.is_set():
            try:
                token = read_token()
                headers = {"Authorization": f"Bearer {token}"}
                with self.session.get(STREAM_URL, headers=headers, stream=True, timeout=(10, READ_TIMEOUT_SEC)) as resp:
                    if resp.status_code != 200:
                        self.get_logger().warn(f"Stream error: {resp.status_code}. Retrying...")
                        time.sleep(RECONNECT_DELAY_SEC)
                        continue

                    for line in resp.iter_lines(decode_unicode=True):
                        if self.stop_event.is_set():
                            break
                        if not line:
                            continue
                        try:
                            obj = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        latlon = try_extract_latlon(obj)
                        mmsi = obj.get("mmsi")
                        if not latlon or not mmsi:
                            continue  # Skip incomplete data

                        lat, lon = latlon
                        dist_km = haversine_km(REFERENCE_LAT, REFERENCE_LON, lat, lon)
                        if dist_km > RADIUS_KM:
                            continue

                        vessel_data = {
                            "courseOverGround": obj.get("courseOverGround"),
                            "latitude": lat,
                            "longitude": lon,
                            "name": obj.get("name"),
                            "rateOfTurn": obj.get("rateOfTurn"),
                            "shipType": obj.get("shipType"),
                            "speedOverGround": obj.get("speedOverGround"),
                            "trueHeading": obj.get("trueHeading"),
                            "mmsi": mmsi,
                            "msgtime": obj.get("msgtime"),
                        }

                        msg = String()
                        msg.data = json.dumps(vessel_data, separators=(",", ":"), ensure_ascii=False)
                        self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Streaming failed: {e}. Retrying...")
                time.sleep(RECONNECT_DELAY_SEC)


def main():
    rclpy.init()
    node = AISStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
