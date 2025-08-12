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
from sensor_msgs.msg import NavSatFix
from pathlib import Path

# =========================
# Configuration
# =========================
DEFAULT_LAT = 63.4305   # Trondheim
DEFAULT_LON = 10.3951
RADIUS_KM = 20.0
STREAM_URL = "https://live.ais.barentswatch.no/v1/combined?modelType=Full&modelFormat=Geojson"
PUBTOPIC = "/ais/stream"
SUBTOPIC = "/navigation/fix"
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
        self.ref_latitude = DEFAULT_LAT
        self.ref_longitude = DEFAULT_LON
        self.publisher_ = self.create_publisher(String, PUBTOPIC, 10)
        self.pose_sub = self.create_subscription(
            NavSatFix,
            SUBTOPIC,
            self.pose_callback,
            10
        )
        self.session = requests.Session()
        self.stop_event = threading.Event()
        self.stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.stream_thread.start()
        self.get_logger().info(f"Streaming AIS data within {RADIUS_KM} km of ({self.ref_latitude}, {self.ref_longitude})")

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.session.close()
        except Exception:
            pass
        super().destroy_node()

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

                        # GeoJSON Feature format
                        if obj.get("type") != "Feature":
                            continue

                        geometry = obj.get("geometry", {})
                        props = obj.get("properties", {})

                        coords = geometry.get("coordinates")
                        if not isinstance(coords, (list, tuple)) or len(coords) < 2:
                            continue

                        lon, lat = coords[0], coords[1]
                        mmsi = props.get("mmsi")
                        if not mmsi or not isinstance(lat, (float, int)) or not isinstance(lon, (float, int)):
                            continue

                        dist_km = haversine_km(self.ref_latitude, self.ref_longitude, lat, lon)
                        if dist_km > RADIUS_KM:
                            continue

                        # Format output to your expected structure
                        vessel_data = {
                            "courseOverGround": props.get("courseOverGround"),
                            "latitude": lat,
                            "longitude": lon,
                            "shipName": props.get("name"),
                            "rateOfTurn": props.get("rateOfTurn"),
                            "shipType": props.get("shipType"),
                            "speedOverGround": props.get("speedOverGround"),
                            "trueHeading": props.get("trueHeading"),
                            "mmsi": mmsi,
                            "destination": props.get("destination"),
                            "dimensionA": props.get("dimensionA"),
                            "dimensionB": props.get("dimensionB"),
                            "dimensionC": props.get("dimensionC"),
                            "dimensionD": props.get("dimensionD"),
                            "draught": props.get("draught"),
                            "shipLength": props.get("shipLength"),
                            "shipWidth": props.get("shipWidth"),
                            "msgtime": props.get("msgtime"),
                        }

                        msg = String()
                        msg.data = json.dumps(vessel_data, separators=(",", ":"), ensure_ascii=False)
                        self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Streaming failed: {e}. Retrying...")
                time.sleep(RECONNECT_DELAY_SEC)
                
    def pose_callback(self, msg):
        self.ref_latitude = msg.latitude
        self.ref_longitude = msg.longitude


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
