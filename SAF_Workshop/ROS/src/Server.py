import socket
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lxml import etree
from saf_workshop_ros.msg import StationReport


class XMLPublisher(Node):
    def __init__(self):
        super().__init__('xml_publisher')
        self.publisher_ = self.create_publisher(StationReport, 'xml_topic', 10)

    def publish_xml(self, xml_content):
        try:
            # Parse the XML content
            root = etree.fromstring(xml_content)
            date = root.findtext('date')
            carrier_id = root.findtext('carrierid')
            station_id = root.findtext('stationid')

            # Create and populate the custom message
            msg = StationReport()
            msg.date = date
            msg.carrierid = carrier_id
            msg.stationid = station_id

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published StationReport: date={msg.date}, carrierid={msg.carrierid}, stationid={msg.stationid}')
        except etree.XMLSyntaxError as e:
            self.get_logger().error(f"Failed to parse XML: {e}")

def start_server(host='127.17.0.1', port=12345):
    rclpy.init()
    xml_publisher = XMLPublisher()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Server listening on {host}:{port}")

    try:
        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address}")

            message = client_socket.recv(1024)
            print(f"Received message: {message}")

            start_time = time.time()
            xml_publisher.publish_xml(message)

            processing_time = (time.time() - start_time) * 1000  # Convert to milliseconds
            client_socket.send(struct.pack('<f', processing_time))
            print(f"Sending processing time: {processing_time} ms")

            client_socket.close()
            print("Client connection closed")
    except KeyboardInterrupt:
        print("Server is shutting down.")
    finally:
        server_socket.close()
        print("Server socket closed")
        rclpy.shutdown()

if __name__ == "__main__":
    start_server()
