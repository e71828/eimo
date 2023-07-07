import struct
from typing import Union, Tuple, List, Optional
import logging

try:
    from .packetID import PacketID
except ModuleNotFoundError:
    print("Please install access to the SDK.")

logger = logging.getLogger(__name__)


class WITProtocol:
    """Class used to encode and decode BPL packets."""

    @staticmethod
    def packet_splitter(buff: bytes) -> Tuple[List[bytes], Optional[bytes]]:
        """
        Split packets coming in along bpl protocol, Packets are split at b'0x55'.

        :param buff: input buffer of bytes
        :return: List of bytes separated by 0x55, and a remaining bytes of an incomplete packet.
        """
        incomplete_packet = None
        if buff[0] != 0x55:
            *_, packet_complete = buff.split(b'\x55', 1)
            packet_complete = b'\x55' + packet_complete
            packets = [packet_complete[i:i + 11] for i in range(0, len(packet_complete), 11)]
        else:
            packets = [buff[i:i + 11] for i in range(0, len(buff), 11)]
        if len(packets[-1]) != 11:
            incomplete_packet = packets.pop()
        return packets, incomplete_packet

    @staticmethod
    def parse_packet(packet_in: Union[bytes, bytearray]) -> Tuple[int, bytes]:
        """
        Parse the packet returning a tuple of [int, bytes].
        If unable to parse the packet, then return 0,b''.
        :param packet_in: bytes of a full packet
        """

        # packet_in = bytearray(packet_in)

        if packet_in[0] == 0x55:
            decoded_packet: bytes = packet_in

            if len(decoded_packet) != 11:
                logger.warning(f"parse_packet(): Incorrect length: length is {len(decoded_packet)} "
                               f"in {[hex(x) for x in list(decoded_packet)]}")
                return 0, b''
            elif sum(decoded_packet[:-1]) & 0xff == decoded_packet[-1]:
                rx_data = decoded_packet[2:]
                packet_id = decoded_packet[1]
                return packet_id, rx_data
            else:
                logger.warning(f"parse_packet(): CRC error in {[hex(x) for x in list(decoded_packet)]} ")
                return 0, b''
        else:
            logger.warning(f"parse_packet(): invalid packet {[hex(x) for x in list(packet_in)]}")
            return 0, b''

    @staticmethod
    def decode_floats(data: Union[bytes, bytearray]) -> List[float]:
        """
        Decode a received byte list, into a float list as specified by the bpl protocol

        Bytes are decoded into 32-bit floats.

        param data: bytes, but be divisible by 4.
        return: decoded list of floats
        """
        list_data = list(struct.unpack(str(int(len(data) / 4)) + "f", data))
        return list_data

    @staticmethod
    def get_gyro(packet_id: int, data_hex: bytes) -> Tuple[float, float, float]:
        if packet_id == PacketID.POSITION:
            k_gyro = 180.0
        elif packet_id == PacketID.VELOCITY:
            k_gyro = 2000.0
        elif packet_id == PacketID.ACCELERATION:
            k_gyro = 16.0
        else:
            logger.warning(f"get_gyro(): can not deal data with packet_id: {packet_id}.")
            return -1.1, -1.1, -1.1

        wxl = data_hex[0]
        wxh = data_hex[1]
        wyl = data_hex[2]
        wyh = data_hex[3]
        wzl = data_hex[4]
        wzh = data_hex[5]

        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        gyro_y = gyro_y/2 if packet_id == PacketID.POSITION else gyro_y
        return gyro_x, gyro_y, gyro_z


class PacketReader:
    """
    Packet Reader
    Helper class to read and decode incoming bytes and account for the incomplete packets.



    """
    incomplete_packets = b''

    def receive_bytes(self, data: bytes) -> List[Tuple[int, bytes]]:
        """
        Decodes packets.
        Accounts for reading incomplete bytes.

        param data: input bytes
        return: a list of decoded packets (Packet ID, data (in bytes))
        """
        # Receive data, and return a decoded packet
        packet_list = []
        encoded_packets, self.incomplete_packets = WITProtocol.packet_splitter(self.incomplete_packets + data)
        if encoded_packets:
            for encoded_packet in encoded_packets:
                decoded_packet = WITProtocol.parse_packet(encoded_packet)
                packet_list.append(decoded_packet)
        return packet_list