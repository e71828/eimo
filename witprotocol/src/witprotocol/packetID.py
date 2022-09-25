from enum import IntEnum


class PacketID(IntEnum):
    """
    Class containing BPL packet IDs.
    Look in The Serial Protocol Document for comprehensive details on packet ids.

    Packet IDs:

    """

    VELOCITY = 0x52
    "1 float - Describes the velocity of a device. Degrees/s for angle."
    POSITION = 0x53
    "1 float - Describes the position of a device. In degrees."
    ACCELERATION = 0x51
    "1 float - Describes the ACCELERATION of a device."
