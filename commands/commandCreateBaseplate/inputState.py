from typing import Dict
from dataclasses import dataclass

@dataclass
class InputState:
    baseWidth: float
    baseLength: float
    xyClearance: float
    plateWidth: float
    plateLength: float

    plateType: str

    hasMagnetSockets: bool
    magnetSocketSize: float
    magnetSocketDepth: float

    hasGlueChannels: bool
    glueChannelWidth: float
    glueChannelDepth: float

    hasScrewHoles: bool
    screwHoleSize: float
    screwHeadSize: float

    hasPadding: bool
    paddingLeft: float
    paddingTop: float
    paddingRight: float
    paddingBottom: float

    extraBottomThickness: float
    verticalClearance: float

    hasConnectionHoles: bool
    connectionHoleSize: float

    tabLeftType: str
    tabRightType: str
    tabTopType: str
    tabBottomType: str
    tabClearance: float
