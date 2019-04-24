#!/usr/bin/env python
"""
Conversions between basic control systems

"""
import numpy as np
from systemcontrol.basic_systems import *


class Uni_Diffeo(SingleUnicycle):
    """ apply near identity diffeomorphism to control
    unicycle using cartesian control"""
    def __init__(self):
        pass

    def u(self):
        pass