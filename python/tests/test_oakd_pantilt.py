# -*- coding: utf-8 -*-
"""
Created on Thu May 13 22:16:31 2021

@author: LattePanda
"""

import move_oak_d as oakd
import time

oakd.initialize()
oakd.initHome()
oakd.setYaw(45)
time.sleep(1)
oakd.setYaw(135)
time.sleep(1)
oakd.setPitch(45)
time.sleep(1)
oakd.shutdown()
