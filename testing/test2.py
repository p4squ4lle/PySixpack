#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  7 12:47:13 2019

@author: pasinger
"""

from Sixpack2Controller import Sixpack2Controller

c = Sixpack2Controller()

c.get_unit_info()

c[0].get_pos()
c[0].get_vel()
c[0].rotate(5)
c[0].start_ramp(1000)

xaxis = c[0]
