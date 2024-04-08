#!/usr/bin/env python
import numpy as np
import pandas as pd
from utils.signals import *

class PIDController():
    def __init__(self, config):
        self._config = config

    def compute_control_cmd(self, ref_v, controller_input):
        preview_length = self._config['preview_length']
        current_v = controller_input[V]
        error = ref_v[min(preview_length,len(ref_v))-1] - current_v
        return error * self._config['propotional_gain']