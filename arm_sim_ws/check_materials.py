#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "src", "arm_description"))

from arm_description.urdf_generator import generate_urdf, load_config  # noqa: E402

cfg_path = os.path.join(HERE, "src", "arm_description", "config", "arm_config.yaml")
urdf = generate_urdf(load_config(cfg_path))
names = re.findall(r'<material name="([^"]+)"', urdf)
print("material names:", names)
print("unique:", len(names) == len(set(names)))
