#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""本地校验脚本：验证 urdf_generator 能产出合法、连通的 URDF（不依赖 ROS2）。

运行：
    python validate_generator.py
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "src", "arm_description"))

from arm_description.urdf_generator import generate_urdf, dh_to_origin  # noqa: E402


def sample_config(convention="dh_std"):
    """与 config/arm_config.yaml 等价的 Python 字典（便于在缺少 PyYAML 时校验）。"""
    if convention == "urdf":
        states = []
        for i in range(1, 7):
            states.append({
                "name": f"joint_{i}",
                "type": "revolute",
                "axis": [0, 0, 1],
                "origin": {"xyz": [0, 0, 0.1 * i], "rpy": [0, 0, 0]},
                "limits": {"lower": -3.14, "upper": 3.14, "effort": 10.0, "velocity": 2.0},
                "link": {"name": f"link_{i}", "geometry": {"shape": "cylinder", "radius": 0.05, "length": 0.1},
                         "material": {"r": 0.5, "g": 0.5, "b": 0.5, "a": 1.0}, "mass": 1.0},
            })
        return {"robot_name": "test_arm", "convention": "urdf",
                "base_link": {"name": "base_link", "geometry": {"shape": "box", "size": [0.2, 0.2, 0.06]},
                              "mass": 5.0},
                "tool_link": {"name": "tool0", "origin": {"xyz": [0, 0, 0.1]},
                              "geometry": {"shape": "sphere", "radius": 0.02}, "mass": 0.1},
                "states": states}

    dh = [
        (0.0, 1.5708, 0.35),
        (0.35, 0.0, 0.0),
        (0.0, 1.5708, 0.0),
        (0.0, -1.5708, 0.35),
        (0.0, 1.5708, 0.0),
        (0.0, 0.0, 0.10),
    ]
    states = []
    for i, (a, alpha, d) in enumerate(dh, start=1):
        states.append({
            "name": f"joint_{i}",
            "type": "revolute",
            "dh": {"a": a, "alpha": alpha, "d": d, "theta": 0.0},
            "limits": {"lower": -3.14159, "upper": 3.14159, "effort": 100.0, "velocity": 2.0},
            "link": {"name": f"link_{i}", "geometry": {"shape": "cylinder", "radius": 0.06, "length": 0.15},
                     "material": {"r": 0.2, "g": 0.5, "b": 0.8, "a": 1.0}, "mass": 2.0},
        })
    return {"robot_name": "test_arm", "convention": convention,
            "base_link": {"name": "base_link", "geometry": {"shape": "box", "size": [0.2, 0.2, 0.06]},
                          "mass": 5.0},
            "tool_link": {"name": "tool0", "origin": {"xyz": [0, 0, 0.1]},
                          "geometry": {"shape": "sphere", "radius": 0.02}, "mass": 0.1},
            "states": states}


def validate(urdf, convention, enable_gazebo=False):
    root = ET.fromstring(urdf)  # 若 XML 非法会直接抛异常

    links = [e.get("name") for e in root.findall("link")]
    joints = root.findall("joint")

    errors = []
    # 1) 名字唯一
    if len(links) != len(set(links)):
        errors.append("存在重名 link")
    # 2) 关节的 parent/child 均存在
    link_set = set(links)
    parent_of = {}
    for j in joints:
        jn = j.get("name")
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        if p not in link_set:
            errors.append(f"关节 {jn} 的 parent {p} 不存在")
        if c not in link_set:
            errors.append(f"关节 {jn} 的 child {c} 不存在")
        parent_of[c] = p

    # 3) 单根 + 全连通
    roots = [l for l in links if l not in parent_of]
    if len(roots) != 1:
        errors.append(f"根节点数量应为 1，实际 {roots}")
    else:
        seen = set()
        frontier = [roots[0]]
        children = {}
        for c, p in parent_of.items():
            children.setdefault(p, []).append(c)
        while frontier:
            n = frontier.pop()
            if n in seen:
                continue
            seen.add(n)
            frontier.extend(children.get(n, []))
        if seen != link_set:
            errors.append(f"存在不连通 link: {link_set - seen}")

    # 4) 旋转/移动关节必须有单位轴与 limit
    for j in joints:
        jn = j.get("name")
        jt = j.get("type")
        if jt in ("revolute", "continous", "continuous", "prismatic"):
            axis = j.find("axis")
            if axis is None:
                errors.append(f"关节 {jn} 缺少 axis")
            else:
                v = [float(x) for x in axis.get("xyz").split()]
                norm = math.sqrt(sum(x * x for x in v))
                if abs(norm - 1.0) > 1e-6:
                    errors.append(f"关节 {jn} axis 未归一化: {v} (|a|={norm})")
            if j.find("limit") is None:
                errors.append(f"关节 {jn} 缺少 limit")

    # 5) Gazebo 模式下必须有 ros2_control 与 transmission
    if enable_gazebo:
        if root.find("ros2_control") is None:
            errors.append("enable_gazebo 但缺少 <ros2_control>")
        if root.findall("transmission") == []:
            errors.append("enable_gazebo 但缺少 <transmission>")

    # 6) 材质名必须唯一（同名不同色会让 RViz 颜色错乱）
    mats = [e.get("name") for e in root.iter("material")]
    if len(mats) != len(set(mats)):
        errors.append(f"存在重名 material: {mats}")

    return {
        "convention": convention,
        "links": len(links),
        "joints": len(joints),
        "root": roots[0] if roots else None,
        "errors": errors,
    }


def check_dh_math():
    """校验标准/改进 DH 换算的数值。"""
    cases = [
        ("dh_std", {"a": 0.0, "alpha": math.pi / 2, "d": 0.35},
         [0.0, 0.0, 0.35], [math.pi / 2, 0.0, 0.0]),
        ("dh_std", {"a": 0.35, "alpha": 0.0, "d": 0.0},
         [0.35, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ("dh_mod", {"a": 0.0, "alpha": math.pi / 2, "d": 0.35},
         [0.0, -0.35, 0.0], [math.pi / 2, 0.0, 0.0]),
    ]
    errs = []
    for conv, dh, exp_xyz, exp_rpy in cases:
        xyz, rpy, axis = dh_to_origin(dh, conv)
        for got, exp, tag in ((xyz, exp_xyz, "xyz"), (rpy, exp_rpy, "rpy")):
            if any(abs(g - e) > 1e-6 for g, e in zip(got, exp)):
                errs.append(f"{conv} {dh} 的 {tag} 期望 {exp} 实际 {got}")
        if axis != [0.0, 0.0, 1.0]:
            errs.append(f"{conv} 的 axis 不是 [0,0,1]")
    return errs


def main():
    ok = True

    # A) DH 数学自检
    errs = check_dh_math()
    print("[A] DH 换算自检:", "PASS" if not errs else "FAIL")
    for e in errs:
        print("    -", e)
    ok = ok and not errs

    # B) 三种约定各生成一次
    for conv in ("dh_std", "dh_mod", "urdf"):
        cfg = sample_config(conv)
        urdf = generate_urdf(cfg)
        r = validate(urdf, conv)
        status = "PASS" if not r["errors"] else "FAIL"
        print(f"[B] {conv:7s} links={r['links']} joints={r['joints']} root={r['root']} -> {status}")
        for e in r["errors"]:
            print("    -", e)
        ok = ok and not r["errors"]

    # C) Gazebo 模式
    cfg = sample_config("dh_std")
    cfg["enable_gazebo"] = True
    urdf = generate_urdf(cfg)
    r = validate(urdf, "dh_std+gazebo", enable_gazebo=True)
    status = "PASS" if not r["errors"] else "FAIL"
    print(f"[C] gazebo  links={r['links']} joints={r['joints']} -> {status}")
    for e in r["errors"]:
        print("    -", e)
    ok = ok and not r["errors"]

    # D) 若 PyYAML 可用，读取真实 YAML 验证
    try:
        import yaml  # noqa: F401
        from arm_description.urdf_generator import load_config
        real = os.path.join(HERE, "src", "arm_description", "config", "arm_config.yaml")
        cfg = load_config(real)
        urdf = generate_urdf(cfg)
        r = validate(urdf, "real-yaml")
        status = "PASS" if not r["errors"] else "FAIL"
        print(f"[D] 真实 YAML links={r['links']} joints={r['joints']} -> {status}")
        for e in r["errors"]:
            print("    -", e)
        ok = ok and not r["errors"]
    except ImportError:
        print("[D] 真实 YAML 校验: SKIP（未安装 PyYAML）")

    print()
    print("RESULT:", "ALL PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
