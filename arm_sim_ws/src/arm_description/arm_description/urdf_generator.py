#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
URDF 生成器：把「状态机式」的机械臂参数 (config/arm_config.yaml) 转换成 URDF。

================================================================================
状态机含义
================================================================================
    states 列表中的每一项 = 机械臂的一个「状态」（一个关节 + 它带的那根连杆）。
    列表从上到下的顺序 = 从基座到末端的装配顺序，也就是「轴序」。
    想改轴序？直接调整列表顺序即可。想增删关节？直接增删一项即可。

    每个状态里可以写：
        name/type       关节名、关节类型（revolute/continuous/prismatic/fixed）
        dh              一组 DH 参数（见下）
        origin/axis     或直接写关节偏置与转轴（URDF 约定，见下）
        limits          关节限位（下限/上限/力矩/速度）
        link            该关节带动的连杆（几何、材质、质量、惯量）

================================================================================
两种参数约定（YAML 顶层 convention 字段二选一）
================================================================================
    urdf   —— 直接给定关节 axis（转轴向量）和 origin（偏置 xyz + 姿态 rpy）。
              适合你心里已想好每个关节相对父坐标系的位姿。

    dh_std —— 标准 DH 参数 (a, alpha, d, theta)，自动换算成 URDF 的 origin/axis。
              theta 是关节变量，这里填初值（单位弧度）。
              T = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
              → URDF origin = (a, 0, d), rpy = (alpha, 0, 0)

    dh_mod —— 改进 DH 参数（Craig 约定），换算规则与标准 DH 不同。
              T = RotX(alpha) * TransX(a) * RotZ(theta) * TransZ(d)
              → URDF origin = (a, -d*sinα, d*cosα), rpy = (alpha, 0, 0)

    参数含义速查（DH 约定下）：
        a      —— 连杆长度：沿 X_{i-1} 方向，Z_{i-1} 到 Z_i 的距离（米）
        alpha  —— 连杆扭转：绕 X_{i-1} 轴，Z_{i-1} 转到 Z_i 的角度（弧度）
        d      —— 关节偏置：沿 Z_i 轴方向，X_{i-1} 到 X_i 的距离（米）
        theta  —— 关节角：绕 Z_{i-1} 轴，X_{i-1} 转到 X_i 的角度（弧度）

================================================================================
用法
================================================================================
    1) 命令行导出 URDF 文件：
       python3 urdf_generator.py config/arm_config.yaml -o urdf/arm.urdf
    2) 作为 Python 模块：
       from arm_description.urdf_generator import generate_urdf, load_config
       urdf_str = generate_urdf(load_config("config/arm_config.yaml"))
    3) ROS 内直接运行（控制台脚本，colcon build 后）：
       ros2 run arm_description generate_urdf \
           $(ros2 pkg prefix arm_description)/share/arm_description/config/arm_config.yaml
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Any, Dict, List, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


# ---------------------------------------------------------------------------
# 数值/字符串小工具
# ---------------------------------------------------------------------------

def _f(x: float) -> str:
    """把浮点数格式化为紧凑字符串，避免 URDF 出现科学计数法或多余 0。"""
    if isinstance(x, bool):
        return "true" if x else "false"
    return ("%.6f" % float(x)).rstrip("0").rstrip(".")


def _vec(values: List[float]) -> str:
    return " ".join(_f(v) for v in values)


def _normalize(axis: List[float]) -> List[float]:
    n = math.sqrt(sum(float(c) * float(c) for c in axis))
    if n < 1e-12:
        raise ValueError(f"关节轴向量模长为 0，无法归一化：{axis}")
    return [float(c) / n for c in axis]


# ---------------------------------------------------------------------------
# DH 参数换算
# ---------------------------------------------------------------------------

def dh_to_origin(dh: Dict[str, Any], convention: str) -> Tuple[List[float], List[float], List[float]]:
    """
    把一组 DH 参数转换为 URDF 关节 origin 的 (xyz, rpy, axis)。

    DH 约定下关节轴恒为 Z 轴（revolute 绕 Z 转、prismatic 沿 Z 移），
    所以 axis 固定返回 [0, 0, 1]；theta 是关节变量初值，不进入静态 origin。
    """
    a = float(dh.get("a", 0.0))
    alpha = float(dh.get("alpha", 0.0))
    d = float(dh.get("d", 0.0))

    if convention == "dh_std":
        # 标准 DH: T = RotZ(theta)·TransZ(d)·TransX(a)·RotX(alpha)
        # 静态部分 O = TransZ(d)·TransX(a)·RotX(alpha)
        xyz = [a, 0.0, d]
        rpy = [alpha, 0.0, 0.0]
    elif convention == "dh_mod":
        # 改进 DH: T = RotX(alpha)·TransX(a)·RotZ(theta)·TransZ(d)
        # 静态部分 O = RotX(alpha)·TransX(a)·TransZ(d)
        xyz = [a, -d * math.sin(alpha), d * math.cos(alpha)]
        rpy = [alpha, 0.0, 0.0]
    else:
        raise ValueError(f"未知 DH 约定：{convention}（应为 dh_std 或 dh_mod）")
    return xyz, rpy, [0.0, 0.0, 1.0]


# ---------------------------------------------------------------------------
# 连杆（link）生成
# ---------------------------------------------------------------------------

def _geometry(cfg: Dict[str, Any]) -> str:
    geom = cfg.get("geometry", {}) or {}
    shape = geom.get("shape", "box")
    if shape == "box":
        s = geom.get("size", [0.1, 0.1, 0.1])
        return f'      <geometry>\n        <box size="{_vec(s)}"/>\n      </geometry>\n'
    if shape == "cylinder":
        r = float(geom.get("radius", 0.05))
        l = float(geom.get("length", 0.1))
        return f'      <geometry>\n        <cylinder radius="{_f(r)}" length="{_f(l)}"/>\n      </geometry>\n'
    if shape == "sphere":
        r = float(geom.get("radius", 0.05))
        return f'      <geometry>\n        <sphere radius="{_f(r)}"/>\n      </geometry>\n'
    if shape == "mesh":
        fn = geom.get("filename")
        if not fn:
            raise ValueError("mesh 几何需要 filename 字段")
        scale = geom.get("scale", [1, 1, 1])
        return f'      <geometry>\n        <mesh filename="{fn}" scale="{_vec(scale)}"/>\n      </geometry>\n'
    raise ValueError(f"未知几何形状：{shape}（可选 box/cylinder/sphere/mesh）")


def _material(cfg: Dict[str, Any]) -> str:
    mat = cfg.get("material", {}) or {}
    if not mat:
        return ""
    if "file" in mat:  # 贴图材质
        name = mat.get("name", "tex")
        return (
            f'      <material name="{name}">\n'
            f'        <texture filename="{mat["file"]}"/>\n'
            f'      </material>\n'
        )
    r = float(mat.get("r", 0.6))
    g = float(mat.get("g", 0.6))
    b = float(mat.get("b", 0.6))
    a = float(mat.get("a", 1.0))
    name = mat.get("name", cfg.get("name", "link") + "_color")
    return (
        f'      <material name="{name}">\n'
        f'        <color rgba="{_vec([r, g, b, a])}"/>\n'
        f'      </material>\n'
    )


def _inertia(cfg: Dict[str, Any]) -> str:
    """按简单规则体（box/cylinder/sphere）解析惯量；假定质心在连杆坐标系原点。"""
    mass = float(cfg.get("mass", 0.0))

    if "inertia" in cfg:  # 用户显式给定惯量张量
        ii = cfg["inertia"]
        return (
            f'      <inertial>\n'
            f'        <mass value="{_f(mass)}"/>\n'
            f'        <inertia ixx="{_f(ii["ixx"])}" ixy="{_f(ii.get("ixy", 0.0))}" '
            f'ixz="{_f(ii.get("ixz", 0.0))}" iyy="{_f(ii["iyy"])}" '
            f'iyz="{_f(ii.get("iyz", 0.0))}" izz="{_f(ii["izz"])}"/>\n'
            f'      </inertial>\n'
        )

    if mass <= 0:
        return ""

    geom = cfg.get("geometry", {}) or {}
    shape = geom.get("shape", "box")

    if shape == "box":
        s = geom.get("size", [0.1, 0.1, 0.1])
        x, y, z = float(s[0]), float(s[1]), float(s[2])
        ixx = mass / 12.0 * (y * y + z * z)
        iyy = mass / 12.0 * (x * x + z * z)
        izz = mass / 12.0 * (x * x + y * y)
    elif shape == "cylinder":
        r = float(geom.get("radius", 0.05))
        l = float(geom.get("length", 0.1))
        ixx = iyy = mass / 12.0 * (3 * r * r + l * l)
        izz = mass / 2.0 * r * r
    else:  # sphere / mesh：按球近似
        r = float(geom.get("radius", 0.05))
        ixx = iyy = izz = 0.4 * mass * r * r

    return (
        f'      <inertial>\n'
        f'        <mass value="{_f(mass)}"/>\n'
        f'        <inertia ixx="{_f(ixx)}" ixy="0" ixz="0" '
        f'iyy="{_f(iyy)}" iyz="0" izz="{_f(izz)}"/>\n'
        f'      </inertial>\n'
    )


def _link_xml(name: str, cfg: Dict[str, Any]) -> str:
    lines = [f'  <link name="{name}">']
    if cfg.get("geometry"):
        view = {"name": name, "geometry": cfg.get("geometry"), "material": cfg.get("material")}
        lines.append("    <visual>")
        lines.append(_geometry(view))
        lines.append(_material(view))
        lines.append("    </visual>")
        lines.append("    <collision>")
        lines.append(_geometry(view))
        lines.append("    </collision>")
    else:
        # 无几何：给一个极小球体，保证 RViz/TF 有东西可显示
        lines.append('    <visual><geometry><sphere radius="0.001"/></geometry></visual>')
        lines.append('    <collision><geometry><sphere radius="0.001"/></geometry></collision>')
    lines.append(_inertia(cfg))
    lines.append("  </link>")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 关节（joint）生成
# ---------------------------------------------------------------------------

def _joint_xml(name, jtype, parent, child, xyz, rpy, axis, st):
    lines = [
        f'  <joint name="{name}" type="{jtype}">',
        f'    <parent link="{parent}"/>',
        f'    <child link="{child}"/>',
        f'    <origin xyz="{_vec(xyz)}" rpy="{_vec(rpy)}"/>',
    ]
    if jtype != "fixed":
        lines.append(f'    <axis xyz="{_vec(axis)}"/>')

    limits = st.get("limits") or {}
    if jtype in ("revolute", "prismatic"):
        lower = limits.get("lower", -math.pi)
        upper = limits.get("upper", math.pi)
        effort = limits.get("effort", 10.0)
        velocity = limits.get("velocity", 1.0)
        lines.append(
            f'    <limit lower="{_f(lower)}" upper="{_f(upper)}" '
            f'effort="{_f(effort)}" velocity="{_f(velocity)}"/>'
        )
    elif jtype == "continuous":
        effort = limits.get("effort", 10.0)
        velocity = limits.get("velocity", 1.0)
        lines.append(f'    <limit effort="{_f(effort)}" velocity="{_f(velocity)}"/>')
    lines.append("  </joint>")
    return "\n".join(lines)


def _fixed_joint_xml(parent, child, origin_cfg):
    origin_cfg = origin_cfg or {}
    xyz = origin_cfg.get("xyz", [0.0, 0.0, 0.0])
    rpy = origin_cfg.get("rpy", [0.0, 0.0, 0.0])
    return (
        f'  <joint name="{child}_joint" type="fixed">\n'
        f'    <parent link="{parent}"/>\n'
        f'    <child link="{child}"/>\n'
        f'    <origin xyz="{_vec(xyz)}" rpy="{_vec(rpy)}"/>\n'
        f'  </joint>'
    )


# ---------------------------------------------------------------------------
# Gazebo / ros2_control 支持
# ---------------------------------------------------------------------------

def _transmission_xml(joint_name: str) -> str:
    return (
        f'  <transmission name="{joint_name}_trans">\n'
        f'    <type>transmission_interface/SimpleTransmission</type>\n'
        f'    <joint name="{joint_name}">\n'
        f'      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n'
        f'    </joint>\n'
        f'    <actuator name="{joint_name}_motor">\n'
        f'      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n'
        f'      <mechanicalReduction>1</mechanicalReduction>\n'
        f'    </actuator>\n'
        f'  </transmission>'
    )


def _ros2_control_xml(joints: List[Tuple[str, Dict[str, Any]]]) -> str:
    lines = [
        '  <ros2_control name="GazeboSystem" type="system">',
        '    <hardware>',
        '      <plugin>gazebo_ros2_control/GazeboSystem</plugin>',
        '    </hardware>',
    ]
    for name, limits in joints:
        lower = float(limits.get("lower", -math.pi))
        upper = float(limits.get("upper", math.pi))
        lines.append(f'    <joint name="{name}">')
        lines.append('      <command_interface name="position">')
        lines.append(f'        <param name="min">{_f(lower)}</param>')
        lines.append(f'        <param name="max">{_f(upper)}</param>')
        lines.append('      </command_interface>')
        lines.append('      <state_interface name="position"/>')
        lines.append('      <state_interface name="velocity"/>')
        lines.append('    </joint>')
    lines.append('  </ros2_control>')
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 主生成函数
# ---------------------------------------------------------------------------

def generate_urdf(config: Dict[str, Any]) -> str:
    robot_name = config.get("robot_name", "arm")
    convention = config.get("convention", "urdf")
    enable_gazebo = bool(config.get("enable_gazebo", False))

    states = config.get("states") or []
    if not states:
        raise ValueError("配置里没有 states 列表（至少需要一个关节/连杆状态）")

    base = config.get("base_link") or {}
    base_name = base.get("name", "base_link")
    tool = config.get("tool_link")

    links_xml: List[str] = []
    joints_xml: List[str] = []
    transmissions_xml: List[str] = []
    ros2_control_joints: List[Tuple[str, Dict[str, Any]]] = []

    links_xml.append(_link_xml(base_name, base))

    parent = base_name
    for i, st in enumerate(states, start=1):
        joint_name = st.get("name", f"joint_{i}")
        link_cfg = st.get("link") or {}
        link_name = link_cfg.get("name", f"link_{i}")
        jtype = st.get("type", "revolute")

        if st.get("dh"):
            xyz, rpy, axis = dh_to_origin(st["dh"], convention)
        else:
            origin = st.get("origin") or {}
            xyz = origin.get("xyz", [0.0, 0.0, 0.0])
            rpy = origin.get("rpy", [0.0, 0.0, 0.0])
            axis = st.get("axis", [0.0, 0.0, 1.0])
        axis = _normalize(axis)

        joints_xml.append(_joint_xml(joint_name, jtype, parent, link_name, xyz, rpy, axis, st))
        links_xml.append(_link_xml(link_name, link_cfg))

        if enable_gazebo and jtype in ("revolute", "prismatic", "continuous"):
            transmissions_xml.append(_transmission_xml(joint_name))
            ros2_control_joints.append((joint_name, st.get("limits") or {}))

        parent = link_name

    if tool:
        tool_name = tool.get("name", "tool0")
        links_xml.append(_link_xml(tool_name, tool))
        joints_xml.append(_fixed_joint_xml(parent, tool_name, tool.get("origin")))

    parts: List[str] = ['<?xml version="1.0"?>', f'<robot name="{robot_name}">', ""]
    parts += ['  <!-- ===================== 连杆 links ===================== -->', ""]
    parts += links_xml
    parts += ["", '  <!-- ===================== 关节 joints ===================== -->', ""]
    parts += joints_xml

    if enable_gazebo:
        parts += ["", '  <!-- ===================== Gazebo / ros2_control ===================== -->', ""]
        parts += transmissions_xml
        parts.append(_ros2_control_xml(ros2_control_joints))
        parts.append('  <gazebo>')
        parts.append('    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so"/>')
        parts.append('  </gazebo>')

    parts.append('</robot>')
    return "\n".join(parts)


# ---------------------------------------------------------------------------
# 配置加载 / CLI
# ---------------------------------------------------------------------------

def load_config(path: str) -> Dict[str, Any]:
    if yaml is None:
        raise ImportError("缺少 PyYAML，请先安装：pip install pyyaml 或 sudo apt install python3-yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def main(argv: List[str] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(description="把状态机式机械臂配置转换成 URDF")
    p.add_argument("config", help="YAML 配置文件路径")
    p.add_argument("-o", "--output", default=None, help="输出 URDF 文件路径（默认打印到标准输出）")
    args = p.parse_args(argv)

    config = load_config(args.config)
    urdf = generate_urdf(config)

    if args.output:
        outdir = os.path.dirname(args.output) or "."
        os.makedirs(outdir, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(urdf + "\n")
        print(f"[OK] 已生成 {args.output}（{len(urdf)} 字符）")
    else:
        print(urdf)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
