"""为仿真生成 MuJoCo 物理模型（MJCF）的上层构建器。

职责与位置：本模块把仓库自带的基线 MJCF 资源（``assets/rebotarm_base.xml``）在内存中改写成
"带物理属性"的完整模型树，供仿真执行器、无头物理检查与查看器加载。它只做 XML 结构层面的
加工，不加载 MuJoCo 运行库、不涉及 ROS、也绝不接触真实硬件，因此可以在没有 ROS 环境时单独
测试；仿真启动自然不会开启任何硬件通道。

加工步骤（见 ``build_physics_profile_tree``）：
  1. 把 ``mesh``/``texture`` 的相对路径改为绝对路径，保证生成文件可被任意工作目录加载；
  2. 把原有的网格几何体降级为纯显示（不参与碰撞、不计入质量）；
  3. 追加与本体几何一致的图元碰撞体，并给指垫设置抓取用接触参数；
  4. 按电机档案写入位置执行器的控制范围、力限与增益；
  5. 排除相邻刚体（含两只手指之间）的接触对，避免自碰撞与求解器发散；
  6. 写入/删除关键帧（``keyframe``），供复位到指定姿态使用。

重要约定：本模块中的关节限位与力矩上限只用于仿真，绝不能当作真实硬件限位使用
（``UPSTREAM_ARM_MOTOR_PROFILES`` 尤其如此）。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from .resource_paths import package_resource


# 基线物理模型与抓取场景的默认资源路径（本包 assets 目录下）。
# 抓取场景通过 <include> 引用物理模型，因此两者必须成对生成。
ASSET_DIR = Path(__file__).resolve().parent / "assets"
DEFAULT_GRIPPER_XML = ASSET_DIR / "rebotarm_base.xml"
DEFAULT_GRASP_SCENE_XML = ASSET_DIR / "rebotarm_grasp_scene.xml"


def _default_mesh_dir() -> Path:
    # 网格文件（STL）随运动规划配置包一起安装；基线 XML 里只写文件名，
    # 找不到同名文件时回退到该共享目录，保证源码树与已安装环境都能解析资源。
    return package_resource("rebotarm_moveit_config", "meshes")


@dataclass(frozen=True)
class CollisionGeom:
    """一个待追加到指定刚体上的 MuJoCo 碰撞图元描述。

    字段（全部为 MuJoCo 属性字符串，不是数值）：
      body: 图元挂载到的刚体名；找不到该刚体时整个条目被跳过。
      name: 图元名称，用于去重（同名图元已存在则不重复添加）。
      geom_type: 图元类型，``cylinder``/``box`` 等；尺寸含义随类型变化。
      size: 尺寸参数，依次为半径/半长等，单位米（圆柱为 "半径 半高"，长方体为半边长）。
      pos: 相对父刚体坐标系的平移，单位米，默认 "0 0 0"。
      rgba: 显示颜色，四个分量为 0~1 的红绿蓝与透明度，默认 "0 0.7 0.1 0.24"（半透明绿）。
      friction: 滑动/扭转/滚动摩擦系数；指垫单独加大以稳定抓取。
      condim: 接触维度，4 表示含扭转摩擦（点接触 + 摩擦锥），指垫用它模拟橡胶垫。
      solref: 接触求解器参考时间常数与阻尼比；数值越小接触越"硬"。
      solimp: 接触求解器阻抗参数（最小/最大阻抗与过渡宽度），影响穿透深度。
    """

    body: str
    name: str
    geom_type: str
    size: str
    pos: str = "0 0 0"
    rgba: str = "0 0.7 0.1 0.24"
    friction: str | None = None
    condim: str | None = None
    solref: str | None = None
    solimp: str | None = None


@dataclass(frozen=True)
class MotorProfile:
    """一个关节位置执行器（MuJoCo ``position`` 执行器）的整定参数。

    字段：
      joint: 被驱动的关节名（必须与 XML 中关节名一致）。
      ctrlrange: 控制量允许范围（"下限 上限"），角度关节单位为弧度，手指为米；
                 超出会被执行器截断。
      forcerange: 执行器输出力/力矩限幅（"下限 上限"），转动关节单位牛·米，手指为牛；
                 它同时被仿真力限检查当作比较基准。
      kp: 位置增益（刚度），越大跟随越快、越容易超调与振荡。
      kv: 速度增益（阻尼），可选；为空则不写入该属性。
    """

    joint: str
    ctrlrange: str
    forcerange: str
    kp: str
    kv: str | None = None


# 本体各连杆的碰撞图元表：网格几何体在 MuJoCo 中一律不参与碰撞，
# 因此这里用圆柱/长方体近似真实外形（group=4 表示"碰撞体"分组）。
# 数值均为本仓库对这款机械臂的建模结果，单位米；仅用于仿真碰撞，
# 不代表真实结构尺寸的验收依据。
COLLISION_GEOMS = [
    CollisionGeom("base_link", "base_link_collision", "cylinder", "0.085 0.045", "0 0 0.045"),
    CollisionGeom("link1", "link1_collision", "cylinder", "0.05 0.07", "0 0 0.025"),
    CollisionGeom("link2", "link2_collision", "box", "0.145 0.05 0.045", "-0.13 0 -0.025"),
    CollisionGeom("link3", "link3_collision", "box", "0.135 0.045 0.04", "0.12 -0.025 -0.025"),
    CollisionGeom("link4", "link4_collision", "box", "0.08 0.04 0.04", "0.055 -0.04 -0.02"),
    CollisionGeom("link5", "link5_collision", "cylinder", "0.045 0.055", "0 0 0.035"),
    CollisionGeom("link6", "link6_collision", "cylinder", "0.045 0.065", "0 0 0.06"),
    CollisionGeom("end_link", "gripper_base_collision", "box", "0.06 0.045 0.035"),
    # 指垫：摩擦系数取 1.2、condim=4、较硬的接触参数，用于让夹取物体时
    # 产生足够的切向摩擦与扭转阻力，减少物体滑落。
    CollisionGeom(
        "left_finger_link",
        "left_finger_pad_collision",
        "box",
        "0.014 0.006 0.035",
        "0.005 -0.052 0",
        friction="1.2 0.02 0.001",
        condim="4",
        solref="0.01 1",
        solimp="0.9 0.95 0.001",
    ),
    CollisionGeom(
        "right_finger_link",
        "right_finger_pad_collision",
        "box",
        "0.014 0.006 0.035",
        "0.005 0.052 0",
        friction="1.2 0.02 0.001",
        condim="4",
        solref="0.01 1",
        solimp="0.9 0.95 0.001",
    ),
]


# 仿真默认电机档案：关节 ctrlrange 与基线 XML 的 joint range 一致，
# forcerange 与 XML 的 actuatorfrcrange 一致，kp/kv 为位置执行器的增益与阻尼。
# joint1~3 力矩 ±27 N·m（kp=270, kv=24），joint4~6 为 ±7 N·m（kp=70, kv=10）。
# 手指只有单侧执行器 "left_finger"：控制量即单侧滑移量（米），
# 两侧经 XML 的 equality 约束联动，故 0.035 对应约 0.07 m 总开口。
MOTOR_PROFILES = [
    MotorProfile("joint1", "-2.8 2.8", "-27 27", "270", "24"),
    MotorProfile("joint2", "-3.14 0.02", "-27 27", "270", "24"),
    MotorProfile("joint3", "-3.14 0.02", "-27 27", "270", "24"),
    MotorProfile("joint4", "-1.87 1.57", "-7 7", "70", "10"),
    MotorProfile("joint5", "-1.57 1.57", "-7 7", "70", "10"),
    MotorProfile("joint6", "-3.14 3.14", "-7 7", "70", "10"),
    MotorProfile("left_finger", "0 0.045", "-20 20", "600", "60"),
]

# 仅用于仿真的标定候选档案：沿用上游对 joint4~6 的力矩上限（±12.5 N·m），
# 其余关节与当前位置执行器控制器、夹爪契约保持一致。
# 它与上一份档案的差别只体现在仿真力矩限幅上，绝不能用于真实硬件限位。
UPSTREAM_ARM_MOTOR_PROFILES = [
    MotorProfile("joint1", "-2.8 2.8", "-27 27", "270", "24"),
    MotorProfile("joint2", "-3.14 0.02", "-27 27", "270", "24"),
    MotorProfile("joint3", "-3.14 0.02", "-27 27", "270", "24"),
    MotorProfile("joint4", "-1.87 1.57", "-12.5 12.5", "70", "10"),
    MotorProfile("joint5", "-1.57 1.57", "-12.5 12.5", "70", "10"),
    MotorProfile("joint6", "-3.14 3.14", "-12.5 12.5", "70", "10"),
    MotorProfile("left_finger", "0 0.045", "-20 20", "600", "60"),
]


# 需要禁用的接触对（顺序无关）：相邻连杆在结构上本就贴合或嵌套，
# 若不排除，求解器会把正常装配间隙判为穿透而产生虚假接触力。
# 两只手指之间同样排除，避免闭合时互推导致宽度不可控。
ADJACENT_BODY_EXCLUDES = [
    ("base_link", "link1"),
    ("link1", "link2"),
    ("link2", "link3"),
    ("link3", "link4"),
    ("link4", "link5"),
    ("link5", "link6"),
    ("link6", "end_link"),
    ("end_link", "left_finger_link"),
    ("end_link", "right_finger_link"),
    ("left_finger_link", "right_finger_link"),
]


def build_physics_profile_tree(
    source_xml: Path = DEFAULT_GRIPPER_XML,
    *,
    include_keyframes: bool = True,
    motor_profiles: list[MotorProfile] = MOTOR_PROFILES,
) -> ET.ElementTree:
    """在内存中构建完整物理模型树（不落盘）。

    参数：
      source_xml: 基线 MJCF 路径；两侧手指的 equality 联动约束即来自该文件。
      include_keyframes: True 表示写入 zero/home/safe_home 三个关键帧；
                         False 表示删除已有 keyframe 节点——抓取场景
                         （rebotarm_grasp_scene.xml）自带更大的初始位形，
                         与其 include 进来的关键帧长度冲突，必须去掉。
      motor_profiles: 执行器整定档案，决定控制范围、力限与增益。
    返回：加工后的 XML 元素树。
    注意：本函数只改 XML 属性，不校验数值合法性；生成物仅供仿真加载。
    """
    source_xml = source_xml.resolve()
    tree = ET.parse(source_xml)
    root = tree.getroot()

    _make_asset_files_absolute(root, source_xml.parent)
    _mark_mesh_geoms_as_visual(root)
    _add_collision_geoms(root)
    _tune_actuators(root, motor_profiles)
    _add_contact_excludes(root)
    if include_keyframes:
        _add_keyframes(root)
    else:
        _remove_keyframes(root)
    return tree


def write_physics_profile(
    output_xml: Path,
    source_xml: Path = DEFAULT_GRIPPER_XML,
    *,
    include_keyframes: bool = True,
    motor_profiles: list[MotorProfile] = MOTOR_PROFILES,
) -> Path:
    """生成物理模型 XML 文件，返回解析后的绝对输出路径。

    会创建输出目录；写入时使用 UTF-8 且带 XML 声明，并做缩进美化，
    便于人工比对生成结果。
    """
    output_xml = output_xml.resolve()
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = build_physics_profile_tree(
        source_xml,
        include_keyframes=include_keyframes,
        motor_profiles=motor_profiles,
    )
    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def write_grasp_scene_profile(
    output_xml: Path,
    robot_xml: Path,
    source_scene_xml: Path = DEFAULT_GRASP_SCENE_XML,
) -> Path:
    """生成抓取场景 XML：把场景模板中的 <include> 指向本次生成的机器人模型。

    背景：场景模板里写的是相对路径的基线模型，若直接使用，MuJoCo 加载到的
    是未经物理加工的版本；这里改写为刚刚生成的 robot_xml 绝对路径，
    保证抓取基准测试用到的碰撞体、执行器与关键帧与主模型完全一致。
    """
    output_xml = output_xml.resolve()
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(source_scene_xml.resolve())
    root = tree.getroot()
    include = root.find("include")
    if include is None:
        include = ET.SubElement(root, "include")
    include.set("file", str(robot_xml.resolve()))
    ET.indent(tree, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    return output_xml


def xml_asset_references_are_readable(xml_path: Path) -> bool:
    """检查 XML 引用的网格/贴图文件是否都能找到。

    供启动前自检使用：文件不存在或 XML 解析失败都返回 False，
    避免等到 MuJoCo 加载时才报错。相对路径按 XML 所在目录解析。
    """
    xml_path = Path(xml_path)
    if not xml_path.exists():
        return False
    try:
        root = ET.parse(xml_path).getroot()
    except ET.ParseError:
        return False
    for element in list(root.findall(".//mesh")) + list(root.findall(".//texture")):
        file_name = element.get("file")
        if not file_name:
            continue
        asset_path = Path(file_name)
        if not asset_path.is_absolute():
            asset_path = xml_path.parent / asset_path
        if not asset_path.exists():
            return False
    return True


def _make_asset_files_absolute(root: ET.Element, asset_dir: Path) -> None:
    # 网格先按 XML 所在目录查找；找不到同名文件时回退到本仓库随运动规划
    # 配置包安装的网格共享目录（基线 XML 只写了文件名）。
    # 贴图不做回退，只按 XML 目录解析。
    mesh_dir = _default_mesh_dir()
    for mesh in root.findall(".//mesh"):
        file_name = mesh.get("file")
        if not file_name:
            continue
        asset_path = Path(file_name)
        if not asset_path.is_absolute():
            local_asset = asset_dir / asset_path
            if not local_asset.exists():
                local_asset = mesh_dir / asset_path.name
            mesh.set("file", str(local_asset.resolve()))
    for texture in root.findall(".//texture"):
        file_name = texture.get("file")
        if not file_name:
            continue
        asset_path = Path(file_name)
        if not asset_path.is_absolute():
            texture.set("file", str((asset_dir / asset_path).resolve()))


def _mark_mesh_geoms_as_visual(root: ET.Element) -> None:
    # 基线由 URDF 转换而来，网格几何体默认参与碰撞；转成纯显示：
    # contype/conaffinity 置 0 表示不与任何分组产生接触，density=0 使质量
    # 只由显式 inertial 提供（编译器的 inertiafromgeom 不会把显示网格算进去），
    # group=1 只影响查看器里的显示分组。缺少名称时按序号补 "_visual" 后缀，
    # 避免与随后追加的碰撞图元重名。
    visual_index = 0
    for geom in root.findall(".//geom"):
        if geom.get("type") != "mesh":
            continue
        visual_index += 1
        if "name" not in geom.attrib:
            mesh_name = geom.get("mesh", f"mesh_{visual_index}")
            geom.set("name", f"{mesh_name}_visual")
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("group", "1")
        geom.set("density", "0")


def _add_collision_geoms(root: ET.Element) -> None:
    # 按刚体名建立索引；目标刚体不存在，或同名图元已存在（幂等重跑）时跳过。
    # group=4 标记为碰撞分组，查看器里与显示网格分开显示。
    bodies = {body.get("name"): body for body in root.findall(".//body") if body.get("name")}
    for spec in COLLISION_GEOMS:
        body = bodies.get(spec.body)
        if body is None or body.find(f"./geom[@name='{spec.name}']") is not None:
            continue
        geom = ET.SubElement(body, "geom")
        geom.set("name", spec.name)
        geom.set("type", spec.geom_type)
        geom.set("pos", spec.pos)
        geom.set("size", spec.size)
        geom.set("rgba", spec.rgba)
        geom.set("group", "4")
        if spec.friction is not None:
            geom.set("friction", spec.friction)
        if spec.condim is not None:
            geom.set("condim", spec.condim)
        if spec.solref is not None:
            geom.set("solref", spec.solref)
        if spec.solimp is not None:
            geom.set("solimp", spec.solimp)


def _tune_actuators(root: ET.Element, motor_profiles: list[MotorProfile]) -> None:
    # 只处理位置执行器；档案里没有的关节保持原样。
    # 手指执行器对外统一叫 "gripper"（适配器按该名查表），其余关节执行器
    # 与关节同名。ctrllimited/forcelimited 置 true 使 ctrlrange/forcerange 生效，
    # 这是仿真不会输出超限力矩的关键开关。
    profiles = {profile.joint: profile for profile in motor_profiles}
    for actuator in root.findall(".//actuator/position"):
        joint = actuator.get("joint")
        if joint not in profiles:
            continue
        profile = profiles[joint]
        actuator.set("name", "gripper" if joint == "left_finger" else joint)
        actuator.set("ctrllimited", "true")
        actuator.set("ctrlrange", profile.ctrlrange)
        actuator.set("forcelimited", "true")
        actuator.set("forcerange", profile.forcerange)
        actuator.set("kp", profile.kp)
        if profile.kv is not None:
            actuator.set("kv", profile.kv)


def _add_contact_excludes(root: ET.Element) -> None:
    # contact 节点不存在时创建；已存在的排除对（正反顺序都算）不重复添加。
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    existing = {
        (exclude.get("body1"), exclude.get("body2"))
        for exclude in contact.findall("exclude")
    }
    for body1, body2 in ADJACENT_BODY_EXCLUDES:
        if (body1, body2) in existing or (body2, body1) in existing:
            continue
        exclude = ET.SubElement(contact, "exclude")
        exclude.set("body1", body1)
        exclude.set("body2", body2)


def _add_keyframes(root: ET.Element) -> None:
    """写入三个具名关键帧（已存在则保持原样，保证幂等）。

    三个关键帧的 qpos 都是 8 维（6 个转动关节 + 左、右手指两个滑动关节），
    ctrl 都是 7 维（与执行器数量一致，手指只写左手指）。
    zero/home 用于无头物理检查与查看器的初始位形；
    safe_home 与运动规划使用的同名姿态保持一致。
    """
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")
    # zero：关节接近零位（joint2/joint3 略作弯曲，避免完全奇异），
    # 张开度 2×0.035=0.07 m。
    if keyframe.find("./key[@name='zero']") is None:
        key = ET.SubElement(keyframe, "key")
        key.set("name", "zero")
        key.set("qpos", "0 -0.4 -1.0 0.4 0 0 0.035 -0.035")
        key.set("ctrl", "0 -0.4 -1.0 0.4 0 0 0.035")
    # home：折叠更深的待机位形，张开度 2×0.04=0.08 m。
    if keyframe.find("./key[@name='home']") is None:
        key = ET.SubElement(keyframe, "key")
        key.set("name", "home")
        key.set("qpos", "0 -0.8 -1.2 0.6 0 0 0.04 -0.04")
        key.set("ctrl", "0 -0.8 -1.2 0.6 0 0 0.04")
    # 与运动规划配置里名为 "safe_home" 的具名状态、以及硬件管理器中
    # _SAFE_HOME_JOINT_POSITIONS 保持同步：驱动器停靠姿态，沿基座 -Y 方向
    # 面向可视工作台。保持同步是为了让同一个姿态名在仿真与真机上含义一致。
    # 注意 joint3 使用的是 -1° 的弧度值（-0.017453292519943295 rad），并非 0。
    if keyframe.find("./key[@name='safe_home']") is None:
        key = ET.SubElement(keyframe, "key")
        key.set("name", "safe_home")
        key.set("qpos", "0 0 -0.017453292519943295 0 0 0 0.04 -0.04")
        key.set("ctrl", "0 0 -0.017453292519943295 0 0 0 0.04")


def _remove_keyframes(root: ET.Element) -> None:
    # 抓取场景自带位形更长的关键帧，与其 include 进来的机器人模型（qpos 长度不同）
    # 冲突，因此在生成给场景使用的机器人模型时需要整体移除。
    keyframe = root.find("keyframe")
    if keyframe is not None:
        root.remove(keyframe)
