"""定位包内声明的资源文件，同时兼容「源码树内直接运行」与「安装后运行」。

本模块是仿真包的基础设施：仿真模型、固件参考与标定文件在开发时可能直接位于
工作区的源码树中，在 `colcon` 安装后则位于对应包的 share 目录下。这里提供唯一
的定位入口，避免各调用方自行拼路径而出现两套不一致的查找规则。

本模块不涉及真实硬件接口，也不发布/订阅任何 ROS 话题、服务或动作；调用方按
「包名 + 包内相对路径」请求资源，得到绝对路径后自行读取。
"""

from pathlib import Path


def package_resource(package: str, relative: str) -> Path:
    """按「源码树优先、安装空间兜底」的顺序定位一个包内资源。

    参数：
        package: 资源所属的包名，仅作为目录名参与拼接（例如模型与网格的归属包）。
        relative: 相对于该包根目录的相对路径，例如 ``meshes`` 或
            ``config/xxx.yaml``；为空字符串时表示包根目录本身。

    返回：
        资源在文件系统中的绝对路径；若命中源码树返回源码树路径，否则返回安装
        空间路径。

    异常：
        FileNotFoundError: 两处都不存在该资源时抛出，异常携带的参数是安装空间
            中的期望路径，便于区分「未安装/未拷贝」与「路径写错」两种情况。
    """
    # parents[0] 是本包的内层模块目录，parents[1] 是包的工程目录，parents[2] 是 src/，
    # 因此该表达式等价于「<工作区>/src/<package>/<relative>」，即源码树中的位置。
    source = Path(__file__).resolve().parents[2] / package / relative
    if source.exists():
        return source
    # 源码树未命中时退回安装空间；此处延迟导入，使本模块在未安装相关依赖、
    # 或仅做静态分析时也能被安全导入。
    from ament_index_python.packages import get_package_share_directory

    installed = Path(get_package_share_directory(package)) / relative
    if not installed.exists():
        raise FileNotFoundError(installed)
    return installed
