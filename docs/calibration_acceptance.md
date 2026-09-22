# 网页手眼标定软件验收清单

此清单不计入历史P0–P6硬件完成率，不将合成数据当作真实标定验收。

| 要求 | 当前证据 | 结论 |
|---|---|---|
| 既有Dashboard同端口/calibration | test_handeye_capture_runtime构造实际TeleopStatusPanelNode并GET页面 | 已验证 |
| HTTP→实际Dashboard→ROS→图像/CameraInfo/TF | 同测试独立domain合成ArUco和动态TF，图像/TF时间一致 | 已验证单姿态及16姿态图像求解链 |
| 会话持久保存、恢复、幂等、版本 | test_calibration_session_store；浏览器刷新恢复 | 已验证 |
| 五方法求解、训练选择、独立验证 | test_handeye_workflow，偏置验证集拒绝通过 | 已验证 |
| 可观测性与覆盖 | test_handeye_residual、test_calibration_quality | 已验证 |
| bootstrap诊断 | test_calibration_quality | 已验证，非绝对精度 |
| 原子报告、失败保留旧文件 | test_handeye_workflow | 已验证 |
| 独立使能/重力补偿按钮 | 实际浏览器页面，calibration_gravity gate测试 | 实际Dashboard+隔离控制器替身验证通过 |
| 禁止软件模式操作硬件 | 实际Dashboard HTTP gravity请求返回400 | 已验证 |
| 超时迟到结果及反馈恢复 | GravityRequestTracker测试 | 正常模式切换节点测试通过；迟到结果由tracker单测覆盖 |
| 采集失败不自动失能 | 采集节点无控制服务客户端；无相机测试不修改会话 | 已验证采集边界 |
| SSE进度/错误 | browser能接收状态，客户端异常清busy测试 | 多客户端SSE断开/重连及状态推进测试通过 |
| 浏览器创建→采样→求解→接受→恢复 | 真实浏览器+真实存储/算法，合成姿态测试后端 | 已验证，浏览器ROS采样/恢复已验证；多姿态求解由HTTP集成覆盖 |
| 候选导出 | 页面JSON下载实现 | Blob内容校验、服务器附件HTTP/ROS一致性、实际浏览器附件下载事件均通过 |
| 相机预览/质量可视化 | 仅文本质量和原始报告 | 已实现快照和算法结果表，快照实际浏览器视觉复查通过；结果表和离群/失败提示实际浏览器视觉复查通过 |
| 参数/阈值来源和审计 | 元数据、内参、时间戳、SHA256、OpenCV版本 | 样本已保存有效阈值、库版本和各源文件哈希 |
| TCP页面与独立TF检查 | TCP网页/纯TF采样、独立tf_check已实现 | 无相机ROS采样、TCP浏览器采样恢复和完整12姿态ROS链均通过 |
| 离线/MuJoCo软件闭环 | 合成ROS图像/TF + canonical MuJoCo FK 14训练6验证通过 | MuJoCo FK→投影图像→PnP→实际网页/ROS求解、确认、导出、恢复通过 |

软件范围验收完成（下述历史检查记录保留发生时状态，以本表及最终审计为准）。真实相机、使能、重力补偿、运动及物理精度验收
必须单独取得现场授权，本软件目标不要求自动执行这些操作。

## 2026-09-19 最新浏览器复核

使用实际TeleopStatusPanelNode、HandeyeCaptureNode、隔离domain183合成相机/TF，
浏览器在18766同端口完成创建、快照可视、TF检查、手眼采样1项；切换TCP创建新会话，
采样1项并刷新恢复，模式与坐标系正确回显。快照显示ArUco图案和0.006s获取帧龄。
此为实际浏览器→HTTP→ROS数据链，未使用硬件。未做该链多姿态求解完整闭环。
点击下载未获得in-app浏览器download事件确认（5秒观察超时），不记浏览器文件下载通过；
已有Node Blob内容测试仍有效。测试tab已关闭，临时ROS测试进程已停止。

## 完整TCP实际ROS会话

test_handeye_capture_runtime 进一步通过实际Dashboard HTTP依次请求12次TCP采样，
TF源发布6训练+6验证不同姿态（同一固定pivot），不直接注入样本。求解通过并恢复
预设偏移（1e-6m容差），显式人工接受后revision14、deployed=false；新SessionStore
实例读取报告一致，服务器附件包含完整6+6样本和accepted状态。
此测试无相机、无硬件，覆盖TCP端到端持久化链；手眼多姿态图像链仍须另行验证。

## 手眼多姿态图像端到端验证

test_handeye_capture_runtime 使用透视投影生成16张不同视角ArUco图像，与固定标记及
预设相机外参对应的base→end动态TF原子配对发布。真实Dashboard HTTP→ROS采集→
PnP→稳定窗口→10训练+6验证→多算法求解→人工接受通过；恢复平移误差<5mm。
样本由图像算法生成，未直接注入camera_to_marker。最终accepted=true/deployed=false。
这是合成图像软件证据，不证明真实打印尺寸、内参或机械安装精度。

## 服务器附件浏览器下载与SSE

实际浏览器恢复已保存TCP会话并点击服务器附件链接，download事件成功返回；
页面仍留在/calibration。附件字节正确性由HTTP/ROS导出集成测试覆盖。Blob下载
未获得事件确认的问题不隐瞒，推荐使用服务器附件入口。临时tab和ROS测试进程已关闭。
两个并发SSE连接均收到busy，断开其中一个后另一个收到revision1成功；重连立刻
读到最新结果，/api/status保持可用。

## 正式会话状态唯一性

移除早期未接线CalibrationSession原型，正式采集/网页/离线持久化仅使用SessionStore。
补充测试证明失败报告即使confirmed=true也不能接受；reopen清除旧报告后不能接受旧结果；
调用方修改返回对象不会改写磁盘样本。重建后旧原型find_spec=None，正式store可导入。

## 最新结果表浏览器验收

使用10训练6验证合成数据，其中验证样本15平移增加50mm，真实算法生成失败报告；
实际浏览器显示五方法表，验证位置RMS20.412mm/最大50mm、验证未通过与未部署，
离群提示定位到样本15且保留原始数据。布局可读，临时HTTP服务和tab已关闭。

## MuJoCo投影图像组合测试

新增test_mujoco_projected_marker_detection_and_handeye：canonical scene的24个限位内
末端FK姿态、固定物理标记和相机外参，经1920×1440透视投影生成图像，实际ArUco/PnP
输出样本，再以16训练+8验证求解；通过默认质量门且平移恢复误差<5mm。
测试生成器使用标记外像素边缘(-0.5,399.5)定义100mm边长，线性插值；初始低分辨率
最近邻图像未通过，保留这一事实，不把数字图像离散误差等同于算法或物理精度。
该测试是MuJoCo运动学与投影成像组合，不是MuJoCo光照/遮挡渲染；ROS与HTTP实际链由
test_handeye_capture_runtime独立覆盖。

## 会话级话题选择

网页新增image_topic/camera_info_topic；节点校验绝对ROS名称并按会话切换订阅，清空
缓存且拒绝旧代次回调。实际ROS测试新增alternate相机话题采样，来源记录为实际话题；
随后默认话题多姿态手眼求解链仍通过。生产代码已重建。

## 最终软件完成审计

- 现有Dashboard同端口页面、HTTP/SSE与ROS边界：实际TeleopStatusPanelNode集成和浏览器验证。
- 同步采集、TF/相机检查、稳定窗口、话题切换：test_handeye_capture_runtime、test_calibration_stability，包含无相机失败和不同话题。
- 会话唯一所有权、恢复/幂等/原子保存/接受门/来源与审计：SessionStore及workflow/provenance测试。
- 训练选择/独立验证/五算法/可观测性/离群诊断/bootstrap：真实图像16姿态ROS链及数学回归。
- 显式enable/gravity start/stop与互斥、失败不disable：隔离控制器替身HTTP/ROS测试；采集节点无硬件客户端。
- 数据包导出、SSE重连、浏览器恢复：HTTP附件测试、两客户端SSE测试与实际浏览器download事件。
- TCP额外分支：12姿态完整实际ROS链及浏览器采样恢复通过。
- MuJoCo组合：canonical FK→24张透视投影图像→实际PnP数据，经SessionStore离线载入；
  实际浏览器→Dashboard→ROS对16训练8验证求解，PARK验证位置RMS0.374mm/最大0.710mm，
  人工software-audit确认accepted/revision26/deployed=false，服务器附件download事件与刷新恢复通过。
  离线导入在测试准备阶段完成；采集实时链由上述图像ROS测试验证。使用投影成像，不声称光照/遮挡物理渲染。
- 构建：msgs/calibration/dashboard均成功重建，bringup重建及show-args通过；四个标定入口已检查。
- 最后一次完整回归：734 passed、7 skipped、1既有MuJoCo默认解释器断言失败；分层18、compileall、diff检查通过。
  最后回归后只增加证据和文档，未改生产代码。

目标明确以软件流程与可复现验证为准，故现场相机/机器人精度验收留作用户另行授权后的下一阶段。
未自动部署外参、未启动真实控制器、未发送真实运动或模式切换请求。工作区既有修改保留，未提交/推送。
