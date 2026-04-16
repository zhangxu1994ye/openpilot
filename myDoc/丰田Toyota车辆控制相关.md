# Toyota CarController 接口分析与调用链路

## 一、CarController 提供的接口

### 1.1 核心接口

Toyota 的 `CarController` 类继承自 `CarControllerBase`，提供了两个核心方法：

```python
class CarController(CarControllerBase):
    def __init__(self, dbc_name, CP, VM):
        """初始化控制器"""

    def update(self, CC, CS, now_nanos):
        """主要控制接口 - 每个控制周期调用一次 (50Hz)"""
        return new_actuators, can_sends
```

### 1.2 `__init__` 初始化接口

**功能**：初始化车辆控制器

**输入参数**：
- `dbc_name`: DBC文件名（如 "toyota_nodsu_pt_generated"）
- `CP`: CarParams - 车辆参数配置
- `VM`: VehicleModel - 车辆模型

**初始化内容**：
```python
def __init__(self, dbc_name, CP, VM):
    self.CP = CP                                    # 车辆参数
    self.params = CarControllerParams(self.CP)      # 控制参数
    self.frame = 0                                  # 帧计数器

    # 转向状态
    self.last_steer = 0                             # 上次转向力矩
    self.last_angle = 0                             # 上次转向角度
    self.steer_rate_counter = 0                     # 转向速率计数器

    # 纵向控制状态
    self.gas = 0                                    # 油门
    self.accel = 0                                  # 加速度
    self.last_standstill = False                    # 上次静止状态
    self.standstill_req = False                     # 静止请求

    # HUD状态
    self.alert_active = False                       # 警告激活状态
    self.distance_button = 0                        # 距离按钮

    # CAN消息打包器
    self.packer = CANPacker(dbc_name)
```

### 1.3 `update` 主控制接口 ⭐

**功能**：生成车辆控制指令并打包成CAN消息

**调用频率**：50Hz（每20ms一次）

**输入参数**：
```python
def update(self, CC, CS, now_nanos):
    """
    CC: CarControl - 控制指令（来自planner/控制器）
        - CC.actuators.steer           # 转向指令 [-1, 1]
        - CC.actuators.accel           # 加速度指令 [m/s²]
        - CC.actuators.steeringAngleDeg # 转向角度 [deg]
        - CC.latActive                 # 横向控制激活
        - CC.longActive                # 纵向控制激活
        - CC.enabled                   # 系统使能
        - CC.hudControl                # HUD显示控制
        - CC.cruiseControl.cancel      # 取消巡航

    CS: CarState - 车辆状态（来自CAN总线读取）
        - CS.out.vEgo                  # 车速 [m/s]
        - CS.out.steeringTorque        # 方向盘力矩
        - CS.out.steeringTorqueEps     # EPS力矩
        - CS.out.steeringRateDeg       # 转向速率 [deg/s]
        - CS.out.steeringAngleDeg      # 转向角度
        - CS.out.standstill            # 静止状态
        - CS.pcm_acc_status            # PCM ACC状态
        - CS.acc_type                  # ACC类型
        - CS.lkas_hud                  # LKAS HUD状态

    now_nanos: int - 当前时间戳（纳秒）
    """
```

**返回值**：
```python
return (new_actuators, can_sends)
```
- `new_actuators`: 实际执行的actuators（反馈给控制系统）
- `can_sends`: CAN消息列表，格式 `[(addr, bus, data, src), ...]`

---

## 二、`update` 方法的内部处理流程

### 2.1 横向控制（转向）处理

```python
# 1. 提取转向指令
actuators = CC.actuators
lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

# 2. 计算转向力矩（力矩控制模式）
new_steer = int(round(actuators.steer * self.params.STEER_MAX))  # [-1500, 1500]
apply_steer = apply_meas_steer_torque_limits(
    new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params
)

# 3. 转向速率保护（防止EPS故障）
if abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE:  # 100 deg/s
    # 如果转向速率过快，计数器累加
    self.steer_rate_counter += 1
    if self.steer_rate_counter > MAX_STEER_RATE_FRAMES:  # 18帧
        apply_steer_req = False  # 禁止转向请求
else:
    self.steer_rate_counter = 0

# 4. 计算转向角度（角度控制模式 - LTA）
if self.CP.steerControlType == SteerControlType.angle:
    apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
    apply_angle = apply_std_steer_angle_limits(...)
    self.last_angle = clip(apply_angle, -MAX_LTA_ANGLE, MAX_LTA_ANGLE)

# 5. 发送转向CAN消息
can_sends.append(toyotacan.create_steer_command(
    self.packer, apply_steer, apply_steer_req
))
# 消息ID: 0x2E4 (STEERING_LKA)
# 频率: 100Hz

# 6. TSS2车型发送LTA转向消息
if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
    can_sends.append(toyotacan.create_lta_steer_command(
        self.packer, self.CP.steerControlType, self.last_angle,
        lta_active, self.frame // 2, torque_wind_down
    ))
    # 消息ID: 0x191 (STEERING_LTA)
    # 频率: 50Hz
```

### 2.2 纵向控制（油门/刹车）处理

```python
# 1. 提取加速度指令并限幅
pcm_accel_cmd = clip(actuators.accel,
                     self.params.ACCEL_MIN,   # -3.5 m/s²
                     self.params.ACCEL_MAX)   # 1.5 m/s²

# 2. 处理静止状态
if CS.out.standstill and not self.last_standstill:
    self.standstill_req = True  # 进入静止状态，请求保持
if CS.pcm_acc_status != 8:
    self.standstill_req = False # PCM退出静止模式

# 3. 发送加速度控制消息（纵向控制模式）
if self.CP.openpilotLongitudinalControl:
    lead = hud_control.leadVisible or CS.out.vEgo < 12.
    can_sends.append(toyotacan.create_accel_command(
        self.packer,
        pcm_accel_cmd,          # 加速度指令
        pcm_cancel_cmd,         # 取消指令
        self.standstill_req,    # 静止请求
        lead,                   # 前车存在
        CS.acc_type,            # ACC类型
        fcw_alert,              # 前向碰撞警告
        self.distance_button    # 跟车距离按钮
    ))
    # 消息ID: 0x343 (ACC_CONTROL)
    # 频率: 约16.67Hz (每3帧)
    self.accel = pcm_accel_cmd
```

### 2.3 HUD显示控制

```python
# 1. 解析警告类型
fcw_alert = hud_control.visualAlert == VisualAlert.fcw
steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

# 2. 发送UI显示命令
if self.frame % 20 == 0 or send_ui:  # 5Hz + 事件触发
    can_sends.append(toyotacan.create_ui_command(
        self.packer,
        steer_alert,                    # 转向警告
        pcm_cancel_cmd,                 # 取消指令
        hud_control.leftLaneVisible,   # 左车道线
        hud_control.rightLaneVisible,  # 右车道线
        hud_control.leftLaneDepart,    # 左偏离
        hud_control.rightLaneDepart,   # 右偏离
        CC.enabled,                     # 系统使能
        CS.lkas_hud                     # LKAS HUD状态
    ))
    # 消息ID: 0x2C1 (LKAS_HUD)
    # 频率: 5Hz

# 3. 发送FCW警告
if (self.frame % 100 == 0 or send_ui):  # 1Hz + 事件触发
    can_sends.append(toyotacan.create_fcw_command(
        self.packer, fcw_alert
    ))
    # 消息ID: 0x2E6 (PCS_HUD)
    # 频率: 1Hz
```

### 2.4 静态消息和雷达禁用

```python
# 1. 发送DSU静态消息（如果启用DSU）
for addr, cars, bus, fr_step, vl in STATIC_DSU_MSGS:
    if self.frame % fr_step == 0 and self.CP.enableDsu:
        can_sends.append(make_can_msg(addr, vl, bus))

# 2. 保持雷达禁用
if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR:
    # Tester Present消息
    can_sends.append([0x750, 0, b"\x0F\x02\x3E\x00\x00\x00\x00\x00", 0])
```

### 2.5 构建返回值

```python
# 构建实际执行的actuators（用于反馈）
new_actuators = actuators.as_builder()
new_actuators.steer = apply_steer / self.params.STEER_MAX
new_actuators.steerOutputCan = apply_steer
new_actuators.steeringAngleDeg = self.last_angle
new_actuators.accel = self.accel
new_actuators.gas = self.gas

self.frame += 1
return new_actuators, can_sends
```

---

## 三、完整的调用链路

### 3.1 系统架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    主控制循环 (50Hz)                          │
│                  controlsd.py - Controls                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  1. data_sample() - 采样传感器数据       │
        │     读取 carState, modelV2 等           │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  2. update_events() - 更新事件          │
        │     安全检查、状态监控                   │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  3. state_control() - 计算控制指令       │
        │     • 纵向控制 LoC.update()             │
        │     • 横向控制 LaC.update()             │
        │     生成 CarControl (CC)                │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  4. publish_logs() - 发布控制消息        │
        │     发送 'carControl' 消息               │
        └─────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                    车辆接口层 (100Hz)                         │
│                  card.py - Car                               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  5. state_update() - 更新车辆状态        │
        │     CI.update() 读取CAN消息              │
        │     生成 CarState (CS)                   │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  6. state_publish() - 发布车辆状态       │
        │     发送 'carState', 'carOutput'         │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  7. controls_update() - 执行控制         │
        │     CI.apply(CC, now_nanos)  ⭐         │
        └─────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  车辆控制器层 (50Hz)                          │
│         interfaces.py - CarInterfaceBase.apply()            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  8. CarController.update(CC, CS, now)   │
        │     toyota/carcontroller.py  ⭐⭐⭐      │
        └─────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   CAN消息打包层                               │
│                  toyotacan.py                                │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  9. 打包CAN消息                          │
        │     • create_steer_command()            │
        │     • create_lta_steer_command()        │
        │     • create_accel_command()            │
        │     • create_ui_command()               │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  10. can_list_to_can_capnp()            │
        │      转换为capnp格式                     │
        └─────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   硬件接口层                                  │
│                  panda (USB-CAN)                             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
                      [车辆CAN总线]
```

### 3.2 详细调用代码

#### Step 1: controlsd.py - 主控制循环

```python
# selfdrive/controls/controlsd.py
class Controls:
    def step(self):
        # 1. 采样数据
        CS = self.data_sample()  # 读取车辆状态

        # 2. 更新事件
        self.update_events(CS)

        # 3. 计算控制指令
        CC, lac_log = self.state_control(CS)

        # 4. 发布日志
        self.publish_logs(CS, start_time, CC, lac_log)

    def state_control(self, CS):
        # 纵向控制
        long_plan = self.sm['longitudinalPlan']
        self.LoC.update(long_plan.speeds, long_plan.accels, CS, self.CP)

        # 横向控制
        lat_plan = self.sm['modelV2']
        self.LaC.update(lat_plan, CS, self.CP, self.VM, self.params)

        # 构建CarControl消息
        CC = car.CarControl.new_message()
        CC.enabled = self.enabled
        CC.latActive = self.active
        CC.longActive = self.enabled

        CC.actuators.steer = lat_control.steer  # 转向指令
        CC.actuators.accel = long_control.accel # 加速度指令
        CC.hudControl = ...                     # HUD控制
        CC.cruiseControl = ...                  # 巡航控制

        return CC, lac_log

    def publish_logs(self, CS, start_time, CC, lac_log):
        # 发布carControl消息到消息队列
        cc_send = messaging.new_message('carControl')
        cc_send.carControl = CC
        self.pm.send('carControl', cc_send)
```

#### Step 2: card.py - 车辆接口循环

```python
# selfdrive/car/card.py
class Car:
    def step(self):
        # 1. 更新车辆状态
        CS = self.state_update()  # 从CAN读取

        # 2. 更新事件
        self.update_events(CS)

        # 3. 发布状态
        self.state_publish(CS)

        # 4. 执行控制
        if not self.CP.passive and initialized:
            self.controls_update(CS, self.sm['carControl'])

    def controls_update(self, CS, CC):
        """关键调用点！"""
        now_nanos = int(time.monotonic() * 1e9)

        # 调用CarInterface.apply() ⭐
        self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)

        # 发送CAN消息到车辆
        self.pm.send('sendcan', can_list_to_can_capnp(
            can_sends, msgtype='sendcan', valid=CS.canValid
        ))
```

#### Step 3: interfaces.py - 车辆接口基类

```python
# selfdrive/car/interfaces.py
class CarInterfaceBase(ABC):
    def apply(self, c: car.CarControl, now_nanos: int):
        """转发到CarController.update()"""
        return self.CC.update(c, self.CS, now_nanos)
```

#### Step 4: toyota/carcontroller.py - Toyota控制器

```python
# selfdrive/car/toyota/carcontroller.py
class CarController(CarControllerBase):
    def update(self, CC, CS, now_nanos):
        """⭐⭐⭐ 核心控制逻辑 ⭐⭐⭐"""
        can_sends = []

        # 1. 处理转向
        apply_steer = self._process_steering(CC, CS)
        can_sends.append(toyotacan.create_steer_command(...))

        # 2. 处理加速度
        pcm_accel = self._process_accel(CC, CS)
        can_sends.append(toyotacan.create_accel_command(...))

        # 3. 处理HUD
        can_sends.append(toyotacan.create_ui_command(...))

        # 4. 返回
        return new_actuators, can_sends
```

---

## 四、数据流详解

### 4.1 输入数据来源

#### CarControl (CC) - 来自planner

```python
CC = car.CarControl.new_message()

# 来自纵向规划器 (longitudinalPlan)
CC.actuators.accel = 1.2  # m/s² (来自LongControl)

# 来自横向规划器 (modelV2 + LatControl)
CC.actuators.steer = -0.15  # 归一化转向 [-1, 1]
CC.actuators.steeringAngleDeg = -5.2  # 转向角度

# 来自状态机
CC.enabled = True        # 系统使能
CC.latActive = True      # 横向控制激活
CC.longActive = True     # 纵向控制激活

# 来自AlertManager
CC.hudControl.visualAlert = VisualAlert.fcw
CC.hudControl.leftLaneVisible = True
CC.hudControl.leadVisible = True
```

#### CarState (CS) - 来自CAN总线

```python
CS = car.CarState.new_message()

# 从CAN解析（carstate.py）
CS.out.vEgo = 25.3                  # m/s
CS.out.steeringTorque = 120         # 驾驶员转向力矩
CS.out.steeringTorqueEps = -50      # EPS电机力矩
CS.out.steeringRateDeg = 15.2       # 转向速率
CS.out.steeringAngleDeg = -3.5      # 转向角度
CS.out.standstill = False           # 是否静止
CS.gasPressed = False               # 油门踏板
CS.brakePressed = False             # 刹车踏板
```

### 4.2 输出数据格式

#### CAN消息列表

```python
can_sends = [
    # 格式: (address, bus, data, src)

    # 转向控制 - STEERING_LKA (0x2E4)
    (0x2E4, 0, b'\x00\x64\x01\x00\x0C', 0),

    # 转向控制 - STEERING_LTA (0x191)
    (0x191, 0, b'\x80\x01\x64\xFF\xC8\x00\x00\x00', 0),

    # 加速度控制 - ACC_CONTROL (0x343)
    (0x343, 0, b'\x00\x78\x00\x01\x00\x00\x00\x00', 0),

    # HUD显示 - LKAS_HUD (0x2C1)
    (0x2C1, 0, b'\x03\x02\x00\x00\x00\x00\x00\x00', 0),
]
```

#### 实际执行的Actuators

```python
new_actuators = car.CarControl.Actuators.new_message()
new_actuators.steer = -0.15           # 实际转向（归一化）
new_actuators.steerOutputCan = -225   # CAN转向值
new_actuators.steeringAngleDeg = -5.2 # 转向角度
new_actuators.accel = 1.2             # 加速度
new_actuators.gas = 0                 # 油门
```

---

## 五、关键控制参数

### 5.1 转向控制参数

```python
# toyota/values.py - CarControllerParams
class CarControllerParams:
    STEER_MAX = 1500                    # 最大转向力矩
    STEER_STEP = 1                      # 转向步进
    STEER_ERROR_MAX = 350               # 最大转向误差
    STEER_DELTA_UP = 10 或 15           # 上升速率限制
    STEER_DELTA_DOWN = 25               # 下降速率限制

    # LTA角度控制限制
    ANGLE_RATE_LIMIT_UP = 0.3~0.15 deg/s
    ANGLE_RATE_LIMIT_DOWN = 0.36~0.26 deg/s

# carcontroller.py - 安全限制
MAX_STEER_RATE = 100           # deg/s - 最大转向速率
MAX_STEER_RATE_FRAMES = 18     # 帧 - 触发保护的帧数
MAX_USER_TORQUE = 500          # 最大用户力矩
MAX_LTA_ANGLE = 94.9461        # deg - 最大LTA角度
```

### 5.2 纵向控制参数

```python
class CarControllerParams:
    ACCEL_MAX = 1.5   # m/s² - 最大加速度
    ACCEL_MIN = -3.5  # m/s² - 最大减速度
```

---

## 六、控制频率和时序

| 消息类型 | CAN ID | 频率 | 发送条件 |
|---------|--------|------|---------|
| STEERING_LKA | 0x2E4 | 100Hz | 每帧 |
| STEERING_LTA | 0x191 | 50Hz | TSS2车型，每2帧 |
| ACC_CONTROL | 0x343 | ~16.67Hz | 纵向控制，每3帧 |
| LKAS_HUD | 0x2C1 | 5Hz + 事件 | 每20帧 + 警告触发 |
| PCS_HUD | 0x2E6 | 1Hz + 事件 | 每100帧 + FCW触发 |
| Tester Present | 0x750 | 5Hz | 禁用雷达时，每20帧 |

**主循环频率**：
- `controlsd`: 50Hz (20ms)
- `card`: 100Hz (10ms) - 由CAN接收驱动
- `CarController.update`: 50Hz (20ms) - 由carControl消息驱动

---

## 七、总结

### 7.1 CarController的作用

Toyota的`CarController`是**控制指令到CAN消息的转换器**：

1. **输入**：高级控制指令（转向、加速度、HUD）
2. **处理**：应用安全限制、速率限制、状态管理
3. **输出**：符合Toyota协议的CAN消息列表

### 7.2 调用流程总结

```
用户/环境 → 传感器数据 → controlsd (规划) → CarControl
                                           ↓
                                      card (接口)
                                           ↓
                                   CarController.update()
                                           ↓
                                      toyotacan (打包)
                                           ↓
                                       CAN总线 → 车辆执行
```

### 7.3 关键设计理念

1. **分层架构**：规划层、接口层、控制层、硬件层清晰分离
2. **安全优先**：多重安全检查（速率限制、力矩限制、故障保护）
3. **品牌定制**：每个品牌有独立的CarController实现
4. **实时性**：50Hz高频控制循环
5. **状态管理**：维护历史状态用于速率限制和平滑控制

这个架构使得OpenPilot能够支持多个品牌，同时保持代码的可维护性和安全性！
