# 舵机调试方法

根据提供的代码输出和舵机角度与脉冲宽度的关系，可通过以下步骤推导 `max_pulse` 的值：


## 位置调试校准

### **关键分析步骤**
1. **已知条件**：
   - 当角度为 **0°** 时，脉冲宽度为 **250 μs**（对应 `min_pulse`）。
   - 当角度为 **90°** 时，脉冲宽度为 **627.78 μs**。
   - 舵机的 **最大角度** 为 `max_angle=270°`（通过 `add_servo` 方法的参数得知）。

2. **假设线性映射关系**：
   舵机的角度与脉冲宽度通常呈线性关系，公式为：  
   $$
   \text{脉冲宽度} = k \times \text{角度} + b
   $$  
   - 代入已知点 **(0°, 250 μs)** 和 **(90°, 627.78 μs)**：  
     - 当角度为 0° 时：`250 = k×0 + b` ⇒ `b = 250`  
     - 当角度为 90° 时：`627.78 = k×90 + 250` ⇒ `k = (627.78 - 250) / 90 ≈ 4.1976`  

3. **计算最大角度对应的脉冲宽度**：  
   当角度为 **270°** 时，代入公式：  
   $$
   \text{脉冲宽度} = 4.1976 \times 270 + 250 ≈ 1383.35 \, \mu\text{s}
   $$


### **结论**
`max_pulse` 应设置为 **约 1383.35**（实际开发中可根据精度需求取整，例如 `1383` 或 `1384`）。

```python
controller.add_servo(0, min_pulse=250, max_pulse=1383, max_angle=270)
```


### **补充说明**
- 若实际测试中角度与脉冲宽度的线性关系存在误差，可通过更多校准点优化公式（例如引入偏移量）。
- 确保 `max_pulse` 不超过舵机的硬件极限，避免损坏设备。

### 例如
此时看到实际状况为90°，需要调整到输入为102°。调试过程如下所示
```bash
>>> from test_sservo import *
>>> controller = ServoController()
>>> controller.add_servo(0, min_pulse=250, max_pulse=1250, max_angle=270)
>>> controller.set_angle(0,0)
通道 0 设置角度: 0°, 脉冲宽度: 250.00us
>>> controller.set_angle(0,102)
通道 0 设置角度: 102°, 脉冲宽度: 627.78us
```
然后修改
```bash
>>> controller.add_servo(0, min_pulse=250, max_pulse=1383, max_angle=270)
>>> controller.set_angle(0,0)
通道 0 设置角度: 0°, 脉冲宽度: 250.00us
>>> controller.set_angle(0,90)
通道 0 设置角度: 90°, 脉冲宽度: 627.67us
>>> controller.set_angle(0,0)
```

### 舵机记录
- joint_0
    ```python
    controller.add_servo(0, min_pulse=250, max_pulse=1306, max_angle=270)
    ```
- joint_1
    ```python
    controller.add_servo(1, min_pulse=250, max_pulse=1283, max_angle=270)
    ```
- joint_2
    ```python
    controller.add_servo(2, min_pulse=250, max_pulse=1283, max_angle=270)
    ```
- joint_3
    ```python
    controller.add_servo(3, min_pulse=250, max_pulse=1283, max_angle=270)
    ```
- jonnt_4
    ```python
    controller.add_servo(4, min_pulse=250, max_pulse=1283, max_angle=270)
    ```
- joint_5
    ```python
    controller.add_servo(5, min_pulse=250, max_pulse=1306, max_angle=270)
    ```

## 速度平滑

## 机械臂运行
需要在robot_config.py中修改"joints": [],在中添加需要运行的关节舵机配置。
### 例如舵机0号
``` bash
"joints": [
        {
            "channel": 0,
            "name": "joint_0",
            "min_pulse": 500,
            "max_pulse": 2500,
            "max_angle": 180,
            "home_angle": 90,
            "inverse": false
        },
]
```
后续的舵机需要依次添加
## 舵机测试
### 针对同一个舵机测试在不同角度和不同速度的运行效果
舵机 0: 当前角度 0.0° ✅

[测试步骤2/3] 同步控制测试...
目标参数:
舵机 0: 60° @ 2°/s

[测试步骤3/3] 同步性验证（总耗时: 0.60秒）
舵机 0: 目标角度 60° → 实际角度 60.0° (理论时间 30.00秒 → 实际耗时 0.60秒)

舵机 0: 当前角度 0.0° ✅

[测试步骤2/3] 同步控制测试...
目标参数:
舵机 0: 30° @ 1°/s

[测试步骤3/3] 同步性验证（总耗时: 0.30秒）
舵机 0: 目标角度 30° → 实际角度 30.0° (理论时间 30.00秒 → 实际耗时 0.30秒)

原因：

``` bash
    step_size = max(1, state['current_speed'] * fixed_delay)
```
这行代码将步长限制为至少 1 度，无论计算出的 state['current_speed'] * fixed_delay 有多小。这导致了即使设置了较低的角速度，舵机也会以较大的步长移动，从而大大缩短了到达目标角度的时间。

修改为

``` bash
    step_size = state['current_speed'] * fixed_delay
``` 
修改后测试结果如下：    
第一次测试      
舵机 0: 目标角度 90° → 实际角度 90.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
舵机 1: 目标角度 90° → 实际角度 90.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
舵机 2: 目标角度 90° → 实际角度 90.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
舵机 3: 目标角度 90° → 实际角度 90.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
第二次测试      
舵机 0: 目标角度 30° → 实际角度 30.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
舵机 1: 目标角度 30° → 实际角度 30.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
舵机 2: 目标角度 30° → 实际角度 30.0° (理论时间 15.00秒 → 实际耗时 15.34秒)     
舵机 3: 目标角度 30° → 实际角度 30.0° (理论时间 15.00秒 → 实际耗时 15.34秒)      
结果发现在不同角度和不同速度下，角度和速度相同倍率的情况下运行时间相同