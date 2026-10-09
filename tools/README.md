# tools —— 上位机脚本

这几个脚本通过串口和板子说话（协议见 [app/console/console.h](../app/console/console.h)）。
它们**不在构建里**，烧录固件不需要它们。

```bash
pip install pyserial
python tools/console_test.py COM7
```

| 脚本 | 用途 | 会不会动电机 |
| --- | --- | --- |
| [console_test.py](console_test.py) | **协议回归测**：30+ 项断言，含错误路径、超长行、CRLF、STREAM。改完协议先跑它 | 不会 |
| [trial.py](trial.py) | 一次完整试验：设增益 → 启摆 → 收 STREAM → 报指标 → 停机 | **会**（全功率启摆） |
| [offset_sweep.py](offset_sweep.py) | 在**已经立住**的状态下，运行中扫某个参数（默认 OFFSET） | 只在 RUN 时才动 |
| [swing_probe.py](swing_probe.py) | 只观测一次启摆，打印轨迹末尾 20 行原样数据 | **会** |

## 两个必须知道的约定

**① 发一条、等一条响应，再发下一条。**
OLED 整屏刷新实测约 122 ms，这期间主循环取不到串口数据，而接收环形缓冲只有 256 字节——
连发必定丢，而且丢得不声不响。

**② `console_test.py` 用开头的 `GET ALL` 做快照、结束时回滚。**
它**不写死**默认值（写死过一次：位置环默认值改了之后脚本没跟着改，于是每跑一次回归
就把运行参数改回错的，现象是"调完参测一次反而更抖"）。
同理它也不假设进来时环境是干净的 —— 每个用例自己把要用的参数设好。

> ⚠️ 这里的 `trial.py` / `offset_sweep.py` 都带 `|POS|` 看门狗：横杆累计位置超限就立刻
> `STOP`。这不是可选的 —— 位置是累积量，外环增益不对时横杆会一直转，**能把电源线绞断**
> （[control.h](../app/control/control.h) 里写过"硬件上真的会"）。
