#!/usr/bin/env python
"""一次启摆的观测：发 SWING，收 STREAM，看它有没有被双环接住。

用法:
    python swing_probe.py [COM口] [观测秒数]

⚠️ 这个脚本会**真的驱动电机**（SWP 是实打实的 35% 占空比脉冲）。
   安全兜底（三条，都必须留着）：
     · 每收一行就查 |POS|，超过 POS_LIMIT 立刻 STOP
       —— 防横杆连续转、把电源线绞断（control.h 里写过"硬件上真的会"）
     · 观测时间到就 STOP
     · SWING 没被受理就退出

列序（与 STAT 前 8 列一一对应）：
    ANGLE, POS, SPD, ATAR, AOUT, POUT, PWM, RUN
"""

import sys
import time

import serial

POS_LIMIT = 1200        # 边沿；408 边沿 = 横杆一圈，这里约 3 圈
DEFAULT_PORT = "COM7"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0

    sp = serial.Serial(port, 115200, timeout=0.5)
    time.sleep(0.4)
    sp.reset_input_buffer()

    def send(cmd, wait=0.25):
        sp.write((cmd + "\r\n").encode())
        time.sleep(wait)
        return sp.readline().decode(errors="replace").strip()

    def stop(reason):
        send("STREAM 0", wait=0.1)
        send("STOP")
        print(f"  [STOP] {reason}")

    print(f"起始: {send('STAT')}")
    print(f"参数: {send('GET SWP')} / {send('GET SWT')}")

    ack = send("SWING")
    print(f"SWING -> {ack}")
    if not ack.startswith("OK"):
        stop("SWING 被拒")
        sp.close()
        return

    send("STREAM 1", wait=0.1)

    t0 = time.time()
    rows = []
    caught_at = None
    stopped = None
    while (time.time() - t0) < dur:
        ln = sp.readline().decode(errors="replace").strip()
        if not ln:
            continue
        f = ln.split(",")
        if len(f) != 8:
            continue
        t = time.time() - t0
        angle, pos, spd, atar, aout, pout, pwm, run = (
            int(f[0]), int(f[1]), int(f[2]), float(f[3]),
            float(f[4]), float(f[5]), int(f[6]), int(f[7]))
        rows.append((t, angle, pos, spd, aout, pout, pwm, run))

        if caught_at is None and run == 1:
            caught_at = t

        if abs(pos) > POS_LIMIT:
            stopped = f"|POS|={pos} 超过 {POS_LIMIT}"
            break

    stop(stopped if stopped else "观测时间到")

    print(f"\n收到 {len(rows)} 行（{len(rows) / dur:.1f} 行/秒）")
    if not rows:
        print("  [!!] 一行都没收到")
        sp.close()
        return

    if caught_at is None:
        print(f"  [!!] 全程 RUN=0 —— 双环没有被接上（启摆没成功）")
    else:
        print(f"  [OK] {caught_at:.2f}s 时 RUN 变 1，双环接上了")

    # 接上之后的角度偏差与输出
    run_rows = [r for r in rows if r[7] == 1]
    if run_rows:
        dev = [abs(r[1] - 2086) for r in run_rows]
        pwm = [abs(r[6]) for r in run_rows]
        print(f"  接管后 {len(run_rows)} 行：|ANGLE-2086| 中位 {sorted(dev)[len(dev)//2]}、"
              f"最大 {max(dev)}；|PWM| 中位 {sorted(pwm)[len(pwm)//2]}、最大 {max(pwm)}")
        print(f"  接管后 POS 从 {run_rows[0][2]} 到 {run_rows[-1][2]}")

    # 末尾 20 行原样打出来看
    print("\n  末尾 20 行 (t, ANGLE, POS, SPD, AOUT, POUT, PWM, RUN):")
    for r in rows[-20:]:
        print(f"    {r[0]:5.2f} {r[1]:5d} {r[2]:6d} {r[3]:5d} {r[4]:9.2f} {r[5]:8.2f} {r[6]:5d} {r[7]}")

    print(f"\n结束后: {send('STAT')}")
    sp.close()


if __name__ == "__main__":
    main()
