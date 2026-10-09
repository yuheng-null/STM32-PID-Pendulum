#!/usr/bin/env python
"""一次调参试验：设增益 → SWING → 收 STREAM → 报指标 → STOP。

用法:
    python trial.py [COM口] [--akp 4.5] [--aki 0.162] [--akd 7.38]
                              [--pkp 0.52] [--pki 0.01] [--pkd 4.56]
                              [--secs 12] [--no-swing]

⚠️ 会真的驱动电机。安全兜底：
    · |POS| 超过 POS_LIMIT 立刻 STOP（防横杆连续转、把电源线绞断）
    · 时间到就 STOP
    · SWING 被拒就退出

指标口径（调参时看这几个数就够）：
    catch    从 SWING 到 RUN 变 1 的时间；None = 没接住
    dev      接管后 |ANGLE-CENTER| 的中位 / p90 / 最大，单位 ADC 码
    pwm      接管后 |PWM| 的中位 / 最大，以及"打满"（≥1700）的比例
    pos      接管后 POS 的起点 / 终点 / 绝对值最大
    fell     接管后 RUN 是否掉回 0（倒了 / 被倒下保护停机）
"""

import argparse
import statistics
import sys
import time

import serial

POS_LIMIT = 1200
CENTER = 2086
SAT = 1700          # |PWM| ≥ 这个值算"打满"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM7")
    ap.add_argument("--akp", type=float)
    ap.add_argument("--aki", type=float)
    ap.add_argument("--akd", type=float)
    ap.add_argument("--pkp", type=float)
    ap.add_argument("--pki", type=float)
    ap.add_argument("--pkd", type=float)
    ap.add_argument("--offset", type=float)
    ap.add_argument("--swp", type=float)
    ap.add_argument("--swt", type=float)
    ap.add_argument("--secs", type=float, default=12.0)
    ap.add_argument("--settle", type=float, default=0.0,
                    help="启摆前先等这么多秒，让摆杆静定（各轮之间才可比）")
    ap.add_argument("--no-swing", action="store_true", help="不启摆，直接照当前姿态 RUN")
    a = ap.parse_args()

    sp = serial.Serial(a.port, 115200, timeout=0.5)
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

    gains = {"AKP": a.akp, "AKI": a.aki, "AKD": a.akd,
             "PKP": a.pkp, "PKI": a.pki, "PKD": a.pkd,
             "OFFSET": a.offset, "SWP": a.swp, "SWT": a.swt}
    for name, val in gains.items():
        if val is not None:
            r = send(f"SET {name} {val}")
            if not r.startswith("OK"):
                print(f"  [!!] {name} 设置失败: {r}")
                sp.close()
                return

    print(f"增益: {send('GET ALL')}")

    if a.settle > 0:
        # 关键：每一轮都从"摆杆已经静定"出发，否则上一轮残留的摆动会让
        # 这一轮的接住时刻、初始状态各不相同，指标没法横向比。
        print(f"静定 {a.settle:.0f}s ...")
        time.sleep(a.settle)

    print(f"起始: {send('STAT')}")

    if a.no_swing:
        ack = send("RUN")
    else:
        ack = send("SWING")
    print(f"启动 -> {ack}")
    if not ack.startswith("OK"):
        stop("启动被拒")
        sp.close()
        return

    send("STREAM 1", wait=0.1)

    t0 = time.time()
    rows = []
    caught_at = None
    fell = False
    why = None
    while (time.time() - t0) < a.secs:
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
        elif caught_at is not None and run == 0:
            fell = True

        if abs(pos) > POS_LIMIT:
            why = f"|POS|={pos} 超过 {POS_LIMIT}"
            break

    stop(why if why else "时间到")

    if not rows:
        print("  [!!] 一行都没收到")
        sp.close()
        return

    rr = [r for r in rows if r[7] == 1]
    print(f"\n收到 {len(rows)} 行；RUN=1 的 {len(rr)} 行 ({100*len(rr)/len(rows):.0f}%)")
    print(f"  catch = {'—' if caught_at is None else f'{caught_at:.2f}s'}"
          f"{'   **[没接住]**' if caught_at is None else ''}")
    print(f"  fell  = {fell}")

    if rr:
        dev = sorted(abs(r[1] - CENTER) for r in rr)
        pwm = sorted(abs(r[6]) for r in rr)
        pos = [r[2] for r in rr]
        n = len(dev)
        sats = sum(1 for p in pwm if p >= SAT)
        print(f"  dev   = {dev[n//2]} / {dev[int(n*0.9)]} / {dev[-1]}   (中位/p90/最大, 码)")
        print(f"  pwm   = {pwm[n//2]} / {pwm[-1]}   打满 {100*sats/n:.0f}%")
        print(f"  pos   = {pos[0]} → {pos[-1]}   |max|={max(abs(p) for p in pos)}")

        # 稳态：只看最后 5 秒。跑得越久越该收敛，最后 5 秒才是真实水平；
        # 全程统计会把"刚被接住那一下"的大偏差算进去，掩盖稳态好坏。
        tail = [r for r in rr if r[0] >= (rr[-1][0] - 8.0)]
        if tail:
            td = [abs(r[1] - CENTER) for r in tail]
            tp = [r[6] for r in tail]
            jumps = [abs(tp[i] - tp[i - 1]) for i in range(1, len(tp))]

            def med(v):
                return sorted(v)[len(v) // 2]

            def std(v):
                m = sum(v) / len(v)
                return (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5

            print(f"  稳态8s= dev 中位 {med(td)} 标准差 {std(td):.0f} 码；"
                  f"PWM 中位 {med(tp)} 标准差 {std(tp):.0f}，相邻样本跳变中位 {med(jumps)}")

    print(f"\n结束后: {send('STAT')}")
    sp.close()


if __name__ == "__main__":
    main()
