#!/usr/bin/env python
"""在**已经立住**的情况下，运行中扫 OFFSET，量平衡时的抖动。

为什么这么做：摆杆放在平衡点上时，靠铰链摩擦能自己停住（用户 2026-10-10 说明）。
所以可以完全不启摆、不甩臂 —— 直接 RUN 进闭环，然后在运行中改 OFFSET。
改的是 32 位对齐的 float，1 ms 中断里读它是原子的（control.c 里有论证）。

用法:
    python offset_sweep.py [COM口] [--vals 90,60,45,30,0] [--seg 8]

安全兜底：|POS| 超限 → STOP；RUN 掉 0（倒了）→ 立即停并报告。
"""

import argparse
import statistics
import sys
import time

import serial

POS_LIMIT = 1200
CENTER = 2086
START = 150


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="COM7")
    ap.add_argument("--vals", default="90,60,45,30,0")
    ap.add_argument("--param", default="OFFSET", help="要扫的参数名")
    ap.add_argument("--fix", default="", help="扫描前先固定设置，如 OFFSET=30")
    ap.add_argument("--seg", type=float, default=8.0, help="每个取值的采样秒数")
    ap.add_argument("--keep", action="store_true", help="跑完不停机（默认停机）")
    a = ap.parse_args()

    vals = [float(v) for v in a.vals.split(",")]

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

    # 先强制关掉可能残留的 STREAM，再排空缓冲。
    # 不这么做的话，上一次跑完如果没关流，这里读到的会是 CSV 数据行而不是
    # STAT 的响应 —— 实测直接就崩在解析上。
    sp.write(b"STREAM 0\r\n")
    time.sleep(0.3)
    sp.reset_input_buffer()

    st = send("STAT")
    print(f"起始: {st}")
    if "RUN=1" not in st:
        ang = int(st.split("ANGLE=")[1].split()[0])
        if abs(ang - CENTER) > START:
            print(f"  [!!] 角度 {ang} 不在 CENTER±{START} 内，RUN 会被拒。"
                  f"请把摆杆放回竖直平衡点再跑（本脚本不发 SWING，不会甩臂）。")
            sp.close()
            return
        r = send("RUN")
        print(f"RUN -> {r}")
        if not r.startswith("OK"):
            print("  [!!] RUN 被拒，退出（不发 SWING）")
            sp.close()
            return

    # 固定项要在开流**之前**设完：开流之后每条 SET 的响应都会混在 CSV 里，
    # 看着像"命令没生效"，其实是读错行了。
    for kv in [s for s in a.fix.split(",") if s.strip()]:
        name, val = kv.split("=")
        r = send(f"SET {name.strip()} {val.strip()}")
        print(f"  fix {name.strip()}={val.strip()} -> {r}")

    send("STREAM 1", wait=0.1)

    results = []
    t0 = time.time()
    aborted = None
    for v in vals:
        send(f"SET {a.param} {v}", wait=0.1)
        rows = []
        tseg = time.time()
        while (time.time() - tseg) < a.seg:
            ln = sp.readline().decode(errors="replace").strip()
            if not ln:
                continue
            f = ln.split(",")
            if len(f) != 8:
                continue
            rows.append((int(f[0]), int(f[1]), int(f[2]), int(f[6]), int(f[7])))
            if abs(int(f[1])) > POS_LIMIT:
                aborted = f"|POS|={f[1]} 超过 {POS_LIMIT}"
                break
            if int(f[7]) != 1:
                aborted = "RUN 掉 0（摆杆倒了 / 倒下保护停机）"
                break
        if aborted:
            break

        dev = [abs(r[0] - CENTER) for r in rows]
        pwm = [r[3] for r in rows]
        jumps = [abs(pwm[i] - pwm[i - 1]) for i in range(1, len(pwm))]
        if not dev:
            continue
        results.append((v, len(rows),
                        statistics.median(dev), statistics.pstdev(dev), max(dev),
                        statistics.median(pwm), statistics.pstdev(pwm),
                        statistics.median(jumps)))
        print(f"  {a.param}={v:5.2f}  n={len(rows):4d}  "
              f"dev 中位 {statistics.median(dev):5.1f} 标准差 {statistics.pstdev(dev):5.1f} 最大 {max(dev):4d}  "
              f"| PWM 中位 {statistics.median(pwm):5.0f} 标准差 {statistics.pstdev(pwm):5.1f} 跳变 {statistics.median(jumps):4.0f}")

    if aborted:
        stop(aborted)
    elif a.keep:
        # 保持 RUN：摆杆仍然被托着，可以接着扫下一个参数，不用重新摆位。
        print("  （扫描结束，按要求保持 RUN）")
    else:
        stop("扫描结束")

    print(f"\n总耗时 {time.time() - t0:.0f}s")
    print(f"结束: {send('STAT')}")
    sp.close()

    if results:
        best = min(results, key=lambda r: r[3])     # dev 标准差最小
        print(f"\n按 dev 标准差最小：OFFSET={best[0]:.0f}  "
              f"（中位 {best[2]:.1f} 标准差 {best[3]:.1f}）")


if __name__ == "__main__":
    main()
