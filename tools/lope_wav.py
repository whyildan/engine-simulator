"""Idle "lope" of a rendered engine sound: how rhythmic the loudness swings are.

usage: python tools/lope_wav.py a.wav [b.wav ...] [--t0 2.5 --t1 6.0]

The loudness envelope (RMS over 10 ms) of the window is analysed for slow modulation (0.5 - 20 Hz): a rhythmic lope - a
bridge-port rotary's "brap", a big cam's loping idle - shows as a strong, narrow peak; a random sputter spreads over the
whole band. Printed per file: modulation depth (envelope std / mean), the strongest modulation frequency, and how much of
the modulation energy sits within +-15 % of that peak (concentration: near 1 = one clean rhythm, near 0 = noise).
"""
import sys, wave
import numpy as np


def analyse(path, t0, t1):
    with wave.open(path) as w:
        sr = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, w.getnchannels()) / 32768.0
    s = x[int(t0 * sr):int(t1 * sr)].mean(axis=1)
    hop = int(0.01 * sr)
    env = np.array([np.sqrt(np.mean(s[i:i + hop] ** 2)) for i in range(0, len(s) - hop, hop)])
    depth = np.std(env) / (np.mean(env) + 1e-12)
    e = (env - env.mean()) * np.hanning(len(env))
    spec = np.abs(np.fft.rfft(e)) ** 2
    f = np.fft.rfftfreq(len(e), 0.01)
    band = (f >= 0.5) & (f <= 20.0)
    fb, sb = f[band], spec[band]
    k = int(np.argmax(sb))
    peak = fb[k]
    near = (fb > 0.85 * peak) & (fb < 1.15 * peak)
    conc = sb[near].sum() / (sb.sum() + 1e-20)
    print(f"{path}\n   window {t0}-{t1} s  modulation depth {depth:.2f}  strongest {peak:.2f} Hz  concentration {conc:.2f}")


def main():
    args = sys.argv[1:]
    t0, t1 = 2.5, 6.0
    if "--t0" in args: i = args.index("--t0"); t0 = float(args[i + 1]); del args[i:i + 2]
    if "--t1" in args: i = args.index("--t1"); t1 = float(args[i + 1]); del args[i:i + 2]
    for a in args:
        analyse(a, t0, t1)


if __name__ == "__main__":
    main()
