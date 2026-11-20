"""Compare two rendered engine sounds objectively (before / after a change to the sound model).

usage: python tools/compare_wav.py a.wav b.wav [t0 t1 rpm cylinders]

Over the same time window of both files it prints
  - level, crest factor;
  - 1/3-octave band levels (31.5 Hz .. 16 kHz) of each and their difference b - a;
  - engine-order levels (multiples of the crank rotation frequency) when the rpm is given: the firing order
    (cylinders / 2 for a four-stroke) and its harmonics carry the tone, half orders the lumpiness.
Levels are in dB relative to full scale of the window's RMS spectrum (Hann window), so the two files compare directly.
"""
import sys, wave
import numpy as np


def load(path, t0, t1):
    with wave.open(path) as w:
        sr = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, w.getnchannels()) / 32768.0
    seg = x[int(t0 * sr):int(t1 * sr)].mean(axis=1)
    return seg, sr


def spectrum(seg, sr):
    n = len(seg)
    win = np.hanning(n)
    spec = np.abs(np.fft.rfft(seg * win)) * 2.0 / np.sum(win)
    return np.fft.rfftfreq(n, 1.0 / sr), spec


def band_levels(f, spec):
    centres = 1000.0 * 2.0 ** (np.arange(-15, 13) / 3.0)   # 31.5 Hz .. 16 kHz
    out = []
    for fc in centres:
        lo, hi = fc / 2 ** (1 / 6), fc * 2 ** (1 / 6)
        m = (f >= lo) & (f < hi)
        e = np.sum(spec[m] ** 2)
        out.append((fc, 10 * np.log10(e + 1e-20)))
    return out


def order_level(f, spec, freq, width):
    m = (f > freq - width) & (f < freq + width)
    return 10 * np.log10(np.sum(spec[m] ** 2) + 1e-20) if np.any(m) else -200.0


def main():
    a, b = sys.argv[1], sys.argv[2]
    t0 = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5
    t1 = float(sys.argv[4]) if len(sys.argv) > 4 else 3.4
    rpm = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    cyl = int(sys.argv[6]) if len(sys.argv) > 6 else 0
    sa, sr = load(a, t0, t1)
    sb, _ = load(b, t0, t1)
    for name, s in (("a", sa), ("b", sb)):
        rms = np.sqrt(np.mean(s ** 2)); pk = np.max(np.abs(s))
        print(f"{name}: {(a if name == 'a' else b)}  rms {20 * np.log10(rms + 1e-12):6.1f} dBFS  crest {pk / (rms + 1e-12):4.1f}")
    fa, pa = spectrum(sa, sr)
    fb, pb = spectrum(sb, sr)
    print("\n  1/3 octave     a dB     b dB   b - a")
    for (fc, la), (_, lb) in zip(band_levels(fa, pa), band_levels(fb, pb)):
        print(f"  {fc:8.0f} Hz  {la:7.1f}  {lb:7.1f}  {lb - la:+6.1f}")
    if rpm > 0:
        f1 = rpm / 60.0
        print(f"\n  engine orders at {rpm:.0f} rpm (order 1 = {f1:.1f} Hz)" + (f", firing order {cyl / 2:g}" if cyl else ""))
        print("  order     a dB     b dB   b - a")
        for k in np.arange(0.5, 12.01, 0.5):
            la = order_level(fa, pa, k * f1, 0.2 * f1)
            lb = order_level(fb, pb, k * f1, 0.2 * f1)
            print(f"  {k:5.1f}  {la:7.1f}  {lb:7.1f}  {lb - la:+6.1f}")


if __name__ == "__main__":
    main()
