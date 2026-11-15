"""Quick objective check of rendered engine sounds.

usage: python tools/analyze_wav.py renders/p0.wav [t0 t1 rpm cylinders]
Prints level, crest factor and the strongest spectral peaks in a time window, expressed
as engine orders (multiples of crank rotation frequency) so different engines can be compared.
"""
import sys, wave
import numpy as np

path = sys.argv[1]
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
t1 = float(sys.argv[3]) if len(sys.argv) > 3 else 3.4
rpm = float(sys.argv[4]) if len(sys.argv) > 4 else 0

with wave.open(path) as w:
    sr = w.getframerate()
    x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, 2) / 32768.0
seg = x[int(t0 * sr):int(t1 * sr)].mean(axis=1)
rms = np.sqrt(np.mean(seg ** 2))
peak = np.max(np.abs(seg))
print(f"{path}  window {t0}-{t1}s  rms {20*np.log10(rms+1e-12):.1f} dBFS  crest {peak/(rms+1e-12):.1f}")
n = len(seg)
spec = np.abs(np.fft.rfft(seg * np.hanning(n)))
f = np.fft.rfftfreq(n, 1 / sr)
spec_db = 20 * np.log10(spec / spec.max() + 1e-9)
# strongest peaks below 1.5 kHz
mask = (f > 15) & (f < 1500)
idx = np.where(mask)[0]
peaks = [i for i in idx[1:-1] if spec[i] > spec[i - 1] and spec[i] > spec[i + 1]]
peaks = sorted(peaks, key=lambda i: -spec[i])[:8]
for i in sorted(peaks):
    order = f"  order {f[i] / (rpm / 60):.2f}" if rpm > 0 else ""
    print(f"   {f[i]:7.1f} Hz  {spec_db[i]:6.1f} dB{order}")
# spectral centroid (brightness)
cen = np.sum(f * spec) / np.sum(spec)
print(f"   spectral centroid {cen:.0f} Hz")
