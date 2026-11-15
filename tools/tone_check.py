"""Checks whether a tonal component (e.g. supercharger whine) stands out of the spectrum.

usage: python tools/tone_check.py file.wav t0 t1 freq1 [freq2 ...]
For each frequency prints the peak level within +-3 % and how far it sits above the
median level of the surrounding band (+-25 %). > ~10 dB means clearly audible as a tone.
"""
import sys, wave
import numpy as np

path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
freqs = [float(f) for f in sys.argv[4:]]
with wave.open(path) as w:
    sr = w.getframerate()
    x = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).reshape(-1, 2) / 32768.0
seg = x[int(t0 * sr):int(t1 * sr)].mean(axis=1)
spec = np.abs(np.fft.rfft(seg * np.hanning(len(seg))))
f = np.fft.rfftfreq(len(seg), 1 / sr)
db = 20 * np.log10(spec / spec.max() + 1e-12)
for fr in freqs:
    peak_band = (f > fr * 0.97) & (f < fr * 1.03)
    around = ((f > fr * 0.75) & (f < fr * 0.95)) | ((f > fr * 1.05) & (f < fr * 1.25))
    pk = db[peak_band].max()
    med = np.median(db[around])
    print(f"{path} {t0}-{t1}s  {fr:7.0f} Hz  peak {pk:6.1f} dB  above surroundings {pk - med:5.1f} dB")
