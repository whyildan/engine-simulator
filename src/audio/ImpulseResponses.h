#pragma once
#include <string>
#include <vector>

namespace es {

// IR choices: 0 off, 1 auto (from the muffler / vehicle), 2..6 built-in, 7 custom WAV file
enum IrChoice { IrOff = 0, IrAuto = 1, IrRawPipe = 2, IrSport = 3, IrStock = 4, IrRace = 5, IrMotorcycle = 6, IrCustom = 7 };
const char* irChoiceNames();                 // ImGui combo string ("Off\0Auto\0...")

// procedurally generated exhaust impulse response (direct pulse + dense early reflections of the
// pipes / perforations, a frequency-dependent decaying tail and a few ringing shell modes)
std::vector<float> makeBuiltinIR(int choice, double fs);

// loads a WAV file (PCM 16/24/32-bit or float, any channel count -> first channel), resamples to fs,
// trims trailing silence and normalises its energy. Returns false and an error message on failure.
bool loadWavIR(const std::wstring& path, double fs, std::vector<float>& out, std::string& err);

// scale an IR to unit energy (so switching IRs does not change loudness much)
void normaliseIR(std::vector<float>& h);

} // namespace es
