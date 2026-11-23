// Game controller input: XInput first (Xbox pads and anything presenting itself as one), then DirectInput for the
// rest - a DualShock 4 / DualSense plugged in or paired directly, other HID pads. XInput is loaded at run time (no
// link dependency); DirectInput 8 through dinput8.
#define DIRECTINPUT_VERSION 0x0800
#include "ui/Gamepad.h"
#include <windows.h>
#include <dinput.h>
#include <xinput.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cwctype>
#include <string>

namespace es::ui {

namespace {
enum Btn : uint32_t {
    kUp = 1u << 0, kDown = 1u << 1, kNeutral = 1u << 2, kStart = 1u << 3, kIgn = 1u << 4, kDrive = 1u << 5,
    kLisPrev = 1u << 6, kLisNext = 1u << 7, kBrake = 1u << 8,
};
typedef DWORD(WINAPI* XInputGetStateFn)(DWORD, XINPUT_STATE*);
typedef DWORD(WINAPI* XInputSetStateFn)(DWORD, XINPUT_VIBRATION*);
float trig(float v, float dead) { return v <= dead ? 0.0f : std::min(1.0f, (v - dead) / (1.0f - dead)); }
}

struct Gamepad::Impl {
    HMODULE xi = nullptr;
    XInputGetStateFn xget = nullptr;
    XInputSetStateFn xset = nullptr;
    int xUser = -1;
    IDirectInput8W* di = nullptr;
    IDirectInputDevice8W* dev = nullptr;
    std::string diName;
    bool ps = false;   // a Sony pad: its known layout (else a generic DirectInput pad, same layout assumed)
    // a DualShock 4 / DualSense on USB: its HID device, opened for the rumble output report
    HANDLE hid = INVALID_HANDLE_VALUE;
    int sony = 0;      // 0 none, 4 DualShock 4, 5 DualSense

    Impl() {
        for (const char* dll : {"xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"})
            if ((xi = LoadLibraryA(dll))) break;
        if (xi) {
            xget = (XInputGetStateFn)GetProcAddress(xi, "XInputGetState");
            xset = (XInputSetStateFn)GetProcAddress(xi, "XInputSetState");
        }
        DirectInput8Create(GetModuleHandleW(nullptr), DIRECTINPUT_VERSION, IID_IDirectInput8W, (void**)&di, nullptr);
    }
    ~Impl() {
        rumble(0, 0);
        dropDev();
        if (di) di->Release();
        if (xi) FreeLibrary(xi);
    }
    void dropDev() {
        if (dev) { dev->Unacquire(); dev->Release(); dev = nullptr; }
        if (hid != INVALID_HANDLE_VALUE) { CloseHandle(hid); hid = INVALID_HANDLE_VALUE; }
        sony = 0;
        diName.clear();
    }
    // the Sony pad's HID interface (DirectInput gives its path): a USB one only - see Gamepad.h
    void openHid(WORD pid) {
        DIPROPGUIDANDPATH gp{};
        gp.diph.dwSize = sizeof(gp); gp.diph.dwHeaderSize = sizeof(DIPROPHEADER); gp.diph.dwHow = DIPH_DEVICE;
        if (FAILED(dev->GetProperty(DIPROP_GUIDANDPATH, &gp.diph))) return;
        std::wstring path = gp.wszPath;
        for (wchar_t& c : path) c = (wchar_t)towlower(c);
        if (path.find(L"00001124-0000-1000-8000-00805f9b34fb") != std::wstring::npos || path.find(L"_vid&0002") != std::wstring::npos)
            return;   // Bluetooth
        const int kind = (pid == 0x05C4 || pid == 0x09CC || pid == 0x0BA0) ? 4 : (pid == 0x0CE6 || pid == 0x0DF2) ? 5 : 0;
        if (!kind) return;
        hid = CreateFileW(gp.wszPath, GENERIC_WRITE | GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        if (hid == INVALID_HANDLE_VALUE)
            hid = CreateFileW(gp.wszPath, GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        if (hid != INVALID_HANDLE_VALUE) sony = kind;
    }
    // motors 0..1 -> the pad. XInput: left = low frequency (big), right = high frequency (small). Sony USB output
    // reports (as SDL's hidapi drivers write them): DualShock 4 report 0x05, 32 bytes - [1] 0x01 = set the rumble only
    // (not the light bar), [4] small motor, [5] big motor; DualSense report 0x02, 48 bytes - [1] 0x03 = rumble
    // emulation on, audio haptics off, [3] small (right), [4] big (left)
    bool rumble(float lo, float hi) {
        const auto b8 = [](float v) { return (BYTE)std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f); };
        if (xUser >= 0 && xset) {
            XINPUT_VIBRATION v{};
            v.wLeftMotorSpeed = (WORD)std::lround(std::clamp(lo, 0.0f, 1.0f) * 65535.0f);
            v.wRightMotorSpeed = (WORD)std::lround(std::clamp(hi, 0.0f, 1.0f) * 65535.0f);
            return xset(xUser, &v) == ERROR_SUCCESS;
        }
        if (hid != INVALID_HANDLE_VALUE && sony) {
            BYTE r[64] = {};
            DWORD len;
            if (sony == 4) { r[0] = 0x05; r[1] = 0x01; r[4] = b8(hi); r[5] = b8(lo); len = 32; }
            else { r[0] = 0x02; r[1] = 0x03; r[3] = b8(hi); r[4] = b8(lo); len = 48; }
            DWORD wrote = 0;
            if (!WriteFile(hid, r, len, &wrote, nullptr)) { CloseHandle(hid); hid = INVALID_HANDLE_VALUE; sony = 0; return false; }
            return true;
        }
        return false;
    }
    static BOOL CALLBACK enumCb(const DIDEVICEINSTANCEW* inst, void* ctx) {
        auto* self = static_cast<Impl*>(ctx);
        // (an Xbox pad shows up here too: XInput already has it - skip "XINPUT" HID devices, VID 045E is Microsoft)
        const DWORD vidpid = inst->guidProduct.Data1;
        const WORD vid = LOWORD(vidpid);
        if (vid == 0x045E) return DIENUM_CONTINUE;
        IDirectInputDevice8W* d = nullptr;
        if (FAILED(self->di->CreateDevice(inst->guidInstance, &d, nullptr))) return DIENUM_CONTINUE;
        if (FAILED(d->SetDataFormat(&c_dfDIJoystick2))) { d->Release(); return DIENUM_CONTINUE; }
        self->dev = d;
        self->ps = vid == 0x054C;   // Sony
        self->pid = HIWORD(vidpid);
        char nm[MAX_PATH];
        WideCharToMultiByte(CP_UTF8, 0, inst->tszProductName, -1, nm, sizeof(nm), nullptr, nullptr);
        self->diName = nm;
        return DIENUM_STOP;
    }
    void scanDi(HWND hwnd) {
        if (!di || dev) return;
        di->EnumDevices(DI8DEVCLASS_GAMECTRL, enumCb, this, DIEDFL_ATTACHEDONLY);
        if (!dev) return;
        if (hwnd) dev->SetCooperativeLevel(hwnd, DISCL_BACKGROUND | DISCL_NONEXCLUSIVE);
        // every axis 0..1000
        DIPROPRANGE r{};
        r.diph.dwSize = sizeof(r); r.diph.dwHeaderSize = sizeof(DIPROPHEADER); r.diph.dwHow = DIPH_DEVICE; r.diph.dwObj = 0;
        r.lMin = 0; r.lMax = 1000;
        dev->SetProperty(DIPROP_RANGE, &r.diph);
        dev->Acquire();
        if (ps) openHid(pid);
    }
    WORD pid = 0;
};

Gamepad::~Gamepad() { delete impl_; }

void Gamepad::poll(void* hwndv) {
    HWND hwnd = (HWND)hwndv;
    if (!impl_) impl_ = new Impl();
    Impl& I = *impl_;
    const double now = GetTickCount64() * 1e-3;
    PadState n;
    uint32_t held = 0;
    bool got = false;
    // ---- XInput (user 0..3, the first connected)
    if (I.xget) {
        if (I.xUser < 0 && now >= nextScan_) {
            for (int u = 0; u < 4 && I.xUser < 0; ++u) { XINPUT_STATE st{}; if (I.xget(u, &st) == ERROR_SUCCESS) I.xUser = u; }
        }
        if (I.xUser >= 0) {
            XINPUT_STATE st{};
            if (I.xget(I.xUser, &st) == ERROR_SUCCESS) {
                const XINPUT_GAMEPAD& g = st.Gamepad;
                n.connected = true; got = true;
                char nm[48]; std::snprintf(nm, sizeof(nm), "Xbox-style controller %d (XInput)", I.xUser + 1);
                n.name = nm;
                n.throttle = trig(g.bRightTrigger / 255.0f, 0.04f);
                n.clutch = trig(g.bLeftTrigger / 255.0f, 0.04f);
                const float ry = g.sThumbRY / 32767.0f;   // (up +)
                n.brake = trig(std::max(0.0f, -ry), 0.25f);
                const WORD b = g.wButtons;
                if (b & XINPUT_GAMEPAD_RIGHT_SHOULDER) held |= kUp;
                if (b & XINPUT_GAMEPAD_LEFT_SHOULDER) held |= kDown;
                if (b & XINPUT_GAMEPAD_Y) held |= kNeutral;
                if (b & XINPUT_GAMEPAD_START) held |= kStart;
                if (b & XINPUT_GAMEPAD_BACK) held |= kIgn;
                if (b & XINPUT_GAMEPAD_B) held |= kDrive;
                if (b & XINPUT_GAMEPAD_DPAD_LEFT) held |= kLisPrev;
                if (b & XINPUT_GAMEPAD_DPAD_RIGHT) held |= kLisNext;
                if (b & XINPUT_GAMEPAD_A) held |= kBrake;
            } else I.xUser = -1;
        }
    }
    // ---- DirectInput (a DualShock 4 / DualSense or another pad), when no XInput pad
    if (!got) {
        if (!I.dev && now >= nextScan_) I.scanDi(hwnd);
        if (I.dev) {
            DIJOYSTATE2 js{};
            HRESULT hr = I.dev->Poll();
            if (FAILED(hr)) { I.dev->Acquire(); hr = I.dev->Poll(); }
            hr = I.dev->GetDeviceState(sizeof(js), &js);
            if (FAILED(hr)) {
                if (hr == DIERR_INPUTLOST || hr == DIERR_NOTACQUIRED) { if (FAILED(I.dev->Acquire())) I.dropDev(); }
                else I.dropDev();
            } else {
                n.connected = true; got = true;
                n.name = I.diName + (I.ps ? " (DirectInput, PlayStation layout)" : " (DirectInput)");
                // Sony pads (DS4 / DualSense, HID): L2 / R2 on the Rx / Ry axes (0 released .. 1000 full), the right
                // stick on Z / Rz (Rz down = larger); buttons 0 Square, 1 Cross, 2 Circle, 3 Triangle, 4 L1, 5 R1,
                // 8 Share / Create, 9 Options; the D-pad on the first POV hat
                n.throttle = trig(js.lRy / 1000.0f, 0.04f);
                n.clutch = trig(js.lRx / 1000.0f, 0.04f);
                n.brake = trig(std::max(0.0f, (js.lRz - 500) / 500.0f), 0.25f);
                auto bt = [&](int i) { return (js.rgbButtons[i] & 0x80) != 0; };
                if (bt(5)) held |= kUp;
                if (bt(4)) held |= kDown;
                if (bt(3)) held |= kNeutral;
                if (bt(9)) held |= kStart;
                if (bt(8)) held |= kIgn;
                if (bt(2)) held |= kDrive;
                if (bt(1)) held |= kBrake;
                const DWORD pov = js.rgdwPOV[0];
                if (LOWORD(pov) != 0xFFFF) {
                    if (pov == 27000 || pov == 22500 || pov == 31500) held |= kLisPrev;
                    if (pov == 9000 || pov == 4500 || pov == 13500) held |= kLisNext;
                }
            }
        }
    }
    if (now >= nextScan_) nextScan_ = now + 2.0;
    if (held & kBrake) n.brake = 1.0f;
    const uint32_t press = held & ~held_;
    held_ = held;
    n.shiftUp = press & kUp; n.shiftDown = press & kDown; n.neutral = press & kNeutral; n.start = press & kStart;
    n.ignition = press & kIgn; n.driveMode = press & kDrive; n.listenerPrev = press & kLisPrev; n.listenerNext = press & kLisNext;
    s_ = n;
    rumbleOk_ = got && ((I.xUser >= 0 && I.xset) || I.sony != 0);
}

// (only when it changes by a step, or every half second to keep it alive)
void Gamepad::setRumble(float low, float high) {
    if (!impl_) return;
    const double now = GetTickCount64() * 1e-3;
    if (std::fabs(low - sentLo_) < 1.0f / 128 && std::fabs(high - sentHi_) < 1.0f / 128 && now - sentAt_ < 0.5) return;
    if (impl_->rumble(low, high)) { sentLo_ = low; sentHi_ = high; sentAt_ = now; }
    else sentLo_ = sentHi_ = -1;
}

} // namespace es::ui
