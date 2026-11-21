// EngineSim - Win32 + Direct3D 11 + Dear ImGui host.
#include "ui/App.h"
#include "ui/Gpu3D.h"
#include "resource.h"
#include "ecu/BaseMap.h"
#include "ecu/CalibrationIO.h"
#include "ecu/EngineFile.h"
#include <string>
#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"
#include "implot3d.h"
#include <d3d11.h>
#include <timeapi.h>
#include <windows.h>
#include <chrono>

static ID3D11Device* g_device = nullptr;
static ID3D11DeviceContext* g_context = nullptr;
static IDXGISwapChain* g_swapChain = nullptr;
static ID3D11RenderTargetView* g_rtv = nullptr;
static UINT g_resizeW = 0, g_resizeH = 0;

static void createRenderTarget() {
    ID3D11Texture2D* back = nullptr;
    g_swapChain->GetBuffer(0, IID_PPV_ARGS(&back));
    if (back) { g_device->CreateRenderTargetView(back, nullptr, &g_rtv); back->Release(); }
}
static void cleanupRenderTarget() { if (g_rtv) { g_rtv->Release(); g_rtv = nullptr; } }

static bool createDevice(HWND hwnd) {
    DXGI_SWAP_CHAIN_DESC sd = {};
    sd.BufferCount = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60;
    sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hwnd;
    sd.SampleDesc.Count = 1;
    sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0};
    D3D_FEATURE_LEVEL got;
    HRESULT hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2, D3D11_SDK_VERSION,
                                               &sd, &g_swapChain, &g_device, &got, &g_context);
    if (hr == DXGI_ERROR_UNSUPPORTED)
        hr = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 2, D3D11_SDK_VERSION,
                                           &sd, &g_swapChain, &g_device, &got, &g_context);
    if (FAILED(hr)) return false;
    createRenderTarget();
    return true;
}

static void cleanupDevice() {
    cleanupRenderTarget();
    if (g_swapChain) g_swapChain->Release();
    if (g_context) g_context->Release();
    if (g_device) g_device->Release();
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

static LRESULT WINAPI wndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;
    switch (msg) {
    case WM_SIZE:
        if (wParam == SIZE_MINIMIZED) return 0;
        g_resizeW = (UINT)LOWORD(lParam);
        g_resizeH = (UINT)HIWORD(lParam);
        return 0;
    case WM_SYSCOMMAND:
        if ((wParam & 0xfff0) == SC_KEYMENU) return 0;
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static void setStyle(float scale) {
    ImGuiStyle& s = ImGui::GetStyle();
    ImGui::StyleColorsDark();
    // dense, dark "tuning software" look: small paddings, flat panels, one warm accent colour
    s.WindowRounding = 2; s.ChildRounding = 2; s.FrameRounding = 2; s.GrabRounding = 2; s.TabRounding = 2;
    s.FramePadding = ImVec2(5, 3); s.ItemSpacing = ImVec2(6, 4); s.WindowPadding = ImVec2(6, 6);
    s.ChildBorderSize = 1; s.FrameBorderSize = 0;
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.065f, 0.067f, 0.075f, 1);
    c[ImGuiCol_ChildBg] = ImVec4(0.085f, 0.087f, 0.097f, 1);
    c[ImGuiCol_Border] = ImVec4(0.20f, 0.21f, 0.24f, 1);
    c[ImGuiCol_Separator] = ImVec4(0.22f, 0.23f, 0.26f, 1);
    c[ImGuiCol_FrameBg] = ImVec4(0.15f, 0.155f, 0.17f, 1);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.22f, 0.25f, 1);
    c[ImGuiCol_SliderGrab] = ImVec4(0.95f, 0.5f, 0.2f, 1);
    c[ImGuiCol_SliderGrabActive] = ImVec4(1.0f, 0.65f, 0.3f, 1);
    c[ImGuiCol_CheckMark] = ImVec4(1.0f, 0.6f, 0.25f, 1);
    c[ImGuiCol_Header] = ImVec4(0.55f, 0.3f, 0.15f, 0.6f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.7f, 0.38f, 0.18f, 0.8f);
    c[ImGuiCol_Button] = ImVec4(0.25f, 0.25f, 0.28f, 1);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.6f, 0.33f, 0.16f, 1);
    c[ImGuiCol_Tab] = ImVec4(0.18f, 0.18f, 0.2f, 1);
    c[ImGuiCol_TabActive] = ImVec4(0.6f, 0.32f, 0.15f, 1);
    c[ImGuiCol_TabHovered] = ImVec4(0.7f, 0.38f, 0.18f, 1);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.2f, 0.12f, 0.08f, 1);
    s.ScaleAllSizes(scale);
}

int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, PWSTR cmdLine, int) {
    // optional: --preset N  (start with preset N loaded)
    int startPreset = 0;
    if (const wchar_t* s = cmdLine ? wcsstr(cmdLine, L"--preset") : nullptr) startPreset = _wtoi(s + 8);
    // optional: --section N  (open navigation section N, 1..12)
    int startSection = 0;
    if (const wchar_t* s = cmdLine ? wcsstr(cmdLine, L"--section") : nullptr) startSection = _wtoi(s + 9);
    // optional: --tab KEY  (open a tab of that section, e.g. --section 9 --tab flowmap) and --start (start the engine)
    char startTab[32] = {};
    if (const wchar_t* s = cmdLine ? wcsstr(cmdLine, L"--tab ") : nullptr)
        for (int i = 0; i < 31 && s[6 + i] && s[6 + i] != L' '; ++i) startTab[i] = (char)s[6 + i];
    const bool startEngine = cmdLine && wcsstr(cmdLine, L"--start");
    ImGui_ImplWin32_EnableDpiAwareness();
    timeBeginPeriod(1); // 1 ms sleep granularity for the simulation thread

    WNDCLASSEXW wc = {sizeof(wc), CS_CLASSDC, wndProc, 0, 0, hInst, nullptr, nullptr, nullptr, nullptr, L"EngineSimWnd", nullptr};
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    // app icon (src/app.rc): the large one for Alt-Tab / taskbar, the small one for the title bar
    wc.hIcon = LoadIconW(hInst, MAKEINTRESOURCEW(IDI_APPICON));
    wc.hIconSm = (HICON)LoadImageW(hInst, MAKEINTRESOURCEW(IDI_APPICON), IMAGE_ICON, GetSystemMetrics(SM_CXSMICON),
                                   GetSystemMetrics(SM_CYSMICON), LR_DEFAULTCOLOR);
    RegisterClassExW(&wc);
    HWND hwnd = CreateWindowW(wc.lpszClassName, L"EngineSim - real-time engine simulator", WS_OVERLAPPEDWINDOW,
                              60, 40, 1680, 1000, nullptr, nullptr, hInst, nullptr);
    if (!createDevice(hwnd)) {
        cleanupDevice();
        UnregisterClassW(wc.lpszClassName, hInst);
        MessageBoxW(nullptr, L"Direct3D 11 is not available.", L"EngineSim", MB_ICONERROR);
        return 1;
    }
    ShowWindow(hwnd, SW_SHOWMAXIMIZED);
    UpdateWindow(hwnd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImPlot3D::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = "enginesim_layout.ini";
    float dpi = ImGui_ImplWin32_GetDpiScaleForHwnd(hwnd);
    ImFontConfig fc;
    fc.SizePixels = 15.0f * dpi;
    io.Fonts->AddFontDefault(&fc);
    setStyle(dpi);
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_device, g_context);
    es::ui::gpu::setDevice(g_device, g_context);   // the engine view's 3D renderer (falls back to the flat view)

    {
        {   // measured base maps are cached next to the executable
            wchar_t exe[MAX_PATH]; GetModuleFileNameW(nullptr, exe, MAX_PATH);
            std::wstring root(exe); root = root.substr(0, root.find_last_of(L"\\/"));
            auto utf8 = [](const std::wstring& w) {
                int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, nullptr, 0, nullptr, nullptr);
                std::string s(n > 0 ? n - 1 : 0, '\0');
                if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w.c_str(), -1, &s[0], n, nullptr, nullptr);
                return s;
            };
            CreateDirectoryW((root + L"\\basemaps").c_str(), nullptr);
            es::setBaseMapCacheDir(utf8(root + L"\\basemaps"));
            es::setCalibrationDir(utf8(root + L"\\calibrations"));   // created on the first save
            es::setEngineDir(utf8(root + L"\\engines"));             // your own / imported engines
        }
        es::App app(startPreset);
        if (startSection >= 1 && startSection <= es::App::kSectionCount) app.setSection(startSection - 1);
        if (startTab[0]) app.openTab(startTab);
        if (startEngine) app.startEngine();
        auto last = std::chrono::steady_clock::now();
        bool done = false;
        while (!done) {
            MSG msg;
            while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
                if (msg.message == WM_QUIT) done = true;
            }
            if (done) break;
            if (g_resizeW != 0 && g_resizeH != 0) {
                cleanupRenderTarget();
                g_swapChain->ResizeBuffers(0, g_resizeW, g_resizeH, DXGI_FORMAT_UNKNOWN, 0);
                g_resizeW = g_resizeH = 0;
                createRenderTarget();
            }
            auto now = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(now - last).count();
            last = now;

            ImGui_ImplDX11_NewFrame();
            ImGui_ImplWin32_NewFrame();
            ImGui::NewFrame();
            app.frame(std::min(dt, 0.1f));
            ImGui::Render();
            const float clear[4] = {0.05f, 0.05f, 0.06f, 1.0f};
            g_context->OMSetRenderTargets(1, &g_rtv, nullptr);
            g_context->ClearRenderTargetView(g_rtv, clear);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
            g_swapChain->Present(1, 0);
        }
    }

    es::ui::gpu::shutdown();
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot3D::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    cleanupDevice();
    DestroyWindow(hwnd);
    UnregisterClassW(wc.lpszClassName, hInst);
    timeEndPeriod(1);
    return 0;
}
