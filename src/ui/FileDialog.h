#pragma once
// Win32 open / save file dialogs (UTF-8 paths).
#include <string>

namespace es::ui {
// filter: double-zero terminated pairs, e.g. L"Logs (*.csv)\0*.csv\0All files\0*.*\0"
bool fileDialog(bool save, const std::string& suggested, std::string& out, const wchar_t* filter, const wchar_t* defExt,
                const wchar_t* title);
} // namespace es::ui
