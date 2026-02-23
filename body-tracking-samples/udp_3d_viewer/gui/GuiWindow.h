// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once

#include <string>
#include <windows.h>

class GuiWindow
{
public:
    GuiWindow() = default;
    ~GuiWindow();

    bool Create(const char* title, int x = 100, int y = 100, int width = 340, int height = 200);
    void Show(int nCmdShow = SW_SHOW);
    void Destroy();
    void PumpMessages();

    bool ConsumeToggleRequest();
    bool ConsumeExportRequest();
    void SetStreamingState(bool streaming);
    bool IsStreaming() const { return m_streaming; }
    std::string GetIpAddress() const { return m_ipAddress; }

private:
    static LRESULT CALLBACK WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    LRESULT HandleMessage(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
    void CreateControls(HWND hwnd);
    void UpdateStatusText();
    void UpdateButtonText();
    void UpdateIpFromEdit();

    HWND m_hwnd = nullptr;
    HWND m_ipEdit = nullptr;
    HWND m_toggleButton = nullptr;
    HWND m_exportButton = nullptr;
    HWND m_statusLabel = nullptr;

    bool m_streaming = false;
    bool m_toggleRequested = false;
    bool m_exportRequested = false;
    std::string m_ipAddress = "127.0.0.1";
};
