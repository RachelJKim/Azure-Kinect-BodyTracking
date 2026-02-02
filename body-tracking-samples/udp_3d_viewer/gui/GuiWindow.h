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

    bool Create(const char* title, int x = 100, int y = 100, int width = 320, int height = 160);
    void Show(int nCmdShow = SW_SHOW);
    void Destroy();
    void PumpMessages();

    bool ConsumeToggleRequest();
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
    HWND m_statusLabel = nullptr;

    bool m_streaming = false;
    bool m_toggleRequested = false;
    std::string m_ipAddress = "127.0.0.1";
};
