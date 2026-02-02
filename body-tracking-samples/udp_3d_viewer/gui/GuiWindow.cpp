// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "GuiWindow.h"

#include <array>

namespace
{
    constexpr int kIpEditId = 1001;
    constexpr int kToggleButtonId = 1002;
    constexpr int kStatusLabelId = 1003;
    constexpr const char* kWindowClassName = "Udp3dViewerGuiWindow";
}

GuiWindow::~GuiWindow()
{
    Destroy();
}

bool GuiWindow::Create(const char* title, int x, int y, int width, int height)
{
    if (m_hwnd != nullptr)
    {
        return true;
    }

    HINSTANCE instance = GetModuleHandle(nullptr);
    WNDCLASSEXA wc = {};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = GuiWindow::WindowProc;
    wc.hInstance = instance;
    wc.lpszClassName = kWindowClassName;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);

    if (!GetClassInfoExA(instance, kWindowClassName, &wc))
    {
        if (!RegisterClassExA(&wc))
        {
            return false;
        }
    }

    HWND hwnd = CreateWindowExA(
        0,
        kWindowClassName,
        title,
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        x,
        y,
        width,
        height,
        nullptr,
        nullptr,
        instance,
        this);

    if (!hwnd)
    {
        return false;
    }

    ShowWindow(hwnd, SW_SHOW);
    UpdateWindow(hwnd);
    return true;
}

void GuiWindow::Show(int nCmdShow)
{
    if (m_hwnd)
    {
        ShowWindow(m_hwnd, nCmdShow);
    }
}

void GuiWindow::Destroy()
{
    if (m_hwnd)
    {
        DestroyWindow(m_hwnd);
        m_hwnd = nullptr;
    }
}

void GuiWindow::PumpMessages()
{
    if (!m_hwnd)
    {
        return;
    }

    MSG msg = {};
    while (PeekMessage(&msg, m_hwnd, 0, 0, PM_REMOVE))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
}

bool GuiWindow::ConsumeToggleRequest()
{
    if (!m_toggleRequested)
    {
        return false;
    }

    m_toggleRequested = false;
    return true;
}

void GuiWindow::SetStreamingState(bool streaming)
{
    m_streaming = streaming;
    UpdateButtonText();
    UpdateStatusText();
}

LRESULT CALLBACK GuiWindow::WindowProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    if (msg == WM_NCCREATE)
    {
        auto* createStruct = reinterpret_cast<CREATESTRUCT*>(lParam);
        auto* self = reinterpret_cast<GuiWindow*>(createStruct->lpCreateParams);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        self->m_hwnd = hwnd;
    }

    auto* self = reinterpret_cast<GuiWindow*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (self)
    {
        return self->HandleMessage(hwnd, msg, wParam, lParam);
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}

LRESULT GuiWindow::HandleMessage(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg)
    {
    case WM_CREATE:
        CreateControls(hwnd);
        SetStreamingState(false);
        return 0;
    case WM_COMMAND:
        switch (LOWORD(wParam))
        {
        case kToggleButtonId:
            if (HIWORD(wParam) == BN_CLICKED)
            {
                m_toggleRequested = true;
            }
            return 0;
        case kIpEditId:
            if (HIWORD(wParam) == EN_CHANGE)
            {
                UpdateIpFromEdit();
            }
            return 0;
        default:
            break;
        }
        break;
    case WM_CLOSE:
        ShowWindow(hwnd, SW_HIDE);
        return 0;
    case WM_DESTROY:
        m_hwnd = nullptr;
        m_ipEdit = nullptr;
        m_toggleButton = nullptr;
        m_statusLabel = nullptr;
        return 0;
    default:
        break;
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}

void GuiWindow::CreateControls(HWND hwnd)
{
    const int margin = 10;
    const int labelWidth = 70;
    const int controlHeight = 22;
    const int editWidth = 220;
    const int buttonWidth = 80;

    HWND ipLabel = CreateWindowExA(
        0,
        "STATIC",
        "IP Address:",
        WS_CHILD | WS_VISIBLE,
        margin,
        margin + 2,
        labelWidth,
        controlHeight,
        hwnd,
        nullptr,
        nullptr,
        nullptr);

    m_ipEdit = CreateWindowExA(
        WS_EX_CLIENTEDGE,
        "EDIT",
        "",
        WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL,
        margin + labelWidth + 6,
        margin,
        editWidth,
        controlHeight,
        hwnd,
        reinterpret_cast<HMENU>(static_cast<intptr_t>(kIpEditId)),
        nullptr,
        nullptr);

    m_toggleButton = CreateWindowExA(
        0,
        "BUTTON",
        "Start",
        WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
        margin,
        margin + controlHeight + 12,
        buttonWidth,
        controlHeight + 4,
        hwnd,
        reinterpret_cast<HMENU>(static_cast<intptr_t>(kToggleButtonId)),
        nullptr,
        nullptr);

    m_statusLabel = CreateWindowExA(
        0,
        "STATIC",
        "Streaming: OFF",
        WS_CHILD | WS_VISIBLE,
        margin + buttonWidth + 10,
        margin + controlHeight + 16,
        160,
        controlHeight,
        hwnd,
        reinterpret_cast<HMENU>(static_cast<intptr_t>(kStatusLabelId)),
        nullptr,
        nullptr);

    if (m_ipEdit)
    {
        SetWindowTextA(m_ipEdit, m_ipAddress.c_str());
    }

    HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
    if (font)
    {
        if (ipLabel)
        {
            SendMessage(ipLabel, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        }
        if (m_ipEdit)
        {
            SendMessage(m_ipEdit, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        }
        if (m_toggleButton)
        {
            SendMessage(m_toggleButton, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        }
        if (m_statusLabel)
        {
            SendMessage(m_statusLabel, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        }
    }
}

void GuiWindow::UpdateStatusText()
{
    if (!m_statusLabel)
    {
        return;
    }

    const char* text = m_streaming ? "Streaming: ON" : "Streaming: OFF";
    SetWindowTextA(m_statusLabel, text);
}

void GuiWindow::UpdateButtonText()
{
    if (!m_toggleButton)
    {
        return;
    }

    const char* text = m_streaming ? "Stop" : "Start";
    SetWindowTextA(m_toggleButton, text);
}

void GuiWindow::UpdateIpFromEdit()
{
    if (!m_ipEdit)
    {
        return;
    }

    std::array<char, 64> buffer = {};
    GetWindowTextA(m_ipEdit, buffer.data(), static_cast<int>(buffer.size()));
    m_ipAddress = buffer.data();
}
