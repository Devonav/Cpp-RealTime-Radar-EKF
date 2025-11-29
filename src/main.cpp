#define WIN32_LEAN_AND_MEAN
#include <atomic>
#include <cmath>
#include <d3d11.h>
#include <iostream>
#include <string>
#include <tchar.h>
#include <thread>
#include <vector>
#include <windows.h>
#include <winsock2.h>

#include "backends/imgui_impl_dx11.h"
#include "backends/imgui_impl_win32.h"
#include "imgui.h"

#include "Protocol.h"
#include "network/UdpSocket.h"
#include "radar/ThreadSafeQueue.h"
#include "radar/TrackManager.h"

// Data
static ID3D11Device *g_pd3dDevice = nullptr;
static ID3D11DeviceContext *g_pd3dDeviceContext = nullptr;
static IDXGISwapChain *g_pSwapChain = nullptr;
static bool g_SwapChainOccluded = false;
static UINT g_ResizeWidth = 0, g_ResizeHeight = 0;
static ID3D11RenderTargetView *g_mainRenderTargetView = nullptr;

// Aegis System Components
aegis::ThreadSafeQueue<aegis::Plot> g_packetQueue;
aegis::TrackManager g_trackManager;
std::atomic<bool> g_running = true;

// Forward declarations of helper functions
bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Receiver Thread Function
void ReceiverThread() {
  try {
    aegis::net::UdpSocket socket;
    socket.Bind(5000);
    // socket.SetNonBlocking(false); // Blocking receive

    std::cout << "Receiver Thread Started on Port 5000" << std::endl;

    while (g_running) {
      aegis::Plot plot;
      std::string senderAddr;
      int senderPort;

      // Use a small timeout or non-blocking if possible to check g_running,
      // but for now blocking is fine as we can kill the app.
      // Actually, if we block forever, joining the thread at exit might hang if
      // no packets come. Let's set a timeout if we could, but UdpSocket wrapper
      // is simple. We'll just rely on OS closing the socket or detaching.

      int bytes =
          socket.ReceiveFrom(&plot, sizeof(plot), senderAddr, senderPort);
      if (bytes == sizeof(plot)) {
        g_packetQueue.Push(plot);
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "Receiver Error: " << e.what() << std::endl;
  }
}

// Visualization Helper
void DrawPPIScope(ImDrawList *draw_list, const ImVec2 &center, float radius,
                  const std::vector<std::shared_ptr<aegis::Track>> &tracks) {
  // Draw Radar Circle
  draw_list->AddCircle(center, radius, IM_COL32(0, 255, 0, 255), 64, 2.0f);
  draw_list->AddCircle(center, radius * 0.75f, IM_COL32(0, 255, 0, 100), 64,
                       1.0f);
  draw_list->AddCircle(center, radius * 0.5f, IM_COL32(0, 255, 0, 100), 64,
                       1.0f);
  draw_list->AddCircle(center, radius * 0.25f, IM_COL32(0, 255, 0, 100), 64,
                       1.0f);

  // Draw Crosshairs
  draw_list->AddLine(ImVec2(center.x - radius, center.y),
                     ImVec2(center.x + radius, center.y),
                     IM_COL32(0, 255, 0, 100));
  draw_list->AddLine(ImVec2(center.x, center.y - radius),
                     ImVec2(center.x, center.y + radius),
                     IM_COL32(0, 255, 0, 100));

  // Draw Tracks
  // Map World Coordinates (-10000 to 10000) to Screen Coordinates
  float scale = radius / 10000.0f; // 10km range

  for (const auto &track : tracks) {
    glm::vec2 pos = track->GetPosition();
    ImVec2 screenPos(center.x + pos.x * scale,
                     center.y - pos.y * scale); // Invert Y for screen

    // Draw History Trail
    const auto &history = track->GetHistory();
    for (size_t i = 1; i < history.size(); ++i) {
      ImVec2 p1(center.x + history[i - 1].x * scale,
                center.y - history[i - 1].y * scale);
      ImVec2 p2(center.x + history[i].x * scale,
                center.y - history[i].y * scale);
      draw_list->AddLine(p1, p2, IM_COL32(0, 255, 0, 150), 1.0f);
    }

    // Draw Current Position (Blip)
    draw_list->AddCircleFilled(screenPos, 4.0f, IM_COL32(255, 0, 0, 255));

    // Draw ID
    std::string idStr = "TRK " + std::to_string(track->GetId());
    draw_list->AddText(ImVec2(screenPos.x + 5, screenPos.y + 5),
                       IM_COL32(255, 255, 255, 255), idStr.c_str());

    // Draw Velocity Vector
    glm::vec2 vel = track->GetVelocity();
    ImVec2 velEnd(screenPos.x + vel.x * scale * 10.0f,
                  screenPos.y -
                      vel.y * scale * 10.0f); // Scale velocity for visibility
    draw_list->AddLine(screenPos, velEnd, IM_COL32(255, 255, 0, 200), 1.0f);
  }

  // Draw Sweeping Line (Simulated)
  static float sweepAngle = 0.0f;
  sweepAngle += 0.05f;
  if (sweepAngle > 6.28f)
    sweepAngle = 0.0f;

  ImVec2 sweepEnd(center.x + cos(sweepAngle) * radius,
                  center.y + sin(sweepAngle) * radius);
  draw_list->AddLine(center, sweepEnd, IM_COL32(0, 255, 0, 200), 2.0f);
}

// Main code
int main(int, char **) {
  // Make process DPI aware and obtain main monitor scale
  ImGui_ImplWin32_EnableDpiAwareness();
  float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(
      ::MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));

  // Create application window
  WNDCLASSEXW wc = {sizeof(wc),
                    CS_CLASSDC,
                    WndProc,
                    0L,
                    0L,
                    GetModuleHandle(nullptr),
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    L"ImGui Example",
                    nullptr};
  ::RegisterClassExW(&wc);
  HWND hwnd = ::CreateWindowW(wc.lpszClassName, L"Aegis Radar Tracker",
                              WS_OVERLAPPEDWINDOW, 100, 100,
                              (int)(1280 * main_scale), (int)(800 * main_scale),
                              nullptr, nullptr, wc.hInstance, nullptr);

  // Initialize Direct3D
  if (!CreateDeviceD3D(hwnd)) {
    CleanupDeviceD3D();
    ::UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 1;
  }

  // Show the window
  ::ShowWindow(hwnd, SW_SHOWDEFAULT);
  ::UpdateWindow(hwnd);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Custom Style for "Military" look
  ImGuiStyle &style = ImGui::GetStyle();
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.05f, 0.05f, 0.9f);
  style.Colors[ImGuiCol_TitleBg] = ImVec4(0.1f, 0.1f, 0.1f, 1.0f);
  style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.2f, 0.2f, 0.2f, 1.0f);
  style.ScaleAllSizes(main_scale);
  style.FontScaleDpi = main_scale;

  // Setup Platform/Renderer backends
  ImGui_ImplWin32_Init(hwnd);
  ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

  // Start Receiver Thread
  std::thread receiver(ReceiverThread);
  receiver.detach(); // Detach for now, simple exit

  ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

  // Main loop
  bool done = false;
  while (!done) {
    // Poll and handle messages
    MSG msg;
    while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
      ::TranslateMessage(&msg);
      ::DispatchMessage(&msg);
      if (msg.message == WM_QUIT)
        done = true;
    }
    if (done)
      break;

    // Handle window resize
    if (g_ResizeWidth != 0 && g_ResizeHeight != 0) {
      CleanupRenderTarget();
      g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight,
                                  DXGI_FORMAT_UNKNOWN, 0);
      g_ResizeWidth = g_ResizeHeight = 0;
      CreateRenderTarget();
    }

    // --- Core Logic ---
    // Process incoming packets
    while (!g_packetQueue.Empty()) {
      auto plot = g_packetQueue.Pop();
      g_trackManager.ProcessPlot(plot.id, plot.x, plot.y, plot.timestamp);
    }

    // Prune old tracks
    double currentTime =
        std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch())
            .count();
    g_trackManager.PruneTracks(currentTime);

    // --- Rendering ---
    ImGui_ImplDX11_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    // 1. PPI Scope Window
    {
      ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(600, 600), ImGuiCond_FirstUseEver);
      ImGui::Begin("PPI Scope", nullptr,
                   ImGuiWindowFlags_NoScrollbar |
                       ImGuiWindowFlags_NoScrollWithMouse);

      ImVec2 winPos = ImGui::GetWindowPos();
      ImVec2 winSize = ImGui::GetWindowSize();
      ImVec2 center(winPos.x + winSize.x * 0.5f,
                    winPos.y + winSize.y * 0.5f + 20); // Offset for title bar
      float radius = (std::min(winSize.x, winSize.y) * 0.4f);

      auto tracks = g_trackManager.GetTracks();
      DrawPPIScope(ImGui::GetWindowDrawList(), center, radius, tracks);

      ImGui::End();
    }

    // 2. Track Table Window
    {
      ImGui::SetNextWindowPos(ImVec2(620, 10), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);
      ImGui::Begin("Track Table");

      if (ImGui::BeginTable("Tracks", 4,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("ID");
        ImGui::TableSetupColumn("Pos X");
        ImGui::TableSetupColumn("Pos Y");
        ImGui::TableSetupColumn("Velocity");
        ImGui::TableHeadersRow();

        auto tracks = g_trackManager.GetTracks();
        for (const auto &track : tracks) {
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          ImGui::Text("%d", track->GetId());
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("%.1f", track->GetPosition().x);
          ImGui::TableSetColumnIndex(2);
          ImGui::Text("%.1f", track->GetPosition().y);
          ImGui::TableSetColumnIndex(3);
          glm::vec2 v = track->GetVelocity();
          float speed = std::sqrt(v.x * v.x + v.y * v.y);
          ImGui::Text("%.1f m/s", speed);
        }
        ImGui::EndTable();
      }
      ImGui::End();
    }

    // Rendering
    ImGui::Render();
    const float clear_color_with_alpha[4] = {
        clear_color.x * clear_color.w, clear_color.y * clear_color.w,
        clear_color.z * clear_color.w, clear_color.w};
    g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView,
                                            nullptr);
    g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView,
                                               clear_color_with_alpha);
    ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

    HRESULT hr = g_pSwapChain->Present(1, 0);
    g_SwapChainOccluded = (hr == DXGI_STATUS_OCCLUDED);
  }

  g_running = false;
  // receiver.join(); // If we didn't detach

  // Cleanup
  ImGui_ImplDX11_Shutdown();
  ImGui_ImplWin32_Shutdown();
  ImGui::DestroyContext();

  CleanupDeviceD3D();
  ::DestroyWindow(hwnd);
  ::UnregisterClassW(wc.lpszClassName, wc.hInstance);

  return 0;
}

// Helper functions (Same as before)
bool CreateDeviceD3D(HWND hWnd) {
  DXGI_SWAP_CHAIN_DESC sd;
  ZeroMemory(&sd, sizeof(sd));
  sd.BufferCount = 2;
  sd.BufferDesc.Width = 0;
  sd.BufferDesc.Height = 0;
  sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  sd.BufferDesc.RefreshRate.Numerator = 60;
  sd.BufferDesc.RefreshRate.Denominator = 1;
  sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
  sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  sd.OutputWindow = hWnd;
  sd.SampleDesc.Count = 1;
  sd.SampleDesc.Quality = 0;
  sd.Windowed = TRUE;
  sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

  UINT createDeviceFlags = 0;
  D3D_FEATURE_LEVEL featureLevel;
  const D3D_FEATURE_LEVEL featureLevelArray[2] = {
      D3D_FEATURE_LEVEL_11_0,
      D3D_FEATURE_LEVEL_10_0,
  };
  HRESULT res = D3D11CreateDeviceAndSwapChain(
      nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags,
      featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain,
      &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
  if (res == DXGI_ERROR_UNSUPPORTED)
    res = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_WARP, nullptr, createDeviceFlags,
        featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain,
        &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
  if (res != S_OK)
    return false;

  CreateRenderTarget();
  return true;
}

void CleanupDeviceD3D() {
  CleanupRenderTarget();
  if (g_pSwapChain) {
    g_pSwapChain->Release();
    g_pSwapChain = nullptr;
  }
  if (g_pd3dDeviceContext) {
    g_pd3dDeviceContext->Release();
    g_pd3dDeviceContext = nullptr;
  }
  if (g_pd3dDevice) {
    g_pd3dDevice->Release();
    g_pd3dDevice = nullptr;
  }
}

void CreateRenderTarget() {
  ID3D11Texture2D *pBackBuffer;
  g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
  g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr,
                                       &g_mainRenderTargetView);
  pBackBuffer->Release();
}

void CleanupRenderTarget() {
  if (g_mainRenderTargetView) {
    g_mainRenderTargetView->Release();
    g_mainRenderTargetView = nullptr;
  }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd,
                                                             UINT msg,
                                                             WPARAM wParam,
                                                             LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
  if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
    return true;

  switch (msg) {
  case WM_SIZE:
    if (wParam == SIZE_MINIMIZED)
      return 0;
    g_ResizeWidth = (UINT)LOWORD(lParam);
    g_ResizeHeight = (UINT)HIWORD(lParam);
    return 0;
  case WM_SYSCOMMAND:
    if ((wParam & 0xfff0) == SC_KEYMENU)
      return 0;
    break;
  case WM_DESTROY:
    ::PostQuitMessage(0);
    return 0;
  }
  return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}
