// Independent Linux/X11 monitor: no ROS, controller headers or control-thread hooks.
#include <X11/Xft/Xft.h>
#include <X11/Xatom.h>
#include <X11/Xutil.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <sched.h>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unordered_map>
#include <unistd.h>

namespace cpu_monitor {
using Ticks = unsigned long long;
using Clock = std::chrono::steady_clock;

struct Process {
  int parent = 0;
  char state = 0;
  Ticks ticks = 0;
  Ticks started = 0;
  bool project = false;
  bool related = false;
  Clock::time_point sampled{};
};

// /proc/PID/stat's comm can contain spaces and parentheses.
bool parseProcess(const std::string& line, Process& result) {
  const auto end = line.rfind(')');
  if (end == std::string::npos) return false;
  std::istringstream fields(line.substr(end + 1));
  if (!(fields >> result.state >> result.parent)) return false;
  std::string ignored;
  for (int field = 5; field <= 13; ++field) {
    if (!(fields >> ignored)) return false;
  }
  Ticks user = 0, system = 0;
  if (!(fields >> user >> system)) return false;
  for (int field = 16; field <= 21; ++field) {
    if (!(fields >> ignored)) return false;
  }
  if (!(fields >> result.started)) return false;
  // utime/stime already include all threads. Do not add children's cumulative times.
  result.ticks = user + system;
  return true;
}

bool readProcess(int pid, Process& result) {
  const auto before = Clock::now();
  std::ifstream file("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  if (!std::getline(file, line) || !parseProcess(line, result)) return false;
  result.sampled = before + (Clock::now() - before) / 2;
  return true;
}

struct System {
  Ticks total = 0;
  Ticks busy = 0;
  Clock::time_point sampled{};
};

bool readSystem(System& result) {
  result = System{};
  const auto before = Clock::now();
  std::ifstream file("/proc/stat");
  std::string name;
  std::array<Ticks, 8> fields{};
  if (!(file >> name) || name != "cpu") return false;
  // guest/guest_nice are already included in user/nice: count only the first 8.
  for (auto& value : fields) {
    if (!(file >> value)) return false;
    result.total += value;
  }
  // Sum busy counters directly; iowait can decrease and is not execution time.
  result.busy = fields[0] + fields[1] + fields[2] + fields[5] + fields[6] + fields[7];
  result.sampled = before + (Clock::now() - before) / 2;
  return true;
}

double percentage(double used, double total) {
  return total > 0 ? std::clamp(100.0 * used / total, 0.0, 100.0) : 0.0;
}

int colorIndex(double percent) {
  if (percent < 0) return 3;  // Neutral text when there is no valid measurement.
  return percent > 75.0 ? 2 : (percent >= 40.0 ? 1 : 0);
}

bool containsWorkspacePath(const std::string& text, const std::string& workspace) {
  std::size_t pos = 0;
  while ((pos = text.find(workspace, pos)) != std::string::npos) {
    // Avoid matching another workspace with the same prefix or embedded strings.
    const bool left = pos == 0 || text[pos - 1] == '\0' || text[pos - 1] == ' ' ||
                      text[pos - 1] == '=';
    const auto end = pos + workspace.size();
    const bool right = end == text.size() || text[end] == '/' || text[end] == '\0';
    if (left && right) return true;
    pos = end;
  }
  return false;
}

bool workspaceProcess(int pid, const std::string& workspace) {
  const auto base = "/proc/" + std::to_string(pid);
  std::array<char, 4096> executable{};
  const auto size = readlink((base + "/exe").c_str(), executable.data(), executable.size());
  if (size > 0 && containsWorkspacePath(std::string(executable.data(), size), workspace)) {
    return true;
  }
  std::ifstream cmdline(base + "/cmdline", std::ios::binary);
  std::string command((std::istreambuf_iterator<char>(cmdline)), {});
  if (containsWorkspacePath(command, workspace)) return true;
  // Finds plugins hosted by system executables (e.g. gazebo/ros2_control_node).
  std::ifstream maps(base + "/maps");
  std::string line;
  while (std::getline(maps, line)) {
    if (containsWorkspacePath(line, workspace)) return true;
  }
  return false;
}

using Processes = std::unordered_map<int, Process>;

void markRelated(Processes& processes, int launch_pid, int monitor_pid) {
  for (auto& entry : processes) {
    entry.second.related = entry.second.project || entry.first == launch_pid;
  }
  // Includes launch children, and children of independently started workspace nodes.
  bool changed;
  do {
    changed = false;
    for (auto& entry : processes) {
      auto parent = processes.find(entry.second.parent);
      if (!entry.second.related && parent != processes.end() && parent->second.related) {
        entry.second.related = true;
        changed = true;
      }
    }
  } while (changed);
  // The measurement tool itself is not part of the project's CPU figure.
  processes.erase(monitor_pid);
}

class Sampler {
 public:
  Sampler(std::string workspace, int launch_pid)
      : workspace_(std::move(workspace)), launch_pid_(launch_pid) {}

  bool sample(double& total_percent, double& project_percent) {
    // Discover ownership first. Reading cmdline/maps must not sit between the
    // system CPU snapshot and the project's CPU snapshot.
    Processes current;
    DIR* directory = opendir("/proc");
    if (!directory) return false;
    const auto now = Clock::now();
    const bool refresh = now >= next_discovery_;
    while (auto* entry = readdir(directory)) {
      char* end = nullptr;
      const long number = std::strtol(entry->d_name, &end, 10);
      if (number <= 0 || *end != '\0') continue;
      const int pid = static_cast<int>(number);
      Process process;
      if (!readProcess(pid, process)) continue;
      // Descendants can have another UID; still collect their stat, but skip maps.
      struct stat info{};
      const auto base = "/proc/" + std::to_string(pid);
      const bool same_user = stat(base.c_str(), &info) == 0 && info.st_uid == getuid();
      const auto previous = previous_.find(pid);
      const bool same_process = previous != previous_.end() &&
                                previous->second.started == process.started;
      if (same_user) {
        process.project = (!refresh && same_process) ? previous->second.project :
                          workspaceProcess(pid, workspace_);
      }
      // Retain descendants which outlive their parent; reset on PID reuse.
      if (same_process && previous->second.related) process.project = true;
      current.emplace(pid, process);
    }
    closedir(directory);
    if (refresh) next_discovery_ = now + std::chrono::seconds(5);
    markRelated(current, launch_pid_, getpid());

    // Keep the counter acquisition pass short and bracket it with system reads.
    System before, after;
    if (!readSystem(before)) return false;
    for (auto entry = current.begin(); entry != current.end();) {
      if (!entry->second.related) {
        ++entry;
        continue;
      }
      Process measured;
      if (!readProcess(entry->first, measured) || measured.started != entry->second.started) {
        entry = current.erase(entry);
        continue;
      }
      entry->second.ticks = measured.ticks;
      entry->second.sampled = measured.sampled;
      ++entry;
    }
    if (!readSystem(after)) return false;
    if (after.total < before.total || after.busy < before.busy) return false;
    System system;
    system.total = before.total + (after.total - before.total) / 2;
    system.busy = before.busy + (after.busy - before.busy) / 2;
    system.sampled = before.sampled + (after.sampled - before.sampled) / 2;
    const double seconds = std::chrono::duration<double>(
        system.sampled - previous_system_.sampled).count();
    double used = 0;
    bool valid = true;
    for (const auto& entry : current) {
      if (!entry.second.related) continue;
      const auto old = previous_.find(entry.first);
      if (old != previous_.end() && old->second.started == entry.second.started) {
        const double process_seconds = std::chrono::duration<double>(
            entry.second.sampled - old->second.sampled).count();
        if (process_seconds <= 0 || entry.second.ticks < old->second.ticks) {
          valid = false;
          continue;
        }
        // Normalize each process's actual reading interval to the system interval.
        // This prevents a late read from being counted as extra CPU usage.
        used += static_cast<double>(entry.second.ticks - old->second.ticks) *
                seconds / process_seconds;
      }
      // Newly observed processes need a baseline. Never add lifetime CPU time
      // to a single interval; count their delta from the next sample onward.
    }
    if (initialized_ && seconds > 0 && system.total > previous_system_.total &&
        system.busy >= previous_system_.busy) {
      const auto elapsed = system.total - previous_system_.total;
      const auto busy = system.busy - previous_system_.busy;
      const double capture_seconds = std::chrono::duration<double>(
          after.sampled - before.sampled).count();
      if (!valid || busy > elapsed || capture_seconds > seconds * 0.05) {
        // E.g. prolonged descheduling of this low-priority monitor during reads.
        total_percent = project_percent = -2.0;
      } else {
        total_percent = percentage(busy, elapsed);
        // /proc counters are not an atomic snapshot and have tick granularity.
        // Reject an inconsistent project sample instead of fabricating a capped value.
        project_percent = used > static_cast<double>(busy) + 1e-6 ?
                          -2.0 : percentage(used, elapsed);
      }
    } else {
      total_percent = project_percent = -1.0;
    }
    previous_system_ = system;
    previous_ = std::move(current);
    initialized_ = true;
    return true;
  }

 private:
  std::string workspace_;
  int launch_pid_;
  Processes previous_;
  System previous_system_;
  Clock::time_point next_discovery_{};
  bool initialized_ = false;
};

volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }

int runWindow(const std::string& workspace, int launch_pid) {
  Process launch;
  if (launch_pid > 0 && !readProcess(launch_pid, launch)) return 0;
  // These calls affect only this process, never the launch process or controller.
  setpriority(PRIO_PROCESS, 0, 19);
  sched_param priority{};
  sched_setscheduler(0, SCHED_IDLE, &priority);
  Display* display = XOpenDisplay(nullptr);
  if (!display) {
    std::cerr << "处理器监视：无法连接图形桌面，监视窗口已跳过。\n";
    return 0;
  }
  const int screen = DefaultScreen(display);
  XftFont* font = XftFontOpenName(display, screen, "Noto Sans CJK SC:lang=zh-cn:pixelsize=16");
  if (!font) {
    std::cerr << "处理器监视：无法加载中文字体，监视窗口已跳过。\n";
    XCloseDisplay(display);
    return 0;
  }
  const Window root = RootWindow(display, screen);
  constexpr int width = 290, height = 76;
  const Window window = XCreateSimpleWindow(display, root,
      std::max(0, DisplayWidth(display, screen) - width - 32), 64,
      width, height, 0, 0, WhitePixel(display, screen));
  const auto atom = [display](const char* name) { return XInternAtom(display, name, False); };
  const std::string title = "处理器占用率";
  const Atom utf8 = atom("UTF8_STRING");
  XChangeProperty(display, window, atom("_NET_WM_NAME"), utf8, 8, PropModeReplace,
                  reinterpret_cast<const unsigned char*>(title.data()), title.size());
  // Borderless, fixed-size, always above ordinary windows, with no focus stealing.
  const std::array<unsigned long, 5> motif{2, 0, 0, 0, 0};
  XChangeProperty(display, window, atom("_MOTIF_WM_HINTS"), atom("_MOTIF_WM_HINTS"),
                  32, PropModeReplace, reinterpret_cast<const unsigned char*>(motif.data()), 5);
  const std::array<Atom, 2> states{atom("_NET_WM_STATE_ABOVE"), atom("_NET_WM_STATE_SKIP_TASKBAR")};
  XChangeProperty(display, window, atom("_NET_WM_STATE"), XA_ATOM, 32, PropModeReplace,
                  reinterpret_cast<const unsigned char*>(states.data()), states.size());
  XSizeHints size{};
  size.flags = PPosition | PMinSize | PMaxSize;
  size.x = std::max(0, DisplayWidth(display, screen) - width - 32);
  size.y = 64;
  size.min_width = size.max_width = width;
  size.min_height = size.max_height = height;
  XSetWMNormalHints(display, window, &size);
  XWMHints hints{};
  hints.flags = InputHint;
  hints.input = False;
  XSetWMHints(display, window, &hints);
  Atom close = atom("WM_DELETE_WINDOW");
  XSetWMProtocols(display, window, &close, 1);
  XSelectInput(display, window, ExposureMask | StructureNotifyMask);
  XftDraw* draw = XftDrawCreate(display, window, DefaultVisual(display, screen),
                              DefaultColormap(display, screen));
  std::array<XftColor, 4> colors{};
  const std::array<const char*, 4> names{"#16803a", "#155ac6", "#d02020", "#555555"};
  for (std::size_t i = 0; i < colors.size(); ++i) {
    XftColorAllocName(display, DefaultVisual(display, screen), DefaultColormap(display, screen),
                     names[i], &colors[i]);
  }
  XMapWindow(display, window);
  Sampler sampler(workspace, launch_pid);
  const bool watch_parent = launch_pid > 0;
  double total = -1, project = -1;
  sampler.sample(total, project);
  auto next_sample = Clock::now() + std::chrono::seconds(1);
  bool repaint = true;
  while (running) {
    while (XPending(display)) {
      XEvent event{};
      XNextEvent(display, &event);
      if (event.type == Expose) repaint = true;
      if (event.type == ClientMessage && static_cast<Atom>(event.xclient.data.l[0]) == close) {
        running = 0;
      }
    }
    if (!running) break;
    const auto now = Clock::now();
    if (now >= next_sample) {
      Process parent;
      if (watch_parent && (!readProcess(launch_pid, parent) || parent.started != launch.started ||
                           parent.state == 'Z')) break;
      if (!sampler.sample(total, project)) total = project = -1;
      next_sample = Clock::now() + std::chrono::seconds(1);
      repaint = true;
    }
    if (repaint) {
      XClearWindow(display, window);
      const std::array<double, 2> values{total, project};
      const std::array<const char*, 2> labels{"总处理器占用率：", "工程处理器占用率："};
      for (int row = 0; row < 2; ++row) {
        char value[32];
        if (values[row] == -2.0) std::snprintf(value, sizeof(value), "采样不同步");
        else if (values[row] < 0) std::snprintf(value, sizeof(value), "采样中");
        else std::snprintf(value, sizeof(value), "%.1f%%", values[row]);
        const auto text = std::string(labels[row]) + value;
        XftDrawStringUtf8(draw, &colors[colorIndex(values[row])], font, 12, 29 + row * 30,
                         reinterpret_cast<const FcChar8*>(text.data()), text.size());
      }
      XFlush(display);
      repaint = false;
    }
    const auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(next_sample - Clock::now());
    pollfd descriptor{ConnectionNumber(display), POLLIN, 0};
    poll(&descriptor, 1, std::max(0, static_cast<int>(wait.count())));
  }
  for (auto& color : colors) {
    XftColorFree(display, DefaultVisual(display, screen), DefaultColormap(display, screen), &color);
  }
  XftDrawDestroy(draw);
  XftFontClose(display, font);
  XDestroyWindow(display, window);
  XCloseDisplay(display);
  return 0;
}
}  // namespace cpu_monitor

#ifndef SYSU219_CPU_MONITOR_TEST
int main(int argc, char** argv) {
  std::string workspace;
  int launch_pid = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "--workspace" && i + 1 < argc) workspace = argv[++i];
    else if (option == "--launch-pid" && i + 1 < argc) launch_pid = std::atoi(argv[++i]);
    else {
      std::cerr << "用法：sysu219_cpu_monitor --workspace 工作空间路径 [--launch-pid 启动进程号]\n";
      return 1;
    }
  }
  std::error_code error;
  const auto root = std::filesystem::canonical(workspace, error);
  if (workspace.empty() || error || root == "/") {
    std::cerr << "处理器监视：请指定有效的工作空间路径。\n";
    return 1;
  }
  std::signal(SIGINT, cpu_monitor::stop);
  std::signal(SIGTERM, cpu_monitor::stop);
  return cpu_monitor::runWindow(root.string(), launch_pid);
}
#endif
