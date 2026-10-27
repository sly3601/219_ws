#include "sysu219_guide_controller/control/OnlineTuning.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <locale>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <spawn.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char** environ;

namespace sysu219::tuning {
namespace {

static_assert(std::atomic<bool>::is_always_lock_free, "RT mailbox requires lock-free bool");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "RT acknowledgement requires lock-free uint64_t");
constexpr std::array<std::array<std::size_t, 2>, 3> kMatrices{{
    {0, 13}, {169, 12}, {313, 12}}};

// Python 原文内嵌，避免增加第三个文件。窗口及快照 IO 全部在独立进程中执行。
// 不读取任何历史快照或持久化参数。父进程只传入实际初值和已应用值。
constexpr char kWindowScript[] = R"SYSU219_PY(
import datetime
import json
import math
import os
import queue
import socket
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

COUNT = 485
WBC_OFFSET = 457
RELAXATION_OFFSET = 478
RELAXATION_NAMES = ('Q_WBC', 'Q_MPC', 'joint_acceleration_weight_ratio')
MOTOR_OFFSET = 481
MOTOR_NAMES = ('kp_stance', 'kd_stance', 'kp_swing', 'kd_swing')
Q_AXES = ('roll', 'pitch', 'yaw', 'px', 'py', 'pz',
          'wx', 'wy', 'wz', 'vx', 'vy', 'vz', 'g')
FORCE_AXES = tuple(leg + '.' + axis for leg in ('FR', 'FL', 'RR', 'RL')
                   for axis in ('fx', 'fy', 'fz'))
WBC_NAMES = tuple(name + '.' + axis for name in
                  ('kp_orientation', 'kd_orientation', 'kp_body_position',
                   'kd_body_position', 'kp_swing', 'kd_swing')
                  for axis in ('x', 'y', 'z')) + (
                      'swing_hip_posture_weight', 'kp_swing_hip_posture',
                      'kd_swing_hip_posture')


def parameter(index):
    if index >= MOTOR_OFFSET:
        return 'MotorMIT.' + MOTOR_NAMES[index - MOTOR_OFFSET], False
    if index >= RELAXATION_OFFSET:
        return 'Relaxation.' + RELAXATION_NAMES[index - RELAXATION_OFFSET], False
    if index in (313, 326, 339):
        return ('S_fx', 'S_fy', 'S_fz')[(index - 313) // 13], False
    for name, offset, size, axes in (('Q', 0, 13, Q_AXES),
                                   ('R', 169, 12, FORCE_AXES),
                                   ('S', 313, 12, FORCE_AXES)):
        if offset <= index < offset + size * size:
            row, col = divmod(index - offset, size)
            return name + '[' + axes[row] + ',' + axes[col] + ']', row != col
    return 'WBC.' + WBC_NAMES[index - WBC_OFFSET], False


def snapshot(values, revision):
    result = {'format': 'sysu219_mpc_wbc_snapshot_v1',
              'created_at': datetime.datetime.now().astimezone().isoformat(),
              'applied_revision': revision,
              'note': '仅记录已应用参数，不会自动加载；矩阵按行，FR/FL/RR/RL。',
              'Q_state_order': list(Q_AXES), 'force_order': list(FORCE_AXES)}
    for name, offset, size in (('Q', 0, 13), ('R', 169, 12), ('S', 313, 12)):
        result[name] = [values[offset + row * size:offset + (row + 1) * size]
                        for row in range(size)]
    result['WBC'] = {name: values[WBC_OFFSET + 3 * i:WBC_OFFSET + 3 * i + 3]
                     for i, name in enumerate(('kp_orientation', 'kd_orientation',
                                               'kp_body_position', 'kd_body_position',
                                               'kp_swing', 'kd_swing'))}
    result['WBC'].update(zip(WBC_NAMES[-3:], values[RELAXATION_OFFSET - 3:RELAXATION_OFFSET]))
    result['Relaxation'] = dict(zip(RELAXATION_NAMES, values[RELAXATION_OFFSET:MOTOR_OFFSET]))
    result['MotorMIT'] = dict(zip(MOTOR_NAMES, values[MOTOR_OFFSET:]))
    return result


class Window:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('MPC + WBC 在线调参')
        self.root.geometry('780x620')
        self.sock = socket.socket(fileno=0)
        self.sock.setblocking(False)
        self.incoming = b''
        self.outgoing = b''
        self.values = None
        self.initial = None
        self.revision = 0
        self.request = 0
        self.pending = None
        self.connected = True
        self.editor = None
        self.buttons = {}
        self.jobs = queue.Queue()
        self.results = queue.Queue()
        # 文件对话框在 GUI 线程；序列化/写盘在窗口进程的后台线程。
        self.writer = threading.Thread(target=self.write_snapshots, daemon=True)
        self.writer.start()
        self.status = tk.StringVar(value='等待控制器提供原始参数……')
        ttk.Label(self.root, textvariable=self.status, wraplength=750).pack(
            fill='x', padx=12, pady=10)
        toolbar = ttk.Frame(self.root)
        toolbar.pack(fill='x', padx=12)
        self.save_button = ttk.Button(toolbar, text='输出全部参数快照',
                                      command=self.save, state='disabled')
        self.save_button.pack(side='left')
        self.all_entries = tk.BooleanVar(value=False)
        ttk.Checkbutton(toolbar, text='显示全部非对角项（对称编辑）',
                        variable=self.all_entries, command=self.rebuild).pack(side='right')
        self.tabs = ttk.Notebook(self.root)
        self.tabs.pack(fill='both', expand=True, padx=12, pady=10)
        ttk.Label(self.root, text='点击参数 → 拖动或输入 → 确定；收到控制周期确认才显示已生效。'
                  '\n关闭窗口保留本次已生效值；快照不自动加载。').pack(pady=(0, 10))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(20, self.poll)

    def rebuild(self):
        selected = self.tabs.index('current') if self.tabs.tabs() else 0
        for tab in self.tabs.tabs():
            self.tabs.nametowidget(tab).destroy()
        self.buttons.clear()
        if self.values is None:
            return
        groups = []
        for name, offset, size in (('Q', 0, 13), ('R', 169, 12)):
            indices = [offset + row * size + col for row in range(size)
                       for col in range(row, size)
                       if row == col or self.all_entries.get()
                       or self.values[offset + row * size + col] != 0.0]
            groups.append((name, indices))
        groups.append(('S', [313, 326, 339]))
        groups.append(('WBC', list(range(WBC_OFFSET, RELAXATION_OFFSET))))
        groups.append(('松弛优化', list(range(RELAXATION_OFFSET, MOTOR_OFFSET))))
        groups.append(('电机 MIT', list(range(MOTOR_OFFSET, COUNT))))
        for name, indices in groups:
            frame = ttk.Frame(self.tabs)
            self.tabs.add(frame, text=name)
            canvas = tk.Canvas(frame, highlightthickness=0)
            scroll = ttk.Scrollbar(frame, orient='vertical', command=canvas.yview)
            body = ttk.Frame(canvas)
            body.bind('<Configure>', lambda event, c=canvas:
                      c.configure(scrollregion=c.bbox('all')))
            body_id = canvas.create_window((0, 0), window=body, anchor='nw')
            canvas.bind('<Configure>', lambda event, c=canvas, item=body_id:
                        c.itemconfigure(item, width=event.width))
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side='right', fill='y')
            canvas.pack(side='left', fill='both', expand=True)
            body.columnconfigure(0, weight=1)
            body.columnconfigure(1, weight=1)
            for order, index in enumerate(indices):
                label, _ = parameter(index)
                button = ttk.Button(body, text=f'{label} = {self.values[index]:.10g}',
                                    command=lambda i=index: self.edit(i))
                button.grid(row=order // 2, column=order % 2,
                            sticky='ew', padx=5, pady=4)
                self.buttons[index] = button
        self.tabs.select(selected)

    def edit(self, index):
        if self.pending is not None or not self.connected or self.values is None:
            return
        if self.editor is not None and self.editor.winfo_exists():
            self.editor.lift()
            return
        label, signed = parameter(index)
        dialog = self.editor = tk.Toplevel(self.root)
        dialog.title(label)
        dialog.transient(self.root)
        # 不使用 grab/wait_window；主窗口与控制算法均不等待对话框。
        ttk.Label(dialog, text=label).pack(padx=15, pady=10)
        text = tk.StringVar(value=format(self.values[index], '.17g'))
        scale_value = tk.DoubleVar(value=self.values[index])
        span = max(abs(self.values[index]), abs(self.initial[index])) * 3.0
        if span == 0.0:
            span = 1.0
        span = max(span, 1e-12)
        scale = ttk.Scale(dialog, from_=-span if signed else 0.0, to=span,
                          variable=scale_value,
                          command=lambda value: text.set(format(float(value), '.12g')))
        scale.pack(fill='x', padx=15, pady=8)
        entry = ttk.Entry(dialog, textvariable=text, width=36)
        entry.pack(padx=15, pady=8)
        entry.focus_set()
        error = tk.StringVar()
        ttk.Label(dialog, textvariable=error, wraplength=440).pack(padx=15)
        controls = ttk.Frame(dialog)
        controls.pack(padx=15, pady=10)

        def resize(factor):
            limit = min(max(float(scale.cget('to')) * factor, 1e-12), 1e100)
            scale.configure(from_=-limit if signed else 0.0, to=limit)

        ttk.Button(controls, text='范围 ÷10', command=lambda: resize(0.1)).pack(side='left')
        ttk.Button(controls, text='范围 ×10', command=lambda: resize(10.0)).pack(side='left')

        def confirm():
            try:
                value = float(text.get())
                if not math.isfinite(value) or (not signed and value < 0.0):
                    raise ValueError('请输入有限数值；对角权重、WBC 和 MIT 增益须 ≥ 0。')
                if RELAXATION_OFFSET <= index < MOTOR_OFFSET and value <= 0.0:
                    raise ValueError('松弛优化权重须 > 0。')
            except ValueError as exc:
                error.set(str(exc))
                return
            self.request += 1
            self.pending = self.request
            self.outgoing += f'SET {self.request} {index} {value:.17g}\n'.encode('ascii')
            self.save_button.configure(state='disabled')
            self.status.set('已提交，等待下一个控制周期应用确认……')
            request = self.pending
            self.root.after(3000, lambda: self.wait_notice(request))
            dialog.destroy()
            self.editor = None

        ttk.Button(controls, text='确定', command=confirm).pack(side='left', padx=10)
        ttk.Button(controls, text='取消', command=dialog.destroy).pack(side='left')
        dialog.bind('<Return>', lambda event: confirm())
        dialog.bind('<Escape>', lambda event: dialog.destroy())

    def wait_notice(self, request):
        if self.pending == request:
            self.status.set('尚未收到应用确认：控制周期未消费请求或尚未运行；算法无需等待窗口。')

    def receive(self, line):
        fields = line.split()
        if not fields:
            return
        if fields[0] == 'STATE':
            if len(fields) != COUNT + 3:
                raise ValueError('控制器返回的参数数量错误')
            request, revision = int(fields[1]), int(fields[2])
            values = [float(item) for item in fields[3:]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError('控制器返回了非有限参数')
            if request != 0 and request != self.pending:
                raise ValueError('应用确认序号不匹配')
            self.values, self.revision = values, revision
            if self.initial is None:
                self.initial = values[:]
            self.pending = None
            self.save_button.configure(state='normal')
            self.rebuild()
            self.status.set('当前为代码原始赋值，尚未调参。' if request == 0
                            else f'已生效，版本 {revision}（控制线程已确认）。')
        elif fields[0] == 'ERROR':
            request = int(fields[1])
            if request == self.pending:
                self.pending = None
                self.save_button.configure(state='normal')
            self.status.set('修改未生效：' + ' '.join(fields[2:]))

    def poll(self):
        try:
            if self.connected:
                if self.outgoing:
                    try:
                        sent = self.sock.send(self.outgoing)
                        self.outgoing = self.outgoing[sent:]
                    except BlockingIOError:
                        pass
                try:
                    block = self.sock.recv(65536)
                    if not block:
                        raise ConnectionError('控制器连接已关闭；可输出最后确认的快照。')
                    self.incoming += block
                except BlockingIOError:
                    pass
                if len(self.incoming) > 200000:
                    raise ValueError('控制器消息过长')
                while b'\n' in self.incoming:
                    line, self.incoming = self.incoming.split(b'\n', 1)
                    self.receive(line.decode('utf-8'))
        except (OSError, ValueError, IndexError) as exc:
            self.connected = False
            self.pending = None
            self.outgoing = b''
            self.status.set(str(exc))
            if self.values is not None:
                self.save_button.configure(state='normal')
        while True:
            try:
                ok, message = self.results.get_nowait()
            except queue.Empty:
                break
            if ok:
                self.status.set(message)
            else:
                messagebox.showerror('快照输出失败', message, parent=self.root)
        self.root.after(20, self.poll)

    def save(self):
        if self.values is None or self.pending is not None:
            return
        # 在选择文件前冻结最后确认值；对话框期间到来的更新不改变本次快照。
        values, revision = self.values[:], self.revision
        filename = filedialog.asksaveasfilename(
            parent=self.root, title='输出当前全部参数快照', defaultextension='.json',
            initialfile='mpc_wbc_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.json',
            filetypes=(('JSON 快照', '*.json'),))
        if filename:
            self.jobs.put((filename, values, revision))

    def write_snapshots(self):
        while True:
            job = self.jobs.get()
            if job is None:
                return
            filename, values, revision = job
            try:
                with open(filename, 'w', encoding='utf-8') as stream:
                    json.dump(snapshot(values, revision), stream, ensure_ascii=False,
                              indent=2, allow_nan=False)
                    stream.write('\n')
                self.results.put((True, f'已输出版本 {revision} 快照：{filename}'))
            except (OSError, ValueError) as exc:
                self.results.put((False, str(exc)))

    def close(self):
        # 等待已经提交的快照写完仅发生在 GUI 进程，控制线程不参与。
        self.jobs.put(None)
        self.writer.join()
        self.sock.close()
        self.root.destroy()


if __name__ == '__main__':
    # 不继承控制器可能使用的实时调度优先级；失败则不启动窗口。
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    os.nice(5)
    window = Window()
    window.root.mainloop()
)SYSU219_PY";

// 仅在初始化/后台线程调用；不会修正原值。对称编辑仍需保证整体半正定。
bool validate(const Parameters& p, std::string& reason) {
  for (const double value : p.values) {
    if (!std::isfinite(value) || std::abs(value) > 1e100) {
      reason = "Values must be finite with abs(value) <= 1e100.";
      return false;
    }
  }
  for (const auto& shape : kMatrices) {
    const auto offset = shape[0], size = shape[1];
    using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    const Eigen::Map<const RowMatrix> matrix(p.values.data() + offset, size, size);
    const double scale = std::max(1e-100, matrix.cwiseAbs().maxCoeff());
    if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff() > scale * 1e-12 ||
        matrix.diagonal().minCoeff() < 0.0) {
      reason = "Q/R/S must be symmetric with nonnegative diagonal.";
      return false;
    }
    // 归一化只用于后台校验，原始矩阵不变；避免小权重被绝对容差误放行。
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(matrix / scale,
                                                        Eigen::EigenvaluesOnly);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() < -1e-12) {
      reason = "Change rejected: Q/R/S must remain positive semidefinite.";
      return false;
    }
  }
  for (std::size_t i = Parameters::kWbcOffset; i < Parameters::kRelaxationOffset; ++i) {
    if (p.values[i] < 0.0) {
      reason = "WBC gains and hip posture weight must be nonnegative.";
      return false;
    }
  }
  const auto relaxation = Parameters::kRelaxationOffset;
  const double joint_weight = p.values[relaxation] * p.values[relaxation + 2];
  if (p.values[relaxation] <= 0.0 || p.values[relaxation + 1] <= 0.0 ||
      p.values[relaxation + 2] <= 0.0 || !std::isfinite(joint_weight) || joint_weight <= 0.0) {
    reason = "Relaxation weights and Q_WBC * joint_acceleration_weight_ratio must be positive and finite.";
    return false;
  }
  for (std::size_t i = Parameters::kMotorOffset; i < Parameters::kCount; ++i) {
    if (p.values[i] < 0.0) {
      reason = "Motor MIT gains must be nonnegative.";
      return false;
    }
  }
  return true;
}

std::string stateMessage(std::uint64_t request, std::uint64_t revision,
                         const Parameters& parameters) {
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << "STATE " << request << ' ' << revision << std::setprecision(17);
  for (double value : parameters.values) text << ' ' << value;
  text << '\n';
  return text.str();
}

// 后台只接受上三角/对角元素，写入时自动对称。初值的所有耦合项均保留。
bool setParameter(Parameters& p, std::size_t index, double value) {
  if (index >= Parameters::kCount) return false;
  for (const auto& shape : kMatrices) {
    const auto offset = shape[0], size = shape[1];
    if (index >= offset && index < offset + size * size) {
      const auto row = (index - offset) / size, col = (index - offset) % size;
      if (row > col) return false;
      if (offset == 313 && row == col) {
        // S 的同一力方向由四条腿共用；一次请求统一更新，保留非对角耦合项。
        for (std::size_t leg = 0; leg < 4; ++leg) {
          const auto axis = 3 * leg + row % 3;
          p.values[offset + axis * size + axis] = value;
        }
        return true;
      }
      p.values[index] = value;
      p.values[offset + col * size + row] = value;
      return true;
    }
  }
  p.values[index] = value;
  return true;
}

}  // namespace

Parameters::Parameters() noexcept {
  values.fill(std::numeric_limits<double>::quiet_NaN());
}

struct OnlineTuning::Impl {
  // 单生产者（worker）/单消费者（控制线程）单槽。
  // ready release/acquire 发布数据；taken 后仍须等待 finished 确认才能再次写槽。
  Parameters pending;
  std::uint64_t pending_revision = 0;
  std::atomic<bool> ready{false};
  std::atomic<std::uint64_t> finished{0}; // revision << 1 | applied
  std::atomic<bool> stop{false};
  int channel = -1;
  pid_t child = -1;
  std::thread worker;

  explicit Impl(const Parameters& initial) {
    const char* display = std::getenv("DISPLAY");
    if (!display || !*display) throw std::runtime_error("DISPLAY 未设置，跳过桌面调参窗口");
    std::string reason;
    if (!validate(initial, reason)) throw std::invalid_argument(reason);
    int pair[2];
    if (::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, pair) != 0)
      throw std::system_error(errno, std::generic_category(), "tuning socketpair");

    posix_spawn_file_actions_t actions;
    posix_spawnattr_t attributes;
    int error = posix_spawn_file_actions_init(&actions);
    const bool actions_initialized = error == 0;
    if (!error) error = posix_spawn_file_actions_addclose(&actions, pair[0]);
    if (!error) error = posix_spawn_file_actions_adddup2(&actions, pair[1], STDIN_FILENO);
    if (!error && pair[1] != STDIN_FILENO)
      error = posix_spawn_file_actions_addclose(&actions, pair[1]);

    bool attributes_initialized = false;
    if (!error) {
      error = posix_spawnattr_init(&attributes);
      attributes_initialized = error == 0;
    }
    const sched_param normal_priority{};
    if (!error) error = posix_spawnattr_setschedpolicy(&attributes, SCHED_OTHER);
    if (!error) error = posix_spawnattr_setschedparam(&attributes, &normal_priority);
    if (!error) error = posix_spawnattr_setflags(
        &attributes, POSIX_SPAWN_SETSCHEDULER | POSIX_SPAWN_SETSCHEDPARAM);

    // posix_spawn 避免在多线程控制进程 fork 后执行 Python/C++ 初始化逻辑。
    // 子进程从 exec 前就使用普通调度，不能继承控制进程的实时优先级。
    char python[] = "python3", option[] = "-c";
    char* args[] = {python, option, const_cast<char*>(kWindowScript), nullptr};
    if (!error) error = ::posix_spawnp(&child, python, &actions, &attributes, args, environ);
    if (attributes_initialized) posix_spawnattr_destroy(&attributes);
    if (actions_initialized) posix_spawn_file_actions_destroy(&actions);
    ::close(pair[1]);
    if (error) {
      ::close(pair[0]);
      throw std::system_error(error, std::generic_category(), "tuning window spawn");
    }
    channel = pair[0];
    const int flags = ::fcntl(channel, F_GETFL, 0);
    if (flags < 0 || ::fcntl(channel, F_SETFL, flags | O_NONBLOCK) < 0) {
      const int code = errno;
      cleanup();
      throw std::system_error(code, std::generic_category(), "tuning nonblocking socket");
    }
    try {
      worker = std::thread([this, initial] {
        // 窗口错误或内存不足仅使后台退出，不跨线程抛异常/终止控制进程。
        const sched_param normal{};
        if (::pthread_setschedparam(::pthread_self(), SCHED_OTHER, &normal) == 0) {
          try { run(initial); } catch (...) {}
        }
        cleanup();
      });
    } catch (...) {
      cleanup();
      throw;
    }
  }

  ~Impl() {
    stop.store(true, std::memory_order_relaxed);
    if (worker.joinable()) worker.join(); // 仅允许非实时生命周期线程析构。
  }

  void cleanup() noexcept {
    if (channel >= 0) { ::close(channel); channel = -1; }
    if (child > 0) {
      // 仅用于回收本模块启动的窗口进程；从不接触其它进程。
      int status = 0;
      pid_t result;
      do { result = ::waitpid(child, &status, WNOHANG); } while (result < 0 && errno == EINTR);
      if (result == 0) {
        ::kill(child, SIGTERM);
        while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
      }
      child = -1;
    }
  }

  void run(Parameters current) {
    std::string incoming, outgoing = stateMessage(0, 0, current);
    std::uint64_t revision = 0, waiting = 0, request_id = 0;
    Parameters candidate;
    while (!stop.load(std::memory_order_relaxed)) {
      if (waiting) {
        const auto result = finished.load(std::memory_order_acquire);
        if ((result >> 1) == waiting) {
          if (result & 1) {
            current = candidate;
            revision = waiting;
            outgoing += stateMessage(request_id, revision, current);
          } else {
            outgoing += "ERROR " + std::to_string(request_id) +
                        " Control thread rejected the update; previous values retained.\n";
          }
          waiting = 0;
        }
      }
      if (outgoing.size() > 65536) break;
      pollfd descriptor{channel, static_cast<short>(POLLIN | (outgoing.empty() ? 0 : POLLOUT)), 0};
      const int result = ::poll(&descriptor, 1, 20);
      if (result < 0) { if (errno == EINTR) continue; break; }
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) break;
      if (descriptor.revents & POLLOUT) {
        const auto sent = ::send(channel, outgoing.data(), outgoing.size(), MSG_NOSIGNAL);
        if (sent > 0) outgoing.erase(0, static_cast<std::size_t>(sent));
        else if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) break;
      }
      if (!(descriptor.revents & POLLIN)) continue;
      char block[2048];
      const auto count = ::recv(channel, block, sizeof(block), 0);
      if (count == 0) break;
      if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
        break;
      }
      incoming.append(block, static_cast<std::size_t>(count));
      if (incoming.size() > 4096) break;
      std::size_t newline;
      while ((newline = incoming.find('\n')) != std::string::npos) {
        const std::string line = incoming.substr(0, newline);
        incoming.erase(0, newline + 1);
        std::istringstream input(line);
        input.imbue(std::locale::classic());
        std::string verb, extra;
        std::uint64_t id = 0;
        std::size_t index = 0;
        double value = 0.0;
        std::string reason;
        if (!(input >> verb >> id >> index >> value) || verb != "SET" || id == 0 ||
            (input >> extra)) {
          reason = "Invalid SET message.";
        } else if (waiting) {
          reason = "An update is awaiting control-thread acknowledgement.";
        } else if (pending_revision >= (std::numeric_limits<std::uint64_t>::max() >> 1)) {
          reason = "Revision limit reached.";
        } else {
          candidate = current;
          if (!setParameter(candidate, index, value)) reason = "Invalid parameter index.";
          else if (validate(candidate, reason)) {
            pending = candidate;
            waiting = ++pending_revision;
            request_id = id;
            ready.store(true, std::memory_order_release);
          }
        }
        if (!reason.empty()) outgoing += "ERROR " + std::to_string(id) + ' ' + reason + '\n';
      }
    }
  }
};

OnlineTuning::OnlineTuning(const Parameters& initial) : impl_(std::make_unique<Impl>(initial)) {}
OnlineTuning::~OnlineTuning() = default;

bool OnlineTuning::tryReadUpdate(Parameters& output, std::uint64_t& revision) noexcept {
  if (!impl_->ready.load(std::memory_order_acquire)) return false;
  output = impl_->pending;
  revision = impl_->pending_revision;
  impl_->ready.store(false, std::memory_order_release);
  return true;
}

void OnlineTuning::finishUpdate(std::uint64_t revision, bool applied) noexcept {
  impl_->finished.store((revision << 1) | static_cast<std::uint64_t>(applied),
                        std::memory_order_release);
}

void OnlineTuning::requestStop() noexcept {
  impl_->stop.store(true, std::memory_order_relaxed);
}

}  // namespace sysu219::tuning
