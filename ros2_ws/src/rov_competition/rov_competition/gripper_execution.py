"""Nonblocking request-correlated servo execution; never claims physical arrival."""
from dataclasses import dataclass
import math


@dataclass
class GripperExecution:
    request_id: str = ""
    source: str = ""
    action: str = ""
    state: str = "idle"
    message: str = ""
    wait_s: float = 0.0
    deadline: float = 0.0
    sent_at: float | None = None

    @property
    def active(self):
        return self.state in {"accepted", "running"}

    def begin(self, request_id, source, action, now, send_duration, wait_s):
        if self.active or not request_id or request_id == self.request_id:
            raise ValueError("动作重叠或重复请求")
        if not all(math.isfinite(v) for v in (now, send_duration, wait_s)) or send_duration < 0 or wait_s < 0:
            raise ValueError("无效舵机时间参数")
        self.request_id, self.source, self.action = request_id, source, action
        self.state, self.message = "accepted", "序列已接受，未确认物理到位"
        self.wait_s = wait_s
        self.deadline = now + send_duration + wait_s + 3.0
        self.sent_at = None

    def poll(self, now, sending):
        if not self.active:
            return
        if now >= self.deadline:
            self.fail("机械序列超时")
            return
        self.state = "running"
        if sending:
            return
        if self.sent_at is None:
            self.sent_at = now
        if now - self.sent_at >= self.wait_s:
            self.state = "completed"
            self.message = "指令发送完成且标定等待结束；无物理到位或入网反馈"

    def fail(self, reason, *, cancelled=False):
        if self.active:
            self.state = "cancelled" if cancelled else "failed"
            self.message = reason
