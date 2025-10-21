import threading

class FeedbackEvent:
    """
    线程安全的反馈信号，封装反馈内容，用于navigate内部
    """
    def __init__(self):
        self._event = threading.Event()
        self._result = None

    def set(self, value=None):
        """设置结果并触发事件"""
        self._result = value
        self._event.set()

    def wait(self, timeout=None):
        """
        等待事件触发。
        返回 set() 传入的 value；超时则返回 None。
        """
        ok = self._event.wait(timeout)
        return self._result if ok else None

    def clear(self):
        """清除事件状态（可重复使用）"""
        self._event.clear()

    def is_set(self):
        """查询事件是否已触发"""
        return self._event.is_set()