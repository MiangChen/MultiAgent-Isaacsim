import logging
import sys
from pathlib import Path


class LogManager:
    """
    一个用于集中配置和管理项目日志的类。
    """

    def __init__(self, log_level: str = "INFO", log_file: str = "logs/app.log"):
        """
        初始化 LogManager。

        Args:
            log_level (str): 日志级别 (e.g., "DEBUG", "INFO", "WARNING").
            log_file (str): 日志文件的输出路径。
        """
        self.log_level_str = log_level.upper()
        self.log_file = log_file
        self.is_configured = False

        self.setup()

    def setup(self):
        """
        配置根日志记录器 (root logger)。
        这个方法应该在应用程序启动时只调用一次。
        """
        if self.is_configured:
            # 防止重复配置
            return

        # 1. 获取日志级别
        log_level = getattr(logging, self.log_level_str, logging.INFO)

        # 2. 创建日志文件目录
        log_path = Path(self.log_file)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        # 3. 创建格式化器 (Formatters)
        console_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        file_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"
        )

        # 4. 创建处理器 (Handlers)
        # 控制台处理器
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(log_level)
        console_handler.setFormatter(console_formatter)

        # 文件处理器
        file_handler = logging.FileHandler(self.log_file, mode='a')
        file_handler.setLevel(log_level)
        file_handler.setFormatter(file_formatter)

        # 5. 配置根日志记录器
        root_logger = logging.getLogger()
        root_logger.setLevel(log_level)

        # 清除任何可能由第三方库（如isaac sim）添加的现有处理器
        root_logger.handlers.clear()

        root_logger.addHandler(console_handler)
        root_logger.addHandler(file_handler)

        self.is_configured = True
        logging.info("LogManager: Logging has been configured successfully.")

    @staticmethod
    def get_logger(name: str) -> logging.Logger:
        """
        获取一个指定名称的日志记录器实例。
        这是一个方便的静态方法，可以在项目的任何地方使用。

        Args:
            name (str): 日志记录器的名称，通常是 __name__。

        Returns:
            logging.Logger: 一个日志记录器实例。
        """
        return logging.getLogger(name)