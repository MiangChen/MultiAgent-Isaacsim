import logging
import os
from pathlib import Path
import sys


class LogManager:
    """
    一个用于集中配置和管理项目日志的类。
    """

    def __init__(self, log_level: str = "INFO", log_file: str = "logs/app.log"):
        """
        初始化并立即配置根日志记录器 (root logger)。
        这个方法由 dependency-injector 的 Singleton provider 保证只执行一次。
        """
        # --- 步骤 1: 健壮的路径构建与规范化 ---

        # 首先，展开路径中可能包含的环境变量和用户主目录 '~'
        expanded_log_file = os.path.expanduser(log_file)
        expanded_log_file = os.path.expandvars(expanded_log_file)

        # 将处理过的字符串转换为 Path 对象
        log_file_path = Path(expanded_log_file)

        # 判断处理后的路径是否为绝对路径
        if log_file_path.is_absolute():
            absolute_log_path = log_file_path
        else:
            # 如果是相对路径，则将其与项目根目录拼接
            project_root = Path(__file__).parent.parent.absolute()
            absolute_log_path = project_root / log_file_path

        # 使用 .resolve() 获取最终的、最干净的绝对路径 (例如解析 '..')
        final_log_path = absolute_log_path.resolve()

        # --- 步骤 2: 创建日志目录，并进行智能的权限错误处理 ---
        try:
            # 确保日志文件的父目录存在
            final_log_path.parent.mkdir(parents=True, exist_ok=True)
        except PermissionError as e:
            # 捕获权限错误，并分析原因
            error_dir = final_log_path.parent
            # 检查是否试图在文件系统的根目录下创建文件夹
            is_root_dir_error = error_dir.parent == error_dir

            if is_root_dir_error:
                raise ValueError(
                    f"\n\n{'=' * 80}\n"
                    f"FATAL: Permission denied to create log directory '{error_dir}'.\n\n"
                    f"This usually means the log file path in your configuration is accidentally an absolute path starting with '/'.\n"
                    f"  > Your original config value: '{log_file}'\n\n"
                    f"  > FIX: If you intended a path relative to the project, please remove the leading slash '/'.\n"
                    f"  > Correct example: 'logs/app.log'\n"
                    f"{'=' * 80}\n"
                ) from e
            else:
                # 如果是其他目录的权限问题，则正常抛出原始异常
                raise e

        # --- 步骤 3: 配置 Python 的 logging 系统 ---
        level = getattr(logging, log_level.upper(), logging.INFO)

        # 创建格式化器
        console_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        file_formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s"
        )

        # 创建处理器
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(console_formatter)

        # 使用最终的绝对路径来创建 FileHandler
        file_handler = logging.FileHandler(final_log_path, mode="a")
        file_handler.setLevel(level)
        file_handler.setFormatter(file_formatter)

        # 配置根日志记录器
        root_logger = logging.getLogger()
        root_logger.setLevel(level)
        # 清除任何可能由第三方库添加的现有处理器，确保我们的配置是唯一的
        root_logger.handlers.clear()
        root_logger.addHandler(console_handler)
        root_logger.addHandler(file_handler)

        logging.info(
            f"LogManager: Logging automatically configured. Log file resolved to: {final_log_path}"
        )

    @staticmethod
    def get_logger(name: str) -> logging.Logger:
        """
        获取一个指定名称的日志记录器实例。
        这是在项目中获取 logger 的唯一推荐方式。
        """
        return logging.getLogger(name)
