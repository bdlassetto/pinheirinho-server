"""Logger centralizado com dois canais:

- ACAdapter.log(msg)   → ac.log() — rápido, mas truncado pelo AC (~2001 linhas)
- FileLogger.critical(msg) → arquivo direto em reports/events_*.txt — nunca truncado
  Use para: LOCK/UNLOCK, GREEN, RT, CHAT, Report saved, erros. Qualquer evento que
  não pode ser perdido num evento com muitos players logados por horas.
"""

import os
import datetime

try:
    import ac
except ImportError:
    class ac:
        @staticmethod
        def log(msg):
            print(msg)


class FileLogger:
    """Grava eventos críticos diretamente em arquivo Python — independente do ac.log."""

    _file_path = None
    _session_start = None

    @classmethod
    def initialize(cls, reports_dir):
        """Deve ser chamado em App.initialize() com o caminho para reports/."""
        try:
            if not os.path.exists(reports_dir):
                os.makedirs(reports_dir)
            cls._session_start = datetime.datetime.now()
            timestamp = cls._session_start.strftime("%Y%m%d_%H%M%S")
            cls._file_path = os.path.join(reports_dir, "events_{}.txt".format(timestamp))
            cls.critical("=== Pinheirinho2 session started at {} ===".format(
                cls._session_start.strftime("%Y-%m-%d %H:%M:%S")))
        except Exception as e:
            try:
                ac.log("FileLogger.initialize failed: {}".format(e))
            except:
                pass

    @classmethod
    def critical(cls, msg):
        """Grava diretamente em arquivo. Nunca truncado. Sempre faz flush."""
        # Always also send to ac.log for py_log.txt visibility
        try:
            ac.log(msg)
        except:
            pass

        if cls._file_path is None:
            return
        try:
            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            with open(cls._file_path, "a") as f:
                f.write("[{}] {}\n".format(ts, msg))
        except Exception as e:
            try:
                ac.log("FileLogger.critical write error: {}".format(e))
            except:
                pass

    @classmethod
    def get_path(cls):
        return cls._file_path


class Logger:
    DEBUG = 0
    INFO = 1
    WARNING = 2
    ERROR = 3

    _level = INFO

    @classmethod
    def set_level(cls, level):
        cls._level = level

    @classmethod
    def debug(cls, msg):
        if cls._level <= cls.DEBUG:
            try:
                ac.log("[DEBUG] {}".format(msg))
            except:
                pass

    @classmethod
    def info(cls, msg):
        if cls._level <= cls.INFO:
            try:
                ac.log("[INFO] {}".format(msg))
            except:
                pass

    @classmethod
    def warning(cls, msg):
        if cls._level <= cls.WARNING:
            try:
                ac.log("[WARN] {}".format(msg))
            except:
                pass

    @classmethod
    def error(cls, msg):
        if cls._level <= cls.ERROR:
            FileLogger.critical("[ERROR] {}".format(msg))
