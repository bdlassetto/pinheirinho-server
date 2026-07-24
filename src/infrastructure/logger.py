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
    """Grava eventos críticos diretamente em arquivo Python — independente do ac.log.

    Durante uma passada (RT, verde, queima, parciais podem disparar varios
    eventos em poucos segundos), cada critical() abrindo+escrevendo+fechando
    o arquivo na hora e I/O de disco na thread do jogo, exatamente quando
    o timing mais importa. begin_buffering()/flush() deixam essas linhas
    acumuladas em RAM durante a corrida e escrevem tudo de uma vez, numa
    unica abertura de arquivo, quando a passada termina.
    """

    _file_path = None
    _session_start = None
    _buffering = False
    _buffer = []

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
    def begin_buffering(cls):
        """Chamar quando a arvore arma: acumula critical() em RAM ate flush()."""
        cls._buffering = True

    @classmethod
    def flush(cls):
        """Chamar quando a passada termina: grava tudo que foi acumulado
        desde begin_buffering() numa unica abertura de arquivo, e volta a
        gravar direto por chamada ate a proxima begin_buffering()."""
        cls._buffering = False
        if not cls._buffer:
            return
        buffered, cls._buffer = cls._buffer, []
        if cls._file_path is None:
            return
        try:
            with open(cls._file_path, "a") as f:
                f.write("".join(buffered))
        except Exception as e:
            try:
                ac.log("FileLogger.flush write error: {}".format(e))
            except:
                pass

    @classmethod
    def critical(cls, msg):
        """Grava em arquivo (ou acumula em RAM se begin_buffering() ativo).
        Nunca truncado."""
        # Always also send to ac.log for py_log.txt visibility
        try:
            ac.log(msg)
        except:
            pass

        if cls._file_path is None:
            return
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = "[{}] {}\n".format(ts, msg)

        if cls._buffering:
            cls._buffer.append(line)
            return

        try:
            with open(cls._file_path, "a") as f:
                f.write(line)
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
