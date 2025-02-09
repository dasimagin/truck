import asyncio
import atexit
import enum
import json
import logging
import time

from threading import Event, Thread
from foxglove_websocket.server import FoxgloveServer
from mcap.writer import CompressionType, Writer
from pydantic import BaseModel
from typing import Any, Dict


class Level(enum.IntEnum):
    """
    Classic log levels: `UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`
    """

    UNKNOWN = 0
    DEBUG   = 1
    INFO    = 2
    WARNING = 3
    ERROR   = 4
    FATAL   = 5

def pylog_level(level: Level) -> int:
    """
    Convert log level from foxglove to python
    """

    if level == Level.UNKNOWN:
        return logging.NOTSET
    elif level == Level.DEBUG:
        return logging.DEBUG
    elif level == Level.INFO:
        return logging.INFO
    elif level == Level.WARNING:
        return logging.WARNING
    elif level == Level.ERROR:
        return logging.ERROR
    elif level == Level.FATAL:
        return logging.CRITICAL
    else:
        raise ValueError(f'Unknown log level {level}')


FOXGLOVE_LOG_TOPIC = "`/log`"

FOXGLOVE_LOG_MSG_TYPE = 'foxglove.Log'

FOXGLOVE_LOGM_SG_SCHEMA = '''
{
  "title": "foxglove.Log",
  "description": "A log message",
  "$comment": "Generated by https://github.com/foxglove/schemas",
  "type": "object",
  "properties": {
    "timestamp": {
      "type": "object",
      "title": "time",
      "properties": {
        "sec": {
          "type": "integer",
          "minimum": 0
        },
        "nsec": {
          "type": "integer",
          "minimum": 0,
          "maximum": 999999999
        }
      },
      "description": "Timestamp of log message"
    },
    "level": {
      "title": "foxglove.Level",
      "description": "Log level",
      "oneOf": [
        {
          "title": "UNKNOWN",
          "const": 0
        },
        {
          "title": "DEBUG",
          "const": 1
        },
        {
          "title": "INFO",
          "const": 2
        },
        {
          "title": "WARNING",
          "const": 3
        },
        {
          "title": "ERROR",
          "const": 4
        },
        {
          "title": "FATAL",
          "const": 5
        }
      ]
    },
    "message": {
      "type": "string",
      "description": "Log message"
    },
    "name": {
      "type": "string",
      "description": "Process or node name"
    },
    "file": {
      "type": "string",
      "description": "Filename"
    },
    "line": {
      "type": "integer",
      "minimum": 0,
      "description": "Line number in the file"
    }
  }
}
'''


NANO: int = 1000000000


def to_ns(t: float) -> int:
    '''
    Convert time in seconds (float) to nanoseconds (int)
    '''
    return int(t * NANO)


def to_stamp(t: int) -> tuple[int, int]:
    '''
    Convert time in nanoseconds to (seconds, nanoseconds).
    '''
    return (t // NANO, t % NANO)


def this_or_now(t: float|None) -> float:
    '''
    Return provided time or current time
    '''
    return t if t is not None else time.time()


class Registration(BaseModel):
    name: str
    channel_id: int


def get_pylogger(name: str, level: Level) -> logging.Logger:
    """
    Create python or get existing logger. Log Level is translated from foxglove to python.

    Parameters
    ----------
    name: str
        logger name
    level: Level
        log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
    """

    logger = logging.getLogger(name)
    logger.setLevel(pylog_level(level))

    # naive check that logger is configured
    if not logger.hasHandlers():
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter("[%(name)s] [%(levelname)s] %(message)s"))
        logger.addHandler(handler)

    return logger


class MCAPLogger:
    """
    Logger to mcap file.

    Example
    -------
    ```python
    with MCAPLogger(log_path='log.mcap') as log:
        obj = ... # some pydantic object
        log.publish('/topic', obj, stamp)
        log.info('message')
    ```

    """

    def __init__(self, log_path: str, level: Level=Level.INFO, compress=True):
        """
        Parameters
        ----------
        log_path: str
            path to mcap log file
        level: Level
            log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)

        compress: bool
            enable compression
        """

        self._pylog = get_pylogger('cartpole.mcap', level)

        self._writer = Writer(
            open(log_path, "wb"),
            compression=CompressionType.ZSTD if compress else CompressionType.NONE,
        )

        self._writer.start()
        self._topic_to_registration: Dict[str, Registration] = {}

        # preventive topic creation
        self.registration_log = self._register(
            FOXGLOVE_LOG_TOPIC,
            FOXGLOVE_LOG_MSG_TYPE,
            FOXGLOVE_LOGM_SG_SCHEMA)

    def __enter__(self):
        return self

    def _register(self, topic_name: str, name: str, schema: str) -> Registration:
        if topic_name in self._topic_to_registration:
            cached = self._topic_to_registration[topic_name]
            assert cached.name == name, f'Topic {topic_name} registered with {cached.name}'
            return cached

        schema_id = self._writer.register_schema(
            name=name,
            encoding="jsonschema",
            data=schema.encode())

        channel_id = self._writer.register_channel(
            schema_id=schema_id,
            topic=topic_name,
            message_encoding="json")

        cached = Registration(name=name, channel_id=channel_id)
        self._topic_to_registration[topic_name] = cached
        self._pylog.debug('id=%i topic=\'%s\', type=\'%s\'', channel_id, topic_name, name)

        return cached

    def _register_class(self, topic_name: str, cls: Any) -> Registration:
        name = cls.__name__
        assert issubclass(cls, BaseModel), 'Required pydantic model, but got {name}'
        return self._register(topic_name, name, json.dumps(cls.schema()))

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        """
        Publish object to topic.

        Parameters:
        -----------
        topic_name:
            topic name
        obj: Any
            object to dump (pydantic model)
        stamp: float
            timestamp in nanoseconds (float)
        """

        registation = self._register_class(topic_name, type(obj))
        self._writer.add_message(
            channel_id=registation.channel_id,
            log_time=to_ns(stamp),
            data=obj.json().encode(),
            publish_time=to_ns(stamp),
        )

    def log(self, msg: str, stamp: float, level: Level) -> None:
        """
        Print message to topic `/log`.

        Parameters:
        -----------
        msg: str
            message to print
        stamp: float
            timestamp in nanoseconds (float)
        level: Level
            log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
        """

        sec, nsec = to_stamp(stamp)
        stamp_ns = to_ns(stamp)
    
        obj = {
            "timestamp": {"sec": sec, "nsec": nsec},
            "level": int(level),
            "message": msg,
            "name": "cartpole",
            "file": "/dev/null",
            "line": 0
        }

        self._writer.add_message(
                channel_id=self.registration_log.channel_id,
                log_time=stamp_ns,
                data=json.dumps(obj).encode(),
                publish_time=stamp_ns)
        
    def debug(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `DEBUG` level.
        """
        self.log(msg, stamp, Level.DEBUG)

    def info(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `INFO` level.
        """
        self.log(msg, stamp, Level.INFO)

    def warning(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `WARNING` level.
        """
        self.log(msg, stamp, Level.WARNING)

    def error(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `ERROR` level.
        """
        self.log(msg, stamp, Level.ERROR)

    def fatal(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `FATAL` level.
        """
        self.log(msg, stamp, Level.FATAL)

    def close(self):
        """
        Free log resources.
        """
        self._writer.finish()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


async def _foxglove_async_entrypoint(queue: asyncio.Queue, stop: Event, level: Level) -> None:
    logger = get_pylogger('cartpole.foxglove', level)

    async with FoxgloveServer("0.0.0.0", 8765, "CartPole", logger=logger) as server:
        topic_to_registration = {}

        async def register(topic_name, name, schema) -> Registration:
            if topic_name in topic_to_registration:
                cached = topic_to_registration[topic_name]
                assert cached.name == name, f'Topic {topic_name} registered with {cached.name}'
                return cached

            spec = {
                "topic": topic_name,
                "encoding": "json",
                "schemaName": name,
                "schema": schema,
            }

            channel_id = await server.add_channel(spec)
            cached = Registration(name=name, channel_id=channel_id)
            topic_to_registration[topic_name] = cached

            logger.debug('id=%i topic=\'%s\', type=\'%s\'', channel_id, topic_name, name)
            return cached


        async def register_class(topic_name, cls) -> Registration:
            name = cls.__name__
            assert issubclass(cls, BaseModel), f'Required pydantic model, but got {name}'
            return await register(topic_name, name, cls.model_json_schema())

        # preventive topic creation
        registration_log = await register(
            FOXGLOVE_LOG_TOPIC,
            FOXGLOVE_LOG_MSG_TYPE,
            FOXGLOVE_LOGM_SG_SCHEMA)

        while not stop.is_set():
            try:
                topic_name, stamp, obj = await asyncio.wait_for(queue.get(), timeout=0.2)

                if topic_name == FOXGLOVE_LOG_TOPIC:
                    # special logic for logs
                    assert isinstance(obj, str), f'Expected string, but got {type(obj)}'
                    registration = registration_log
                    sec, nsec = to_stamp(stamp)

                    msg = {
                        "timestamp": {"sec": sec, "nsec": nsec},
                        "level": int(level),
                        "message": obj,
                        "name": "cartpole",
                        "file": "/dev/null",
                        "line": 0
                    } 

                    data = json.dumps(msg).encode()
                else:
                    registration = await register_class(topic_name, type(obj))
                    data = obj.model_dump_json().encode()

                await server.send_message(registration.channel_id, to_ns(stamp), data)

            except asyncio.TimeoutError:
                pass


async def _foxglove_async_wrapping(
        input_queue: asyncio.Queue,
        stop: Event,
        exception_queue: asyncio.Queue,
        level: int) -> None:
    try:
        await _foxglove_async_entrypoint(input_queue, stop, level)
    except Exception as e:
        await exception_queue.put(e)


def foxglove_main(
        loop: asyncio.AbstractEventLoop,
        input_queue: asyncio.Queue,
        stop: Event,
        exception_queue: asyncio.Queue,
        level: int) -> None:
    asyncio.set_event_loop(loop)
    loop.run_until_complete(_foxglove_async_wrapping(input_queue, stop, exception_queue, level))


class FoxgloveWebsocketLogger:
    """
    Logger to foxglove websocket, messages are available in real time.
    Class will start new thread with asyncio event loop.

    Example
    -------
    ```
    with FoxgloveWebsocketLogger() as log:
        obj = ... # some pydantic object
        log.publish('/topic', obj, stamp)
        log.info('message')
    ```
    """

    def __init__(self, level: int = Level.INFO):
        """
        Parameters
        ----------
        level: Level
            log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
        """

        self._loop = asyncio.new_event_loop()
        self._input_queue = asyncio.Queue()
        self._exception_queue = asyncio.Queue()
        self._stop = Event()

        args = (self._loop, self._input_queue, self._stop, self._exception_queue, level)

        self._foxlgove_thread = Thread(
            target=foxglove_main,
            name='foxglove_main_loop',
            daemon=True,
            args=args)

        self._foxlgove_thread.start()

    def __enter__(self):
        return self

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        """
        Publish object to topic.

        Parameters
        ----------
        topic_name: str
            topic name
        obj: BaseModel
            object to dump (pydantic model)
        stamp: float
            timestamp in nanoseconds (float)
        """

        if not (self._loop.is_running() and self._foxlgove_thread.is_alive()):
            if not self._exception_queue.empty():
                raise self._exception_queue.get_nowait()

            raise AssertionError('Foxglove logger is not running')

        item = (topic_name, stamp, obj)
        asyncio.run_coroutine_threadsafe(self._input_queue.put(item), self._loop)

    def log(self, msg: str, stamp: float, level: int) -> None:
        """
        Print message to topic `/log`.

        Parameters
        ----------
        msg: str
            message to print
        stamp: float
            timestamp in nanoseconds (float)
        level: Level
            log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
        """

        self.publish("`/log`", msg, stamp)

    def debug(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `DEBUG` level.
        """
        self.log(msg, stamp, Level.DEBUG)

    def info(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `INFO` level.
        """
        self.log(msg, stamp, Level.INFO)

    def warning(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `WARNING` level.
        """
        self.log(msg, stamp, Level.WARNING)

    def error(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `ERROR` level.
        """
        self.log(msg, stamp, Level.ERROR)

    def fatal(self, msg: str, stamp: float) -> None:
        """
        Print message to topic `/log` with `FATAL` level.
        """
        self.log(msg, stamp, Level.FATAL)

    def close(self):
        """
        Free log resources.
        """

        self._stop.set()
        self._foxlgove_thread.join()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class Logger:
    """
    Compound Logger class that logs to console, foxglove and mcap.

    Example
    -------
    ```
    with Logger(log_path='log.mcap', level=INFO) as log:
        obj = ... # some pydantic object

        log.publish('/topic', obj)
        log.info('message')
    ```
    """

    def __init__(self, log_path: str = '', level: Level = Level.INFO):
        """
        Parameters
        ----------
        log_path: str
            path to mcap log file, if not provided, no mcap log will be created
        level: Level
            log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
        """

        self._pylog = get_pylogger('cartpole', level)
        self._foxglove_log = FoxgloveWebsocketLogger()
        self._mcap_log = None

        if log_path:
            self._mcap_log = MCAPLogger(log_path, level=level)

    def publish(self, topic_name: str, obj: BaseModel, stamp: float) -> None:
        """
        Parameters
        ----------
        topic_name: str
            topic name
        obj: BaseModel
            pydantic object
        stamp: float
            timestamp in nanoseconds (float), if not provided, current time used
        """

        if self._mcap_log:
            self._mcap_log.publish(topic_name, obj, stamp)

        self._foxglove_log.publish(topic_name, obj, stamp)

    def log(self, msg: str, stamp: float, level: Level = Level.INFO) -> None:
        """
        Print message to console and topic `/log`.

        Parameters
        ----------
        msg: str
            message to print
        stamp: float
            timestamp in nanoseconds (float), if not provided, current time used
        level: Level
            log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
        """

        self._pylog.log(pylog_level(level), f'{stamp:.3f}: {msg}')
        self._foxglove_log.log(msg, stamp, level)

        if self._mcap_log:
            self._mcap_log.log(msg, stamp, level)

    def debug(self, msg: str, stamp: float) -> None:
        """
        Print message to console and topic `/log` with `DEBUG` level.
        """
        self.info(msg, stamp, Level.DEBUG)

    def info(self, msg: str, stamp: float) -> None:
        """
        Print message to console and topic `/log` with `INFO` level.
        """
        self.log(msg, stamp, Level.INFO)

    def warning(self, msg: str, stamp: float) -> None:
        """
        Print message to console and topic `/log` with `WARNING` level.
        """
        self.log(msg, stamp, Level.WARNING)

    def error(self, msg: str, stamp: float) -> None:
        """
        Print message to console and topic `/log` with `ERROR` level.
        """
        self.log(msg, stamp, Level.ERROR)

    def fatal(self, msg: str, stamp: float) -> None:
        """
        Print message to console and topic `/log` with `FATAL` level.
        """
        self.log(msg, stamp, Level.FATAL)

    def close(self):
        """
        Free log resources.
        """

        self._foxglove_log.close()
        if self._mcap_log:
            self._mcap_log.close()

    def __exit__(self):
        self.close()


__logger = None


def setup(log_path: str = '', level: Level = Level.INFO) -> None:
    """
    Setup gloval logger.

    Parameters
    ----------
    log_path: str
        path to mcap log file, if not provided, no mcap log will be created
    level: Level
        log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
    """

    global __logger
    close()
    __logger = Logger(log_path=log_path, level=level)


def close():
    """
    Close global logger
    """

    global __logger

    if __logger:
        __logger.close()
        __logger = None


atexit.register(close)


def get_logger() -> Logger:
    """
    Get global logger instance
    """

    global __logger
    if not __logger:
        setup()

    return __logger
 

def publish(topic_name: str, obj: BaseModel, stamp: float|None = None) -> None:
    """
    Publish object to topic of global logger.
    If logger is not set, it will be created with default settings.

    Parameters
    ----------
    topic_name: str
        topic name
    obj: BaseModel
        pydantic model
    stamp: float
        timestamp in nanoseconds (float), if not provided, current time used
    """

    get_logger().publish(topic_name, obj, this_or_now(stamp))


def log(msg: str, stamp: float|None = None, level: Level = Level.INFO) -> None:
    """
    Print message to console and topic `/log`.
    If logger is not set, it will be created with default settings.

    Parameters
    ----------
    msg: str
        message to print
    stamp: float
        timestamp in nanoseconds (float), if not provided, current time used
    level: Level
        log level (`UNKNOWN`, `DEBUG`, `INFO`, `WARNING`, `ERROR`, `FATAL`)
    """

    get_logger().log(msg, this_or_now(stamp), level)


def debug(msg: str, stamp: float|None = None) -> None:
    """
    Print message to console and topic `/log` with `DEBUG` level.
    If logger is not set, it will be created with default settings.
    """
    log(msg, stamp, Level.DEBUG)


def info(msg: str, stamp: float|None = None) -> None:
    """
    Print message to console and topic `/log` with `INFO` level.
    If logger is not set, it will be created with default settings.
    """
    log(msg, stamp, Level.INFO)


def warning(msg: str, stamp: float|None = None) -> None:
    """
    Print message to console and topic `/log` with `WARNING` level.
    If logger is not set, it will be created with default settings.
    """
    log(msg, stamp, Level.WARNING)


def error(msg: str, stamp: float|None = None) -> None:
    """
    Print message to console and topic `/log` with `ERROR` level.
    If logger is not set, it will be created with default settings.
    """
    log(msg, stamp, Level.ERROR)


def fatal(msg: str, stamp: float|None = None) -> None:
    """
    Print message to console and topic `/log` with `FATAL` level.
    If logger is not set, it will be created with default settings.
    """
    log(msg, stamp, Level.FATAL)
