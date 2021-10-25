#!/usr/bin/env python
import configparser
from enum import Enum
from datetime import datetime

config = configparser.ConfigParser()
config.read('./resources/config.ini')
config.sections()

class LogLevel(Enum):
    DEBUG = 1
    INFO = 2
    WARNING = 3
    ERROR = 4
    MSG = 99

    def __str__(self):
        if self.value == 1:
            return 'DEBUG'
        elif self.value == 2:
            return 'INFO'
        elif self.value == 3:
            return 'WARNING'
        elif self.value == 4:
            return 'ERROR'
        elif self.value == 99:
            return 'MSG'
        else:
            return ''

    @staticmethod
    def parse_str(s):
        if s == 'DEBUG':
            return LogLevel.DEBUG
        elif s == 'INFO':
            return LogLevel.INFO
        elif s == 'WARNING':
            return LogLevel.WARNING
        elif s == 'ERROR':
            return LogLevel.ERROR
        elif s == 'MSG':
            return LogLevel.MSG
        else:
            return None


# INIT LOGGING LEVEL BASED ON CONFIG FILE
if 'LOGGING_LEVEL' in config['GENERAL SETTINGS']:
    MIN_LOG_LEVEL: LogLevel = LogLevel.parse_str(config['GENERAL SETTINGS']['LOGGING_LEVEL'])
else:
    MIN_LOG_LEVEL: LogLevel = LogLevel.WARNING
#


class Logger():
    MSG_STR = "[{}] {} [{}]: {}"

    def __init__(self, speaker: str):
       self.speaker = speaker

    def log(self, msg: str):
       ts = datetime.now()
       print(self.MSG_STR.format(LogLevel.__str__(MIN_LOG_LEVEL), ts, self.speaker, msg))

    def debug(self, msg: str):
        if(LogLevel.DEBUG.value>=MIN_LOG_LEVEL.value):
            ts = datetime.now()
            print(self.MSG_STR.format(LogLevel.__str__(LogLevel.DEBUG), ts, self.speaker, msg))

    def info(self, msg: str):
        if(LogLevel.INFO.value>=MIN_LOG_LEVEL.value):
            ts = datetime.now()
            print(self.MSG_STR.format(LogLevel.__str__(LogLevel.INFO), ts, self.speaker, msg))

    def warn(self, msg: str):
        if(LogLevel.WARNING.value>=MIN_LOG_LEVEL.value):
            ts = datetime.now()
            print(self.MSG_STR.format(LogLevel.__str__(LogLevel.WARNING), ts, self.speaker, msg))

    def error(self, msg: str):
        if(LogLevel.ERROR.value>=MIN_LOG_LEVEL.value):
            ts = datetime.now()
            print(self.MSG_STR.format(LogLevel.__str__(LogLevel.ERROR), ts, self.speaker, msg))


    
