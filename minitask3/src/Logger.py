import logging
from logging.handlers import RotatingFileHandler
import logging.config
import yaml
import os

cur_dir, filename = os.path.split(os.path.abspath(__file__))

logger = logging.getLogger("Robot-G31")
logger.setLevel(logging.DEBUG)

# set two handlers
log_file = "{}.log".format("robot-G31")
print(log_file)

#fileHandler = logging.FileHandler(os.path.join(cur_dir + "/../logs/", log_file), mode = 'w')
fileHandler = RotatingFileHandler(os.path.join(cur_dir + "/../logs/", log_file), maxBytes=5*1024*1024, backupCount=0)
fileHandler.setLevel(logging.INFO)
consoleHandler = logging.StreamHandler()
consoleHandler.setLevel(logging.INFO)

# set formatter
file_formatter = logging.Formatter('[%(asctime)s] [%(filename)s:%(lineno)d] [%(funcName)s] [%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
console_formatter = logging.Formatter('[%(filename)s:%(lineno)d] [%(levelname)s] %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
consoleHandler.setFormatter(console_formatter)
fileHandler.setFormatter(file_formatter)

# add
logger.addHandler(fileHandler)
logger.addHandler(consoleHandler)
