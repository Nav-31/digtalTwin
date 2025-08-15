import logging
import sys

def setup_logging(console_level=logging.INFO):
    """
    Configures a centralized logger that writes to both a file and the console.
    """
    # The root logger's level should be the lowest of its handlers to capture all messages.
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    # Prevent handlers from being added multiple times in interactive environments
    if root_logger.hasHandlers():
        root_logger.handlers.clear()

    # Create a formatter for the log messages
    log_formatter = logging.Formatter(
        '%(asctime)s - %(levelname)-8s - [%(name)s] - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )


    file_handler = logging.FileHandler("simulation.log", mode='w')
    file_handler.setFormatter(log_formatter)
    file_handler.setLevel(logging.DEBUG)
    root_logger.addHandler(file_handler)

    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(log_formatter)
    console_handler.setLevel(console_level)
    root_logger.addHandler(console_handler)