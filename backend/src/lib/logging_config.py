import logging
import sys
from logging import config


def setup_logging():
    """Configure logging for the application."""
    logging_config = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "default": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            },
            "detailed": {
                "format": "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s",
            },
            "ai_agent": {
                "format": "%(asctime)s - AI_AGENT - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s",
            }
        },
        "handlers": {
            "console": {
                "class": "logging.StreamHandler",
                "level": "INFO",
                "formatter": "default",
                "stream": sys.stdout,
            },
            "file": {
                "class": "logging.FileHandler",
                "level": "INFO",
                "formatter": "detailed",
                "filename": "search_service.log",
            },
            "ai_agent_file": {
                "class": "logging.FileHandler",
                "level": "INFO",
                "formatter": "ai_agent",
                "filename": "ai_agent.log",
            }
        },
        "loggers": {
            "search_service": {
                "level": "INFO",
                "handlers": ["console", "file"],
                "propagate": False,
            },
            "ai_agent": {
                "level": "INFO",
                "handlers": ["console", "ai_agent_file"],
                "propagate": False,
            }
        },
        "root": {
            "level": "INFO",
            "handlers": ["console", "file"],
        }
    }

    logging.config.dictConfig(logging_config)


# Create a logger for the search service
search_logger = logging.getLogger("search_service")

# Create a logger for the AI agent
ai_agent_logger = logging.getLogger("ai_agent")