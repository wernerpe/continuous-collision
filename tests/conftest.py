"""Silences Drake's one-line warning about rigid half spaces and deformable contact, which
several test worlds trigger and which has no bearing on distance queries."""

import logging

import pydrake.common  # noqa: F401 -- installs the "drake" logger.

logging.getLogger("drake").setLevel(logging.ERROR)
