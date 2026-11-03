"""Job functions a worker process can import, for the parallel tests."""
import os

STATE = {"marked": False}


def square(x, offset=0):
    return x * x + offset


def mark():
    STATE["marked"] = True


def marked_in(pid_of_parent):
    return STATE["marked"], os.getpid() != pid_of_parent
