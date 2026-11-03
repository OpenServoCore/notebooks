"""Jobs run on separate processes, for the fits a notebook repeats.

A job is a (function, args, kwargs) triple. Each worker unpickles its
job's function by module and name, so the function lives at the top of a
module the worker can import. A module built at run time, a second copy of
a notebook module pointed at another motor, registers itself in the worker
through `initializer`, which runs once in each worker before any job.
"""
import os
from concurrent.futures import ProcessPoolExecutor


def call(job):
    """One job, (function, args, kwargs), called."""
    fn, args, kwargs = job
    return fn(*args, **kwargs)


def run_parallel(jobs, workers=None, chunksize=1, cost=None,
                 initializer=None):
    """The jobs' results in the order given, the costliest run first."""
    workers = workers or os.cpu_count()
    if cost is None:
        order = list(range(len(jobs)))
    else:
        order = sorted(range(len(jobs)), key=lambda i: -cost[i])
    with ProcessPoolExecutor(max_workers=workers,
                             initializer=initializer) as ex:
        res = list(ex.map(call, [jobs[i] for i in order],
                          chunksize=chunksize))
    out = [None] * len(jobs)
    for i, r in zip(order, res):
        out[i] = r
    return out
