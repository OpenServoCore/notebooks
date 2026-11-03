"""Jobs on separate processes: results in the order given whatever order
they ran in, and the initializer run in each worker."""
import os

import parallel_jobs as jobs

from oscnb import parallel


def test_call_unpacks_a_job():
    assert parallel.call((jobs.square, (3,), {"offset": 1})) == 10


def test_results_come_back_in_the_order_given():
    work = [(jobs.square, (k,), {}) for k in range(6)]
    cost = [0, 5, 1, 4, 2, 3]
    out = parallel.run_parallel(work, workers=2, cost=cost)
    assert out == [k * k for k in range(6)]
    assert parallel.run_parallel(work, workers=2, chunksize=3) == out


def test_the_initializer_runs_in_each_worker():
    work = [(jobs.marked_in, (os.getpid(),), {}) for _ in range(4)]
    out = parallel.run_parallel(work, workers=2, initializer=jobs.mark)
    assert out == [(True, True)] * 4
    plain = parallel.run_parallel(work, workers=2)
    assert plain == [(False, True)] * 4

