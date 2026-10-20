# Copyright 2026 The NiFT authors
# SPDX-License-Identifier: Apache-2.0
"""Tests for lanelet2/_stdorder.py.

Every case in fixtures/cleanroom_vectors.json.gz must match exactly. The
property tests below check the general contracts on random input.
"""

import gzip
import importlib.util
import json
import random
from functools import lru_cache
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
_VECTORS = _HERE / "fixtures" / "cleanroom_vectors.json.gz"
_MODULE = _HERE.parents[1] / "py" / "fakelanelet2" / "lanelet2" / "_stdorder.py"

# Load the file directly so the test does not depend on the package init.
_spec = importlib.util.spec_from_file_location("_stdorder_under_test", _MODULE)
so = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(so)


def _less(a, b):
    return a[0] < b[0]


@lru_cache(maxsize=1)
def _vectors():
    with gzip.open(_VECTORS, "rt", encoding="utf-8") as fh:
        return json.load(fh)


def _items(values):
    return [(v, i) for i, v in enumerate(values)]


def _indices(items):
    return [i for _, i in items]


# ---------------------------------------------------------------------------
# Fixture vectors
# ---------------------------------------------------------------------------

def test_vector_counts():
    v = _vectors()
    assert len(v["sort"]) == 1351
    assert len(v["select"]) == 4125
    assert len(v["heap"]) == 400
    assert len(v["id_map"]) == 223


def test_sort_vectors():
    bad = []
    for k, case in enumerate(_vectors()["sort"]):
        items = _items(case["values"])
        so.sort_range(items, _less, case["lo"], case["hi"])
        if _indices(items) != case["result"]:
            bad.append(k)
    assert bad == []


def test_select_vectors():
    bad = []
    for k, case in enumerate(_vectors()["select"]):
        items = _items(case["values"])
        so.select_range(items, case["lo"], case["nth"], case["hi"], _less)
        if _indices(items) != case["result"]:
            bad.append(k)
    assert bad == []


def test_heap_vectors():
    bad = []
    for k, case in enumerate(_vectors()["heap"]):
        items = _items(case["values"])
        op = case["op"]
        if op == "build":
            so.heap_build(items, case["lo"], case["hi"], _less)
        elif op == "select":
            so.heap_select(items, case["lo"], case["mid"], case["hi"], _less)
        elif op == "sort":
            so.heap_sort(items, case["lo"], case["hi"], _less)
        else:
            raise AssertionError(f"unknown heap op {op!r}")
        if _indices(items) != case["result"]:
            bad.append(k)
    assert bad == []


def test_id_map_vectors():
    bad = []
    for k, case in enumerate(_vectors()["id_map"]):
        order, buckets = so.hash_map_order(case["keys"])
        if list(order) != case["order"] or buckets != case["bucket_count"]:
            bad.append((k, case["tag"]))
    assert bad == []


def test_depth_fallback_cases_present():
    v = _vectors()
    assert sum(c["depth_fallback"] for c in v["sort"]) == 48
    assert sum(c["depth_fallback"] for c in v["select"]) == 57


# ---------------------------------------------------------------------------
# Worked examples from the spec
# ---------------------------------------------------------------------------

def test_hash_map_worked_examples():
    assert so.hash_map_order([]) == ([], 1)
    assert so.hash_map_order([1, 2, 3, 4, 5]) == ([5, 4, 3, 2, 1], 13)
    assert so.hash_map_order(list(range(1, 15))) == (
        [14] + list(range(1, 14)), 29)
    assert so.hash_map_order([0, 13, 26, 1, 14]) == ([14, 1, 26, 13, 0], 13)


def test_hash_map_growth_points():
    expected = {1: 13, 13: 13, 14: 29, 29: 29, 30: 59, 59: 59, 60: 127,
                127: 127, 128: 257}
    for n, buckets in expected.items():
        assert so.hash_map_order(list(range(n)))[1] == buckets


# ---------------------------------------------------------------------------
# Property tests
# ---------------------------------------------------------------------------

def _random_values(rng, n):
    kind = rng.randrange(4)
    if kind == 0:
        return [rng.random() for _ in range(n)]
    if kind == 1:
        return [rng.randrange(3) for _ in range(n)]
    if kind == 2:
        return [rng.randrange(max(1, n // 4)) for _ in range(n)]
    return [rng.randrange(-10**9, 10**9) for _ in range(n)]


def _random_range(rng, n):
    lo = rng.randint(0, n)
    hi = rng.randint(lo, n)
    return lo, hi


def test_sort_property():
    rng = random.Random(1234)
    for _ in range(400):
        n = rng.randint(0, 300)
        values = _random_values(rng, n)
        lo, hi = _random_range(rng, n)
        items = _items(values)
        so.sort_range(items, _less, lo, hi)
        assert items[:lo] == _items(values)[:lo]
        assert items[hi:] == _items(values)[hi:]
        assert sorted(items[lo:hi]) == sorted(_items(values)[lo:hi])
        keys = [v for v, _ in items[lo:hi]]
        assert keys == sorted(keys)


def test_sort_default_bounds():
    rng = random.Random(7)
    values = _random_values(rng, 200)
    items = _items(values)
    so.sort_range(items, _less)
    assert [v for v, _ in items] == sorted(values)


def test_select_property():
    rng = random.Random(4321)
    for _ in range(600):
        n = rng.randint(0, 300)
        values = _random_values(rng, n)
        lo, hi = _random_range(rng, n)
        nth = rng.randint(lo, hi)
        items = _items(values)
        so.select_range(items, lo, nth, hi, _less)
        assert items[:lo] == _items(values)[:lo]
        assert items[hi:] == _items(values)[hi:]
        assert sorted(items[lo:hi]) == sorted(_items(values)[lo:hi])
        if nth < hi:
            pivot = items[nth][0]
            assert pivot == sorted(values[lo:hi])[nth - lo]
            assert all(v <= pivot for v, _ in items[lo:nth])
            assert all(v >= pivot for v, _ in items[nth + 1:hi])
        else:
            assert items == _items(values)


def _is_max_heap(keys):
    return all(not keys[(c - 1) // 2] < keys[c] for c in range(1, len(keys)))


def test_heap_build_property():
    rng = random.Random(99)
    for _ in range(300):
        n = rng.randint(0, 200)
        values = _random_values(rng, n)
        lo, hi = _random_range(rng, n)
        items = _items(values)
        so.heap_build(items, lo, hi, _less)
        assert _is_max_heap([v for v, _ in items[lo:hi]])
        assert sorted(items[lo:hi]) == sorted(_items(values)[lo:hi])
        assert items[:lo] + items[hi:] == (_items(values)[:lo]
                                            + _items(values)[hi:])


def test_heap_select_property():
    rng = random.Random(77)
    for _ in range(300):
        n = rng.randint(0, 200)
        values = _random_values(rng, n)
        lo, hi = _random_range(rng, n)
        mid = rng.randint(lo, hi)
        items = _items(values)
        so.heap_select(items, lo, mid, hi, _less)
        if mid == lo:
            assert items == _items(values)
            continue
        head = [v for v, _ in items[lo:mid]]
        assert _is_max_heap(head)
        assert sorted(head) == sorted(values[lo:hi])[:mid - lo]
        assert sorted(items[lo:hi]) == sorted(_items(values)[lo:hi])


def test_heap_sort_property():
    rng = random.Random(55)
    for _ in range(300):
        n = rng.randint(0, 200)
        values = _random_values(rng, n)
        lo, hi = _random_range(rng, n)
        items = _items(values)
        so.heap_sort(items, lo, hi, _less)
        keys = [v for v, _ in items[lo:hi]]
        assert keys == sorted(values[lo:hi])
        assert sorted(items[lo:hi]) == sorted(_items(values)[lo:hi])


def test_hash_map_property():
    rng = random.Random(2026)
    for _ in range(100):
        n = rng.randint(0, 500)
        keys = [rng.randrange(-2**63, 2**63) for _ in range(n)]
        keys += [rng.choice(keys) for _ in range(n // 5)] if keys else []
        order, buckets = so.hash_map_order(keys)
        assert sorted(order) == sorted(set(keys))
        assert len(order) == len(set(keys))
        assert buckets >= len(order)
        # Each bucket's ids form one contiguous run.
        bucket_seq = [(k % 2**64) % buckets for k in order]
        seen, prev = set(), None
        for b in bucket_seq:
            if b != prev:
                assert b not in seen
                seen.add(b)
                prev = b


def test_elements_are_moved_not_copied():
    rng = random.Random(3)
    objs = [[rng.randrange(5)] for _ in range(120)]
    items = list(objs)
    so.sort_range(items, lambda a, b: a[0] < b[0])
    assert {id(x) for x in items} == {id(x) for x in objs}


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
