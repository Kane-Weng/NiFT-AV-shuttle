# Copyright 2026 The NiFT authors
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
"""Container order helpers for the Lanelet2 stand-in.

This module was written from a behavioural specification only
(py/fakelanelet2/CLEANROOM_SPEC.md) and checked against its test vectors.
No other implementation was consulted.

It reproduces three things that decide how Lanelet2 breaks ties:

- the iteration order of a hash map keyed by integer ids,
- the final order of an unstable in-place sort,
- the final order of an in-place selection around one position.

Pure Python, standard library only, so it runs in Pyodide. Ranges are half
open. ``less(a, b)`` is True when ``a`` is strictly smaller than ``b``.
"""

__all__ = [
    "hash_map_order",
    "sort_range",
    "select_range",
    "heap_build",
    "heap_select",
    "heap_sort",
]

# ---------------------------------------------------------------------------
# Hash map iteration order
# ---------------------------------------------------------------------------

# Sizes used for small requests, indexed by the request itself.
_SMALL_SIZES = (1, 2, 2, 3, 5, 5, 7, 7, 11, 11, 11, 11, 13, 13)

# Sizes used for requests of 14 and above.
_PRIME_SIZES = (
    17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89,
    97, 103, 109, 113, 127, 137, 139, 149, 157, 167, 179, 193, 199, 211, 227,
    241, 257, 277, 293, 313, 337, 359, 383, 409, 439, 467, 503, 541, 577, 619,
    661, 709, 761, 823, 887, 953, 1031, 1109, 1193, 1289, 1381, 1493, 1613,
    1741, 1879, 2029, 2179, 2357, 2549, 2753, 2971, 3209, 3469, 3739, 4027,
    4349, 4703, 5087, 5503, 5953, 6427, 6949, 7517, 8123, 8783, 9497, 10273,
    11113, 12011, 12983, 14033, 15173, 16411, 17749, 19183, 20753, 22447,
    24281, 26267, 28411, 30727, 33223, 35933, 38873, 42043, 45481, 49201,
    53201, 57557, 62233, 67307, 72817, 78779, 85229, 92203, 99733, 107897,
    116731, 126271, 136607, 147793, 159871, 172933, 187091, 202409, 218971,
    236897, 256279, 277261, 299951, 324503, 351061, 379787, 410857, 444487,
    480881, 520241, 562841, 608903, 658753, 712697, 771049, 834181, 902483,
    976369, 1056323, 1142821, 1236397, 1337629, 1447153, 1565659, 1693859,
    1832561, 1982627, 2144977, 2320627, 2510653, 2716249, 2938679, 3179303,
    3439651, 3721303, 4026031, 4355707, 4712381, 5098259, 5515729, 5967347,
    6456007, 6984629, 7556579, 8175383, 8844859, 9569143, 10352717, 11200489,
    12117689, 13109983, 14183539, 15345007, 16601593, 17961079, 19431899,
    21023161, 22744717, 24607243, 26622317, 28802401, 31160981, 33712729,
    36473443, 39460231, 42691603, 46187573, 49969847, 54061849, 58488943,
    63278561, 68460391, 74066549, 80131819, 86693767, 93793069, 101473717,
)

_U64 = 1 << 64


def _bucket_size_for(request):
    """Return the bucket count chosen for a size request."""
    if request < len(_SMALL_SIZES):
        return _SMALL_SIZES[request]
    for size in _PRIME_SIZES:
        if size >= request:
            return size
    # Past the table the map stops growing. Out of scope in practice.
    return _PRIME_SIZES[-1]


class _RunList:
    """The map's single element sequence, kept as runs per bucket.

    Newest runs and newest ids go to the front. To keep placing cheap we
    store both levels oldest first and reverse them when reading back.
    """

    def __init__(self, buckets):
        self.buckets = buckets
        self.run_of = {}       # bucket -> ids in that run, oldest first
        self.run_order = []    # buckets in the order their run was opened

    def place(self, key):
        bucket = (key % _U64) % self.buckets
        run = self.run_of.get(bucket)
        if run is None:
            self.run_of[bucket] = [key]
            self.run_order.append(bucket)
        else:
            run.append(key)

    def sequence(self):
        out = []
        for bucket in reversed(self.run_order):
            out.extend(reversed(self.run_of[bucket]))
        return out


def hash_map_order(keys):
    """Return ``(iteration_order, bucket_count)`` after inserting ``keys``."""
    seen = set()
    runs = _RunList(1)
    capacity = 0
    for key in keys:
        if key in seen:
            continue
        count = len(seen)
        if count + 1 > capacity:
            if count == 0:
                new_buckets = _bucket_size_for(12)
            else:
                new_buckets = _bucket_size_for(max(count + 2, 2 * runs.buckets))
            capacity = new_buckets
            old = runs.sequence()
            runs = _RunList(new_buckets)
            for old_key in old:
                runs.place(old_key)
        runs.place(key)
        seen.add(key)
    return runs.sequence(), runs.buckets


# ---------------------------------------------------------------------------
# Heap helpers (max-heap stored inside items[base:base + size])
# ---------------------------------------------------------------------------

def _sift(items, base, size, start, value, less):
    """Drop ``value`` into the subtree rooted at ``start``.

    The hole first walks all the way down to a leaf, taking the larger child
    (the right one on ties). Then ``value`` climbs back up while its parent
    is smaller than it.
    """
    hole = start
    while 2 * hole + 2 < size:
        child = 2 * hole + 2
        if less(items[base + child], items[base + child - 1]):
            child -= 1
        items[base + hole] = items[base + child]
        hole = child
    if 2 * hole + 2 == size:
        child = 2 * hole + 1
        items[base + hole] = items[base + child]
        hole = child
    while hole > start:
        parent = (hole - 1) // 2
        if not less(items[base + parent], value):
            break
        items[base + hole] = items[base + parent]
        hole = parent
    items[base + hole] = value


def _pop_into(items, base, size, target, less):
    """Move the root to ``target`` and sift the old target element in."""
    value = items[target]
    items[target] = items[base]
    _sift(items, base, size, 0, value, less)


def heap_build(items, lo, hi, less):
    """Turn ``items[lo:hi]`` into a max-heap in place."""
    size = hi - lo
    if size < 2:
        return
    for pos in range((size - 2) // 2, -1, -1):
        _sift(items, lo, size, pos, items[lo + pos], less)


def _heap_drain(items, lo, hi, less):
    while hi - lo > 1:
        hi -= 1
        _pop_into(items, lo, hi - lo, hi, less)


def heap_select(items, lo, mid, hi, less):
    """Keep the ``mid - lo`` smallest elements of ``items[lo:hi]`` as a heap
    on ``items[lo:mid]``."""
    if mid == lo:
        return
    heap_build(items, lo, mid, less)
    size = mid - lo
    for i in range(mid, hi):
        if less(items[i], items[lo]):
            _pop_into(items, lo, size, i, less)


def heap_sort(items, lo, hi, less):
    """Sort ``items[lo:hi]`` ascending with a heap (not stable)."""
    heap_build(items, lo, hi, less)
    _heap_drain(items, lo, hi, less)


# ---------------------------------------------------------------------------
# Sort and select
# ---------------------------------------------------------------------------

_SMALL_SORT = 16


def _depth_budget(n):
    return 2 * (n.bit_length() - 1)


def _partition(items, s, e, less):
    """Median-of-three pivot to ``s``, then a two-marker scan.

    Returns the cut position. Needs at least 4 elements.
    """
    a, m, z = s + 1, s + (e - s) // 2, e - 1
    va, vm, vz = items[a], items[m], items[z]
    if less(va, vm):
        if less(vm, vz):
            pick = m
        elif less(va, vz):
            pick = z
        else:
            pick = a
    else:
        if less(va, vz):
            pick = a
        elif less(vm, vz):
            pick = z
        else:
            pick = m
    items[s], items[pick] = items[pick], items[s]

    pivot = items[s]
    left, right = s + 1, e
    while True:
        while less(items[left], pivot):
            left += 1
        right -= 1
        while less(pivot, items[right]):
            right -= 1
        if left >= right:
            return left
        items[left], items[right] = items[right], items[left]
        left += 1


def _insertion_sort(items, lo, hi, less):
    for i in range(lo + 1, hi):
        value = items[i]
        j = i
        while j > lo and less(value, items[j - 1]):
            items[j] = items[j - 1]
            j -= 1
        items[j] = value


def _coarse_sort(items, lo, hi, budget, less):
    while hi - lo > _SMALL_SORT:
        if budget == 0:
            heap_sort(items, lo, hi, less)
            return
        budget -= 1
        cut = _partition(items, lo, hi, less)
        _coarse_sort(items, cut, hi, budget, less)
        hi = cut


def sort_range(items, less, lo=0, hi=None):
    """Sort ``items[lo:hi]`` in place (unstable). Returns None."""
    if hi is None:
        hi = len(items)
    n = hi - lo
    if n <= 0:
        return
    _coarse_sort(items, lo, hi, _depth_budget(n), less)
    _insertion_sort(items, lo, hi, less)


def select_range(items, lo, nth, hi, less):
    """Partially sort ``items[lo:hi]`` so the element at ``nth`` is in its
    sorted place, with nothing greater before it and nothing smaller after.
    Returns None."""
    if hi <= lo or nth == hi:
        return
    budget = _depth_budget(hi - lo)
    while hi - lo > 3:
        if budget == 0:
            heap_select(items, lo, nth + 1, hi, less)
            items[lo], items[nth] = items[nth], items[lo]
            return
        budget -= 1
        cut = _partition(items, lo, hi, less)
        if cut <= nth:
            lo = cut
        else:
            hi = cut
    _insertion_sort(items, lo, hi, less)
