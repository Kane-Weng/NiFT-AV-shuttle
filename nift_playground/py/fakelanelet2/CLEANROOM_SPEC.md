# Clean-room spec: container order helpers for the Lanelet2 stand-in

## Why this exists

Lanelet2 picks between primitives at exactly the same distance (for example a point inside two overlapping lanelets) in an order that the C++ standard containers and algorithms decide. The stand-in has to give the same pick. So it needs a small helper module that reproduces three observable behaviours of the C++ runtime that Lanelet2 is built with (GCC 11 on Ubuntu 22.04, ROS Humble).

1. The iteration order of a hash map keyed by 64-bit integer ids, after inserting ids one by one.
2. The exact final order of an unstable in-place sort.
3. The exact final order of an in-place "partial sort around one position" (selection).

This document describes the behaviour only. It is written so that you can build the module without looking at any other implementation. Follow the rules here exactly. Small "harmless" changes (a different pivot, a different heap layout, a stable sort) change the order of equal elements, and that changes which lanelet findNearest returns.

A behavioural fixture checks your result. It lives in `nift_playground/tests/lanelet2/fixtures/cleanroom_vectors.json` and is described at the end. Real C++ output is also in `nift_playground/tests/lanelet2/fixtures/cxx_ground_truth.json.gz`. Your module must match both.

## Module to provide

Put the module at `nift_playground/py/fakelanelet2/lanelet2/_stdorder.py` (the name is a suggestion). Plain Python, no dependencies, so it runs in Pyodide.

The comparison argument `less` is always a function of two elements that returns True when the first is strictly smaller. Callers only ever use a strict weak order (they compare floats or ints that are never NaN). Elements are opaque objects. You move whole elements around, never copy or rebuild them. All ranges are half open: `lo` is the first position, `hi` is one past the last.

| Function | Meaning |
|---|---|
| `hash_map_order(keys)` | Takes a list of integer ids in insertion order. Returns a pair: the list of distinct ids in the map's iteration order, and the final bucket count (int). |
| `sort_range(items, less, lo=0, hi=None)` | Sorts `items[lo:hi]` in place following section 2. `hi=None` means `len(items)`. Returns None. |
| `select_range(items, lo, nth, hi, less)` | Rearranges `items[lo:hi]` in place following section 3. Returns None. `nth` may equal `hi`, then nothing happens. |
| `heap_build(items, lo, hi, less)` | Section 4, build. Exposed for the tests. |
| `heap_select(items, lo, mid, hi, less)` | Section 4, select. Exposed for the tests. Does nothing when `mid == lo`. |
| `heap_sort(items, lo, hi, less)` | Section 4, build and then drain. Exposed for the tests. |

The argument order of `sort_range` and `select_range` is the same as the functions they replace, so the callers only change their import.

## Who uses what

- `lanelet2/core.py`, in the layer method that builds the R-tree. It calls `hash_map_order` with the layer's ids in the order they were added. The loader adds them in ascending id order. Only the order is used, the bucket count is ignored.
- `lanelet2/_rtree.py`, when packing the tree. It calls `select_range` on a list of `(centre_x, centre_y, index)` tuples, on sub-ranges `[first, last)` with `nth` at the split position, comparing on either `centre_x` or `centre_y` only. Ties are common, because many boxes share a centre coordinate.
- `lanelet2/_rtree.py`, in the nearest-neighbour walk. It calls `sort_range` on the whole list, on `(distance, node)` pairs of child branches and on `(distance, (box, value))` pairs of found neighbours, comparing on the distance only. Ties are common here too.
- `lanelet2/geometry.py` does not call the module. `findNearest` depends on it only through the R-tree order above.
- `lanelet2/routing.py` does not use it. Dijkstra there uses Python's own heap with an insertion counter as tie break. Nothing in this spec applies to routing.
- `tests/lanelet2/test_standin.py` calls the old functions directly against the C++ ground truth. Point it at the new module. The docstrings in `lanelet2/__init__.py` and `lanelet2/_rtree.py` mention the old module name and should be updated too.

## 1. Hash map iteration order

### Model

The map stores every element in one single sequence. Iteration walks this sequence from front to back. Elements of the same bucket always sit next to each other in the sequence, so the sequence is a series of "runs", one run per non-empty bucket. The map has a bucket count `B` and a capacity `C`. It starts with `B = 1`, `C = 0` and no elements.

### Which bucket

The hash of an id is the id itself. Read the id as an unsigned 64-bit number (a negative id `k` becomes `k + 2^64`). The bucket is that number modulo `B`.

### Placing one id (used both for a new insert and during a rehash)

- If the id's bucket already has a run, the id goes to the front of that run. So inside a run the most recently placed id comes first.
- If the bucket has no run yet, a new run holding only this id goes to the very front of the whole sequence.

So runs appear newest first (by the moment the bucket got its first element since the last rehash), and inside each run ids appear newest first.

### Inserting

Process the input ids one at a time.

1. If the id is already in the map, skip it. It keeps its place and nothing else changes.
2. Let `n` be the number of ids already in the map. If `n + 1 > C`, grow before placing (see below).
3. Place the id with the rule above.

### Growing (rehash)

- On the very first insert (`n = 0`) the new bucket count is the size chosen for a request of 12. That gives 13.
- Otherwise the request is the larger of `n + 2` and `2 × B`, and the new bucket count is the size chosen for that request. In practice the request is always `2 × B`.
- Set the capacity `C` to the new bucket count (the maximum load factor is 1.0).
- Take the current sequence as a list from front to back. Empty the map. Set `B` to the new bucket count. Then place every id again, in that old front-to-back order, with the placing rule above and the new `B`.

Only after this is the new id placed.

Growth happens when inserting the 1st, 14th, 30th, 60th, 128th, ... distinct id. The bucket counts go 13, 29, 59, 127, 257, 541, 1109, 2357, 5087, 10273, ...

### Choosing a size for a request `r`

- For `r` from 0 to 13, use this table (position `r` gives the size): 1, 2, 2, 3, 5, 5, 7, 7, 11, 11, 11, 11, 13, 13. Only `r = 12` ever occurs.
- For `r` of 14 or more, use the smallest value in the published table below that is at least `r`.
- If `r` is larger than every table value, use the last value and never grow again. This only matters past about 50 million ids and is out of scope.

The published size table (202 primes, from the GCC 11 runtime on 64-bit Linux):

17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 103, 109, 113, 127, 137, 139, 149, 157, 167, 179, 193, 199, 211, 227, 241, 257, 277, 293, 313, 337, 359, 383, 409, 439, 467, 503, 541, 577, 619, 661, 709, 761, 823, 887, 953, 1031, 1109, 1193, 1289, 1381, 1493, 1613, 1741, 1879, 2029, 2179, 2357, 2549, 2753, 2971, 3209, 3469, 3739, 4027, 4349, 4703, 5087, 5503, 5953, 6427, 6949, 7517, 8123, 8783, 9497, 10273, 11113, 12011, 12983, 14033, 15173, 16411, 17749, 19183, 20753, 22447, 24281, 26267, 28411, 30727, 33223, 35933, 38873, 42043, 45481, 49201, 53201, 57557, 62233, 67307, 72817, 78779, 85229, 92203, 99733, 107897, 116731, 126271, 136607, 147793, 159871, 172933, 187091, 202409, 218971, 236897, 256279, 277261, 299951, 324503, 351061, 379787, 410857, 444487, 480881, 520241, 562841, 608903, 658753, 712697, 771049, 834181, 902483, 976369, 1056323, 1142821, 1236397, 1337629, 1447153, 1565659, 1693859, 1832561, 1982627, 2144977, 2320627, 2510653, 2716249, 2938679, 3179303, 3439651, 3721303, 4026031, 4355707, 4712381, 5098259, 5515729, 5967347, 6456007, 6984629, 7556579, 8175383, 8844859, 9569143, 10352717, 11200489, 12117689, 13109983, 14183539, 15345007, 16601593, 17961079, 19431899, 21023161, 22744717, 24607243, 26622317, 28802401, 31160981, 33712729, 36473443, 39460231, 42691603, 46187573, 49969847, 54061849, 58488943, 63278561, 68460391, 74066549, 80131819, 86693767, 93793069, 101473717

### Worked examples

- Ids 1, 2, 3, 4, 5. All go into 13 buckets, each in its own bucket, each new run at the front. Order 5, 4, 3, 2, 1. Bucket count 13.
- Ids 1 to 14. After 13 ids the order is 13, 12, ..., 1. The 14th id triggers growth to 29. Re-placing 13, 12, ..., 1 one by one, each into its own new bucket at the front, gives 1, 2, ..., 13. Then 14 goes in front. Order 14, 1, 2, ..., 13. Bucket count 29.
- Ids 0, 13, 26, 1, 14 with 13 buckets. 0, 13 and 26 share bucket 0, and 1 and 14 share bucket 1. Order 14, 1, 26, 13, 0.

## 2. Sort

The sort is not stable. Equal elements end up in an order decided by the steps below. Only the `less` function is used to compare. Let `N = hi - lo`.

If `N` is 0, do nothing.

### Phase A, coarse partitioning

Set a depth budget to twice the floor of the base-2 logarithm of `N` (for `N = 100` that is 12). Then process the range as follows.

While the current range has more than 16 elements:

1. If the depth budget is 0, sort the current range with the heap method (section 4, build then drain) and stop processing this range.
2. Otherwise lower the budget by 1 and partition the range (see "Partition step" below). This splits it at a cut position into a left part (from the range start up to the cut) and a right part (from the cut to the range end).
3. Process the right part with the same loop, starting from the current budget (the right part gets its own copy of the budget).
4. Continue the loop with the left part as the current range, keeping the same budget.

A range of 16 elements or fewer is left as it is in Phase A.

### Partition step (shared by sort and select)

Call the range positions `s` (first) to `e` (one past last), with at least 4 elements.

Choosing the pivot. Look at three positions. Call them A = `s + 1`, M = `s + (e - s) // 2` (integer division), and Z = `e - 1`. Pick one of them using these comparisons of the elements stored there.

- If A is less than M:
  - if M is less than Z, pick M;
  - else if A is less than Z, pick Z;
  - else pick A.
- If A is not less than M:
  - if A is less than Z, pick A;
  - else if M is less than Z, pick Z;
  - else pick M.

Swap the element at `s` with the element at the picked position. The pivot now sits at `s` and stays there for the whole partition step. It is not moved into the middle afterwards.

Scanning. Use a left marker starting at `s + 1` and a right marker starting at `e`. Repeat:

1. Move the left marker right while its element is less than the pivot.
2. Move the right marker one step left. Then keep moving it left while the pivot is less than its element.
3. If the left marker is not before the right marker, stop. The cut position is the left marker.
4. Otherwise swap the two elements under the markers, move the left marker one step right, and go back to step 1.

Notes. Elements equal to the pivot stop both scans, so they get swapped. That is what spreads equal keys across both sides. The scans never run past the range. The pivot and the median choice guarantee a stopping element on each side, so no bounds check is needed, and adding one does not change the result. After the step, no element in the right part is less than any element in the left part.

### Phase B, final pass

After Phase A, run a straight insertion sort over the whole range `lo` to `hi`. Go through positions from left to right. Take each element out and move it left past every neighbour that is strictly greater than it (stop at the first neighbour that it is not less than, or at `lo`). This pass does not reorder equal elements.

(The C++ runtime does this pass as an insertion sort of the first 16 elements followed by an unchecked insertion of the rest. The outcome is the same as the plain insertion sort described here, because Phase A always leaves a smallest element inside the first 16 positions.)

If `N` is 16 or less, Phase A does nothing and only Phase B runs.

## 3. Select (partial sort around position `nth`)

After the call, the element at `nth` is one that a full sort could put there, nothing before it is greater, and nothing after it is less. The exact arrangement of every other element is also checked by the tests, so follow these steps exactly.

1. If the range is empty or `nth` equals `hi`, do nothing.
2. Set a depth budget to twice the floor of the base-2 logarithm of `hi - lo`.
3. While the current range has more than 3 elements:
   - If the budget is 0, use the heap fallback. Run heap select (section 4) with the heap covering positions `lo` to `nth + 1` (exclusive) and scanning up to `hi`. Then swap the elements at `lo` and `nth`. Stop, the call is finished (skip step 4).
   - Otherwise lower the budget by 1 and run the same partition step as the sort (section 2). If the cut is at or before `nth`, the range becomes cut to `hi`. Otherwise it becomes `lo` to cut.
4. Run the straight insertion sort from Phase B on the final small range only.

Here `lo` and `hi` mean the current range bounds, which move as the loop narrows the range.

## 4. Heap operations

These are used only as the depth-budget fallback of sort and select. They are a max-heap stored in the range itself. Position `i` counted from the start of the range has children at `2i + 1` and `2i + 2`. The root is the first position.

### Sift (the one primitive)

Inputs are a heap range of length `L`, a starting position `t`, and a value `v` that is to be placed somewhere in the subtree of `t` (the slot at `t` is treated as empty).

1. Going down. Start with the empty slot at `t`. While the empty slot has two children, pick the right child unless the right child is less than the left child (so on a tie the right child is picked). Move the picked child up into the empty slot. The slot moves down to the child's position. If the empty slot then has exactly one child (the left one, which only happens at the very end of an even-length heap), move that child up and the slot moves down to it. The descent always goes all the way to a leaf. It does not compare against `v` on the way down.
2. Going up. From the empty slot, while the slot is below `t` and the parent of the slot is less than `v`, move the parent down into the slot and move the slot up to the parent. Then put `v` in the slot.

The choice "always descend to a leaf, then climb back" matters for equal elements. A sift that stops early on the way down gives a different order.

### Build

For a range of length below 2, do nothing. Otherwise, for each position from `(L - 2) // 2` down to 0, take the element at that position as `v` and sift it with `t` equal to that position.

### Pop one into a given slot

Inputs are a heap of length `L` and a target position outside the heap (or the heap's last slot). Take the element at the target as `v`. Copy the root element into the target. Then sift `v` from the root (`t = 0`) within the heap of length `L`.

### Drain (turns a heap into ascending order)

While the heap has more than one element, shrink it by one at the end. Pop the root into the slot that was just released (its old element becomes `v`, and the sift uses the shrunken length).

### Heap select (`lo`, `mid`, `hi`)

Build a heap on `lo` to `mid`. Then for each position `i` from `mid` to `hi - 1` in order, if the element at `i` is less than the current root, pop the root into position `i` (the old element at `i` is sifted into the heap of length `mid - lo`).

### Heap sort (`lo`, `hi`)

Build a heap on the range, then drain it. The sort fallback in section 2 is exactly this.

## Test vectors

File `nift_playground/tests/lanelet2/fixtures/cleanroom_vectors.json`. It was produced by running the previous implementation, which matches the real C++ ground truth. It has 6099 cases.

Element model for every sort, select and heap case. Build a list of items `(values[i], i)` for each `i`. The comparison looks only at the value (first item less than second item by value). `result` is the list of the original indices `i` at every position of the whole list after the operation, including positions outside `lo..hi`, which must be unchanged.

| Key | Count | Fields | How to run |
|---|---|---|---|
| `sort` | 1351 | `values`, `lo`, `hi`, `result`, `depth_fallback` | `sort_range(items, less, lo, hi)` |
| `select` | 4125 | `values`, `lo`, `nth`, `hi`, `result`, `depth_fallback` | `select_range(items, lo, nth, hi, less)` |
| `heap` | 400 | `op`, `values`, `lo`, `hi`, `mid` (only for `op = "select"`), `result` | `op` is `build`, `select` or `sort`, calling `heap_build`, `heap_select` or `heap_sort` |
| `id_map` | 223 | `tag`, `keys`, `order`, `bucket_count` | `hash_map_order(keys)` must return `(order, bucket_count)` |

Coverage. Sizes run from 0 to 200, with distinct values, few distinct values, three values, all equal, ascending, descending, organ pipe, wide random ints, floats, and floats with many ties. There are random sub-ranges and R-tree-like select calls. Cases where `depth_fallback` is true (48 sort, 57 select) are adversarial inputs that run the depth budget down to 0, so they exercise the heap fallback. The map cases cover ascending ids from 0 to 60 elements and sizes across every growth point up to 4100 ids, sparse ascending ids, random ids, duplicates, negative ids, ids anywhere in the signed 64-bit range, large OSM-style ids, and ids that are multiples of a bucket count (long runs in one bucket).
