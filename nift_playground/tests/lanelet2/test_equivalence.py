"""The stand-in against saved results of the real Lanelet2 (fixtures/*.json.gz, made by compare_real.py).

Checks, on the NiFT map (odr2024) and Lanelet2's example map: projection both ways within 1 mm, every map node within
1 mm, every centerline point within 1 cm, the same traffic rules and graph links for every lanelet (both directions),
the same nearest lanelets for 500 points, the same route for 250 lanelet pairs (3 variants each) and the same
nearest-plus-route chain for 250 point pairs. The check is strict: where several lanelets are equally near, the
stand-in must pick the one Lanelet2 picks.
"""
import pytest

import equivalence as eq
import lanelet2
import lanelet2_dump


@pytest.mark.parametrize('use_cache', [True, False], ids=['node-cache', 'projected'])
@pytest.mark.parametrize('name', sorted(eq.MAPS))
def test_matches_real_lanelet2(name, use_cache, monkeypatch):
    """With the node cache (odr2024 ships one) and without it (the stand-in projects every node itself)."""
    if not use_cache:
        monkeypatch.setenv('LANELET2_STANDIN_NO_CACHE', '1')
    fixture = eq.load_fixture(name)
    queries = dict(fixture['queries'])
    queries['map'] = eq.map_path(name)
    assert getattr(lanelet2, 'IS_STAND_IN', False)
    stand = lanelet2_dump.dump(lanelet2, queries)
    stats, fails = eq.compare(fixture['real'], stand, fixture['queries'], eq._CostOracle(lanelet2, fixture['queries']),
                              strict=True)
    print(name, stats)
    assert not fails, '\n'.join(fails[:30])
    assert stats['nearest_points'] >= 200 and stats['routes_checked'] >= 600 and stats['pipeline_pairs'] >= 200
