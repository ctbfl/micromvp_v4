from micromvp.core.planner import refine_paths, shuffle_paths


def test_refine_paths_keeps_lengths_equal():
    paths = [[(0.0, 0.0), (1.0, 0.0)], [(0.0, 0.0)]]
    refined = refine_paths(paths)
    assert len(refined[0]) == len(refined[1])


def test_shuffle_paths_assigns_closest():
    locs = [(0.0, 0.0), (10.0, 0.0)]
    paths = [[(9.0, 0.0)], [(1.0, 0.0)]]
    shuffled = shuffle_paths(locs, paths)
    assert shuffled[0][0] == (1.0, 0.0)
    assert shuffled[1][0] == (9.0, 0.0)
