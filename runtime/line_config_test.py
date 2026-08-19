from runtime import line_config


def test_resolve_returns_value_source_pairs():
    result = line_config.resolve()
    assert result
    for entry in result.values():
        assert set(entry.keys()) == {"value", "source"}
        assert entry["source"] == "line_default"


def test_resolve_includes_buffer_size():
    result = line_config.resolve()
    assert "buffer_size" in result
    assert result["buffer_size"]["value"] == 1


def test_resolve_accepts_an_edge_id_and_is_currently_edge_id_independent():
    # Phase 2 stub: per-edge overrides aren't wired in yet, so every
    # edge_id currently resolves to the same line defaults. This should
    # start failing (correctly) once layering is implemented — see the
    # TODO in runtime/line_config.py.
    assert line_config.resolve("edge_a") == line_config.resolve("edge_b")
    assert line_config.resolve() == line_config.resolve("edge_a")
