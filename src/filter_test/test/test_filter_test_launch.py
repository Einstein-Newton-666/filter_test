import importlib.util
from pathlib import Path


def load_filter_test_launch():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "filter_test.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "filter_test_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tracker_backend_default_reads_yaml_global_parameter(tmp_path):
    module = load_filter_test_launch()
    config = tmp_path / "config.yaml"
    config.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    tracker_backend: filter_graph\n",
        encoding="utf-8")

    assert module.tracker_backend_default_from_yaml(str(config)) == "filter_graph"


def test_tracker_backend_default_rejects_unknown_value(tmp_path):
    module = load_filter_test_launch()
    config = tmp_path / "config.yaml"
    config.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    tracker_backend: typo\n",
        encoding="utf-8")

    assert module.tracker_backend_default_from_yaml(str(config)) == "graph"
