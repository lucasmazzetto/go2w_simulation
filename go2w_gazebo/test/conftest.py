"""Exercise the same Xacro preparation used by the installed launch file."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


@pytest.fixture(scope='session')
def resources():
    simulation = Path(__file__).resolve().parents[1]
    paths = []
    for name in ('go2w_description', 'go2w_gazebo'):
        try:
            paths.append(Path(get_package_share_directory(name)))
        except PackageNotFoundError:
            paths.append(simulation.parent / name)
    return tuple(paths)


@pytest.fixture(scope='session')
def launch_module(resources):
    spec = importlib.util.spec_from_file_location(
        'go2w_spawn_launch', resources[1] / 'launch/spawn.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def compose(launch_module, resources, tmp_path_factory):
    def expand(sensors=None, settings=None, robot_name='example', bridge_clock=False,
               description=None, sensors_file=None):
        simulation = resources[1]
        description = resources[0] if description is None else description
        directory = tmp_path_factory.mktemp('go2w_xacro')
        settings_file = simulation / 'config/parameters.yaml'
        if settings is not None:
            settings_file = directory / 'parameters.yaml'
            settings_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
        if sensors is not None:
            sensors_file = directory / 'sensors.yaml'
            sensors_file.write_text(yaml.safe_dump(sensors))
        return launch_module.prepare_simulation(
            description, simulation, settings_file, sensors_file,
            directory / 'controllers.yaml', robot_name, directory,
            world='test_world', bridge_clock=bridge_clock)
    return expand
