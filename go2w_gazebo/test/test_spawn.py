"""Check insertion ordering, failure handling and responsive launch shutdown."""

import asyncio
import gc
import os
from functools import partial
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueCoroutine,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited


SERVICES = '/world/test/create\n/world/test/scene/info\n'
POSE = dict(x=1.0, y=-2.0, z=0.43, R=0.1, P=-0.2, Y=0.3)


def context():
    return SimpleNamespace(environment=dict(os.environ), is_shutdown=False, emit_event=AsyncMock())


def test_scene_models_excludes_nested_entities(launch_module, monkeypatch):
    command = AsyncMock(return_value='''name: "test"
model {
  name: "robot"
  model {
    name: "nested"
  }
}
model {
  name: "wall"
}
''')
    monkeypatch.setattr(launch_module, 'run_command', command)
    assert asyncio.run(launch_module.scene_models(context(), 'test', 1)) == ['robot', 'wall']
    assert '/world/test/scene/info' in command.call_args.args[1]


def test_spawn_waits_for_services_and_actual_insertion(launch_module, monkeypatch, tmp_path):
    ctx, calls = context(), []
    service_calls = scene_calls = 0
    model_file = tmp_path / 'robot model.urdf'
    after = OpaqueFunction(function=lambda _: None)

    async def command(_context, args, timeout):
        nonlocal service_calls, scene_calls
        calls.append(args)
        assert timeout > 0
        assert ctx.emit_event.await_count == 0
        if args == ['gz', 'service', '--list']:
            service_calls += 1
            return SERVICES if service_calls > 1 else '/world/test/create\n'
        if args[0] == 'gz':
            scene_calls += 1
            return 'model {\n name: "robot"\n}\n' if scene_calls == 3 else 'name: "test"\n'
        assert scene_calls == 1  # Duplicate check precedes the creation request.
        assert args[1:9] == ['-world', 'test', '-name', 'robot', '-file', str(model_file),
                             '-allow_renaming', 'false']
        assert args[9:] == [item for key, value in POSE.items() for item in (f'-{key}', str(value))]
        return 'Entity creation request accepted\n'

    monkeypatch.setattr(launch_module, 'run_command', command)
    asyncio.run(launch_module.spawn_robot(ctx, 'test', 'robot', model_file, POSE, 2, [after]))
    assert service_calls == 2 and scene_calls == 3
    assert sum(Path(args[0]).name == 'create' for args in calls) == 1
    ctx.emit_event.assert_awaited_once()
    assert ctx.emit_event.call_args.args[0].launch_description.entities == [after]


@pytest.mark.parametrize('failure,message', [
    ('world', 'Timed out waiting'), ('duplicate', 'already exists'),
    ('scene', 'Timed out checking'), ('creator', 'creator failed'),
    ('invisible', 'did not appear'),
])
def test_spawn_failures_do_not_start_controllers(launch_module, monkeypatch, tmp_path, failure, message):
    ctx, created = context(), []

    async def command(_context, args, timeout):
        if args == ['gz', 'service', '--list']:
            return '' if failure == 'world' else SERVICES
        if args[0] == 'gz':
            if failure == 'scene':
                raise asyncio.TimeoutError
            return 'model {\n name: "robot"\n}\n' if failure == 'duplicate' else 'name: "test"\n'
        created.append(args)
        if failure == 'creator':
            raise RuntimeError('creator failed')
        return 'request accepted'

    monkeypatch.setattr(launch_module, 'run_command', command)
    with pytest.raises(RuntimeError, match=message):
        asyncio.run(launch_module.spawn_robot(ctx, 'test', 'robot', tmp_path / 'robot.urdf',
                                             POSE, 0.02, []))
    assert bool(created) == (failure in ('creator', 'invisible'))
    ctx.emit_event.assert_not_awaited()


@pytest.mark.parametrize('returncode', [0, 7, -15])
def test_checked_exit_keeps_bindings_independent(launch_module, monkeypatch, tmp_path, returncode):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    ctx = LaunchContext()
    process = ExecuteProcess(cmd=[sys.executable, '-c', 'pass'])
    event = ProcessExited(action=process, name='test_process', cmd=[], cwd=None,
                          env=None, pid=1234, returncode=returncode)
    bindings = []
    for label in ('Controller activation', 'Sensor readiness'):
        actions = []
        handler = OnProcessExit(target_action=process,
                                on_exit=partial(launch_module.checked_exit,
                                                label=label, actions=actions))
        bindings.append((label, actions, handler))

    for label, actions, handler in bindings:
        actions.append(OpaqueFunction(function=lambda _: None))
        assert handler.matches(event)
        if returncode == 0:
            assert handler.handle(event, ctx) is actions
        else:
            with pytest.raises(RuntimeError) as failure:
                handler.handle(event, ctx)
            assert str(failure.value) == f'{label} failed with exit code {returncode}'


@pytest.mark.parametrize('returncode', [0, 7])
def test_checked_exit_controls_launch_progression(launch_module, monkeypatch, tmp_path, returncode):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    events = []
    process = ExecuteProcess(cmd=[sys.executable, '-c', f'raise SystemExit({returncode})'])
    actions = [OpaqueFunction(function=lambda _: events.append('ready')),
               EmitEvent(event=Shutdown(reason='readiness complete'))]
    handler = OnProcessExit(target_action=process,
                            on_exit=partial(launch_module.checked_exit,
                                            label='Controller activation', actions=actions))
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(handler), process]))

    result = asyncio.run(service.run_async())
    assert (result == 0) == (returncode == 0)
    assert events == (['ready'] if returncode == 0 else [])


def test_spawn_shutdown_cleans_temporary_resources(launch_module, resources, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    shares = dict(zip(('go2w_description', 'go2w_gazebo'), resources))
    monkeypatch.setattr(launch_module, 'get_package_share_directory', lambda name: str(shares[name]))
    directories = []

    def temporary_directory(prefix, dir):
        directory = tempfile.TemporaryDirectory(prefix=prefix, dir=tmp_path)
        directories.append(Path(directory.name))
        return directory

    monkeypatch.setattr(launch_module, 'tempfile',
                        SimpleNamespace(TemporaryDirectory=temporary_directory))
    ctx = LaunchContext()
    for action in launch_module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    ctx.launch_configurations.update(enable_sensors='false', enable_lowstate='false', rviz='false')
    actions = launch_module.spawn_actions(ctx)
    directory, = directories
    before_shutdown = []

    def check_files(context):
        gc.collect()
        for name in ('description.urdf.xacro', 'controllers.yaml', 'go2w.urdf', 'bridges.yaml'):
            assert (directory / name).is_file()
        before_shutdown.append(True)

    try:
        service = LaunchService()
        service.include_launch_description(LaunchDescription([
            actions[0], OpaqueFunction(function=check_files),
            EmitEvent(event=Shutdown(reason='cleanup test'))]))
        assert asyncio.run(service.run_async()) == 0
        assert before_shutdown == [True]
        assert not directory.exists()
    finally:
        shutil.rmtree(directory, ignore_errors=True)


def test_command_environment_and_error_output(launch_module):
    ctx = context()
    ctx.environment['GO2W_SPAWN_TEST'] = 'launch environment'
    output = asyncio.run(launch_module.run_command(
        ctx, [sys.executable, '-c', 'import os; print(os.environ["GO2W_SPAWN_TEST"])'], 3))
    assert output.strip() == 'launch environment'
    with pytest.raises(RuntimeError, match='exit code 7: invalid request'):
        asyncio.run(launch_module.run_command(
            ctx, [sys.executable, '-c', 'import sys; print("invalid request"); sys.exit(7)'], 3))


def running(pid):
    try:
        return 'State:\tZ' not in Path(f'/proc/{pid}/status').read_text()
    except FileNotFoundError:
        return False


@pytest.mark.parametrize('cancel', [False, True], ids=['timeout', 'cancel'])
def test_command_cleanup_includes_child_processes(launch_module, tmp_path, cancel):
    parent_file, child_file = tmp_path / 'parent.pid', tmp_path / 'child.pid'
    child_code = ('import os, signal; from pathlib import Path; '
                  'signal.signal(signal.SIGTERM, signal.SIG_IGN); '
                  f'Path({str(child_file)!r}).write_text(str(os.getpid())); signal.pause()')
    parent_code = ('import os, signal, subprocess, sys; from pathlib import Path; '
                   f'Path({str(parent_file)!r}).write_text(str(os.getpid())); '
                   f'subprocess.Popen([sys.executable, "-c", {child_code!r}]); signal.pause()')

    async def check():
        task = asyncio.create_task(launch_module.run_command(
            context(), [sys.executable, '-c', parent_code], 10 if cancel else 1))
        try:
            deadline = asyncio.get_running_loop().time() + 3
            while not child_file.exists():
                assert not task.done(), 'Command exited before child startup'
                assert asyncio.get_running_loop().time() < deadline
                await asyncio.sleep(0.01)
            if cancel:
                task.cancel()
            with pytest.raises(asyncio.CancelledError if cancel else asyncio.TimeoutError):
                await task
            deadline = asyncio.get_running_loop().time() + 3
            while any(running(int(path.read_text())) for path in (parent_file, child_file)):
                assert asyncio.get_running_loop().time() < deadline, 'Owned process survived cleanup'
                await asyncio.sleep(0.01)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            if parent_file.exists():
                try:
                    os.killpg(int(parent_file.read_text()), signal.SIGKILL)
                except ProcessLookupError:
                    pass

    asyncio.run(check())


def test_launch_remains_responsive_while_waiting_for_world(launch_module, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    events = []

    async def waiting_command(*args):
        events.append('waiting')
        try:
            await asyncio.sleep(10)
        finally:
            events.append('cancelled')

    monkeypatch.setattr(launch_module, 'run_command', waiting_command)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        OpaqueCoroutine(coroutine=launch_module.spawn_robot,
                        args=['test', 'robot', tmp_path / 'robot.urdf', POSE, 20, []]),
        TimerAction(period=0.05, actions=[
            OpaqueFunction(function=lambda _: events.append('timer')),
            EmitEvent(event=Shutdown(reason='test shutdown')),
        ]),
    ]))
    started = time.monotonic()
    assert asyncio.run(service.run_async()) == 0
    assert time.monotonic() - started < 3
    assert events == ['waiting', 'timer', 'cancelled']


def test_spawn_failure_propagates_to_launch(launch_module, monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    monkeypatch.setattr(launch_module, 'run_command', AsyncMock(return_value=''))
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        OpaqueCoroutine(coroutine=launch_module.spawn_robot,
                        args=['test', 'robot', tmp_path / 'robot.urdf', POSE, 0.02, []]),
    ]))
    assert asyncio.run(service.run_async()) != 0
