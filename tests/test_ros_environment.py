import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import unittest

HELPER = Path(__file__).resolve().parents[1] / 'scripts/ros_environment.sh'


class ViewerEnvironmentTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='viewer environment ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.workspace = self.root / 'workspace'
        (self.workspace / 'install').mkdir(parents=True)
        (self.workspace / 'install/local_setup.bash').write_text('export OVERLAY_LOADED=yes\n')

    def add_ros(self, distro):
        path = self.root / 'ros' / distro
        path.mkdir(parents=True)
        (path / 'setup.bash').write_text('export ROS_DISTRO=' + distro + '\n')

    def invoke(self, distro=''):
        return subprocess.run(['bash', '-c',
            'set -eu; source "$1"; cpp_control_source_ros "$2" "$3"; '
            'printf "%s %s" "$ROS_DISTRO" "$OVERLAY_LOADED"',
            'test', str(HELPER), str(self.workspace), str(self.root / 'ros')],
            env=dict(os.environ, ROS_DISTRO=distro), text=True, capture_output=True)

    def test_jazzy_fresh_shell_and_paths_with_spaces(self):
        self.add_ros('jazzy')
        result = self.invoke()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'jazzy yes')

    def test_selected_distribution_wins_and_invalid_selection_fails(self):
        self.add_ros('humble')
        self.add_ros('jazzy')
        self.assertNotEqual(self.invoke().returncode, 0)
        self.assertEqual(self.invoke('jazzy').stdout, 'jazzy yes')
        self.assertNotEqual(self.invoke('foxy').returncode, 0)

    def test_wrong_overlay_is_rejected_before_sourcing(self):
        self.add_ros('jazzy')
        marker = self.workspace / 'install/vibe_build.json'
        marker.write_text(json.dumps({'ros_distro': 'humble',
            'architecture': platform.machine(), 'python': '{}.{}'.format(*sys.version_info[:2])}))
        result = self.invoke('jazzy')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('differs', result.stderr)


if __name__ == '__main__':
    unittest.main()
