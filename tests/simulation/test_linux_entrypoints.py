"""Entry parsing and desktop environment checks; no ROS nodes or GUI launches."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class LinuxEntrypoints(unittest.TestCase):
    def run_entry(self, *args):
        with tempfile.TemporaryDirectory() as directory:
            return subprocess.run(['bash', str(ROOT/'edgeai.sh'), *args], cwd=directory,
                                  text=True, capture_output=True)

    def test_help_from_other_directory(self):
        for command in (('--help',), ('mission','--help'), ('navigation','--help'), ('cooperative','--help')):
            with self.subTest(command=command):
                result=self.run_entry(*command)
                self.assertEqual(result.returncode,0,result.stderr)
                self.assertIn('Usage:',result.stdout)
        result=self.run_entry('mission','--help')
        self.assertIn('default: rpp',result.stdout)
        self.assertIn('default: 3600',result.stdout)

    def test_invalid_arguments_fail_before_launch(self):
        for command in (('unknown',), ('mission','--not-an-option'), ('cooperative','--duration','0'),
                        ('cooperative','--domain','233'), ('cooperative','--params')):
            with self.subTest(command=command):
                self.assertEqual(self.run_entry(*command).returncode,2)

    def desktop(self, platform, settings):
        env=os.environ.copy()
        for key in ('DISPLAY','WAYLAND_DISPLAY','XAUTHORITY','QT_QPA_PLATFORM','GALLIUM_DRIVER',
                    'MESA_D3D12_DEFAULT_ADAPTER_NAME','EDGEAI_GALLIUM_DRIVER'):
            env.pop(key,None)
        env.update(settings, EDGEAI_DISPLAY_PLATFORM=platform)
        result=subprocess.run(['bash','-c','set -eu; source "$1"; env -0','desktop-test',
                               str(ROOT/'scripts/display/configure-desktop.sh')],
                              env=env,capture_output=True,check=True)
        return dict(item.split('=',1) for item in result.stdout.decode().split('\0') if item)

    def test_native_preserves_desktop_authentication_and_gpu(self):
        settings=dict(DISPLAY=':7',WAYLAND_DISPLAY='wayland-2',XAUTHORITY='/tmp/custom-auth',
                      GALLIUM_DRIVER='radeonsi',QT_QPA_PLATFORM='wayland')
        actual=self.desktop('native',settings)
        for key,value in settings.items():self.assertEqual(actual[key],value)
        self.assertNotIn('MESA_D3D12_DEFAULT_ADAPTER_NAME',actual)

    def test_native_does_not_invent_display_or_driver(self):
        actual=self.desktop('native',{})
        self.assertNotIn('DISPLAY',actual)
        self.assertNotIn('GALLIUM_DRIVER',actual)

    def test_wsl_defaults_and_explicit_overrides(self):
        actual=self.desktop('wslg',{})
        self.assertEqual(actual['DISPLAY'],':0');self.assertEqual(actual['GALLIUM_DRIVER'],'d3d12')
        actual=self.desktop('wslg',dict(DISPLAY=':3',XAUTHORITY='/tmp/user-auth',EDGEAI_GALLIUM_DRIVER='llvmpipe'))
        self.assertEqual(actual['DISPLAY'],':3');self.assertEqual(actual['XAUTHORITY'],'/tmp/user-auth')
        self.assertEqual(actual['GALLIUM_DRIVER'],'llvmpipe')


if __name__=='__main__':unittest.main()
