import os
from pathlib import Path
import subprocess
import tempfile
import unittest

PROFILES = Path(__file__).resolve().parent


class BuildProfiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.13)\nproject(ProfileCheck NONE)\n'
            f'include("{PROFILES / "profile.cmake"}")\n'
            'get_directory_property(definitions COMPILE_DEFINITIONS)\n'
            'file(WRITE "${CMAKE_BINARY_DIR}/definitions.txt" "${definitions}")\n'
        )

    def env(self, profile):
        env = os.environ.copy()
        env.pop("ADSBEE_PROFILE", None)
        if profile is not None:
            env["ADSBEE_PROFILE"] = profile
        return env

    def configure(self, profile):
        return subprocess.run(
            ["cmake", "-S", str(self.root), "-B", str(self.root / "build")],
            env=self.env(profile), capture_output=True, text=True,
        )

    def definitions(self):
        return set((self.root / "build/definitions.txt").read_text().split(";"))

    def test_uart_settings(self):
        for profile, console, parity in [(None, 0, "NONE"), ("airgrep", 1, "EVEN")]:
            with self.subTest(profile=profile):
                result = self.configure(profile)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f"ADSBEE_COMMS_UART_CONSOLE={console}", self.definitions())
                self.assertIn(f"ADSBEE_COMMS_UART_PARITY=UART_PARITY_{parity}", self.definitions())
                self.assertIn(f'ADSBEE_BUILD_PROFILE="{profile or "default"}"', self.definitions())

    def test_unset_restores_defaults_in_existing_build(self):
        self.assertEqual(self.configure("airgrep").returncode, 0)
        self.assertEqual(self.configure(None).returncode, 0)
        self.assertIn("ADSBEE_COMMS_UART_CONSOLE=0", self.definitions())
        self.assertIn("ADSBEE_COMMS_UART_PARITY=UART_PARITY_NONE", self.definitions())

    def test_build_and_flash_variants(self):
        for profile, suffix in [(None, ""), ("default", ""), ("airgrep", "-airgrep")]:
            for mode in ["Release", "Debug", "Test"]:
                result = subprocess.run(
                    ["bash", "-ec", 'source "$1"; profile_build_variant "$2"',
                     "profile-test", str(PROFILES / "profile.sh"), mode],
                    env=self.env(profile), capture_output=True, text=True,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, mode + suffix)

    def test_unknown_and_unsafe_profiles_fail(self):
        for profile in ["missing-profile", "../airgrep", "airgrep;true", "airgrep with spaces"]:
            with self.subTest(profile=profile):
                self.assertNotEqual(self.configure(profile).returncode, 0)
                result = subprocess.run(
                    ["bash", "-ec", 'source "$1"', "profile-test", str(PROFILES / "profile.sh")],
                    env=self.env(profile), capture_output=True, text=True,
                )
                self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
