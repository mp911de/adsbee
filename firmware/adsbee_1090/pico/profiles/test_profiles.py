import os
from pathlib import Path
import subprocess
import shutil
import tempfile
import unittest

PROFILES = Path(__file__).resolve().parent


class BuildProfiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.profiles = self.root / "profiles"
        shutil.copytree(PROFILES, self.profiles)
        (self.root / "CMakeLists.txt").write_text(
            'cmake_minimum_required(VERSION 3.13)\nproject(ProfileCheck NONE)\n'
            f'include("{self.profiles / "profile.cmake"}")\n'
            'file(WRITE "${CMAKE_BINARY_DIR}/ti_build_dir.txt" "${ADSBEE_TI_BUILD_DIR}")\n'
            'get_directory_property(definitions COMPILE_DEFINITIONS)\n'
            'file(WRITE "${CMAKE_BINARY_DIR}/definitions.txt" "${definitions}")\n'
        )

    def env(self, profile):
        env = os.environ.copy()
        env.pop("ADSBEE_PROFILE", None)
        if profile is not None:
            env["ADSBEE_PROFILE"] = profile
        return env

    def configure(self, profile, build_type="Release"):
        return subprocess.run(
            ["cmake", "-S", str(self.root), "-B", str(self.root / "build"),
             f"-DCMAKE_BUILD_TYPE={build_type}"],
            env=self.env(profile), capture_output=True, text=True,
        )

    def definitions(self):
        return set((self.root / "build/definitions.txt").read_text().split(";"))

    def test_uart_settings(self):
        for profile, console, parity in [(None, 0, "NONE"), ("airgrep", 1, "EVEN")]:
            with self.subTest(profile=profile):
                result = self.configure(profile)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("ADSBEE_SUBGHZ_PROTOCOL=ADSBEE_SUBGHZ_PROTOCOL_" +
                              ("FLARM" if profile == "airgrep" else "UAT"), self.definitions())
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

    def test_ti_build_directory_matches_packaging(self):
        for profile in [None, "default", "airgrep"]:
            for mode in ["Release", "Debug"]:
                self.assertEqual(self.configure(profile, mode).returncode, 0)
                expected = "build" if profile in [None, "default"] else f"build/{mode}-{profile}"
                self.assertEqual((self.root / "build/ti_build_dir.txt").read_text(), expected)
                result = subprocess.run(
                    ["bash", "-ec", 'source "$1"; profile_ti_build_dir "$2"',
                     "profile-test", str(PROFILES / "profile.sh"), mode],
                    env=self.env(profile), capture_output=True, text=True,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout, expected)

    def test_invalid_protocol_fails_configuration(self):
        (self.profiles / "invalid.cmake").write_text("set(ADSBEE_SUBGHZ_PROTOCOL TYPO)\n")
        result = self.configure("invalid")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Invalid ADSBEE_SUBGHZ_PROTOCOL", result.stderr)

    def test_receiver_selection_rejects_invalid_backends(self):
        compiler = shutil.which("c++")
        if compiler is None:
            self.skipTest("C++ compiler unavailable")
        header = PROFILES.parent.parent / "ti/sub_ghz_radio/sub_ghz_radio/receiver_config.hh"
        for selection, error in [
            (None, None),
            ("ADSBEE_SUBGHZ_PROTOCOL_UAT", None),
            ("ADSBEE_SUBGHZ_PROTOCOL_FLARM", None),
            ("0", "Invalid ADSBEE_SUBGHZ_PROTOCOL"),
            ("TYPO", "Invalid ADSBEE_SUBGHZ_PROTOCOL"),
        ]:
            with self.subTest(selection=selection):
                args = [compiler, "-x", "c++", "-fsyntax-only", "-include", str(header), "-"]
                if selection is not None:
                    args.append(f"-DADSBEE_SUBGHZ_PROTOCOL={selection}")
                result = subprocess.run(args, input="", capture_output=True, text=True)
                if error is None:
                    self.assertEqual(result.returncode, 0, result.stderr)
                else:
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(error, result.stderr)

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
