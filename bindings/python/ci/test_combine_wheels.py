"""Combined compatibility tags must remain readable as WHEEL headers."""

from email.parser import Parser
from pathlib import Path
import tempfile
import unittest

from combine_wheels import WheelInfo, _update_wheel_file


class CombinedWheelMetadataTest(unittest.TestCase):
    def test_tags_precede_the_header_terminator(self):
        tags = [
            "cp312-cp312-macosx_11_0_arm64",
            "cp315-cp315t-macosx_11_0_arm64",
        ]
        infos = [WheelInfo.from_wheel_name(f"depthai-1.0-{tag}.whl") for tag in tags]
        for newline in ("\n", "\r\n"):
            with self.subTest(newline=repr(newline)), tempfile.TemporaryDirectory() as tmp:
                wheel = Path(tmp) / "WHEEL"
                wheel.write_text(newline.join([
                    "Wheel-Version: 1.0",
                    "Generator: regression-fixture",
                    "Root-Is-Purelib: false",
                    "Tag: cp39-cp39-macosx_11_0_arm64",
                    "Build: 7",
                    "",
                    "",
                ]), encoding="utf-8")

                _update_wheel_file(tmp, infos)

                metadata = Parser().parsestr(wheel.read_text(encoding="utf-8"))
                self.assertEqual(metadata.get_all("Tag"), tags)
                self.assertEqual(metadata["Wheel-Version"], "1.0")
                self.assertEqual(metadata["Generator"], "regression-fixture")
                self.assertEqual(metadata["Root-Is-Purelib"], "false")
                self.assertEqual(metadata["Build"], "7")
                self.assertFalse(metadata.get_payload().strip())


if __name__ == "__main__":
    unittest.main()
