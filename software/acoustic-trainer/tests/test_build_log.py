"""Regression for CDT clean-only completion accidentally terminating compilation."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / ".vscode/scripts/Invoke-RaBuild.sh"
FUNCTION = SCRIPT.read_text().split("managed_build_succeeded() {", 1)[1].split("\ninvoke_managed_build()", 1)[0]
FUNCTION = "managed_build_succeeded() {" + FUNCTION


class BuildLogTests(unittest.TestCase):
    def check_log(self, log, expected):
        with tempfile.NamedTemporaryFile(mode="w") as file:
            file.write(log)
            file.flush()
            result = subprocess.run(["bash", "-c", FUNCTION + '\nmanaged_build_succeeded "$1" CPU0', "test", file.name])
            self.assertEqual(result.returncode == 0, expected)

    def test_clean_and_other_project_are_not_success(self):
        self.check_log("**** Clean-only build of configuration Debug for project CPU0 ****\n"
                       "Build Finished. 0 errors, 0 warnings.\n", False)
        self.check_log("**** Build of configuration Debug for project CPU1 ****\n"
                       "Build Finished. 0 errors, 0 warnings.\n", False)

    def test_actual_build_completion(self):
        self.check_log("**** Clean-only build of configuration Debug for project CPU0 ****\n"
                       "Build Finished. 0 errors, 0 warnings.\n"
                       "**** Build of configuration Debug for project CPU0 ****\nBuilding file: a.cc\n", False)
        self.check_log("**** Build of configuration Debug for project CPU0 ****\n"
                       "Build Finished. 0 errors, 4 warnings.\n", True)

    def test_failure_and_new_build_invalidate_success(self):
        self.check_log("**** Build of configuration Debug for project CPU0 ****\n"
                       "Build Failed. 1 errors.\n", False)
        self.check_log("**** Build of configuration Debug for project CPU0 ****\n"
                       "Build Finished. 0 errors.\n"
                       "**** Build of configuration Debug for project CPU0 ****\n", False)


if __name__ == "__main__":
    unittest.main()
