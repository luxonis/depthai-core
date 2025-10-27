import unittest


class TestDepthAI(unittest.TestCase):
    def test_import(self):
        try:
            import depthai
        except ImportError:
            print("Unable to import DepthAI module!")
            raise


if __name__ == "__main__":
    unittest.main()
