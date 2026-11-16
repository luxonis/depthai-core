import pickle
import unittest

import depthai as dai


class HeadSerializationTest(unittest.TestCase):
    def test_head_and_metadata_pickle(self):
        metadata = dai.nn_archive.v1.Metadata()
        metadata.nClasses = 2
        metadata.classes = ["first", "second"]
        metadata.strides = [8, 16, 32]
        metadata.extraParams = {"custom_options": {"enabled": True, "labels": [1, 3]}}
        head = dai.nn_archive.v1.Head()
        head.name = "detections"
        head.parser = "YOLO"
        head.outputs = ["boxes", "masks"]
        head.metadata = metadata
        for protocol in (2, pickle.HIGHEST_PROTOCOL):
            with self.subTest(protocol=protocol):
                restored = pickle.loads(pickle.dumps(head, protocol=protocol))
                self.assertEqual(restored.name, head.name)
                self.assertEqual(restored.parser, head.parser)
                self.assertEqual(restored.outputs, head.outputs)
                self.assertEqual(restored.metadata.nClasses, 2)
                self.assertEqual(restored.metadata.classes, metadata.classes)
                self.assertEqual(restored.metadata.strides, metadata.strides)
                self.assertEqual(restored.metadata.extraParams["custom_options"], metadata.extraParams["custom_options"])
                restored_metadata = pickle.loads(pickle.dumps(metadata, protocol=protocol))
                self.assertEqual(restored_metadata.classes, metadata.classes)
                self.assertEqual(restored_metadata.extraParams["custom_options"], metadata.extraParams["custom_options"])

    def test_empty_optional_fields(self):
        head = dai.nn_archive.v1.Head()
        head.parser = "custom"
        restored = pickle.loads(pickle.dumps(head))
        self.assertIsNone(restored.name)
        self.assertIsNone(restored.outputs)
        self.assertIsNone(restored.metadata.nClasses)
        self.assertIsNone(restored.metadata.classes)


if __name__ == "__main__":
    unittest.main()
