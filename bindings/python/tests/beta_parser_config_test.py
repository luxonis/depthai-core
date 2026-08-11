import numpy as np
import depthai as dai


PARSER_CONFIGS = [
    ("FastSAMParser", "FastSAMParserConfig", "FastSAMParserProperties"),
    ("HRNetParser", "HRNetParserConfig", "HRNetParserProperties"),
    ("MLSDParser", "MLSDParserConfig", "MLSDParserProperties"),
    ("MPPalmDetectionParser", "MPPalmDetectionParserConfig", "MPPalmDetectionParserProperties"),
    ("PPTextDetectionParser", "PPTextDetectionParserConfig", "PPTextDetectionParserProperties"),
    ("RFDETRParser", "RFDETRParserConfig", "RFDETRParserProperties"),
    ("SCRFDParser", "SCRFDParserConfig", "SCRFDParserProperties"),
    ("SuperAnimalParser", "SuperAnimalParserConfig", "SuperAnimalParserProperties"),
    ("YuNetParser", "YuNetParserConfig", "YuNetParserProperties"),
    ("ClassificationSequenceParser", "ClassificationSequenceParserConfig", "ClassificationSequenceParserProperties"),
    ("MapOutputParser", "MapOutputParserConfig", "MapOutputParserProperties"),
    ("XFeatMonoParser", "XFeatMonoParserConfig", "XFeatMonoParserProperties"),
    ("XFeatStereoParser", "XFeatStereoParserConfig", "XFeatStereoParserProperties"),
]


PROPERTY_FIELDS = {
    "FastSAMParserProperties": ("numClasses", "yoloOutputs", "maskOutputs", "protosOutput"),
    "HRNetParserProperties": ("outputLayerName", "labelNames", "edges"),
    "MLSDParserProperties": ("outputLayerTPMap", "outputLayerHeat", "inputSize"),
    "MPPalmDetectionParserProperties": ("outputLayerNames", "scale", "labelNames"),
    "PPTextDetectionParserProperties": ("outputLayerName",),
    "RFDETRParserProperties": ("labelNames", "outputLayerNames", "inputSize"),
    "SCRFDParserProperties": ("outputLayerNames", "inputSize", "featStrideFpn", "numAnchors", "labelNames"),
    "SuperAnimalParserProperties": ("outputLayerName", "scaleFactor", "nKeypoints", "labelNames", "edges"),
    "YuNetParserProperties": ("locOutputLayerName", "confOutputLayerName", "iouOutputLayerName", "inputSize", "labelNames"),
    "ClassificationSequenceParserProperties": ("outputLayerName", "classes", "nClasses", "isSoftmax"),
    "MapOutputParserProperties": ("outputLayerName",),
    "XFeatMonoParserProperties": ("outputLayerFeats", "outputLayerKeypoints", "outputLayerHeatmaps", "originalSize", "inputSize"),
    "XFeatStereoParserProperties": ("outputLayerFeats", "outputLayerKeypoints", "outputLayerHeatmaps", "originalSize", "inputSize"),
}

def test_every_parser_config_and_node_config_surface_is_bound():
    with dai.Pipeline(False) as pipeline:
        for node_name, config_name, properties_name in PARSER_CONFIGS:
            config_type = getattr(dai.beta, config_name)
            properties_type = getattr(dai.beta, properties_name)
            node_type = getattr(dai.beta.node, node_name)

            config = config_type()
            node = pipeline.create(node_type)

            assert config.validate()
            assert isinstance(node.initialConfig, config_type)
            assert node.inputConfig is not None
            assert node_type.Properties is properties_type
            for field in ("initialConfig", *PROPERTY_FIELDS[properties_name]):
                assert hasattr(properties_type, field)


def test_map_output_parser_accepts_and_applies_a_runtime_config():
    with dai.Pipeline(False) as pipeline:
        parser = pipeline.create(dai.beta.node.MapOutputParser)
        parser.setOutputLayerName("map")
        parser.inputConfig.setWaitForMessage(True)

        input_queue = parser.input.createInputQueue()
        config_queue = parser.inputConfig.createInputQueue()
        output_queue = parser.out.createOutputQueue()

        config = dai.beta.MapOutputParserConfig()
        config.minMaxScaling = True

        nn_data = dai.NNData()
        nn_data.addTensor("map", np.array([[2.0, 4.0]], dtype=np.float32))

        config_queue.send(config)
        pipeline.start()
        input_queue.send(nn_data)
        output = output_queue.get(timeout=1.0)

    np.testing.assert_allclose(output.getMap(), np.array([[0.0, 1.0]], dtype=np.float32))
