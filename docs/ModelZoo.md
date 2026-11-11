# Model descriptions

`NNModelDescription` selects an NN archive to download from the Luxonis model zoo.
The [complete YAML example](../examples/models/nn_model_description.yaml) lists
every key read by `NNModelDescription::fromYamlFile()`.

| YAML key | C++ / Python field | Meaning |
| --- | --- | --- |
| `model` | `model` | Required model slug, optionally including team and variant, e.g. `luxonis/yolov6-nano:r2-coco-512x288`. |
| `platform` | `platform` | One hardware platform, such as `RVC2` or `RVC4`. |
| `optimization_level` | `optimizationLevel` | Optional Hailo export optimization selector: `"-100"`, `"0"`, `"1"`, `"2"`, `"3"`, or `"4"`. Example: `"2"`. |
| `compression_level` | `compressionLevel` | Optional Hailo export compression selector: `"0"` through `"5"`. Example: `"2"`. This does not control NN archive compression. |
| `snpe_version` | `snpeVersion` | Optional SNPE version of an RVC4 export, e.g. `"2.23.0"`. |
| `model_precision_type` | `modelPrecisionType` | Optional export precision, e.g. `"INT8"` for the example's RVC4 export or `"FP16"` for an RVC2 export. |

All values are strings; quote numeric selectors and version numbers. These
settings select existing exports, rather than converting a model. Use a
combination available for the chosen model and platform in the model zoo.
Omitting an optional key or setting it to `""` leaves that request setting
unspecified.

The example specifies the SNPE version and precision of the available
[YOLOv6 Nano R2 COCO 512x288 RVC4 export](https://models.luxonis.com/luxonis/yolov6-nano/aim_RFFiRGVUcFAVE894kxRbRm).
The optimization and compression selectors apply to Hailo exports, as described
in the [model instance reference](https://docs.luxonis.com/cloud/hubai/model-registry/hubai-sdk/hubai-sdk-api-reference/services/instances#list_instances),
so they remain empty in this RVC4 example. Their comments show concrete Hailo
values. For RVC4, choose an SNPE version compatible with your device's runtime;
see [SNPE compatibility](https://docs.luxonis.com/software-v3/ai-inference/conversion/troubleshooting).

`platform` is a scalar, not a list. It may be omitted when a `NeuralNetwork.build()`
call fills it from its device. For a direct `getModelFromZoo()` call, specify it:
`NNModelDescription.check()` requires both a model and a platform.
It only checks that these two strings are nonempty; it does not validate the
optional settings or the availability of a matching export.
`globalMetadataEntryName` is derived from the input filename; it is not a YAML key.

From the repository root, read the example without downloading a model:

```python
import depthai as dai

description = dai.NNModelDescription.fromYamlFile(
    "./examples/models/nn_model_description.yaml"
)
assert description.check()
print(description)
```

```cpp
#include <depthai/modelzoo/Zoo.hpp>
#include <iostream>

int main() {
    const auto description = dai::NNModelDescription::fromYamlFile(
        "./examples/models/nn_model_description.yaml");
    std::cout << description << '\n';
    return description.check() ? 0 : 1;
}
```

The optional second argument to `fromYamlFile(modelName, modelsPath)` supplies a
model-description directory. With no directory argument, bare names use
`DEPTHAI_ZOO_MODELS_PATH`; a path beginning with `./`, `../`, or an absolute path
is used directly. `saveToYamlFile()` writes the supported keys, omitting empty
optional fields.
