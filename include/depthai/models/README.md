## Modules

### Models
A wrapper around a model and its settings.

### ModelSerializer
Serializing and Deserializing models. This is useful for both sending models from and to device as well as storing models in memory

### ModelLoader
Everything related to model loading and management. Depends on Model and ModelSerializer.

### ModelZoo
A collection of functions for downloading a model from a remote repository. Depends on ModelLoader and ModelSerializer (for interpreting bytes as a model + metadata).

### ModelArchive
A helper class for working with NNArchives. Depends on Models, ModelSerializer and ModelLoader

### ModelOptimizer
A set of functions for optimizing model settings (e.g. number of shaves in case of a Superblob) for a specific node.


Ideally we can provide smart default settings for each model (i.e. without head) for each of the supported model types (Blob, SuperBlob,
Dlc, Onnx, Pytorch, etc.)

I think NNArchive is a neat thing and we should keep it, as playing around with parameters settable in a json file is much easier than fiddling 
with bits stored in a data structure containing the model and the settings themselves ([ModelBytes; SettingsBytes]);

