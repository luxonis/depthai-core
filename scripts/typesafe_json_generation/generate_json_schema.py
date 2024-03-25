from luxonis_ml.luxonis_ml.nn_archive.config import Config
import json

# try mode="serialization"
print(json.dumps(Config.model_json_schema(mode="validation"), sort_keys=True, indent=2))
