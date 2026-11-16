"""Model-zoo I/O must allow a Python HTTP server thread to make progress."""

import os
import subprocess
import sys

import pytest


@pytest.mark.parametrize("operation", ["get", "download"])
@pytest.mark.parametrize("update_settings", [False, True])
def test_model_zoo_releases_gil(tmp_path, operation, update_settings):
    """Allow Python HTTP replies and concurrent settings updates during loading."""
    # Run in a child to isolate the global zoo endpoints and timeout a deadlock.
    script = r'''
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import depthai as dai

root = Path(sys.argv[1])
operation = sys.argv[2]
update_settings = sys.argv[3] == "True"
settings_updated = threading.Event()
stop_updates = threading.Event()
model_bytes = b"local model payload"

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if update_settings:
            settings_updated.clear()
            assert settings_updated.wait(timeout=3), "Settings update blocked during HTTP I/O"
        path = urlsplit(self.path).path
        if path == "/health":
            body = b"ok"
        elif path == "/download":
            body = json.dumps({
                "hash": "gil-regression",
                "download_links": [base_url + "/model.onnx"],
            }).encode()
        elif path == "/model.onnx":
            body = model_bytes
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass

server = HTTPServer(("127.0.0.1", 0), Handler)
base_url = "http://127.0.0.1:" + str(server.server_port)
thread = threading.Thread(target=server.serve_forever, daemon=True)
thread.start()
dai.modelzoo.setHealthEndpoint(base_url + "/health")
dai.modelzoo.setDownloadEndpoint(base_url + "/download")

def change_settings():
    generation = 0
    while not stop_updates.is_set():
        suffix = "?generation=" + str(generation % 2)
        dai.modelzoo.setHealthEndpoint(base_url + "/health" + suffix)
        dai.modelzoo.setDownloadEndpoint(base_url + "/download" + suffix)
        dai.modelzoo.setDefaultCachePath(root / ("unused-cache-" + str(generation % 2)))
        dai.modelzoo.setDefaultModelsPath(root / ("unused-models-" + str(generation % 2)))
        settings_updated.set()
        generation += 1
        stop_updates.wait(0.001)

updater = threading.Thread(target=change_settings, daemon=True)
if update_settings:
    updater.start()
description = dai.NNModelDescription(model="luxonis/gil-regression:model", platform="RVC4")
cache = root / "cache"
try:
    if operation == "get":
        path = dai.getModelFromZoo(description, cacheDirectory=cache)
        assert Path(path).read_bytes() == model_bytes
    else:
        models = root / "models"
        models.mkdir()
        description.saveToYamlFile(models / "model.yaml")
        assert dai.downloadModelsFromZoo(models, cacheDirectory=cache)
        assert [p.read_bytes() for p in cache.rglob("model.onnx")] == [model_bytes]
finally:
    stop_updates.set()
    if update_settings:
        updater.join(timeout=2)
        assert not updater.is_alive(), "Settings updater did not finish"
    server.shutdown()
    thread.join(timeout=2)
    server.server_close()
'''
    env = os.environ.copy()
    env.update({
        "DEPTHAI_ZOO_INTERNET_CHECK": "1",
        "DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT": "2000",
        "DEPTHAI_TELEMETRY": "0",
        "NO_PROXY": "127.0.0.1,localhost",
        "no_proxy": "127.0.0.1,localhost",
    })
    result = subprocess.run(
        [sys.executable, "-c", script, str(tmp_path), operation, str(update_settings)],
        capture_output=True,
        text=True,
        timeout=15,
        env=env,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
