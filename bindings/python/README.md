# DepthAI Python Library

![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![PyPI](https://img.shields.io/pypi/v/depthai.svg)
![Downloads](https://img.shields.io/pepy/dt/depthai)


![Luxonis OAK camera](https://docs.luxonis.com/static/images/home-camera-animated.webp)

Python bindings for the [DepthAI C++ SDK](https://github.com/luxonis/depthai-core),
for building vision pipelines with Luxonis OAK cameras.

This README describes **DepthAI v3**. For existing v2 applications, use
`depthai<3` and the [v2 documentation](https://docs.luxonis.com/software/depthai/).
See the [v2 to v3 porting guide](https://github.com/luxonis/depthai-core/blob/main/V2V3PortinGuide.md)
when migrating.

## Installation

Install the released package from PyPI in a Python 3.9 or newer environment:

```sh
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade depthai
```

Verify the installed version and discover connected devices:

```sh
python3 -c "import depthai as dai; print(dai.__version__); print(dai.Device.getAllAvailableDevices())"
```



Run all camera streams of a connected OAK device. `cam_test` uses OpenCV,
so install it first:

```sh
python3 -m pip install opencv-python
depthai cam_test
```

## Next steps

- [DepthAI v3 documentation](https://docs.luxonis.com/software-v3/depthai/): guides, API reference and node descriptions
- [Python examples](https://github.com/luxonis/depthai-core/tree/main/examples/python): short examples for each node and feature
- [OAK examples](https://github.com/luxonis/oak-examples): complete applications that you can run and change
- [Model Zoo](https://models.luxonis.com/): ready-to-use neural network models for OAK cameras
- [depthai-nodes](https://github.com/luxonis/depthai-nodes): Python host nodes, such as parsers for Model Zoo models

## Need help?

Ask [Luxonis Chat](https://support.luxonis.com/) or the [forum](https://discuss.luxonis.com/),
or open a [GitHub issue](https://github.com/luxonis/depthai-core/issues).

## Luxonis MCP server

Connect your AI coding agent to the [Luxonis MCP server](https://mcp.luxonis.com) at
`https://mcp.luxonis.com/mcp`. It gives the agent current Luxonis documentation,
examples and model information. For setup steps, see the
[Luxonis Agent Toolkit](https://docs.luxonis.com/software-v3/agent-tools/agent-skills/).

**Happy building!**
