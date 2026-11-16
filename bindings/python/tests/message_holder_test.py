"""Python message state and virtual callbacks must survive native ownership."""

import gc
import weakref

import depthai as dai
import pytest
from depthai_pybind11_tests.message_holder import MessageOwner


def bound_message_types():
    found = {dai.ADatatype}
    pending = [dai.ADatatype]
    while pending:
        for cls in pending.pop().__subclasses__():
            if cls not in found and cls.__module__ in {"depthai", "depthai.beta"}:
                found.add(cls)
                pending.append(cls)
    return sorted(found, key=lambda cls: (cls.__module__, cls.__qualname__))


MESSAGE_TYPES = bound_message_types()
VISUALIZABLE_TYPES = [
    cls for cls in MESSAGE_TYPES
    if issubclass(cls, dai.Buffer) and not issubclass(cls, dai.TransformableBuffer)
]


def type_name(cls):
    return f"{cls.__module__}.{cls.__qualname__}"


@pytest.mark.parametrize("message_type", MESSAGE_TYPES, ids=type_name)
def test_queue_retains_python_message_until_final_release(message_type):
    class CustomMessage(message_type):
        def __init__(self):
            super().__init__()
            self.marker = object()

    message = CustomMessage()
    marker = message.marker
    ref = weakref.ref(message)
    queue = dai.MessageQueue(maxSize=1, blocking=False)
    queue.send(message)
    del message
    gc.collect()
    assert ref() is not None, "Native queue lost the Python instance"
    restored = queue.get()
    assert restored is ref()
    assert restored.marker is marker
    del restored, queue
    gc.collect()
    assert ref() is None, "Python message leaked after native ownership ended"


@pytest.mark.parametrize("message_type", VISUALIZABLE_TYPES, ids=type_name)
def test_native_owner_preserves_python_virtual_callback(message_type):
    calls = []

    class CustomMessage(message_type):
        def __init__(self):
            super().__init__()
            self.marker = "Python callback survived"

        def getVisualizationMessage(self):
            calls.append(self.marker)
            return None

    message = CustomMessage()
    ref = weakref.ref(message)
    owner = MessageOwner(message)
    del message
    gc.collect()
    # Invoke the C++ virtual without first recovering the Python wrapper.
    assert owner.visualize() is None
    assert calls == ["Python callback survived"]
    assert ref() is not None
    del owner
    gc.collect()
    assert ref() is None


def test_transformable_message_callback_survives_native_ownership():
    calls = []

    class CustomMessage(dai.TransformableBuffer):
        def __init__(self):
            super().__init__()
            self.marker = 73

        def transformTo(self, target):
            calls.append(self.marker)
            result = dai.TransformableBuffer()
            result.setTransformation(target)
            result.setSequenceNum(self.marker)
            return result

    message = CustomMessage()
    ref = weakref.ref(message)
    owner = MessageOwner(message)
    del message
    gc.collect()
    target = dai.ImgTransformation(320, 240)
    transformed = owner.transform(target)
    assert transformed.getSequenceNum() == 73
    assert transformed.getTransformation().isEqualTransformation(target)
    assert calls == [73]
    assert ref() is not None
    del owner
    gc.collect()
    assert ref() is None


def constructor_cases():
    cases = [
        (dai.Buffer, (32,)),
        (dai.ImgFrame, (32,)),
        (dai.NNData, (32,)),
        (dai.GateControl, (True, 5, 20)),
        (dai.CalibrationQuality, ("quality",)),
        (dai.CalibrationQuality, (dai.CalibrationQualityData(), "quality data")),
        (dai.DynamicCalibrationResult, ("calibration",)),
        (
            dai.DynamicCalibrationResult,
            (dai.DynamicCalibrationResultData(), "calibration data"),
        ),
        (
            dai.DynamicCalibrationControl,
            (dai.DynamicCalibrationControl.Commands.Calibrate(),),
        ),
        (dai.AutoCalibrationResult, (0.2, 0.4, True, dai.CalibrationHandler())),
    ]
    beta = getattr(dai, "beta", None)
    if beta is not None and hasattr(beta, "MultiDeviceCalibrationControl"):
        control = beta.MultiDeviceCalibrationControl
        cases.append((control, (control.Commands.Start(),)))
    if beta is not None and hasattr(beta, "MultiDeviceCalibrationResult"):
        cases.append((beta.MultiDeviceCalibrationResult, ("multi-device",)))
    return [
        pytest.param(cls, args, id=f"{type_name(cls)}-{index}")
        for index, (cls, args) in enumerate(cases)
    ]


@pytest.mark.parametrize("message_type,constructor_args", constructor_cases())
def test_constructor_overloads_preserve_python_instance(message_type, constructor_args):
    class CustomMessage(message_type):
        def __init__(self):
            super().__init__(*constructor_args)
            self.marker = "overloaded constructor"

    message = CustomMessage()
    ref = weakref.ref(message)
    owner = MessageOwner(message)
    del message
    gc.collect()
    assert ref() is not None
    restored = owner.get()
    assert restored is ref()
    assert restored.marker == "overloaded constructor"
    del restored, owner
    gc.collect()
    assert ref() is None
