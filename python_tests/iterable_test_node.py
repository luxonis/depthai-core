from pathlib import Path
import sys
sys.path.insert(0, '/home/jakub/Code/depthai-core/build/bindings/python/')
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "build" / "bindings" / "python"))

import depthai as dai


def make_frame(width: int = 20, height: int = 20) -> dai.ImgFrame:
    frame = dai.ImgFrame()
    frame.setType(dai.ImgFrame.Type.GRAY8)
    frame.setWidth(width)
    frame.setHeight(height)
    frame.setSize(width, height)
    frame.setStride(width)
    frame.setData(np.arange(width * height, dtype=np.uint8))
    return frame


def plot_batch(data: list[dai.ImgFrame | None]) -> None:
    fig, axes = plt.subplots(1, len(data), figsize=(2 * len(data), 2.5), squeeze=False)
    for index, (ax, img) in enumerate(zip(axes[0], data)):
        ax.set_title(str(index))
        ax.set_xticks([])
        ax.set_yticks([])
        if img is None:
            ax.text(0.5, 0.5, "None", ha="center", va="center", fontsize=12)
            ax.set_facecolor("#eeeeee")
            continue
        ax.imshow(img.getCvFrame(), cmap="gray", vmin=0, vmax=255)
    fig.tight_layout()
    plt.show()


def main() -> None:
    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        test_node = pipeline.create(dai.node.TestNode)
        gather = pipeline.create(dai.node.BatchAssembler)
        gather.input.setMaxSize(16)
        gather.input.setBlocking(True)
        test_node.output.link(gather.input)
        input_queue = test_node.input.createInputQueue()
        output_queue = gather.output.createOutputQueue()

        pipeline.start()
        input_queue.send(make_frame())

        data = output_queue.get()
        assert isinstance(data, dai.MessageBatch)
        data = data.getBuffers()
        print(f"size = {len(data)}")
        plot_batch(data)

        pipeline.stop()
        pipeline.wait()


if __name__ == "__main__":
    main()
