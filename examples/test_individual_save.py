import argparse
import json
import os
import signal
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterable, Optional
from mcap.writer import Writer

import cv2
import depthai as dai
import numpy as np

def normalize_point2f(point: dai.Point2f, width: int, height: int) -> dai.Point2f:
    return dai.Point2f(x=float(point.x) / float(width), y=float(point.y) / float(height), normalized=True)


def april_tag_to_points(april_tag_message: dai.AprilTags, width: int, height: int) -> list[dai.Point2f]:
    data_points = []
    for tag in april_tag_message.aprilTags:
        data_points.append(normalize_point2f(tag.topLeft, width, height))
        data_points.append(normalize_point2f(tag.topRight, width, height))
        data_points.append(normalize_point2f(tag.bottomRight, width, height))
        data_points.append(normalize_point2f(tag.bottomLeft, width, height))
    return data_points


def draw_points_on_image(image: np.ndarray, points: list[dai.Point2f], depth_frame: list[float], color: tuple[int, int, int] = (0, 255, 0), name: str = "Image with Points") -> None:
    for point, depth in zip(points, depth_frame):
        # Skip invalid depth/point projections (NaN/Inf/non-positive depth).
        if not np.isfinite(depth) or float(depth) <= 0:
            continue
        if not np.isfinite(point.x) or not np.isfinite(point.y):
            continue

        if point.isNormalized():
            x = int(point.x * image.shape[1])
            y = int(point.y * image.shape[0])
        else:
            x = int(point.x)
            y = int(point.y)

        cv2.circle(image, (x, y), 5, color, -1)
        cv2.putText(image, f"{depth}mm", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color)

    cv2.imshow(name, image)
    
    

def get_depth_at_points(depth_frame: np.ndarray, points: list[dai.Point2f]) -> list[float]:
    depth_values = []
    for point in points:
        if point.isNormalized():
            x = int(point.x * depth_frame.shape[1])
            y = int(point.y * depth_frame.shape[0])
        else:
            x = int(point.x)
            y = int(point.y)
        
        if 0 <= x < depth_frame.shape[1] and 0 <= y < depth_frame.shape[0]:
            depth_values.append(depth_frame[y, x])
        else:
            depth_values.append(float('nan'))  # Out of bounds, append NaN
    return depth_values


def spatialProjection(point: dai.Point2f, depth: float, source: dai.ImgTransformation):
    intrinsics = source.getIntrinsicMatrix()
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    cx = intrinsics[0][2]
    cy = intrinsics[1][2]
    
    x_mm = (point.x - cx) * depth / fx
    y_mm = (point.y - cy) * depth / fy
    z_mm = depth
    return dai.Point3f(x=x_mm/10, y=y_mm/10, z=z_mm/10)

def project_3D_points_to(points: list[dai.Point2f], depths: list[float], reference: dai.ImgTransformation, to: dai.ImgTransformation) -> tuple[list[dai.Point2f], list[float]]:
    projected_points = []
    projected_depths = []
    w, h = reference.getSize()
    for point, depth in zip(points, depths):
        if not np.isfinite(depth) or float(depth) <= 0:
            continue

        if point.isNormalized():
            x = int(point.x * w)
            y = int(point.y * h)
        else:
            x = int(point.x)
            y = int(point.y)
        
        # Reconstruct the 3D point in the reference camera frame, then project to target.
        spatial_point: dai.Point3f = spatialProjection(dai.Point2f(x=x, y=y), depth, reference)
        projected_point: dai.Point2f = reference.project3DPointTo(to, spatial_point)
        if not np.isfinite(projected_point.x) or not np.isfinite(projected_point.y):
            continue

        projected_points.append(projected_point)
        projected_depths.append(float(depth))
    
    return projected_points, projected_depths

def main():
    fps = 30
    with dai.Pipeline() as pipeline:
        def signal_handler(sig, frame):
            print("Interrupted, stopping the pipeline")
            pipeline.stop()

        signal.signal(signal.SIGINT, signal_handler)
        
        
        syncNode = pipeline.create(dai.node.Sync)
        syncNode.setRunOnHost(False)
        syncNode.setSyncThreshold(timedelta(milliseconds=1000/ fps * 0.5))
        syncNode.setSyncAttempts(-1)


        camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        camera_a_output = camera_a.requestOutput(
            (1280, 600), type=dai.ImgFrame.Type.BGR888i, fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.STRETCH
        )
        
        april_tag_node = pipeline.create(dai.node.AprilTag)
        camera_a_output.link(april_tag_node.inputImage)

        camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        # camera_c_output = camera_c.requestFullResolutionOutput(fps=fps)
        camera_c_output = camera_c.requestOutput(
            (1280, 800),  fps=fps, enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP
        )
        camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        # camera_b_output = camera_b.requestFullResolutionOutput(fps=fps)
        camera_b_output = camera_b.requestOutput(
            (1280, 800),  fps=fps, enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP
        )

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
        # stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
        camera_b_output.link(stereo.left)
        camera_c_output.link(stereo.right)
        
        image_align = pipeline.create(dai.node.ImageAlign)
        stereo.depth.link(image_align.input)
        camera_a_output.link(image_align.inputAlignTo)
        
        camera_c_rectified_output = stereo.rectifiedLeft
        # camera_c_rectified_output = camera_c_output

        camera_c_rectified_output.link(syncNode.inputs["rectified_right"])
        # camera_a_output.link(syncNode.inputs["cam_a"])
        
        
        
        
        image_align.outputAligned.link(syncNode.inputs["depth_aligned"])
        april_tag_node.out.link(syncNode.inputs["april"])
        april_tag_node.passthroughInputImage.link(syncNode.inputs["cam_a"])
        
        syncedMessage = syncNode.out.createOutputQueue()


        pipeline.start()
        while pipeline.isRunning():
        
        
        
            fullMessage = syncedMessage.get()
            assert isinstance(fullMessage, dai.MessageGroup)
            assert fullMessage.isSynced(int(1000000000 / fps * 0.5))
            
            cam_a = fullMessage["cam_a"]
            cam_c = fullMessage["rectified_right"]
            depth_msg = fullMessage["depth_aligned"]
            april_msg = fullMessage["april"]
            
            assert isinstance(cam_a, dai.ImgFrame)
            assert isinstance(cam_c, dai.ImgFrame)
            assert isinstance(depth_msg, dai.ImgFrame)
            assert isinstance(april_msg, dai.AprilTags)
            
            cam_a_ts = cam_a.getTimestamp()
            cam_c_ts = cam_c.getTimestamp()
            depth_ts = depth_msg.getTimestamp()
            april_ts = april_msg.getTimestamp()
            
            # print(f"Timestamp diffs to april tag message: Camera A: {abs(cam_a_ts - april_ts)}, Camera C: {abs(cam_c_ts - april_ts)}, Depth: {abs(depth_ts - april_ts)}")

            points: list[dai.Point2f] = april_tag_to_points(april_msg, cam_a.getWidth(), cam_a.getHeight())
            depth_values = get_depth_at_points(depth_msg.getFrame(), points)
            draw_points_on_image(cam_a.getCvFrame(), points, depth_values, (0, 255, 0), "Reference Camera (CAM_A)")
            draw_points_on_image(depth_msg.getCvFrame(), points, depth_values, (0, 255, 0), "Aligned Depth")
            
            projected_points, projected_depths = project_3D_points_to(points, depth_values, cam_a.getTransformation(), cam_c.getTransformation())
            draw_points_on_image(cam_c.getCvFrame(), projected_points, projected_depths, (255, 0, 0), "Rectified Right Camera (CAM_C)")
            
            

            cv2.waitKey(1)


if __name__ == "__main__":
    main()
