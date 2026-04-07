#!/usr/bin/env python3
"""PyQt GUI for GPUStereo — same pipeline as gpu_stereo.py with live parameter tuning via inputConfig.

Requires: PyQt6, depthai (with GPUStereo), opencv-python, numpy.

  pip install PyQt6

Run:
  python3 gpu_stereo_gui.py --device 10.11.0.51 --resolution 1280x800

Temporal filtering (when 0 < temporalAlpha < 1) uses RVC4-style logic aligned with StereoDepth
post-processing: EMA when stable, edge test via temporalDelta, and invalid-fill persistency over an
8-frame history. State is kept in OpenCL on the device (no host previous-frame disparity).

Host buffers / OpenCL platform index are fixed by the device node (see GPUStereo docs).
"""

from __future__ import annotations

import argparse
import sys
import threading
import time

import cv2
import depthai as dai
import numpy as np

try:
    from PyQt6.QtCore import Qt, QThread, pyqtSignal
    from PyQt6.QtGui import QCloseEvent, QImage, QKeySequence, QMouseEvent, QPixmap, QShortcut
    from PyQt6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QSpinBox,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError as e:
    print("PyQt6 is required: pip install PyQt6", file=sys.stderr)
    raise SystemExit(1) from e


def colorize_disparity_u16(frame_u16: np.ndarray, min_value: int, max_value: int) -> np.ndarray:
    valid = frame_u16[frame_u16 > 0]
    if valid.size == 0:
        return np.zeros((*frame_u16.shape, 3), dtype=np.uint8)
    if max_value <= min_value:
        max_value = float(np.max(valid)) + 1.0
    norm = np.clip((frame_u16.astype(np.float32) - min_value) / (max_value - min_value), 0.0, 1.0)
    return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)


def numpy_bgr_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
    return QPixmap.fromImage(qimg.copy())


class AspectRatioLabel(QLabel):
    def __init__(self) -> None:
        super().__init__()
        self._src: QPixmap | None = None

    def setSourcePixmap(self, pixmap: QPixmap) -> None:
        self._src = pixmap
        self.setText("")
        self._apply_scaled()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._apply_scaled()

    def _apply_scaled(self) -> None:
        if self._src is None or self._src.isNull():
            return
        if self.width() <= 1 or self.height() <= 1:
            return
        scaled = self._src.scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        super().setPixmap(scaled)


class HoverImageLabel(AspectRatioLabel):
    pixel_hovered = pyqtSignal(int, int)
    hover_cleared = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self.setMouseTracking(True)

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        super().mouseMoveEvent(event)
        if self._src is None or self._src.isNull():
            self.hover_cleared.emit()
            return
        sw, sh = self.width(), self.height()
        if sw <= 1 or sh <= 1:
            self.hover_cleared.emit()
            return
        scaled = self._src.scaled(
            sw,
            sh,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        pw, ph = scaled.width(), scaled.height()
        ox = (sw - pw) // 2
        oy = (sh - ph) // 2
        mx = int(event.position().x())
        my = int(event.position().y())
        lx, ly = mx - ox, my - oy
        if lx < 0 or ly < 0 or lx >= pw or ly >= ph:
            self.hover_cleared.emit()
            return
        iw, ih = self._src.width(), self._src.height()
        ix = min(iw - 1, max(0, int(lx * iw / pw)))
        iy = min(ih - 1, max(0, int(ly * ih / ph)))
        self.pixel_hovered.emit(ix, iy)

    def leaveEvent(self, event) -> None:
        self.hover_cleared.emit()
        super().leaveEvent(event)


_GPUS_CONFIG_FIELDS = (
    "maxDisparity",
    "numPyramidLevels",
    "downsampleMethod",
    "prefilterMethod",
    "blockMatchRadius",
    "adaptiveSupportRangeSigma",
    "prefilterBilateralSigmaSpatial",
    "prefilterBilateralSigmaRange",
    "refinementRadius",
    "refinementRadiusFull",
    "subpixelBits",
    "lrCheck",
    "lrCheckFast",
    "medianSize",
    "minDisp",
    "confidenceThreshold",
    "useCostVolume",
    "costVolumeAggregation",
    "boxAggregationRadius",
    "bilateralSpatialSigma",
    "bilateralRangeSigma",
    "bilateralAggregationRadius",
    "costMethod",
    "pathAggregation",
    "sgmP1",
    "sgmP2",
    "sgmAdaptiveP2",
    "useFp16",
    "useQcomAcceleratedOps",
    "secondPeakThreshold",
    "secondPeakMinDisparityGap",
    "censusRadiusX",
    "censusRadiusY",
    "speckleMaxSize",
    "speckleMaxDiff",
    "textureFilterRadius",
    "textureThreshold",
    "featureMaskEdgeThresh",
    "featureMaskCornerThresh",
    "featureMaskMorphRadius",
    "edgeAwareRadius",
    "edgeAwareEps",
    "holeFillRadius",
    "holeFillSigmaSpatial",
    "holeFillSigmaRange",
    "temporalAlpha",
    "temporalDelta",
    "temporalPersistencyMode",
    "regionRefine",
    "regionRefineCellSize",
    "regionRefinePlaneResidualThresh",
)


def _gpustereo_config_assign(dst: dai.GPUStereoConfig, src: dai.GPUStereoConfig) -> None:
    for name in _GPUS_CONFIG_FIELDS:
        setattr(dst, name, getattr(src, name))
    dst.algorithmControl.depthUnit = src.algorithmControl.depthUnit
    dst.algorithmControl.customDepthUnitMultiplier = src.algorithmControl.customDepthUnitMultiplier


def gpu_stereo_pipeline_defaults() -> dai.GPUStereoConfig:
    c = dai.GPUStereoConfig()
    c.algorithmControl.depthUnit = dai.DepthUnit.MILLIMETER
    c.algorithmControl.customDepthUnitMultiplier = 1000.0
    c.temporalAlpha = 0.0
    c.temporalDelta = 0
    c.temporalPersistencyMode = 3
    return c


class PipelineThread(QThread):
    frame_ready = pyqtSignal(object)

    def __init__(
        self,
        pipeline: dai.Pipeline,
        disp_q: dai.OutputQueue,
        depth_q: dai.OutputQueue,
        rect_left_q: dai.OutputQueue,
        rect_right_q: dai.OutputQueue,
        min_d: int,
        max_d: int,
    ) -> None:
        super().__init__()
        self.pipeline = pipeline
        self.disp_q = disp_q
        self.depth_q = depth_q
        self.rect_left_q = rect_left_q
        self.rect_right_q = rect_right_q
        self.min_d = min_d
        self.max_d = max_d
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()
        try:
            self.pipeline.stop()
        except Exception:
            pass

    def run(self) -> None:
        with self.pipeline:
            self.pipeline.start()
            while not self._stop.is_set() and self.pipeline.isRunning():
                try:
                    disp = self.disp_q.get(timeout=0.2)
                    depth_if = self.depth_q.get(timeout=0.2)
                    rl_fr = self.rect_left_q.get(timeout=0.2)
                    rr_fr = self.rect_right_q.get(timeout=0.2)
                except BaseException:
                    continue
                if disp is None or depth_if is None or rl_fr is None or rr_fr is None:
                    continue
                assert isinstance(disp, dai.ImgFrame)
                assert isinstance(depth_if, dai.ImgFrame)
                assert isinstance(rl_fr, dai.ImgFrame)
                assert isinstance(rr_fr, dai.ImgFrame)
                d_u16 = np.asarray(disp.getFrame())
                depth_u16 = np.asarray(depth_if.getFrame())
                rl_gray = np.asarray(rl_fr.getFrame())
                rr_gray = np.asarray(rr_fr.getFrame())
                disp_rgb = colorize_disparity_u16(d_u16, self.min_d, self.max_d)
                self.frame_ready.emit(
                    (
                        d_u16.copy(),
                        depth_u16.copy(),
                        rl_gray.copy(),
                        rr_gray.copy(),
                        disp_rgb.copy(),
                    )
                )


class StereoConfigPanel(QWidget):
    def __init__(self, initial: dai.GPUStereoConfig) -> None:
        super().__init__()
        self.G = dai.GPUStereoConfig
        root = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        scroll.setWidget(inner)
        form_root = QVBoxLayout(inner)

        def gb(title: str) -> tuple[QGroupBox, QFormLayout]:
            g = QGroupBox(title)
            fl = QFormLayout(g)
            form_root.addWidget(g)
            return g, fl

        g, fl = gb("Pyramid")
        self.max_disp = QComboBox()
        for v in (16, 32, 64, 128, 256, 512):
            self.max_disp.addItem(str(v), v)
        fl.addRow("Max disparity", self.max_disp)
        self.levels = QComboBox()
        for v in range(1, 7):
            self.levels.addItem(str(v), v)
        fl.addRow("Pyramid levels", self.levels)
        self.downsample = QComboBox()
        for label, val in (
            ("box", self.G.DownsampleMethod.BOX_FILTER),
            ("bilinear", self.G.DownsampleMethod.BILINEAR),
            ("gauss3", self.G.DownsampleMethod.GAUSSIAN_3x3),
            ("gauss5", self.G.DownsampleMethod.GAUSSIAN_5x5),
            ("nearest", self.G.DownsampleMethod.NEAREST),
        ):
            self.downsample.addItem(label, val)
        fl.addRow("Downsample", self.downsample)

        g, fl = gb("Prefilter")
        self.prefilter = QComboBox()
        for label, val in (
            ("none", self.G.PrefilterMethod.NONE),
            ("sobel", self.G.PrefilterMethod.SOBEL_X),
            ("box_mean", self.G.PrefilterMethod.BOX_MEAN_SUBTRACT),
            ("gaussian3", self.G.PrefilterMethod.GAUSSIAN_3x3),
            ("bilateral", self.G.PrefilterMethod.BILATERAL_PREFILTER),
        ):
            self.prefilter.addItem(label, val)
        fl.addRow("Prefilter", self.prefilter)

        g, fl = gb("Matching")
        self.subpixel = QComboBox()
        for v in (0, 4, 5):
            self.subpixel.addItem(str(v), v)
        fl.addRow("Subpixel bits", self.subpixel)
        self.cost_method = QComboBox()
        for val in (
            self.G.CostMethod.ZNCC,
            self.G.CostMethod.SAD,
            self.G.CostMethod.CENSUS,
            self.G.CostMethod.GRADIENT,
            self.G.CostMethod.RANK,
        ):
            self.cost_method.addItem(val.name, val)
        fl.addRow("Cost method", self.cost_method)
        self.block_radius = QSpinBox()
        self.block_radius.setRange(1, 7)
        fl.addRow("Block match radius", self.block_radius)
        self.adaptive_sigma = QDoubleSpinBox()
        self.adaptive_sigma.setRange(0.0, 1.0)
        self.adaptive_sigma.setDecimals(3)
        self.adaptive_sigma.setSingleStep(0.01)
        fl.addRow("Adaptive σ (SAD)", self.adaptive_sigma)
        self.census_rx = QSpinBox()
        self.census_rx.setRange(1, 2)
        self.census_ry = QSpinBox()
        self.census_ry.setRange(1, 2)
        fl.addRow("Census radius X", self.census_rx)
        fl.addRow("Census radius Y", self.census_ry)
        self.second_peak = QDoubleSpinBox()
        self.second_peak.setRange(0.0, 0.5)
        self.second_peak.setDecimals(3)
        self.second_peak.setSingleStep(0.01)
        fl.addRow("Second-peak threshold", self.second_peak)

        g, fl = gb("Cost volume")
        self.use_cost_volume = QCheckBox()
        fl.addRow("Use cost volume", self.use_cost_volume)
        self.second_peak_gap = QSpinBox()
        self.second_peak_gap.setRange(0, 32)
        fl.addRow("Second-peak min disp gap", self.second_peak_gap)
        self.cost_agg = QComboBox()
        for val in (
            self.G.CostVolumeAggregation.BOX,
            self.G.CostVolumeAggregation.WTA,
            self.G.CostVolumeAggregation.BILATERAL,
        ):
            self.cost_agg.addItem(val.name, val)
        fl.addRow("Aggregation", self.cost_agg)
        self.box_agg_r = QSpinBox()
        self.box_agg_r.setRange(1, 10)
        fl.addRow("Box aggregation radius", self.box_agg_r)
        self.bilat_agg_r = QSpinBox()
        self.bilat_agg_r.setRange(1, 10)
        fl.addRow("Bilateral aggregation radius", self.bilat_agg_r)
        self.bilat_sigma_s = QDoubleSpinBox()
        self.bilat_sigma_s.setRange(0.1, 10.0)
        self.bilat_sigma_s.setDecimals(2)
        fl.addRow("Bilateral spatial σ", self.bilat_sigma_s)
        self.bilat_sigma_r = QDoubleSpinBox()
        self.bilat_sigma_r.setRange(0.01, 0.5)
        self.bilat_sigma_r.setDecimals(3)
        fl.addRow("Bilateral range σ", self.bilat_sigma_r)
        self.path_agg = QComboBox()
        for val in (
            self.G.PathAggregation.NONE,
            self.G.PathAggregation.SGM_2,
            self.G.PathAggregation.SGM_4,
            self.G.PathAggregation.SGM_8,
            self.G.PathAggregation.SGM_MGM_INPLACE,
        ):
            self.path_agg.addItem(val.name, val)
        fl.addRow("Path aggregation", self.path_agg)
        self.sgm_p1 = QDoubleSpinBox()
        self.sgm_p1.setRange(0.0, 10.0)
        self.sgm_p1.setDecimals(2)
        fl.addRow("SGM P1", self.sgm_p1)
        self.sgm_p2 = QDoubleSpinBox()
        self.sgm_p2.setRange(0.0, 10.0)
        self.sgm_p2.setDecimals(2)
        fl.addRow("SGM P2", self.sgm_p2)
        self.sgm_adaptive_p2 = QCheckBox()
        fl.addRow("SGM adaptive P2", self.sgm_adaptive_p2)
        self.use_fp16 = QCheckBox()
        fl.addRow("Use FP16 (cost volume)", self.use_fp16)

        g, fl = gb("Post-processing")
        self.lr_check = QCheckBox()
        fl.addRow("LR check", self.lr_check)
        self.lr_check_fast = QCheckBox()
        fl.addRow("LR check fast", self.lr_check_fast)
        self.median = QComboBox()
        for v in (0, 3, 5):
            self.median.addItem(str(v), v)
        fl.addRow("Median", self.median)
        self.conf_thr = QSpinBox()
        self.conf_thr.setRange(0, 255)
        fl.addRow("Confidence threshold", self.conf_thr)
        self.tex_r = QSpinBox()
        self.tex_r.setRange(0, 8)
        fl.addRow("Texture radius", self.tex_r)
        self.tex_t = QSpinBox()
        self.tex_t.setRange(0, 255)
        fl.addRow("Texture threshold", self.tex_t)

        g, fl = gb("Feature mask")
        self.fm_edge = QSpinBox()
        self.fm_edge.setRange(0, 255)
        fl.addRow("Edge threshold", self.fm_edge)
        self.fm_corner = QDoubleSpinBox()
        self.fm_corner.setRange(0.0, 10.0)
        self.fm_corner.setDecimals(2)
        fl.addRow("Corner threshold", self.fm_corner)
        self.fm_morph = QSpinBox()
        self.fm_morph.setRange(0, 5)
        fl.addRow("Morph radius", self.fm_morph)

        g, fl = gb("Speckle")
        self.speckle_size = QSpinBox()
        self.speckle_size.setRange(0, 5000)
        fl.addRow("Speckle max size (0=off)", self.speckle_size)
        self.speckle_diff = QSpinBox()
        self.speckle_diff.setRange(1, 65535)
        fl.addRow("Speckle max diff", self.speckle_diff)

        g, fl = gb("Edge-aware / Hole fill / Region")
        self.ea_r = QSpinBox()
        self.ea_r.setRange(0, 8)
        fl.addRow("Edge-aware radius", self.ea_r)
        self.ea_eps = QDoubleSpinBox()
        self.ea_eps.setRange(0.001, 0.5)
        self.ea_eps.setDecimals(3)
        fl.addRow("Edge-aware ε", self.ea_eps)
        self.hole_r = QSpinBox()
        self.hole_r.setRange(0, 8)
        fl.addRow("Hole fill radius", self.hole_r)
        self.hole_ss = QDoubleSpinBox()
        self.hole_ss.setRange(0.1, 10.0)
        fl.addRow("Hole spatial σ", self.hole_ss)
        self.hole_sr = QDoubleSpinBox()
        self.hole_sr.setRange(0.01, 0.5)
        self.hole_sr.setDecimals(3)
        fl.addRow("Hole range σ", self.hole_sr)
        self.region_refine = QCheckBox()
        fl.addRow("Region refine", self.region_refine)
        self.region_cell = QSpinBox()
        self.region_cell.setRange(1, 64)
        fl.addRow("Region cell size", self.region_cell)
        self.region_res = QDoubleSpinBox()
        self.region_res.setRange(0.5, 20.0)
        self.region_res.setDecimals(2)
        fl.addRow("Region residual thresh", self.region_res)

        g, fl = gb("Temporal")
        self.temporal_alpha = QDoubleSpinBox()
        self.temporal_alpha.setRange(0.0, 1.0)
        self.temporal_alpha.setDecimals(2)
        self.temporal_alpha.setToolTip(
            "EMA blend weight when current and previous disparity agree within δ. "
            "Use 0 to disable temporal filtering, or strictly between 0 and 1 to enable."
        )
        fl.addRow("Temporal α (0=off)", self.temporal_alpha)
        self.temporal_delta = QSpinBox()
        self.temporal_delta.setRange(0, 65535)
        self.temporal_delta.setToolTip(
            "Max disparity difference (same units as output disp) for blending; above this, copy current. "
            "0 = automatic: 3 × 2^subpixelBits (three integer disparity steps in subpixel units)."
        )
        fl.addRow("Temporal δ disparity (0=auto)", self.temporal_delta)
        self.temporal_persistency = QComboBox()
        for val, label in (
            (0, "0 — persistency off"),
            (1, "1 — valid 8 of last 8"),
            (2, "2 — valid 2 in last 3"),
            (3, "3 — valid 2 in last 4 (default)"),
            (4, "4 — valid 2 of last 8"),
            (5, "5 — valid 1 in last 2"),
            (6, "6 — valid 1 in last 5"),
            (7, "7 — valid 1 in last 8"),
            (8, "8 — persistency indefinitely"),
        ):
            self.temporal_persistency.addItem(label, val)
        self.temporal_persistency.setToolTip(
            "When the current disparity is invalid (0), whether to reuse a recent value from history "
            "(same semantics as StereoDepth post-processing temporal filter persistency)."
        )
        fl.addRow("Temporal persistency", self.temporal_persistency)
        g, fl = gb("Device")
        self.qcom_ops = QCheckBox()
        fl.addRow("Qualcomm accelerated ops", self.qcom_ops)

        root.addWidget(scroll)
        self._load_from_config(initial)

    def _set_combo_by_data(self, combo: QComboBox, value) -> None:
        for i in range(combo.count()):
            if combo.itemData(i) == value:
                combo.setCurrentIndex(i)
                return
        combo.setCurrentIndex(0)

    def _load_from_config(self, cfg: dai.GPUStereoConfig) -> None:
        self._set_combo_by_data(self.max_disp, cfg.maxDisparity)
        self._set_combo_by_data(self.levels, cfg.numPyramidLevels)
        self._set_combo_by_data(self.downsample, cfg.downsampleMethod)
        self._set_combo_by_data(self.prefilter, cfg.prefilterMethod)
        self._set_combo_by_data(self.subpixel, cfg.subpixelBits)
        self._set_combo_by_data(self.cost_method, cfg.costMethod)
        self.block_radius.setValue(cfg.blockMatchRadius)
        self.adaptive_sigma.setValue(float(cfg.adaptiveSupportRangeSigma))
        self.census_rx.setValue(cfg.censusRadiusX)
        self.census_ry.setValue(cfg.censusRadiusY)
        self.second_peak.setValue(float(cfg.secondPeakThreshold))
        self.use_cost_volume.setChecked(cfg.useCostVolume)
        self.second_peak_gap.setValue(cfg.secondPeakMinDisparityGap)
        self._set_combo_by_data(self.cost_agg, cfg.costVolumeAggregation)
        self.box_agg_r.setValue(cfg.boxAggregationRadius)
        self.bilat_agg_r.setValue(cfg.bilateralAggregationRadius)
        self.bilat_sigma_s.setValue(float(cfg.bilateralSpatialSigma))
        self.bilat_sigma_r.setValue(float(cfg.bilateralRangeSigma))
        self._set_combo_by_data(self.path_agg, cfg.pathAggregation)
        self.sgm_p1.setValue(float(cfg.sgmP1))
        self.sgm_p2.setValue(float(cfg.sgmP2))
        self.sgm_adaptive_p2.setChecked(cfg.sgmAdaptiveP2)
        self.use_fp16.setChecked(cfg.useFp16)
        self.lr_check.setChecked(cfg.lrCheck)
        self.lr_check_fast.setChecked(cfg.lrCheckFast)
        self._set_combo_by_data(self.median, cfg.medianSize)
        self.conf_thr.setValue(cfg.confidenceThreshold)
        self.tex_r.setValue(cfg.textureFilterRadius)
        self.tex_t.setValue(int(cfg.textureThreshold))
        self.fm_edge.setValue(int(cfg.featureMaskEdgeThresh))
        self.fm_corner.setValue(float(cfg.featureMaskCornerThresh))
        self.fm_morph.setValue(cfg.featureMaskMorphRadius)
        self.speckle_size.setValue(cfg.speckleMaxSize)
        self.speckle_diff.setValue(cfg.speckleMaxDiff)
        self.ea_r.setValue(cfg.edgeAwareRadius)
        self.ea_eps.setValue(float(cfg.edgeAwareEps))
        self.hole_r.setValue(cfg.holeFillRadius)
        self.hole_ss.setValue(float(cfg.holeFillSigmaSpatial))
        self.hole_sr.setValue(float(cfg.holeFillSigmaRange))
        self.region_refine.setChecked(cfg.regionRefine)
        self.region_cell.setValue(cfg.regionRefineCellSize)
        self.region_res.setValue(float(cfg.regionRefinePlaneResidualThresh))
        self.temporal_alpha.setValue(float(cfg.temporalAlpha))
        self.temporal_delta.setValue(int(getattr(cfg, "temporalDelta", 0)))
        self._set_combo_by_data(self.temporal_persistency, int(getattr(cfg, "temporalPersistencyMode", 3)))
        self.qcom_ops.setChecked(cfg.useQcomAcceleratedOps)

    def to_config(self, base: dai.GPUStereoConfig) -> dai.GPUStereoConfig:
        c = dai.GPUStereoConfig()
        c.algorithmControl.depthUnit = base.algorithmControl.depthUnit
        c.algorithmControl.customDepthUnitMultiplier = base.algorithmControl.customDepthUnitMultiplier
        c.maxDisparity = self.max_disp.currentData()
        c.numPyramidLevels = self.levels.currentData()
        c.downsampleMethod = self.downsample.currentData()
        c.prefilterMethod = self.prefilter.currentData()
        c.subpixelBits = self.subpixel.currentData()
        c.costMethod = self.cost_method.currentData()
        c.blockMatchRadius = self.block_radius.value()
        c.adaptiveSupportRangeSigma = self.adaptive_sigma.value()
        c.censusRadiusX = self.census_rx.value()
        c.censusRadiusY = self.census_ry.value()
        c.secondPeakThreshold = self.second_peak.value()
        c.useCostVolume = self.use_cost_volume.isChecked()
        c.secondPeakMinDisparityGap = self.second_peak_gap.value()
        c.costVolumeAggregation = self.cost_agg.currentData()
        c.boxAggregationRadius = self.box_agg_r.value()
        c.bilateralAggregationRadius = self.bilat_agg_r.value()
        c.bilateralSpatialSigma = self.bilat_sigma_s.value()
        c.bilateralRangeSigma = self.bilat_sigma_r.value()
        c.pathAggregation = self.path_agg.currentData()
        c.sgmP1 = self.sgm_p1.value()
        c.sgmP2 = self.sgm_p2.value()
        c.sgmAdaptiveP2 = self.sgm_adaptive_p2.isChecked()
        c.useFp16 = self.use_fp16.isChecked()
        c.lrCheck = self.lr_check.isChecked()
        c.lrCheckFast = self.lr_check_fast.isChecked()
        c.medianSize = self.median.currentData()
        c.confidenceThreshold = self.conf_thr.value()
        c.textureFilterRadius = self.tex_r.value()
        c.textureThreshold = float(self.tex_t.value())
        c.featureMaskEdgeThresh = float(self.fm_edge.value())
        c.featureMaskCornerThresh = self.fm_corner.value()
        c.featureMaskMorphRadius = self.fm_morph.value()
        c.speckleMaxSize = self.speckle_size.value()
        c.speckleMaxDiff = self.speckle_diff.value()
        c.edgeAwareRadius = self.ea_r.value()
        c.edgeAwareEps = self.ea_eps.value()
        c.holeFillRadius = self.hole_r.value()
        c.holeFillSigmaSpatial = self.hole_ss.value()
        c.holeFillSigmaRange = self.hole_sr.value()
        c.regionRefine = self.region_refine.isChecked()
        c.regionRefineCellSize = self.region_cell.value()
        c.regionRefinePlaneResidualThresh = self.region_res.value()
        c.temporalAlpha = self.temporal_alpha.value()
        c.temporalDelta = self.temporal_delta.value()
        c.temporalPersistencyMode = int(self.temporal_persistency.currentData())
        c.useQcomAcceleratedOps = self.qcom_ops.isChecked()
        c.minDisp = base.minDisp
        c.refinementRadius = base.refinementRadius
        c.refinementRadiusFull = base.refinementRadiusFull
        c.prefilterBilateralSigmaSpatial = base.prefilterBilateralSigmaSpatial
        c.prefilterBilateralSigmaRange = base.prefilterBilateralSigmaRange
        return c


class MainWindow(QMainWindow):
    def __init__(
        self,
        pipeline: dai.Pipeline,
        gpu: dai.node.GPUStereo,
        disp_q: dai.OutputQueue,
        depth_q: dai.OutputQueue,
        rect_left_q: dai.OutputQueue,
        rect_right_q: dai.OutputQueue,
        cfg_q,
        left_cam_ctrl_q,
        right_cam_ctrl_q,
        initial_cfg: dai.GPUStereoConfig,
    ) -> None:
        super().__init__()
        self.setWindowTitle("GPUStereo — controls")
        self._pipeline = pipeline
        self._gpu = gpu
        self._disp_q = disp_q
        self._cfg_q = cfg_q
        self._left_cam_ctrl_q = left_cam_ctrl_q
        self._right_cam_ctrl_q = right_cam_ctrl_q
        self._base_algo = initial_cfg

        self._min_d = 0
        self._max_d = initial_cfg.maxDisparity * (1 << int(initial_cfg.subpixelBits))

        self._last_bundle: tuple | None = None

        self._view_mode = QComboBox()
        self._view_mode.addItems(["Disparity", "Rectified left", "Rectified right"])
        self._view_mode.setCurrentIndex(0)
        self._view_mode.currentIndexChanged.connect(lambda _i: self._refresh_left_pane())

        self._disp_label = HoverImageLabel()
        self._disp_label.setMinimumSize(320, 240)
        self._disp_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._disp_label.setText("Starting…")
        self._disp_label.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self._disp_label.pixel_hovered.connect(self._on_pixel_hover)
        self._disp_label.hover_cleared.connect(self._on_hover_clear)

        self._hover_dist = QLabel("")
        self._hover_dist.setAlignment(Qt.AlignmentFlag.AlignLeft)

        left_pane = QWidget()
        left_lay = QVBoxLayout(left_pane)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.addWidget(self._view_mode)
        left_lay.addWidget(self._disp_label, stretch=1)
        left_lay.addWidget(self._hover_dist)

        cam_outer = QGroupBox("Cameras — CAM_B + CAM_C")
        cam_fl = QFormLayout(cam_outer)
        self._cam_exp_us = QSpinBox()
        self._cam_exp_us.setRange(1, 200000)
        self._cam_exp_us.setValue(10000)
        self._cam_exp_us.setSuffix(" µs")
        self._cam_iso = QSpinBox()
        self._cam_iso.setRange(100, 3200)
        self._cam_iso.setValue(400)
        self._cam_sharpness = QSpinBox()
        self._cam_sharpness.setRange(0, 4)
        self._cam_sharpness.setValue(1)
        self._cam_luma_denoise = QSpinBox()
        self._cam_luma_denoise.setRange(0, 4)
        self._cam_luma_denoise.setValue(1)
        cam_fl.addRow("Exposure", self._cam_exp_us)
        cam_fl.addRow("ISO", self._cam_iso)
        cam_fl.addRow("Sharpness", self._cam_sharpness)
        cam_fl.addRow("Luma denoise", self._cam_luma_denoise)
        btn_cam = QPushButton("Apply to both cameras")
        btn_cam.clicked.connect(self._on_apply_cam_exposure)
        cam_fl.addRow(btn_cam)

        self._panel = StereoConfigPanel(initial_cfg)
        self._apply_btn = QPushButton("Apply config to device")
        self._apply_btn.clicked.connect(self._on_apply)
        self._status = QLabel("")
        self._status.setWordWrap(True)
        bottom_bar = QHBoxLayout()
        bottom_bar.addWidget(self._apply_btn)
        bottom_bar.addWidget(self._status, stretch=1)

        right_pane = QWidget()
        right_lay = QVBoxLayout(right_pane)
        right_lay.setContentsMargins(0, 0, 0, 0)
        right_lay.addWidget(cam_outer)
        right_lay.addWidget(self._panel, stretch=1)
        right_lay.addLayout(bottom_bar)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_pane)
        splitter.addWidget(right_pane)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)

        self.setCentralWidget(splitter)

        self._fps_label = QLabel("FPS: —")
        self.statusBar().addPermanentWidget(self._fps_label)
        self._fps_t0 = time.monotonic()
        self._fps_frames = 0

        for seq in ("Ctrl+Q", "Ctrl+W"):
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ShortcutContext.ApplicationShortcut)
            sc.activated.connect(self.close)

        self._worker = PipelineThread(
            pipeline,
            disp_q,
            depth_q,
            rect_left_q,
            rect_right_q,
            self._min_d,
            self._max_d,
        )
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.start()

    def _bgr_for_view(
        self,
        disp_rgb: np.ndarray,
        rl_gray: np.ndarray,
        rr_gray: np.ndarray,
    ) -> np.ndarray:
        idx = self._view_mode.currentIndex()
        if idx == 1:
            return cv2.cvtColor(rl_gray, cv2.COLOR_GRAY2BGR)
        if idx == 2:
            return cv2.cvtColor(rr_gray, cv2.COLOR_GRAY2BGR)
        return disp_rgb

    def _refresh_left_pane(self) -> None:
        if self._last_bundle is None:
            return
        _, _, rl, rr, disp_rgb = self._last_bundle
        bgr = self._bgr_for_view(disp_rgb, rl, rr)
        self._disp_label.setSourcePixmap(numpy_bgr_to_qpixmap(bgr))

    def _on_pixel_hover(self, ix: int, iy: int) -> None:
        if self._last_bundle is None:
            self._hover_dist.setText("")
            return
        depth_u16 = self._last_bundle[1]
        if depth_u16.size == 0:
            self._hover_dist.setText("")
            return
        h, w = depth_u16.shape[:2]
        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            self._hover_dist.setText("")
            return
        d = int(depth_u16[iy, ix])
        if d <= 0:
            self._hover_dist.setText("Distance: —")
            return
        self._hover_dist.setText(f"Distance: {d / 1000.0:.3f} m")

    def _on_hover_clear(self) -> None:
        self._hover_dist.setText("")

    def _on_frame(self, bundle: tuple) -> None:
        self._last_bundle = bundle
        self._fps_frames += 1
        now = time.monotonic()
        dt = now - self._fps_t0
        if dt >= 0.5:
            self._fps_label.setText(f"FPS: {self._fps_frames / dt:.1f}")
            self._fps_t0 = now
            self._fps_frames = 0
        _, _, rl, rr, disp_rgb = bundle
        bgr = self._bgr_for_view(disp_rgb, rl, rr)
        self._disp_label.setSourcePixmap(numpy_bgr_to_qpixmap(bgr))

    def _on_apply(self) -> None:
        try:
            cfg = self._panel.to_config(self._base_algo)
            self._cfg_q.send(cfg)
            self._max_d = cfg.maxDisparity * (1 << cfg.subpixelBits)
            self._worker.max_d = self._max_d
            self._status.setText("Config sent (pipeline rebuild on device)")
        except Exception as e:
            QMessageBox.warning(self, "Apply failed", str(e))

    def _on_apply_cam_exposure(self) -> None:
        try:
            exp = self._cam_exp_us.value()
            iso = self._cam_iso.value()
            sharp = self._cam_sharpness.value()
            luma = self._cam_luma_denoise.value()
            for q in (self._left_cam_ctrl_q, self._right_cam_ctrl_q):
                ctrl = dai.CameraControl()
                ctrl.setManualExposure(exp, iso)
                ctrl.setSharpness(sharp)
                ctrl.setLumaDenoise(luma)
                q.send(ctrl)
            self._status.setText("Cameras (CAM_B, CAM_C): controls sent")
        except Exception as e:
            QMessageBox.warning(self, "Camera control failed", str(e))

    def closeEvent(self, event: QCloseEvent) -> None:
        self._worker.stop()
        self._worker.wait(5000)
        event.accept()


def main() -> None:
    if not hasattr(dai.node, "GPUStereo"):
        print(
            "dai.node.GPUStereo missing: install depthai built from this depthai-core tree.",
            file=sys.stderr,
        )
        sys.exit(1)

    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=str, default="10.11.0.51", help="Device IP")
    parser.add_argument("--resolution", type=str, default="1280x800", help="WxH")
    parser.add_argument(
        "--fps",
        type=float,
        default=60.0,
        metavar="N",
        help="Camera output FPS (default: 60)",
    )
    args = parser.parse_args()
    w, h = (int(x) for x in args.resolution.split("x"))

    device = dai.Device(args.device)
    device.setIrLaserDotProjectorIntensity(0.9)

    pipeline = dai.Pipeline(device)
    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    gpu = pipeline.create(dai.node.GPUStereo)
    mono_left.requestOutput((w, h), type=dai.ImgFrame.Type.GRAY8, fps=args.fps).link(gpu.left)
    mono_right.requestOutput((w, h), type=dai.ImgFrame.Type.GRAY8, fps=args.fps).link(gpu.right)
    gpu.setRectification(True)

    ic = gpu.initialConfig
    _gpustereo_config_assign(ic, gpu_stereo_pipeline_defaults())

    disp_q = gpu.disparity.createOutputQueue()
    depth_q = gpu.depth.createOutputQueue()
    rect_left_q = gpu.rectifiedLeft.createOutputQueue()
    rect_right_q = gpu.rectifiedRight.createOutputQueue()
    cfg_q = gpu.inputConfig.createInputQueue()
    left_cam_ctrl_q = mono_left.inputControl.createInputQueue()
    right_cam_ctrl_q = mono_right.inputControl.createInputQueue()

    app = QApplication(sys.argv)
    win = MainWindow(
        pipeline,
        gpu,
        disp_q,
        depth_q,
        rect_left_q,
        rect_right_q,
        cfg_q,
        left_cam_ctrl_q,
        right_cam_ctrl_q,
        ic,
    )
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
