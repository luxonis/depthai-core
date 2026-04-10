#!/usr/bin/env python3
"""PyQt GUI for GPUStereo — same pipeline as gpu_stereo.py with live parameter tuning via inputConfig.

Requires: PyQt6, depthai (with GPUStereo), opencv-python, numpy, PyYAML.

  pip install PyQt6 pyyaml

Run:
  python3 gpu_stereo_gui.py --device 10.11.0.51 --resolution 1280x800

Temporal filtering (when 0 < temporalAlpha < 1) uses RVC4-style logic aligned with StereoDepth
post-processing: EMA when stable, edge test via temporalDelta, and invalid-fill persistency over an
8-frame history. State is kept in OpenCL on the device (no host previous-frame disparity).

Host buffers / OpenCL platform index are fixed by the device node (see GPUStereo docs).

Pyramid debug: gray per level; per-level disparity uses Turbo colormap with values scaled by 2^L
to match full-res disparity sliders; block matching cost vs disparity on hover (device OpenCL) for
SAD (plain or adaptive), ZNCC (1−ρ), Census/Rank (Hamming), and GRADIENT (SAD on pyramid). Census/Rank
curves require pyramid matching (cost volume off). Range: maxDisparity>>level, capped to 768 samples.
"""

from __future__ import annotations

import argparse
import enum
import sys
import threading
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np

try:
    import yaml
except ImportError as e:
    print("PyYAML is required: pip install pyyaml", file=sys.stderr)
    raise SystemExit(1) from e

try:
    from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
    from PyQt6.QtGui import (
        QColor,
        QCursor,
        QCloseEvent,
        QFont,
        QImage,
        QKeySequence,
        QMouseEvent,
        QPainter,
        QPen,
        QPixmap,
        QShortcut,
    )
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
        QSlider,
        QSpinBox,
        QSplitter,
        QToolTip,
        QVBoxLayout,
        QWidget,
    )
except ImportError as e:
    print("PyQt6 is required: pip install PyQt6", file=sys.stderr)
    raise SystemExit(1) from e


def colorize_disparity_u16(
    frame_u16: np.ndarray,
    min_value: int,
    max_value: int,
    *,
    value_scale: float = 1.0,
    colormap: int = cv2.COLORMAP_JET,
) -> np.ndarray:
    f = frame_u16.astype(np.float32) * value_scale
    valid_m = frame_u16 > 0
    valid = f[valid_m]
    if valid.size == 0:
        return np.zeros((*frame_u16.shape, 3), dtype=np.uint8)
    if max_value <= min_value:
        max_value = float(np.max(valid)) + 1.0
    norm = np.clip((f - float(min_value)) / (float(max_value) - float(min_value)), 0.0, 1.0)
    bgr = cv2.applyColorMap((norm * 255).astype(np.uint8), colormap)
    return np.where(valid_m[..., np.newaxis], bgr, np.uint8(0))


def numpy_bgr_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
    return QPixmap.fromImage(qimg.copy())


def estimate_depth_scale_k(d_u16: np.ndarray, depth_u16: np.ndarray) -> float | None:
    valid = (d_u16 > 0) & (depth_u16 > 0)
    if not np.any(valid):
        return None
    prod = depth_u16[valid].astype(np.float64) * d_u16[valid].astype(np.float64)
    return float(np.median(prod))


def parse_match_cost_curve_frame(fr: dai.ImgFrame | None) -> tuple[int, int, np.ndarray | None]:
    if fr is None:
        return 0, -1, None
    raw = fr.getData()
    if len(raw) < 12:
        return 0, -1, None
    hdr = np.frombuffer(raw, dtype=np.int32, count=3)
    dmin, dmax, n = int(hdr[0]), int(hdr[1]), int(hdr[2])
    need = 12 + n * 4
    if n <= 0 or len(raw) < need:
        return dmin, dmax, None
    costs = np.frombuffer(raw, dtype=np.float32, count=n, offset=12).copy()
    return dmin, dmax, costs


class MatchCostPlotWidget(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._d_min: int | None = None
        self._d_max: int | None = None
        self._costs: np.ndarray | None = None
        self._message: str | None = None
        self._y_axis_short = "cost"
        self.setMinimumHeight(145)
        self.setMaximumHeight(220)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMouseTracking(True)

    def set_curve(self, d_min: int, d_max: int, costs: np.ndarray | None) -> None:
        self._message = None
        self._d_min, self._d_max, self._costs = d_min, d_max, costs
        self.update()

    def set_message(self, text: str) -> None:
        self._d_min = self._d_max = None
        self._costs = None
        self._message = text
        self.update()

    def set_y_axis_short(self, label: str) -> None:
        self._y_axis_short = label or "cost"
        self.update()

    def _curve_layout(self):
        if self._message or self._costs is None or self._costs.size == 0 or self._d_min is None or self._d_max is None:
            return None
        c = self._costs.astype(np.float64)
        c = np.where(np.isfinite(c) & (c < 1e29), c, np.nan)
        if not np.any(np.isfinite(c)):
            return None
        y_hi = float(np.nanmax(c))
        y_lo = float(np.nanmin(c))
        if y_hi <= y_lo:
            y_hi = y_lo + 1e-6
        pad = (y_hi - y_lo) * 0.1
        y_lo -= pad
        y_hi += pad
        w_pix = self.width()
        h_pix = self.height()
        margin_l, margin_r, margin_t, margin_b = 48, 14, 14, 36
        plot_w = max(1, w_pix - margin_l - margin_r)
        plot_h = max(1, h_pix - margin_t - margin_b)
        n = int(c.size)
        d0, d1 = self._d_min, self._d_max
        return {
            "c": c,
            "y_lo": y_lo,
            "y_hi": y_hi,
            "margin_l": margin_l,
            "margin_r": margin_r,
            "margin_t": margin_t,
            "margin_b": margin_b,
            "plot_w": plot_w,
            "plot_h": plot_h,
            "n": n,
            "d0": d0,
            "d1": d1,
            "w_pix": w_pix,
            "h_pix": h_pix,
        }

    def _index_to_x(self, i: int, lay: dict) -> float:
        n = lay["n"]
        return lay["margin_l"] + (i / max(1, n - 1)) * lay["plot_w"]

    def _cost_to_y(self, v: float, lay: dict) -> float:
        t = (v - lay["y_lo"]) / (lay["y_hi"] - lay["y_lo"])
        return lay["margin_t"] + lay["plot_h"] - t * lay["plot_h"]

    def _disparity_tick_values(self, d0: int, d1: int) -> list[int]:
        span = d1 - d0
        if span <= 0:
            return [d0]
        if span <= 8:
            return list(range(d0, d1 + 1))
        target = 7
        step = max(1, (span + target - 1) // target)
        ticks = list(range(d0, d1 + 1, step))
        if ticks[-1] != d1:
            ticks.append(d1)
        return ticks

    def _global_min_index(self, c: np.ndarray) -> int | None:
        valid = np.isfinite(c) & (c < 1e29)
        idxs = np.flatnonzero(valid)
        if idxs.size == 0:
            return None
        vals = c[idxs]
        order = np.argsort(vals, kind="stable")
        return int(idxs[order[0]])

    def _second_interior_peak_index(self, c: np.ndarray, ib: int | None) -> int | None:
        if ib is None:
            return None
        n = int(c.size)
        if n < 3:
            return None
        best_c = np.inf
        best_i: int | None = None
        for i in range(1, n - 1):
            if i == ib:
                continue
            a, b, cc = float(c[i - 1]), float(c[i]), float(c[i + 1])
            if not (np.isfinite(a) and np.isfinite(b) and np.isfinite(cc)):
                continue
            if a >= 1e29 or b >= 1e29 or cc >= 1e29:
                continue
            if b < a and b < cc and b < best_c:
                best_c = b
                best_i = i
        return best_i

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        super().mouseMoveEvent(event)
        lay = self._curve_layout()
        if lay is None:
            QToolTip.hideText()
            return
        mx = float(event.position().x())
        my = float(event.position().y())
        ml, mt, pw, ph = lay["margin_l"], lay["margin_t"], lay["plot_w"], lay["plot_h"]
        c, n, d0 = lay["c"], lay["n"], lay["d0"]
        if not (ml <= mx <= ml + pw and mt <= my <= mt + ph):
            QToolTip.hideText()
            return
        t = (mx - ml) / pw
        fi = t * max(1, n - 1)
        i = int(round(fi))
        i = max(0, min(n - 1, i))
        d = d0 + i
        v = c[i]
        if not np.isfinite(v) or v >= 1e29:
            QToolTip.hideText()
            return
        ylab = self._y_axis_short
        QToolTip.showText(
            QCursor.pos(),
            f"d = {d}\n{ylab} = {v:.6f}",
            self,
            self.rect(),
            4000,
        )

    def leaveEvent(self, event) -> None:
        QToolTip.hideText()
        super().leaveEvent(event)

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.fillRect(self.rect(), QColor(28, 28, 30))
        painter.setPen(QColor(160, 160, 168))
        font = QFont()
        font.setPointSize(9)
        painter.setFont(font)
        if self._message:
            painter.drawText(self.rect(), int(Qt.AlignmentFlag.AlignCenter), self._message)
            return
        lay = self._curve_layout()
        if lay is None:
            painter.drawText(
                self.rect(),
                int(Qt.AlignmentFlag.AlignCenter),
                "Hover pyramid gray or disparity view to plot block cost vs d",
            )
            return
        c = lay["c"]
        margin_l = lay["margin_l"]
        margin_t = lay["margin_t"]
        margin_b = lay["margin_b"]
        plot_w = lay["plot_w"]
        plot_h = lay["plot_h"]
        h_pix = lay["h_pix"]
        n = lay["n"]
        d0, d1 = lay["d0"], lay["d1"]

        painter.setPen(QColor(70, 70, 78))
        painter.drawRect(margin_l, margin_t, plot_w, plot_h)
        painter.setPen(QColor(120, 120, 130))
        painter.drawText(4, margin_t + 12, self._y_axis_short)

        tick_font = QFont(font)
        tick_font.setPointSize(8)
        painter.setFont(tick_font)
        for td in self._disparity_tick_values(d0, d1):
            i = td - d0
            if i < 0 or i >= n:
                continue
            x = self._index_to_x(i, lay)
            painter.setPen(QColor(90, 90, 98))
            painter.drawLine(int(x), margin_t + plot_h, int(x), margin_t + plot_h + 5)
            painter.setPen(QColor(150, 150, 158))
            tw = painter.fontMetrics().horizontalAdvance(str(td))
            painter.drawText(int(x - tw / 2), h_pix - 10, str(td))
        painter.setFont(font)
        painter.setPen(QColor(110, 110, 120))
        painter.drawText(margin_l, h_pix - 2, "disparity")

        painter.setPen(QPen(QColor(80, 200, 255), 2))
        for i in range(n - 1):
            x0 = self._index_to_x(i, lay)
            x1 = self._index_to_x(i + 1, lay)
            v0, v1 = c[i], c[i + 1]
            if not (np.isfinite(v0) and np.isfinite(v1)):
                continue
            y0 = self._cost_to_y(v0, lay)
            y1 = self._cost_to_y(v1, lay)
            painter.drawLine(int(x0), int(y0), int(x1), int(y1))

        ib = self._global_min_index(c)
        i2 = self._second_interior_peak_index(c, ib)
        if ib is not None and np.isfinite(c[ib]):
            xb = self._index_to_x(ib, lay)
            yb = self._cost_to_y(float(c[ib]), lay)
            painter.setPen(QPen(QColor(80, 255, 120), 1))
            painter.setBrush(QColor(80, 255, 120, 200))
            painter.drawEllipse(int(xb - 5), int(yb - 5), 10, 10)
        if i2 is not None and np.isfinite(c[i2]):
            x2 = self._index_to_x(i2, lay)
            y2 = self._cost_to_y(float(c[i2]), lay)
            painter.setPen(QPen(QColor(255, 180, 60), 1))
            painter.setBrush(QColor(255, 180, 60, 200))
            painter.drawEllipse(int(x2 - 5), int(y2 - 5), 10, 10)

        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QColor(150, 150, 158))
        painter.drawText(margin_l, margin_t - 2, f"d ∈ [{d0}, {d1}]  (n={n})   ● min   ● 2nd peak")


def vertical_colormap_bar_bgr(height: int, width: int, colormap: int) -> np.ndarray:
    ramp = np.linspace(255, 0, height, dtype=np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(ramp, colormap)
    return cv2.resize(bar, (width, height), interpolation=cv2.INTER_NEAREST)


def jet_vertical_colorbar_bgr(height: int = 180, width: int = 22) -> np.ndarray:
    return vertical_colormap_bar_bgr(height, width, cv2.COLORMAP_JET)


class AspectRatioLabel(QLabel):
    def __init__(self) -> None:
        super().__init__()
        self._src: QPixmap | None = None
        self._nearest = False

    def setNearestNeighborScaling(self, on: bool) -> None:
        if self._nearest == on:
            return
        self._nearest = on
        self._apply_scaled()

    def setSourcePixmap(self, pixmap: QPixmap) -> None:
        self._src = pixmap
        self.setText("")
        self._apply_scaled()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._apply_scaled()

    def _scale_mode(self) -> Qt.TransformationMode:
        return Qt.TransformationMode.FastTransformation if self._nearest else Qt.TransformationMode.SmoothTransformation

    def _apply_scaled(self) -> None:
        if self._src is None or self._src.isNull():
            return
        if self.width() <= 1 or self.height() <= 1:
            return
        scaled = self._src.scaled(
            self.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            self._scale_mode(),
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
            self._scale_mode(),
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


def _apply_gpustereo_config_header_defaults(c: dai.GPUStereoConfig) -> None:
    G = dai.GPUStereoConfig
    c.maxDisparity = 128
    c.numPyramidLevels = 3
    c.downsampleMethod = G.DownsampleMethod.BOX_FILTER
    c.prefilterMethod = G.PrefilterMethod.GAUSSIAN_3x3
    c.blockMatchRadius = 3
    c.adaptiveSupportRangeSigma = 0.0
    c.prefilterBilateralSigmaSpatial = 2.0
    c.prefilterBilateralSigmaRange = 0.08
    c.refinementRadius = 6
    c.refinementRadiusFull = 3
    c.subpixelBits = 4
    c.lrCheck = True
    c.lrCheckFast = False
    c.medianSize = 3
    c.minDisp = 0
    c.confidenceThreshold = 10
    c.useCostVolume = False
    c.costVolumeAggregation = G.CostVolumeAggregation.BOX
    c.boxAggregationRadius = 2
    c.bilateralSpatialSigma = 2.0
    c.bilateralRangeSigma = 0.08
    c.bilateralAggregationRadius = 2
    c.costMethod = G.CostMethod.ZNCC
    c.pathAggregation = G.PathAggregation.NONE
    c.sgmP1 = 0.5
    c.sgmP2 = 2.0
    c.sgmAdaptiveP2 = True
    c.useFp16 = True
    c.useQcomAcceleratedOps = False
    if hasattr(c, "debugPyramidLevel"):
        c.debugPyramidLevel = -1
        c.debugPyramidDisparityLevel = -1
        c.debugZnccPlotX = -1
        c.debugZnccPlotY = -1
    c.secondPeakThreshold = 0.0
    c.secondPeakMinDisparityGap = 0
    c.censusRadiusX = 2
    c.censusRadiusY = 2
    c.speckleMaxSize = 0
    c.speckleMaxDiff = 1
    c.textureFilterRadius = 0
    c.textureThreshold = 25.0
    c.featureMaskEdgeThresh = 0.0
    c.featureMaskCornerThresh = 0.0
    c.featureMaskMorphRadius = 1
    c.edgeAwareRadius = 0
    c.edgeAwareEps = 0.01
    c.holeFillRadius = 0
    c.holeFillSigmaSpatial = 2.0
    c.holeFillSigmaRange = 0.05
    c.temporalAlpha = 0.0
    c.temporalDelta = 0
    c.temporalPersistencyMode = 3
    c.regionRefine = False
    c.regionRefineCellSize = 16
    c.regionRefinePlaneResidualThresh = 3.0
    c.algorithmControl.depthUnit = dai.DepthUnit.MILLIMETER
    c.algorithmControl.customDepthUnitMultiplier = 1000.0


def gpu_stereo_pipeline_defaults() -> dai.GPUStereoConfig:
    c = dai.GPUStereoConfig()
    _apply_gpustereo_config_header_defaults(c)
    return c


DEFAULT_VIZ_DISP_MAX = 128

VIEW_PYR_LEVELS_UI = 7

_PYRAMID_DISP_COLORMAP = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)


def _pyramid_view_ranges(has_gray: bool, has_disp: bool) -> tuple[tuple[int, int] | None, tuple[int, int] | None]:
    idx = 3
    gr: tuple[int, int] | None = None
    dr: tuple[int, int] | None = None
    if has_gray:
        gr = (idx, idx + VIEW_PYR_LEVELS_UI)
        idx += VIEW_PYR_LEVELS_UI
    if has_disp:
        dr = (idx, idx + VIEW_PYR_LEVELS_UI)
        idx += VIEW_PYR_LEVELS_UI
    return gr, dr


def _vm_in_range(vm: int, rng: tuple[int, int] | None) -> bool:
    return rng is not None and rng[0] <= vm < rng[1]


def _match_cost_curve_supported(cm: object) -> bool:
    CM = dai.GPUStereoConfig.CostMethod
    return cm in (CM.SAD, CM.ZNCC, CM.CENSUS, CM.RANK, CM.GRADIENT)


def _match_cost_curve_descriptor_pair(cm: object) -> bool:
    CM = dai.GPUStereoConfig.CostMethod
    return cm in (CM.CENSUS, CM.RANK)


def _vm_needs_match_cost_plot(vm: int, gray_rng: tuple[int, int] | None, disp_rng: tuple[int, int] | None) -> bool:
    return _vm_in_range(vm, gray_rng) or _vm_in_range(vm, disp_rng)


def default_config_path() -> Path:
    return Path(__file__).resolve().parent / "config.yaml"


def load_config_file(path: Path) -> dict:
    if not path.is_file():
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save_config_file(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False, allow_unicode=True)


def _serialize_gpustereo_config(cfg: dai.GPUStereoConfig) -> dict:
    out: dict = {}
    for name in _GPUS_CONFIG_FIELDS:
        v = getattr(cfg, name)
        if isinstance(v, enum.Enum):
            out[name] = v.name
        elif isinstance(v, (float, np.floating)):
            out[name] = float(v)
        elif isinstance(v, bool):
            out[name] = bool(v)
        else:
            out[name] = int(v)
    out["depthUnit"] = cfg.algorithmControl.depthUnit.name
    out["customDepthUnitMultiplier"] = float(cfg.algorithmControl.customDepthUnitMultiplier)
    return out


def _deserialize_gpustereo_config(d: dict) -> dai.GPUStereoConfig:
    tmpl = gpu_stereo_pipeline_defaults()
    out = dai.GPUStereoConfig()
    _gpustereo_config_assign(out, tmpl)
    for name in _GPUS_CONFIG_FIELDS:
        if name not in d:
            continue
        raw = d[name]
        ref = getattr(tmpl, name)
        if isinstance(ref, enum.Enum):
            setattr(out, name, type(ref)[raw] if isinstance(raw, str) else raw)
        elif isinstance(ref, bool):
            setattr(out, name, bool(raw))
        elif isinstance(ref, float):
            setattr(out, name, float(raw))
        elif isinstance(ref, int):
            setattr(out, name, int(raw))
        else:
            setattr(out, name, float(raw))
    if "depthUnit" in d:
        out.algorithmControl.depthUnit = dai.DepthUnit[d["depthUnit"]]
    if "customDepthUnitMultiplier" in d:
        out.algorithmControl.customDepthUnitMultiplier = float(d["customDepthUnitMultiplier"])
    return out


class PipelineThread(QThread):
    frame_ready = pyqtSignal(object)

    def __init__(
        self,
        pipeline: dai.Pipeline,
        disp_q: dai.OutputQueue,
        depth_q: dai.OutputQueue,
        rect_left_q: dai.OutputQueue,
        rect_right_q: dai.OutputQueue,
        pyr_q: dai.OutputQueue | None = None,
        match_curve_q: dai.OutputQueue | None = None,
        pyr_disp_q: dai.OutputQueue | None = None,
    ) -> None:
        super().__init__()
        self.pipeline = pipeline
        self.disp_q = disp_q
        self.depth_q = depth_q
        self.rect_left_q = rect_left_q
        self.rect_right_q = rect_right_q
        self.pyr_q = pyr_q
        self.match_curve_q = match_curve_q
        self.pyr_disp_q = pyr_disp_q
        self._stop = threading.Event()
        self._last_match_curve: dai.ImgFrame | None = None
        self._last_pyr_disp: np.ndarray | None = None

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
                pyr_gray = None
                if self.pyr_q is not None:
                    pdbg = self.pyr_q.tryGet()
                    if pdbg is not None and isinstance(pdbg, dai.ImgFrame):
                        pyr_gray = np.asarray(pdbg.getFrame()).copy()
                match_curve_fr = None
                if self.match_curve_q is not None:
                    zg = self.match_curve_q.tryGet()
                    if zg is not None and isinstance(zg, dai.ImgFrame):
                        self._last_match_curve = zg
                    match_curve_fr = self._last_match_curve
                pyr_disp_arr = None
                if self.pyr_disp_q is not None:
                    pdg = self.pyr_disp_q.tryGet()
                    if pdg is not None and isinstance(pdg, dai.ImgFrame):
                        self._last_pyr_disp = np.asarray(pdg.getFrame()).copy()
                    pyr_disp_arr = self._last_pyr_disp
                self.frame_ready.emit(
                    (
                        d_u16.copy(),
                        depth_u16.copy(),
                        rl_gray.copy(),
                        rr_gray.copy(),
                        pyr_gray,
                        match_curve_fr,
                        pyr_disp_arr,
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
        self.second_peak.setToolTip(
            "Minimum cost margin between the WTA winner and the next-best strict local minimum "
            "(interior disparity where cost is lower than at d−1 and d+1). "
            "Applies to cost-volume WTA and the pyramid second-peak pass. 0 = off."
        )
        fl.addRow("Second-peak threshold", self.second_peak)
        self.second_peak_gap = QSpinBox()
        self.second_peak_gap.setRange(0, 32)
        self.second_peak_gap.setToolTip(
            "If > 0, only invalidate on ambiguous margin when the second local-minimum disparity is farther "
            "than this (pixels) from the best. Same as second_peak_min_disparity_gap in GPUStereoConfig."
        )
        fl.addRow("Second-peak min disparity gap", self.second_peak_gap)

        g, fl = gb("Cost volume")
        self.use_cost_volume = QCheckBox()
        fl.addRow("Use cost volume", self.use_cost_volume)
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
        config_path: Path,
        cli_device: str,
        cli_resolution: str,
        cli_fps: float,
        saved: dict | None,
        pyr_q: dai.OutputQueue | None = None,
        match_curve_q: dai.OutputQueue | None = None,
        pyr_disp_q: dai.OutputQueue | None = None,
        device: dai.Device | None = None,
        ir_dot_projector: float = 0.9,
    ) -> None:
        super().__init__()
        self.setWindowTitle("GPUStereo — controls")
        self._device = device
        self._pipeline = pipeline
        self._gpu = gpu
        self._disp_q = disp_q
        self._cfg_q = cfg_q
        self._left_cam_ctrl_q = left_cam_ctrl_q
        self._right_cam_ctrl_q = right_cam_ctrl_q
        self._base_algo = initial_cfg
        self._config_path = config_path
        self._cli_device = cli_device
        self._cli_resolution = cli_resolution
        self._cli_fps = cli_fps
        self._pyr_q = pyr_q
        self._match_curve_q = match_curve_q
        self._pyr_disp_q = pyr_disp_q
        self._pending_plot_xy: tuple[int, int] | None = None
        self._committed_plot_xy: tuple[int, int] | None = None

        self._min_d = 0
        self._max_d = initial_cfg.maxDisparity * (1 << int(initial_cfg.subpixelBits))

        self._last_bundle: tuple | None = None

        self._view_mode = QComboBox()
        self._view_mode.addItems(["Disparity", "Rectified left", "Rectified right"])
        if pyr_q is not None:
            for li in range(VIEW_PYR_LEVELS_UI):
                self._view_mode.addItem(f"Pyramid L{li}")
        if pyr_disp_q is not None:
            for li in range(VIEW_PYR_LEVELS_UI):
                self._view_mode.addItem(f"Pyramid disp L{li}")
        self._vr_gray, self._vr_disp = _pyramid_view_ranges(self._pyr_q is not None, self._pyr_disp_q is not None)
        self._view_mode.setCurrentIndex(0)
        self._view_mode.currentIndexChanged.connect(self._on_view_mode_changed)

        self._disp_viz_widget = QWidget()
        dv_lay = QVBoxLayout(self._disp_viz_widget)
        dv_lay.setContentsMargins(0, 0, 0, 0)
        self._slider_disp_min = QSlider(Qt.Orientation.Horizontal)
        self._slider_disp_max = QSlider(Qt.Orientation.Horizontal)
        self._lbl_disp_min_val = QLabel("0")
        self._lbl_disp_max_val = QLabel("0")
        row_min = QHBoxLayout()
        row_min.addWidget(QLabel("Min disp"))
        row_min.addWidget(self._slider_disp_min, stretch=1)
        row_min.addWidget(self._lbl_disp_min_val)
        row_max = QHBoxLayout()
        row_max.addWidget(QLabel("Max disp"))
        row_max.addWidget(self._slider_disp_max, stretch=1)
        row_max.addWidget(self._lbl_disp_max_val)
        dv_lay.addLayout(row_min)
        dv_lay.addLayout(row_max)
        self._sync_viz_slider_range(use_defaults=True)
        self._slider_disp_min.valueChanged.connect(self._on_viz_disp_min_changed)
        self._slider_disp_max.valueChanged.connect(self._on_viz_disp_max_changed)

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

        self._colorbar_widget = QWidget()
        cb_row = QHBoxLayout(self._colorbar_widget)
        cb_row.setContentsMargins(0, 0, 0, 0)
        depth_lbl_col = QVBoxLayout()
        self._lbl_near_depth = QLabel("Near: —")
        self._lbl_far_depth = QLabel("Far: —")
        depth_lbl_col.addWidget(self._lbl_near_depth)
        depth_lbl_col.addStretch()
        depth_lbl_col.addWidget(self._lbl_far_depth)
        cb_row.addLayout(depth_lbl_col)
        self._colorbar_label = QLabel()
        self._colorbar_label.setFixedSize(22, 180)
        self._colorbar_label.setPixmap(numpy_bgr_to_qpixmap(jet_vertical_colorbar_bgr(180, 22)))
        cb_row.addWidget(self._colorbar_label)

        self._image_row = QWidget()
        image_row_lay = QHBoxLayout(self._image_row)
        image_row_lay.setContentsMargins(0, 0, 0, 0)
        image_row_lay.addWidget(self._disp_label, stretch=1)
        image_row_lay.addWidget(self._colorbar_widget)

        self._cost_plot = MatchCostPlotWidget()
        self._cost_plot.setVisible(False)

        self._hover_dist = QLabel("")
        self._hover_dist.setAlignment(Qt.AlignmentFlag.AlignLeft)

        left_pane = QWidget()
        left_lay = QVBoxLayout(left_pane)
        left_lay.setContentsMargins(0, 0, 0, 0)
        left_lay.addWidget(self._view_mode)
        left_lay.addWidget(self._disp_viz_widget)
        left_lay.addWidget(self._image_row, stretch=1)
        left_lay.addWidget(self._cost_plot)
        left_lay.addWidget(self._hover_dist)

        cam_outer = QGroupBox("Cameras — CAM_B + CAM_C")
        cam_fl = QFormLayout(cam_outer)
        self._cam_auto_exp = QCheckBox("Auto-exposure")
        self._cam_auto_exp.setChecked(True)
        self._cam_auto_exp.stateChanged.connect(self._on_cam_auto_exposure_changed)
        cam_fl.addRow(self._cam_auto_exp)
        self._cam_ae_comp = QSlider(Qt.Orientation.Horizontal)
        self._cam_ae_comp.setRange(-9, 9)
        self._cam_ae_comp.setValue(0)
        self._cam_ae_comp.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._cam_ae_comp.setTickInterval(1)
        self._lbl_ae_comp = QLabel("0")
        self._cam_ae_comp.valueChanged.connect(self._on_cam_ae_comp_changed)
        ae_row = QWidget()
        ae_row_lay = QHBoxLayout(ae_row)
        ae_row_lay.setContentsMargins(0, 0, 0, 0)
        ae_row_lay.addWidget(self._cam_ae_comp, stretch=1)
        ae_row_lay.addWidget(self._lbl_ae_comp)
        cam_fl.addRow("AE compensation", ae_row)
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
        self._slider_ir_dot = QSlider(Qt.Orientation.Horizontal)
        self._slider_ir_dot.setRange(0, 10)
        self._slider_ir_dot.setSingleStep(1)
        self._slider_ir_dot.setPageStep(1)
        self._slider_ir_dot.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._slider_ir_dot.setTickInterval(1)
        self._lbl_ir_dot = QLabel("")
        ir_row = QWidget()
        ir_row_lay = QHBoxLayout(ir_row)
        ir_row_lay.setContentsMargins(0, 0, 0, 0)
        ir_row_lay.addWidget(self._slider_ir_dot, stretch=1)
        ir_row_lay.addWidget(self._lbl_ir_dot)
        cam_fl.addRow("IR dot projector (0–1)", ir_row)
        btn_cam = QPushButton("Apply to both cameras")
        btn_cam.clicked.connect(self._on_apply_cam_exposure)
        cam_fl.addRow(btn_cam)
        self._update_cam_exposure_widgets_enabled()
        ir_clamped = max(0.0, min(1.0, float(ir_dot_projector)))
        ir_step = int(round(ir_clamped * 10.0))
        self._slider_ir_dot.blockSignals(True)
        self._slider_ir_dot.setValue(ir_step)
        self._slider_ir_dot.blockSignals(False)
        self._lbl_ir_dot.setText(f"{ir_step * 0.1:.1f}")

        self._panel = StereoConfigPanel(initial_cfg)
        self._wire_panel_autosave()
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

        self._plot_debounce = QTimer(self)
        self._plot_debounce.setSingleShot(True)
        self._plot_debounce.timeout.connect(self._commit_debug_plot_pixel)

        self._worker = PipelineThread(
            pipeline,
            disp_q,
            depth_q,
            rect_left_q,
            rect_right_q,
            pyr_q,
            match_curve_q,
            pyr_disp_q,
        )
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.start()

        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.timeout.connect(self._save_config_to_file)
        self._cam_exp_us.valueChanged.connect(self._schedule_save)
        self._cam_iso.valueChanged.connect(self._schedule_save)
        self._cam_sharpness.valueChanged.connect(self._schedule_save)
        self._cam_luma_denoise.valueChanged.connect(self._schedule_save)
        self._slider_ir_dot.valueChanged.connect(self._on_ir_dot_changed)
        self._view_mode.currentIndexChanged.connect(lambda _i: self._schedule_save())

        self._panel.levels.currentIndexChanged.connect(self._on_panel_pyramid_levels_changed)
        self._panel.cost_method.currentIndexChanged.connect(self._on_panel_cost_method_changed)
        self._panel.use_cost_volume.stateChanged.connect(lambda _s: self._refresh_cost_plot_hint_if_visible())
        self._panel.adaptive_sigma.valueChanged.connect(lambda _v: self._refresh_cost_plot_hint_if_visible())

        if saved:
            self._apply_saved_ui(saved)

    def _vm_gray_pyr(self, vm: int) -> bool:
        return _vm_in_range(vm, self._vr_gray)

    def _vm_disp_pyr(self, vm: int) -> bool:
        return _vm_in_range(vm, self._vr_disp)

    def _vm_match_cost_plot(self, vm: int) -> bool:
        return _vm_needs_match_cost_plot(vm, self._vr_gray, self._vr_disp)

    def _match_curve_y_label(self) -> str:
        G = self._panel.G.CostMethod
        cm = self._panel.cost_method.currentData()
        if cm == G.ZNCC:
            return "1−ρ"
        if cm == G.SAD:
            return "w-SAD" if self._panel.adaptive_sigma.value() > 0.0 else "SAD"
        if cm == G.GRADIENT:
            return "SAD"
        if cm in (G.CENSUS, G.RANK):
            return "Hamming"
        return "cost"

    def _refresh_cost_plot_hint_if_visible(self) -> None:
        if self._cost_plot.isVisible():
            self._update_match_cost_plot_hint()

    def _sync_match_cost_plot(self, curve_fr: dai.ImgFrame | None) -> None:
        if not self._cost_plot.isVisible():
            return
        cm = self._panel.cost_method.currentData()
        if not _match_cost_curve_supported(cm):
            self._update_match_cost_plot_hint()
            return
        if self._panel.use_cost_volume.isChecked() and _match_cost_curve_descriptor_pair(cm):
            self._update_match_cost_plot_hint()
            return
        self._cost_plot.set_y_axis_short(self._match_curve_y_label())
        self._cost_plot.set_message("")
        if self._committed_plot_xy is None:
            self._cost_plot.set_curve(0, -1, None)
            return
        dm, dx, costs = parse_match_cost_curve_frame(curve_fr)
        if costs is not None and costs.size > 0:
            self._cost_plot.set_curve(dm, dx, costs)
        else:
            self._cost_plot.set_curve(0, -1, None)

    def _sync_disp_colorbar_ramp(self) -> None:
        vm = self._view_mode.currentIndex()
        cmap = _PYRAMID_DISP_COLORMAP if self._vm_disp_pyr(vm) else cv2.COLORMAP_JET
        self._colorbar_label.setPixmap(numpy_bgr_to_qpixmap(vertical_colormap_bar_bgr(180, 22, cmap)))

    def _config_with_debug(self) -> dai.GPUStereoConfig:
        c = self._panel.to_config(self._base_algo)
        vm = self._view_mode.currentIndex()
        if hasattr(c, "debugPyramidLevel"):
            c.debugPyramidLevel = (vm - self._vr_gray[0]) if self._vm_gray_pyr(vm) else -1
        if hasattr(c, "debugPyramidDisparityLevel"):
            c.debugPyramidDisparityLevel = (vm - self._vr_disp[0]) if self._vm_disp_pyr(vm) else -1
        if hasattr(c, "debugZnccPlotX"):
            if self._vm_match_cost_plot(vm) and self._committed_plot_xy is not None:
                c.debugZnccPlotX = self._committed_plot_xy[0]
                c.debugZnccPlotY = self._committed_plot_xy[1]
            else:
                c.debugZnccPlotX = -1
                c.debugZnccPlotY = -1
        return c

    def _send_view_config_to_device(self) -> None:
        if self._pyr_q is None and self._match_curve_q is None and self._pyr_disp_q is None:
            return
        try:
            self._cfg_q.send(self._config_with_debug())
        except Exception:
            pass

    def _commit_debug_plot_pixel(self) -> None:
        if self._pending_plot_xy is None:
            return
        self._committed_plot_xy = self._pending_plot_xy
        self._send_view_config_to_device()

    def _on_panel_pyramid_levels_changed(self, _i: int) -> None:
        self._schedule_save()
        if self._vm_match_cost_plot(self._view_mode.currentIndex()):
            self._send_view_config_to_device()

    def _on_panel_cost_method_changed(self, _i: int) -> None:
        self._schedule_save()
        if self._vm_match_cost_plot(self._view_mode.currentIndex()):
            self._update_match_cost_plot_hint()
            self._send_view_config_to_device()

    def _schedule_save(self) -> None:
        self._save_timer.start(500)

    def _wire_panel_autosave(self) -> None:
        for w in self._panel.findChildren(QSpinBox):
            w.valueChanged.connect(self._schedule_save)
        for w in self._panel.findChildren(QDoubleSpinBox):
            w.valueChanged.connect(self._schedule_save)
        for w in self._panel.findChildren(QComboBox):
            w.currentIndexChanged.connect(self._schedule_save)
        for w in self._panel.findChildren(QCheckBox):
            w.stateChanged.connect(self._schedule_save)

    def _collect_config_dict(self) -> dict:
        cfg = self._panel.to_config(self._base_algo)
        return {
            "device": self._cli_device,
            "resolution": self._cli_resolution,
            "fps": float(self._cli_fps),
            "view_mode": int(self._view_mode.currentIndex()),
            "viz_disp_min": int(self._slider_disp_min.value()),
            "viz_disp_max": int(self._slider_disp_max.value()),
            "camera": {
                "auto_exposure": self._cam_auto_exp.isChecked(),
                "ae_compensation": int(self._cam_ae_comp.value()),
                "exposure_us": int(self._cam_exp_us.value()),
                "iso": int(self._cam_iso.value()),
                "sharpness": int(self._cam_sharpness.value()),
                "luma_denoise": int(self._cam_luma_denoise.value()),
            },
            "gpustereo": _serialize_gpustereo_config(cfg),
            "ir_dot_projector": round(self._slider_ir_dot.value() * 0.1, 1),
        }

    def _save_config_to_file(self) -> None:
        try:
            save_config_file(self._config_path, self._collect_config_dict())
        except Exception:
            pass

    def _apply_saved_ui(self, d: dict) -> None:
        cam = d.get("camera")
        if isinstance(cam, dict):
            if "auto_exposure" in cam:
                self._cam_auto_exp.blockSignals(True)
                self._cam_auto_exp.setChecked(bool(cam["auto_exposure"]))
                self._cam_auto_exp.blockSignals(False)
            if "ae_compensation" in cam:
                self._cam_ae_comp.blockSignals(True)
                ac = max(-9, min(9, int(cam["ae_compensation"])))
                self._cam_ae_comp.setValue(ac)
                self._lbl_ae_comp.setText(str(ac))
                self._cam_ae_comp.blockSignals(False)
            if "exposure_us" in cam:
                self._cam_exp_us.setValue(int(cam["exposure_us"]))
            if "iso" in cam:
                self._cam_iso.setValue(int(cam["iso"]))
            if "sharpness" in cam:
                self._cam_sharpness.setValue(int(cam["sharpness"]))
            if "luma_denoise" in cam:
                self._cam_luma_denoise.setValue(int(cam["luma_denoise"]))
            self._update_cam_exposure_widgets_enabled()
        if "view_mode" in d:
            self._view_mode.setCurrentIndex(max(0, min(int(d["view_mode"]), self._view_mode.count() - 1)))
        vm = self._view_mode.currentIndex()
        use_pyr = (self._pyr_q is not None and self._vm_gray_pyr(vm)) or (
            self._pyr_disp_q is not None and self._vm_disp_pyr(vm)
        )
        self._disp_label.setNearestNeighborScaling(use_pyr)
        self._cost_plot.setVisible(self._match_curve_q is not None and self._vm_match_cost_plot(vm))
        if self._cost_plot.isVisible():
            self._update_match_cost_plot_hint()
        if "viz_disp_min" in d and "viz_disp_max" in d:
            self._slider_disp_min.setValue(int(d["viz_disp_min"]))
            self._slider_disp_max.setValue(int(d["viz_disp_max"]))
            self._sync_viz_slider_range(use_defaults=False)
        if "ir_dot_projector" in d and hasattr(self, "_slider_ir_dot"):
            try:
                x = max(0.0, min(1.0, float(d["ir_dot_projector"])))
                t = int(round(x * 10.0))
                self._slider_ir_dot.blockSignals(True)
                self._slider_ir_dot.setValue(t)
                self._slider_ir_dot.blockSignals(False)
                self._lbl_ir_dot.setText(f"{t * 0.1:.1f}")
                if self._device is not None:
                    self._device.setIrLaserDotProjectorIntensity(float(t * 0.1))
            except (TypeError, ValueError):
                pass
        self._refresh_left_pane()
        if self._view_mode.currentIndex() == 0:
            self._update_disparity_depth_labels()
        self._sync_disp_colorbar_ramp()
        self._send_view_config_to_device()

    def _sync_viz_slider_range(self, use_defaults: bool = False) -> None:
        md = max(1, int(self._max_d))
        self._slider_disp_min.blockSignals(True)
        self._slider_disp_max.blockSignals(True)
        self._slider_disp_min.setRange(0, md)
        self._slider_disp_max.setRange(0, md)
        if use_defaults:
            vmin, vmax = 0, min(DEFAULT_VIZ_DISP_MAX, md)
        else:
            vmin = self._slider_disp_min.value()
            vmax = self._slider_disp_max.value()
            vmin = max(0, min(int(vmin), md - 1)) if md > 1 else 0
            vmax = max(1, min(int(vmax), md))
            if vmin >= vmax:
                vmin, vmax = 0, min(DEFAULT_VIZ_DISP_MAX, md)
        self._slider_disp_min.setValue(vmin)
        self._slider_disp_max.setValue(vmax)
        self._slider_disp_min.blockSignals(False)
        self._slider_disp_max.blockSignals(False)
        self._lbl_disp_min_val.setText(str(self._slider_disp_min.value()))
        self._lbl_disp_max_val.setText(str(self._slider_disp_max.value()))

    def _on_viz_disp_min_changed(self, _v: int) -> None:
        if self._slider_disp_min.value() >= self._slider_disp_max.value():
            self._slider_disp_max.blockSignals(True)
            if self._slider_disp_min.value() < self._slider_disp_max.maximum():
                self._slider_disp_max.setValue(self._slider_disp_min.value() + 1)
            else:
                self._slider_disp_min.blockSignals(True)
                self._slider_disp_min.setValue(max(0, self._slider_disp_max.value() - 1))
                self._slider_disp_min.blockSignals(False)
            self._slider_disp_max.blockSignals(False)
        self._lbl_disp_min_val.setText(str(self._slider_disp_min.value()))
        self._refresh_left_pane()
        self._update_disparity_depth_labels()
        self._schedule_save()

    def _on_viz_disp_max_changed(self, _v: int) -> None:
        if self._slider_disp_max.value() <= self._slider_disp_min.value():
            self._slider_disp_min.blockSignals(True)
            if self._slider_disp_max.value() > self._slider_disp_min.minimum():
                self._slider_disp_min.setValue(self._slider_disp_max.value() - 1)
            else:
                self._slider_disp_max.blockSignals(True)
                self._slider_disp_max.setValue(min(self._slider_disp_min.value() + 1, self._slider_disp_max.maximum()))
                self._slider_disp_max.blockSignals(False)
            self._slider_disp_min.blockSignals(False)
        self._lbl_disp_max_val.setText(str(self._slider_disp_max.value()))
        self._refresh_left_pane()
        self._update_disparity_depth_labels()
        self._schedule_save()

    def _on_view_mode_changed(self, _i: int) -> None:
        vm = self._view_mode.currentIndex()
        disp = vm == 0
        pyr_disp_viz = self._pyr_disp_q is not None and self._vm_disp_pyr(vm)
        self._disp_viz_widget.setVisible(disp or pyr_disp_viz)
        self._colorbar_widget.setVisible(disp or pyr_disp_viz)
        use_pyr = (self._pyr_q is not None and self._vm_gray_pyr(vm)) or (
            self._pyr_disp_q is not None and self._vm_disp_pyr(vm)
        )
        self._disp_label.setNearestNeighborScaling(use_pyr)
        show_plot = self._match_curve_q is not None and self._vm_match_cost_plot(vm)
        self._cost_plot.setVisible(show_plot)
        if show_plot:
            self._update_match_cost_plot_hint()
        self._refresh_left_pane()
        if disp:
            self._update_disparity_depth_labels()
        self._sync_disp_colorbar_ramp()
        self._send_view_config_to_device()

    def _update_match_cost_plot_hint(self) -> None:
        if not self._cost_plot.isVisible():
            return
        cm = self._panel.cost_method.currentData()
        if not _match_cost_curve_supported(cm):
            self._cost_plot.set_y_axis_short("cost")
            self._cost_plot.set_message("Cost curve not available for this cost method")
            return
        if self._panel.use_cost_volume.isChecked() and _match_cost_curve_descriptor_pair(cm):
            self._cost_plot.set_y_axis_short(self._match_curve_y_label())
            self._cost_plot.set_message("Census/Rank curves need pyramid matching (turn off cost volume)")
            return
        self._cost_plot.set_y_axis_short(self._match_curve_y_label())
        self._cost_plot.set_message("")
        self._cost_plot.set_curve(0, -1, None)

    def _disparity_bgr(
        self,
        d_u16: np.ndarray,
        *,
        value_scale: float = 1.0,
        colormap: int = cv2.COLORMAP_JET,
    ) -> np.ndarray:
        return colorize_disparity_u16(
            d_u16,
            self._slider_disp_min.value(),
            self._slider_disp_max.value(),
            value_scale=value_scale,
            colormap=colormap,
        )

    def _update_disparity_depth_labels(self) -> None:
        if self._last_bundle is None:
            self._lbl_near_depth.setText("Near: —")
            self._lbl_far_depth.setText("Far: —")
            return
        if self._view_mode.currentIndex() != 0:
            return
        b = self._last_bundle
        d_u16, depth_u16 = b[0], b[1]
        k = estimate_depth_scale_k(d_u16, depth_u16)
        vmin = self._slider_disp_min.value()
        vmax = self._slider_disp_max.value()
        if k is None:
            self._lbl_near_depth.setText("Near: —")
            self._lbl_far_depth.setText("Far: —")
            return
        if vmax > 0:
            near_m = (k / float(vmax)) / 1000.0
            self._lbl_near_depth.setText(f"Near: {near_m:.3f} m")
        else:
            self._lbl_near_depth.setText("Near: —")
        if vmin > 0:
            far_m = (k / float(vmin)) / 1000.0
            self._lbl_far_depth.setText(f"Far: {far_m:.3f} m")
        else:
            self._lbl_far_depth.setText("Far: ∞")

    def _bgr_for_view(
        self,
        d_u16: np.ndarray,
        rl_gray: np.ndarray,
        rr_gray: np.ndarray,
        pyr_gray: np.ndarray | None,
        pyr_disp: np.ndarray | None,
    ) -> np.ndarray:
        idx = self._view_mode.currentIndex()
        if self._vm_disp_pyr(idx):
            if pyr_disp is None or pyr_disp.size == 0:
                return np.zeros((max(1, rl_gray.shape[0]), max(1, rl_gray.shape[1]), 3), dtype=np.uint8)
            level = idx - self._vr_disp[0]
            return self._disparity_bgr(
                pyr_disp,
                value_scale=float(1 << level),
                colormap=_PYRAMID_DISP_COLORMAP,
            )
        if self._pyr_q is not None and self._vm_gray_pyr(idx):
            if pyr_gray is None or pyr_gray.size == 0:
                return np.zeros((max(1, rl_gray.shape[0]), max(1, rl_gray.shape[1]), 3), dtype=np.uint8)
            return cv2.cvtColor(pyr_gray, cv2.COLOR_GRAY2BGR)
        if idx == 1:
            return cv2.cvtColor(rl_gray, cv2.COLOR_GRAY2BGR)
        if idx == 2:
            return cv2.cvtColor(rr_gray, cv2.COLOR_GRAY2BGR)
        return self._disparity_bgr(d_u16)

    def _refresh_left_pane(self) -> None:
        if self._last_bundle is None:
            return
        b = self._last_bundle
        d_u16, rl, rr, pyr = b[0], b[2], b[3], b[4]
        pyr_d = b[6] if len(b) > 6 else None
        bgr = self._bgr_for_view(d_u16, rl, rr, pyr, pyr_d)
        self._disp_label.setSourcePixmap(numpy_bgr_to_qpixmap(bgr))

    def _on_pixel_hover(self, ix: int, iy: int) -> None:
        if self._last_bundle is None:
            self._hover_dist.setText("")
            return
        vm = self._view_mode.currentIndex()
        if self._vm_gray_pyr(vm):
            g = self._last_bundle[4]
            if g is not None and g.size > 0 and 0 <= iy < g.shape[0] and 0 <= ix < g.shape[1]:
                self._hover_dist.setText(f"Pyramid gray: {int(g[iy, ix])}")
                self._pending_plot_xy = (ix, iy)
                self._plot_debounce.stop()
                self._plot_debounce.start(75)
            else:
                self._hover_dist.setText("")
            return
        if self._vm_disp_pyr(vm):
            pd = self._last_bundle[6] if len(self._last_bundle) > 6 else None
            if pd is not None and pd.size > 0 and 0 <= iy < pd.shape[0] and 0 <= ix < pd.shape[1]:
                v = int(pd[iy, ix])
                li = vm - self._vr_disp[0]
                if v > 0:
                    eq = v * (1 << li)
                    self._hover_dist.setText(f"L{li} disp: {v} raw, {eq} ×2^{li} (slider scale)")
                else:
                    self._hover_dist.setText("Pyramid disp: —")
                self._pending_plot_xy = (ix, iy)
                self._plot_debounce.stop()
                self._plot_debounce.start(75)
            else:
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
        d_u16, _, rl, rr, pyr = bundle[0], bundle[1], bundle[2], bundle[3], bundle[4]
        pyr_d = bundle[6] if len(bundle) > 6 else None
        bgr = self._bgr_for_view(d_u16, rl, rr, pyr, pyr_d)
        self._disp_label.setSourcePixmap(numpy_bgr_to_qpixmap(bgr))
        self._update_disparity_depth_labels()
        match_curve_fr = bundle[5] if len(bundle) > 5 else None
        self._sync_match_cost_plot(match_curve_fr)

    def _on_apply(self) -> None:
        try:
            cfg = self._config_with_debug()
            self._cfg_q.send(cfg)
            self._max_d = cfg.maxDisparity * (1 << cfg.subpixelBits)
            self._sync_viz_slider_range()
            self._refresh_left_pane()
            self._update_disparity_depth_labels()
            self._status.setText("Config sent (pipeline rebuild on device)")
            self._schedule_save()
        except Exception as e:
            QMessageBox.warning(self, "Apply failed", str(e))

    def _update_cam_exposure_widgets_enabled(self) -> None:
        ae_on = self._cam_auto_exp.isChecked()
        self._cam_ae_comp.setEnabled(ae_on)
        self._cam_exp_us.setEnabled(not ae_on)
        self._cam_iso.setEnabled(not ae_on)

    def _on_cam_auto_exposure_changed(self, _state: int) -> None:
        self._update_cam_exposure_widgets_enabled()
        self._schedule_save()
        try:
            self._send_camera_control()
        except Exception:
            pass

    def _on_cam_ae_comp_changed(self, v: int) -> None:
        ae_comp = max(-9, min(9, int(v)))
        self._lbl_ae_comp.setText(str(ae_comp))
        self._schedule_save()
        if not self._cam_auto_exp.isChecked():
            return
        try:
            self._send_camera_control()
        except Exception:
            pass

    def _send_camera_control(self) -> None:
        exp = int(self._cam_exp_us.value())
        iso = int(self._cam_iso.value())
        sharp = int(self._cam_sharpness.value())
        luma = int(self._cam_luma_denoise.value())
        ae_comp = max(-9, min(9, int(self._cam_ae_comp.value())))
        for q in (self._left_cam_ctrl_q, self._right_cam_ctrl_q):
            ctrl = dai.CameraControl()
            if self._cam_auto_exp.isChecked():
                ctrl.setAutoExposureEnable()
                ctrl.setAutoExposureCompensation(ae_comp)
            else:
                ctrl.setManualExposure(exp, iso)
            ctrl.setSharpness(sharp)
            ctrl.setLumaDenoise(luma)
            q.send(ctrl)

    def _on_apply_cam_exposure(self) -> None:
        try:
            self._send_camera_control()
            self._status.setText("Cameras (CAM_B, CAM_C): controls sent")
            self._schedule_save()
        except Exception as e:
            QMessageBox.warning(self, "Camera control failed", str(e))

    def _on_ir_dot_changed(self, v: int) -> None:
        intensity = max(0.0, min(1.0, int(v) * 0.1))
        self._lbl_ir_dot.setText(f"{intensity:.1f}")
        if self._device is not None:
            try:
                self._device.setIrLaserDotProjectorIntensity(float(intensity))
            except Exception:
                pass
        self._schedule_save()

    def closeEvent(self, event: QCloseEvent) -> None:
        self._save_config_to_file()
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

    config_path = default_config_path()
    saved = load_config_file(config_path)

    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=str, default=saved.get("device") or "10.11.0.51", help="Device IP")
    parser.add_argument("--resolution", type=str, default=saved.get("resolution") or "1280x800", help="WxH")
    parser.add_argument(
        "--fps",
        type=float,
        default=float(saved.get("fps", 60.0)),
        metavar="N",
        help="Camera output FPS (default: 60)",
    )
    _ir_dot_default = 0.9
    if isinstance(saved, dict) and "ir_dot_projector" in saved:
        try:
            _ir_dot_default = max(0.0, min(1.0, float(saved["ir_dot_projector"])))
        except (TypeError, ValueError):
            pass
    parser.add_argument(
        "--ir-dot",
        type=float,
        default=_ir_dot_default,
        metavar="I",
        help="IR laser dot projector intensity in [0, 1] (step 0.1 in GUI; saved in config file)",
    )
    args = parser.parse_args()
    args.ir_dot = max(0.0, min(1.0, float(args.ir_dot)))
    w, h = (int(x) for x in args.resolution.split("x"))

    ic = gpu_stereo_pipeline_defaults()
    gs = saved.get("gpustereo")
    if isinstance(gs, dict) and gs:
        try:
            loaded = _deserialize_gpustereo_config(gs)
            _gpustereo_config_assign(ic, loaded)
        except Exception:
            pass

    vm_saved = saved.get("view_mode", 0)
    try:
        vm_saved = int(vm_saved)
    except (TypeError, ValueError):
        vm_saved = 0

    device = dai.Device(args.device)
    device.setIrLaserDotProjectorIntensity(float(args.ir_dot))

    pipeline = dai.Pipeline(device)
    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    gpu = pipeline.create(dai.node.GPUStereo)
    mono_left.requestOutput((w, h), type=dai.ImgFrame.Type.GRAY8, fps=args.fps).link(gpu.left)
    mono_right.requestOutput((w, h), type=dai.ImgFrame.Type.GRAY8, fps=args.fps).link(gpu.right)
    gpu.setRectification(True)

    disp_q = gpu.disparity.createOutputQueue()
    depth_q = gpu.depth.createOutputQueue()
    rect_left_q = gpu.rectifiedLeft.createOutputQueue()
    rect_right_q = gpu.rectifiedRight.createOutputQueue()
    pyr_q = None
    if hasattr(gpu, "debugPyramid"):
        pyr_q = gpu.debugPyramid.createOutputQueue(maxSize=4, blocking=False)
    match_curve_q = None
    if hasattr(gpu, "debugZnccCurve"):
        match_curve_q = gpu.debugZnccCurve.createOutputQueue(maxSize=4, blocking=False)
    pyr_disp_q = None
    if hasattr(gpu, "debugPyramidDisparity"):
        pyr_disp_q = gpu.debugPyramidDisparity.createOutputQueue(maxSize=4, blocking=False)
    vr_gray, vr_disp = _pyramid_view_ranges(pyr_q is not None, pyr_disp_q is not None)
    if hasattr(ic, "debugPyramidLevel"):
        ic.debugPyramidLevel = (vm_saved - vr_gray[0]) if _vm_in_range(vm_saved, vr_gray) else -1
    if hasattr(ic, "debugPyramidDisparityLevel"):
        ic.debugPyramidDisparityLevel = (vm_saved - vr_disp[0]) if _vm_in_range(vm_saved, vr_disp) else -1
    _gpustereo_config_assign(gpu.initialConfig, ic)
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
        config_path,
        args.device,
        args.resolution,
        float(args.fps),
        saved,
        pyr_q,
        match_curve_q,
        pyr_disp_q,
        device=device,
        ir_dot_projector=float(args.ir_dot),
    )
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
