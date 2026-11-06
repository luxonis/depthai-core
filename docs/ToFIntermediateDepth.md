# Intermediate S5K63D depth updates

`ToFConfig.setEnableIntermediateDepth(bool)` selects the depth outputs published
by RVC2 S5K63D when phase unwrapping is enabled. The default is `false` for
production: publish the result using both frequency pairs from the current
four-raw-frame stack. Setting it to `true` additionally publishes an update using
the new frequency pair and opposite-frequency history from the previous stack.
It changes output selection, not sensor calibration or correction settings.

```python
tof = pipeline.create(dai.node.ToF).build(dai.CameraBoardSocket.CAM_A, fps=45)
tof.tofBase.initialConfig.setEnableIntermediateDepth(False)  # production default
# Opt in to the additional history-based depth updates:
tof.tofBase.initialConfig.setEnableIntermediateDepth(True)
```

In C++ the corresponding call is
`tof->tofBase->initialConfig->setEnableIntermediateDepth(true)`.
The public `enableIntermediateDepth` field is also exposed in C++ and Python,
consistent with the other ToF configuration fields.

The setting can also be changed through the existing `tof.tofBase.inputConfig`
queue at runtime. Send a complete `ToFConfig` preserving the other active
settings. Profile selection does not reset this setting. Burst mode suppresses
the intermediate output regardless of this option. Other sensors and operation
without phase unwrapping retain their existing output selection.

When intermediate depth is disabled, firmware may skip depth postprocessing for
that discarded output while still preparing corrected frequency history. When
enabled, the additional output receives full depth processing before publication.
Amplitude, intensity and phase output selection is unchanged.

Use matching SDK and firmware builds that include this option. The serialized
`ToFConfig` schema gains a field; older SDK/firmware combinations are not wire
compatible with the new configuration. This API is not an accuracy certification
for the additional updates. Output FPS depends on the API FPS request, enabled
processing, sensor and host delivery; enabling it does not double raw acquisition.
