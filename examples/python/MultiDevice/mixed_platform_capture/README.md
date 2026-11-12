# Mixed-platform trigger capture (OAK4 + RVC2)

One DepthAI pipeline spans an OAK4 (RVC4) and one or more RVC2 cameras. The OAK4 runs hand-pose
detection on its camera. When a fist appears, the OAK4 triggers one capture on each RVC2
camera. The visualizer shows the OAK4 video and the last batch of captures until the next fist.

Each RVC2 runs a `Camera` and a `Script` gate. The gate forwards the newest frame only when a
trigger arrives, so only the triggered frames go over the link.

The hand-pose detection comes from the
[oak-examples hand-pose example](https://github.com/luxonis/oak-examples/tree/main/neural-networks/pose-estimation/hand-pose).
To test without a person in front of the OAK4, use `--media <video>` to replay a video with a fist.

## Standalone on the OAK4

1. In `oakapp.toml`, replace the `--rvc2` IPs with the IPs of your RVC2 cameras.
2. Run `oakctl app run . -d <OAK4 IP>`.
3. Open the frontend URL that `oakctl` prints.

In standalone mode the OAK4 opens the RVC2 cameras over the network, so use PoE RVC2 cameras.

## Host mode

```bash
pip install -r requirements.txt
python main.py --oak4 <OAK4 IP> --rvc2 <RVC2 IP> <RVC2 IP>
```

Open `http://localhost:8082`. `--rvc2` also accepts device IDs and USB names (from
`dai.Device.getAllAvailableDevices()`). Add `--save-dir <folder>` to write each batch as a JPEG,
and `--max-batches <n>` to stop after n batches.

## Limits

- There is no hardware sync between the devices. A capture is the next RVC2 frame after the
  trigger, so it can come up to 200 ms late (the capture cameras run at 5 FPS).
- `--capture-size` must be a size that each capture sensor supports. The default is 1280x800.
