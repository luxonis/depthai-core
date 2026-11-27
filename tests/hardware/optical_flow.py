"""Hardware acceptance test for the dedicated native EVA node. Never stops apps.
Use a matching patched host+firmware build and an available RVC4.
"""
import argparse
import json
from datetime import timedelta
from pathlib import Path
import cv2
import depthai as dai
import numpy as np

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--device', required=True)
p.add_argument('--output', type=Path, default=Path('native-flow-hardware-results'))
a=p.parse_args()
if not hasattr(dai.node, 'OpticalFlow') or not hasattr(dai.node.OpticalFlow, 'outputFlow'):
    raise SystemExit('BLOCKED: installed DepthAI lacks OpticalFlow.outputFlow; build the supplied patch and matching firmware first.')
a.output.mkdir(parents=True, exist_ok=True)
rng=np.random.default_rng(27)
base=cv2.GaussianBlur(rng.integers(0,256,(384,640),np.uint8),(3,3),.6)
right=cv2.warpAffine(base,np.float32([[1,0,4],[0,1,0]]),(640,384),borderMode=cv2.BORDER_REFLECT)
moving_box=base.copy()
moving_box[112:304,192:448]=base[112:304,186:442]
frames=[base,base,right,base,moving_box]
sequences=[10,11,14,15,16]
times=[1.,1.033,1.133,1.166,1.199]
expected=[0.,4.,-4.,6.]
rows=[]
with dai.Device(dai.DeviceInfo(a.device)) as device:
    if device.getPlatform()!=dai.Platform.RVC4:
        raise RuntimeError('This test requires RVC4')
    with dai.Pipeline(device) as pipeline:
        flow_node=pipeline.create(dai.node.OpticalFlow)
        incoming=flow_node.inputImage.createInputQueue(maxSize=2,blocking=True)
        passthrough_queue=flow_node.passthroughInputImage.createOutputQueue(maxSize=2,blocking=True)
        flowqueue=flow_node.outputFlow.createOutputQueue(maxSize=2,blocking=True)
        pipeline.start()
        held=None
        for i,frame in enumerate(frames):
            msg=dai.ImgFrame()
            msg.setCvFrame(frame,dai.ImgFrame.Type.GRAY8)
            msg.setSequenceNum(sequences[i])
            msg.setTimestamp(timedelta(seconds=times[i]))
            msg.setTimestampDevice(timedelta(seconds=times[i]))
            incoming.send(msg)
            passthrough=passthrough_queue.get(timedelta(seconds=20))
            assert passthrough is not None and passthrough.getSequenceNum()==sequences[i]
            assert np.array_equal(passthrough.getCvFrame(),frame), "Passthrough pixels changed"
            if not i:
                assert flowqueue.tryGet() is None, 'First frame must not emit a fake flow field'
                continue
            group=flowqueue.get(timedelta(seconds=20))
            assert group is not None, 'No hardware optical flow output'
            tensors=group['flow']
            forward=tensors.getTensor('forward')
            backward=tensors.getTensor('backward')
            confidence=tensors.getTensor('confidence')[...,0].astype(np.uint8)
            assert forward.shape==backward.shape==(384,640,2)
            assert confidence.shape==(384,640)
            assert np.isfinite(forward).all() and np.isfinite(backward).all()
            assert group.getSequenceNum()==tensors.getSequenceNum()==sequences[i]
            assert group['reference'].getSequenceNum()==sequences[i-1]
            assert group['target'].getSequenceNum()==sequences[i]
            for packet in [group, tensors, group['target']]:
                assert packet.getTimestampDevice()==timedelta(seconds=times[i])
                assert packet.getTimestamp()==timedelta(seconds=times[i])
            assert group['reference'].getTimestamp()==timedelta(seconds=times[i-1])
            dt=(group['target'].getTimestampDevice()-group['reference'].getTimestampDevice()).total_seconds()
            assert abs(dt-(times[i]-times[i-1]))<1e-6
            roi=(slice(136,280),slice(216,424)) if i==4 else (slice(48,-48),slice(48,-48))
            mask=confidence[roi]>=100
            assert mask.mean()>.1, 'Insufficient confidence on textured synthetic image'
            f=forward[roi][mask]
            b=backward[roi][mask]
            median=np.median(f,axis=0)
            median_b=np.median(b,axis=0)
            epe=np.linalg.norm(f-[expected[i-1],0],axis=1)
            row=dict(sequence=sequences[i],interval_s=dt,forward_median=median.tolist(),
                     backward_median=median_b.tolist(),median_epe_px=float(np.median(epe)),
                     confident_fraction=float(mask.mean()))
            if i==4:
                bgmask=confidence[48:336,48:160]>=100
                assert bgmask.mean()>.1
                bg=forward[48:336,48:160][bgmask]
                row['background_median_epe_px']=float(np.median(np.linalg.norm(bg,axis=1)))
                assert row['background_median_epe_px']<1.5, 'Stationary background moved with the box'
            rows.append(row)
            np.savez(a.output/f'pair-{i}.npz',forward=forward,backward=backward,confidence=confidence)
            assert np.median(epe)<1.5, f'Wrong sign, scaling or alignment: {row}'
            assert np.linalg.norm(median_b-[-expected[i-1],0])<1.5, f'Backward motion mismatch: {row}'
            if held is not None:
                for name,value in held[1].items():
                    assert np.array_equal(held[0].getTensor(name),value), 'Previously returned '+name+' buffer was overwritten'
            held=(tensors,{name:tensors.getTensor(name).copy() for name in ['forward','backward','confidence']})
(a.output/'summary.json').write_text(json.dumps({'passed':True,'node':'OpticalFlow','depthai':dai.__version__,'pairs':rows},indent=2))
print(json.dumps(rows,indent=2))
