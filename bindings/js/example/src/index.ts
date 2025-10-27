import depthaiInitialize, { parseMessage, arrayToVector } from "depthai-js";

(async () => {
  const dai = await depthaiInitialize();
  console.log(Object.keys(dai));
  const req = new XMLHttpRequest();
  req.open("GET", "/img_frame.dump", true);
  req.responseType = "arraybuffer";
  req.onload = () => {
    const arrayBuffer = req.response;
    if (arrayBuffer) {
      const message = parseMessage(arrayBuffer);
      console.log(message);
      const msgType = message.type;
      const vectorMetadata = arrayToVector(message.metadata);
      let res;
      if (msgType === dai.DatatypeEnum.ImgFrame) {
        console.log("message type is ImgFrame");
        res = dai.deserializeImgFrame(vectorMetadata);
        console.log("deserializeImgFrame result: ", res);
        console.log("DIMS from javascript: ", res.width, "x", res.height);
      } else if (msgType === dai.DatatypeEnum.ImgDetections) {
        console.log("message type is ImgDetections");
        res = dai.deserializeImgDetections(vectorMetadata);
      } else {
        throw Error("Unhandled message type: " + msgType);
      }
      console.log(res);
    } else {
      console.error("Not an array buffer");
    }
  };
  req.send(null);
})();
