function uint8ArrayToVector(byteArray) {
  const bytes = new Module.Uint8Vector();
  byteArray.forEach((element) => {
    bytes.push_back(element);
  });
  return bytes;
}

function parseMessage(arrayBuffer) {
  if (arrayBuffer.length < 8) {
    throw Error();
  }
  const headerArray = new Uint8Array(
    arrayBuffer,
    arrayBuffer.byteLength - 8,
    8
  );
  const headerBytes = uint8ArrayToVector(headerArray);
  const header = Module.getMessageType(headerBytes);
  const dataLength =
    arrayBuffer.byteLength - // whole packet
    8 - // header
    header.serializedObjectSize; // metadata
  if (dataLength < 0) {
    throw Error(
      "Invalid data length: " +
        dataLength +
        " = " +
        arrayBuffer.byteLength +
        " - 8 - " +
        header.serializedObjectSize
    );
  }
  const data = new Uint8Array(arrayBuffer, 0, dataLength);
  const metadata = new Uint8Array(
    arrayBuffer,
    dataLength,
    header.serializedObjectSize
  );
  const metadataVector = uint8ArrayToVector(metadata);
  const msgType = header.objectType;
  var res;
  if (msgType === Module.DatatypeEnum.ImgFrame) {
    console.log("message type is ImgFrame");
    res = Module.deserializeImgFrame(metadataVector);
    console.log("deserializeImgFrame result: ", res);
    console.log("DIMS from javascript: ", res.width, "x", res.height);
  } else if (msgType === Module.DatatypeEnum.ImgDetections) {
    console.log("message type is ImgDetections");
    res = Module.deserializeImgDetections(metadataVector);
  } else {
    throw Error("Unhandled message type: " + msgType);
  }
  return res;
}

var Module = {
  onRuntimeInitialized: function () {
    console.log(Object.keys(Module));
    const req = new XMLHttpRequest();
    req.open("GET", "/img_frame.dump", true);
    req.responseType = "arraybuffer";
    req.onload = (event) => {
      const arrayBuffer = req.response;
      if (arrayBuffer) {
        const message = parseMessage(arrayBuffer);
        console.log(message);
      } else {
        console.error("Not an array buffer");
      }
    };
    req.send(null);
  },
};
