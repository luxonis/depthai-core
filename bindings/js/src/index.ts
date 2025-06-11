import depthaiJsModuleInitialize, {
  DatatypeEnum,
  Uint8Vector,
  MainModule,
} from "./depthai-js.js";

export * from "./depthai-js.js";

interface ParsedMessage {
  type: DatatypeEnum;
  metadata: Uint8Array;
  data: Uint8Array;
}

export type Dai = MainModule;

let dai: { value: Dai | null } = { value: null };
async function initialize(): Promise<Dai> {
  const depthai = (await (depthaiJsModuleInitialize as any)()) as Dai;
  dai.value = depthai;
  return depthai;
}

export function arrayToVector(byteArray: Uint8Array): Uint8Vector {
  if (dai.value === null) {
    throw Error();
  }
  const bytes = new dai.value.Uint8Vector();
  byteArray.forEach((element) => {
    bytes.push_back(element);
  });
  return bytes;
}

export function parseMessage(arrayBuffer: ArrayBuffer): ParsedMessage {
  if (dai.value === null) {
    throw Error();
  }
  if (arrayBuffer.byteLength < 8) {
    throw Error();
  }
  const headerArray = new Uint8Array(
    arrayBuffer,
    arrayBuffer.byteLength - 8,
    8
  );
  const headerBytes = arrayToVector(headerArray);
  const header = dai.value.getMessageType(headerBytes);
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
  return {
    type: header.objectType,
    data,
    metadata,
  };
}

export default initialize;
