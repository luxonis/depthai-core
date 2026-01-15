#!/usr/bin/env python3
import depthai as dai
import asyncio
import json
import websockets
import struct

uri = "ws://localhost:8765"
async def call_service(ws, service_id):
    # Call a service and return the response payload
    msg = bytearray([0x02])  # opcode
    msg.extend(struct.pack('<I', service_id))
    msg.extend(struct.pack('<I', 1))  # call_id
    msg.extend(struct.pack('<I', 4))  # encoding length
    msg.extend(b"json")
    msg.extend(b"{}")
    await ws.send(bytes(msg))
    
    response = await asyncio.wait_for(ws.recv(), timeout=5.0)
    if isinstance(response, bytes) and response[0] == 0x03:
        encoding_len = struct.unpack('<I', response[9:13])[0]
        payload = response[13 + encoding_len:]
        return payload.decode('utf-8')
    return None


async def call_binary_service(ws, service_id):
    # Call a service and return the response payload
    msg = bytearray([0x02])  # opcode
    msg.extend(struct.pack('<I', service_id))
    msg.extend(struct.pack('<I', 1))  # call_id
    encoding = b"binary"
    msg.extend(struct.pack('<I', len(encoding)))  # encoding length
    msg.extend(encoding)
    payload = b"\x01\x02\x03\x04"
    msg.extend(payload)
    await ws.send(bytes(msg))
    
    response = await asyncio.wait_for(ws.recv(), timeout=5.0)
    if isinstance(response, bytes) and response[0] == 0x03:
        # Format: [opcode(1)][serviceId(4)][callId(4)][encodingLength(4)][encoding][data]
        encoding_len = struct.unpack('<I', response[9:13])[0]
        payload = response[13 + encoding_len:]
        return payload
    return None


async def get_available_services():
    async with websockets.connect(uri, subprotocols=["foxglove.websocket.v1"]) as ws:
        while True:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            if msg.get("op") == "advertiseServices":
                services = msg.get("services", [])
                print(f"Available services: {[s['name'] for s in services]}")
                break
        
        # Test each service
        results = {}
        for service in services:
            try:
                encoding = service['type']
                result = await call_service(ws, service['id']) if encoding == 'json' else await call_binary_service(ws, service['id'])
                results[service['name']] = result
                print(f"{service['name']}: {result}")
            except Exception as e:
                print(f"{service['name']}: ERROR - {e}")

        return results


def custom_service(input):
    return {"result": "custom_service_result"}

def custom_binary_service(input_data):
    # Input data is in bytes
    return input_data[::-1]


def test_remote_connection_services():
    remoteConnection = dai.RemoteConnection()
    remoteConnection.registerService("custom_service", custom_service)
    remoteConnection.registerBinaryService("custom_binary_service", custom_binary_service)

    results = asyncio.run(get_available_services())
    assert(json.loads(results["custom_service"]) == {"result": "custom_service_result"})
    assert(results["libraryVersion"] in dai.__version__)
    assert(results["topicGroups"] == "{}")
    assert(results["keyPressed"] == None)
    assert(results["custom_binary_service"] == b"\x04\x03\x02\x01")


if __name__ == '__main__':
  test_remote_connection_services()