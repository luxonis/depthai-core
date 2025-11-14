#!/usr/bin/env python3
import depthai as dai
import signal
import sys
import time
import asyncio
import json
import websockets
import struct

uri = "ws://localhost:8765"
async def call_service(ws, service_name, service_id):
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

async def test_available_services():
    async with websockets.connect(uri, subprotocols=["foxglove.websocket.v1"]) as ws:
        while True:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=5.0))
            if msg.get("op") == "advertiseServices":
                services = {s['name']: s['id'] for s in msg.get("services", [])}
                print(f"Available services: {list(services.keys())}")
                break
        
        # Test each service
        for service_name in services:
            try:
                result = await call_service(ws, service_name, services[service_name])
                print(f"{service_name}: {result}")
            except Exception as e:
                print(f"{service_name}: ERROR - {e}")


isRunning = True
def stop():
    global isRunning
    isRunning = False

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

remoteConnection = dai.RemoteConnection()

def testService(input):
    print("Test service called with input:", input)
    return {"result": "testService result"}

remoteConnection.registerService("myService", testService)
while isRunning:
    time.sleep(1)
    asyncio.run(test_available_services())
