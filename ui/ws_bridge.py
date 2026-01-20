#!/usr/bin/env python3
"""
WebSocket bridge: TCP(9000) -> WebSocket(9001)
Forwards ridersense JSON messages to browser clients.
"""

import asyncio
import signal
import sys

try:
    import websockets
except ImportError:
    print("Install websockets: pip install websockets")
    sys.exit(1)

TCP_HOST = "127.0.0.1"
TCP_PORT = 9000
WS_PORT = 9001

clients = set()
running = True


async def tcp_reader():
    """Connect to ridersense TCP server and forward messages."""
    global running
    
    while running:
        try:
            reader, writer = await asyncio.open_connection(TCP_HOST, TCP_PORT)
            print(f"[bridge] connected to ridersense on {TCP_HOST}:{TCP_PORT}")
            
            while running:
                line = await reader.readline()
                if not line:
                    break
                
                message = line.decode().strip()
                if message and clients:
                    await broadcast(message)
            
            writer.close()
            await writer.wait_closed()
            
        except ConnectionRefusedError:
            print("[bridge] ridersense not running, retrying in 2s...")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"[bridge] TCP error: {e}, reconnecting...")
            await asyncio.sleep(1)


async def broadcast(message):
    """Send message to all connected WebSocket clients."""
    if clients:
        await asyncio.gather(
            *[client.send(message) for client in clients],
            return_exceptions=True
        )


async def ws_handler(websocket):
    """Handle WebSocket client connection."""
    clients.add(websocket)
    addr = websocket.remote_address
    print(f"[bridge] browser connected: {addr}")
    
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"[bridge] browser disconnected: {addr}")


async def main():
    global running
    
    loop = asyncio.get_event_loop()
    
    def shutdown():
        global running
        running = False
        print("\n[bridge] shutting down...")
    
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, shutdown)
    
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        print(f"[bridge] WebSocket server on ws://localhost:{WS_PORT}")
        
        tcp_task = asyncio.create_task(tcp_reader())
        
        while running:
            await asyncio.sleep(0.1)
        
        tcp_task.cancel()


if __name__ == "__main__":
    print("[bridge] ridersense WebSocket bridge")
    asyncio.run(main())
