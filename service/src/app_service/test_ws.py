import asyncio
import websockets
import json

async def test_order_state():
    uri = "ws://localhost:8000/ws/cafe/order/state"
    async with websockets.connect(uri) as websocket:
        orders = {
            "orders": [
                {"OrderID": 1, "OrderStatus": 1},
                {"OrderID": 2, "OrderStatus": 0}
            ]
        }
        await websocket.send(json.dumps(orders))
        response = await websocket.recv()
        print("Server response:", response)

asyncio.run(test_order_state())
