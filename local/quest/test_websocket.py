"""Real local websocket disconnect/reconnect without a robot or controller."""
import time
import unittest
from aiohttp import web
from aiohttp.test_utils import TestServer, TestClient
from bridge import Bridge, Mapping, instantiate_g1_robot_model
from test_bridge import Publisher, frame, refs


class RecoveryTests(unittest.IsolatedAsyncioTestCase):
    async def test_disconnect_stands_and_reconnect_requires_explicit_toggle(self):
        b=Bridge(Publisher(),instantiate_g1_robot_model())
        b.receive(frame()); b.mapping=Mapping(b.latest[0],refs())
        b.active=b.standing=True
        b.lowstate=[0]*29; b.lowstate_at=time.monotonic()
        app=web.Application(); app.router.add_get('/ws',b.websocket)
        async with TestClient(TestServer(app)) as client:
            origin='https://'+str(client.make_url('/')).split('//')[1].rstrip('/')
            ws=await client.ws_connect('/ws',headers={'Origin':origin})
            self.assertTrue((await ws.receive_json())['active'])
            await ws.close()
            # Wait for the server's finally block, not an arbitrary sleep.
            import asyncio
            for _ in range(100):
                if b.ws is None: break
                await asyncio.sleep(.001)
            self.assertIsNone(b.ws); self.assertFalse(b.active); self.assertTrue(b.standing)
            self.assertIsNone(b.mapping)
            ws=await client.ws_connect('/ws',headers={'Origin':origin})
            state=await ws.receive_json()
            self.assertFalse(state['active']); self.assertFalse(state['start_pending'])
            data=frame(); data.update(input_mode='controllers',locomotion='sticks',drive=[0,0,0])
            await ws.send_json(data)
            self.assertFalse((await ws.receive_json())['active'])
            await ws.send_json({'type':'command','command':'toggle'})
            state=await ws.receive_json()
            self.assertTrue(state['start_pending']); self.assertFalse(state['active'])
            await ws.close()
            self.assertFalse(any(m.startswith(b'command') and m[-3:]==bytes([0,1,1]) for m in b.pub.messages))
