# -*- coding: utf-8 -*-
"""
Offline unit tests for sdp_client_manager.

These do NOT spawn a 32-bit server / require the robot:
  * the cross-thread tripwire raises before any IPC, so no server is needed;
  * the owner-thread and reader-serialization paths mock the base
    Client64.request32 so the guard/lock logic is exercised in isolation.

Run: python -m unittest tests.test_sdp_client_manager   (from the python/ dir)
"""

import os
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from msl.loadlib import Client64
from sdp_client_manager import (
    GuardedClient, SharedReaderClient, SdpClientMisuseError,
)


def _make(cls):
    """Build an instance without running __init__ (which would spawn a server)."""
    return object.__new__(cls)


class TripwireTest(unittest.TestCase):

    def test_misuse_error_bypasses_exception_handlers(self):
        # BaseException-derived on purpose: the retry decorator in my_sdp_client
        # catches (Exception,), so it must NOT be able to swallow this.
        self.assertTrue(issubclass(SdpClientMisuseError, BaseException))
        self.assertFalse(issubclass(SdpClientMisuseError, Exception))

    def test_cross_thread_use_raises(self):
        c = _make(GuardedClient)
        c._label = 'test'
        c._owner_ident = threading.get_ident()
        c._owner_name = threading.current_thread().name

        box = {}

        def worker():
            try:
                c.request32('pose')          # different thread than owner
            except SdpClientMisuseError:
                box['misuse'] = True
            except BaseException as e:       # noqa: BLE001 - want to see anything else
                box['other'] = repr(e)

        t = threading.Thread(target=worker, name='Worker')
        t.start()
        t.join()
        self.assertTrue(box.get('misuse'), box)

    def test_owner_thread_passes_through(self):
        c = _make(GuardedClient)
        c._label = 'test'
        c._owner_ident = threading.get_ident()
        c._owner_name = threading.current_thread().name
        with mock.patch.object(Client64, 'request32', return_value='ok') as m:
            self.assertEqual(c.request32('pose'), 'ok')
            m.assert_called_once()


class ReaderSerializationTest(unittest.TestCase):

    def test_concurrent_reads_are_serialized(self):
        r = _make(SharedReaderClient)
        r._lock = threading.Lock()
        state = {'inside': 0, 'max_concurrent': 0}

        def fake_base(self, name, *a, **k):
            state['inside'] += 1
            state['max_concurrent'] = max(state['max_concurrent'], state['inside'])
            time.sleep(0.02)
            state['inside'] -= 1
            return name

        with mock.patch.object(Client64, 'request32', new=fake_base):
            threads = [threading.Thread(target=r.request32, args=('pose',))
                       for _ in range(8)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()

        # The blocking lock must ensure at most one call is inside at any time.
        self.assertEqual(state['max_concurrent'], 1)


if __name__ == '__main__':
    unittest.main()
