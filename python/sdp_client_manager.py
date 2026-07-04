# -*- coding: utf-8 -*-
"""
sdp_client_manager.py

Centralized lifecycle + thread-safety for SLAMWARE SDP clients (MyClient /
msl.loadlib Client64).

Background
---------
Each MyClient spawns a full 32-bit server process and its own TCP connection to
the robot, and is NOT thread-safe: request32 reuses a single HTTPConnection and a
single temp pickle file. Two threads using the same client concurrently produce
http.client.CannotSendRequest('Request-sent') (and pickle corruption), which the
retry decorator in my_sdp_client silently swallows with a bare print() and then
returns a wrong default value.

This module provides:
  * reader()        - ONE lock-guarded client shared by all quick read-only calls
                      (pose/heading/battery/...). Serializing reads is safe and
                      collapses several 32-bit processes into one.
  * dedicated(label)- a per-thread client for command/navigation threads that
                      issue moves + the blocking waitUntilMoveActionDone. These
                      MUST stay isolated per thread; a thread-affinity tripwire
                      raises loudly (SdpClientMisuseError) if one is used from a
                      thread other than the one that created it.

The tripwire error derives from BaseException on purpose so the Exception-only
retry decorator in my_sdp_client cannot catch and hide it.
"""

import threading

from msl.loadlib import Client64
from my_sdp_client import MyClient
import sdp_comm


class SdpClientMisuseError(BaseException):
    """Raised when a dedicated SDP client is used from a thread other than its owner.

    Derives from BaseException (not Exception) so the `except (Exception,)` retry
    decorator in my_sdp_client.py cannot swallow it -- the whole point is that this
    surfaces loudly and points at the exact call site that needs its own client.
    """
    pass


class GuardedClient(MyClient):
    """A MyClient bound to the thread that created it.

    Every request32 asserts it is called from the owner thread. A single-owner
    thread cannot collide with itself, so this one check catches the cross-thread
    reuse that was producing the mystery "Request-sent x3" warnings.
    """

    def __init__(self, label):
        # Set guard state BEFORE super().__init__(): Client64.__init__ issues a
        # request32(METADATA) during construction, which must pass the guard.
        self._label = label
        self._owner_ident = threading.get_ident()
        self._owner_name = threading.current_thread().name
        super(GuardedClient, self).__init__()

    def request32(self, name, *args, **kwargs):
        cur = threading.current_thread()
        if cur.ident != self._owner_ident:
            raise SdpClientMisuseError(
                "SDP client '{label}' is owned by thread '{owner}' (id={oid}) but "
                "'{name}' was called from thread '{caller}' (id={cid}). This thread "
                "needs its own client: create one with "
                "SdpClientManager.dedicated('<name>') at thread start, or use "
                "SdpClientManager.reader() for read-only calls.".format(
                    label=self._label, owner=self._owner_name, oid=self._owner_ident,
                    name=name, caller=cur.name, cid=cur.ident,
                )
            )
        return super(GuardedClient, self).request32(name, *args, **kwargs)


class SharedReaderClient(MyClient):
    """A single MyClient shared by all threads for quick read-only calls.

    request32 is serialized with a blocking lock. This is a correctness mechanism,
    not a tripwire: concurrent reads from many threads are legal and simply queue.
    By convention only read-only methods (pose/heading/battery/...) are called on
    this client; long-blocking calls (waitUntilMoveActionDone) and mutating
    commands must go through a dedicated() client instead.
    """

    def __init__(self):
        # Create the lock BEFORE super().__init__(): Client64.__init__ issues a
        # request32(METADATA) during construction, which takes the lock.
        self._lock = threading.Lock()
        super(SharedReaderClient, self).__init__()

    def request32(self, name, *args, **kwargs):
        with self._lock:
            return super(SharedReaderClient, self).request32(name, *args, **kwargs)


class SdpClientManager(object):
    """Owns SDP client lifecycle: the shared reader plus labeled dedicated clients."""

    def __init__(self):
        self._lock = threading.Lock()
        self._reader = None
        self._dedicated = {}  # id(client) -> GuardedClient (label is client._label)

    # -- shared read-only client ------------------------------------------------

    def reader(self):
        """Return the single shared, lock-guarded read-only client (lazily created)."""
        with self._lock:
            if self._reader is None:
                r = SharedReaderClient()
                sdp_comm.connectToSdp(r)
                self._reader = r
            return self._reader

    # -- dedicated per-thread command clients -----------------------------------

    def dedicated(self, label, connect=True):
        """Create, (optionally) connect, track and return a thread-affine command client.

        Call this from the thread that will use the client (its owner is bound to
        the calling thread). `label` identifies the client in tracking/errors; it
        need not be unique (e.g. come_here may run concurrently), so release by the
        returned instance. Pass connect=False to spawn the client now but defer
        sdp_comm.connectToSdp to the caller (used by run(), which connects
        conditionally based on no_move / _slamtec_on).
        """
        client = GuardedClient(label)
        if connect:
            sdp_comm.connectToSdp(client)
        with self._lock:
            self._dedicated[id(client)] = client
        return client

    def unguarded(self, label, connect=True):
        """Create, connect and track a PLAIN (non-thread-affine) client.

        Use for a client legitimately driven by a *succession* of short-lived
        threads (one at a time), not a single fixed owner -- e.g. the langgraph
        tool client, invoked from a fresh per-request `_stream_thread`. A
        GuardedClient would false-trip on the second such thread, so no tripwire
        is applied here; the caller guarantees single-threaded-at-a-time use
        (langgraph's `is_processing` gate). Tracked so shutdown_all()/active()
        include it and it is not leaked on shutdown.
        """
        client = MyClient()
        client._label = label
        client._owner_name = '<unguarded>'
        if connect:
            sdp_comm.connectToSdp(client)
        with self._lock:
            self._dedicated[id(client)] = client
        return client

    def release(self, label_or_client):
        """Disconnect, shut down and untrack dedicated client(s).

        Pass the client instance (preferred, unambiguous) to release exactly that
        one, or a label string to release every live client with that label.
        """
        with self._lock:
            if isinstance(label_or_client, str):
                clients = [c for c in self._dedicated.values()
                           if c._label == label_or_client]
            else:
                clients = [label_or_client] if id(label_or_client) in self._dedicated else []
            for c in clients:
                self._dedicated.pop(id(c), None)
        for c in clients:
            _shutdown_client(c)

    def shutdown_all(self):
        """Tear down every tracked dedicated client and the shared reader."""
        with self._lock:
            clients = list(self._dedicated.values())
            self._dedicated.clear()
            reader = self._reader
            self._reader = None
        for c in clients:
            _shutdown_client(c)
        if reader is not None:
            _shutdown_client(reader)

    def active(self):
        """Return [(label, owner_thread_name, alive)] for the live 32-bit processes."""
        with self._lock:
            out = [(c._label, c._owner_name, _is_alive(c))
                   for c in self._dedicated.values()]
            if self._reader is not None:
                out.append(('<reader>', 'shared', _is_alive(self._reader)))
        return out


def _shutdown_client(client):
    # Bypass the thread-affinity guard: shutdown_all() runs on the main thread but
    # a client may be owned by another (already-finished) thread. Call the base
    # Client64.request32 directly so disconnect() doesn't trip the tripwire.
    try:
        Client64.request32(client, 'disconnect')
        client.connected = False
    except Exception:
        pass
    try:
        # shutdown_server32 posts SHUTDOWN over the socket directly (no request32),
        # so it does not hit the guard.
        client.shutdown_server32(kill_timeout=1)
    except Exception:
        pass


def _is_alive(client):
    proc = getattr(client, '_proc', None)
    return bool(proc is not None and proc.poll() is None)


# Module-level singleton -- import and use `manager` everywhere.
manager = SdpClientManager()
