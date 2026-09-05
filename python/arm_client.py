#!/usr/bin/env python3
# coding: utf-8
"""
DOFBOT Arm XML-RPC Client.

Talks to arm_server.py. Import ArmClient to drive the arm from other code, or
run this file for a command line:

    python3 arm_client.py status
    python3 arm_client.py enable
    python3 arm_client.py state ready
    python3 arm_client.py pick 0.30 0.0 0.039
    python3 arm_client.py place
    python3 arm_client.py held               # is the can still in the jaws?
    python3 arm_client.py wave 3
    python3 arm_client.py reset              # after a pick that failed partway
    python3 arm_client.py disable --park init
    python3 arm_client.py -i             # interactive

The server address comes from --url, then $DOFBOT_ARM_URL, then the default
below (the Jetson over the USB-device network link).

Two proxies, not one: motion calls block for as long as the move takes, while
ping/status/stop use a short timeout so they still answer -- and can still
interrupt -- while a pick is in flight.
"""

import argparse
import http.client
import os
import sys
import xmlrpc.client
from typing import Any, Dict, List, Optional

DEFAULT_SERVER_URL = os.environ.get('DOFBOT_ARM_URL',
                                    'http://192.168.55.1:8001/')

# Seconds. Long enough for a whole pick; short enough that a server which died
# mid-command does not hang the caller forever.
MOTION_TIMEOUT = 300
QUERY_TIMEOUT = 10


class _TimeoutTransport(xmlrpc.client.Transport):
    """xmlrpc.client has no timeout knob; this is the documented way in."""

    def __init__(self, timeout):
        super().__init__()
        self._timeout = timeout

    def make_connection(self, host):
        if self._connection and host == self._connection[0]:
            return self._connection[1]
        chost, self._extra_headers, _x509 = self.get_host_info(host)
        self._connection = (host, http.client.HTTPConnection(
            chost, timeout=self._timeout))
        return self._connection[1]


def _proxy(url, timeout):
    return xmlrpc.client.ServerProxy(url, allow_none=True,
                                     transport=_TimeoutTransport(timeout))


def _failed(command: str, error: str) -> Dict[str, Any]:
    """A server-shaped result for something that never reached the server, so
    callers only ever have one dict shape to handle."""
    return {'ok': False, 'command': command, 'returncode': -1, 'output': '',
            'error': error, 'seconds': 0.0}


class ArmClient:
    """Client for the DOFBOT arm server.

    Every motion method returns the server's result dict:
        {'ok', 'command', 'returncode', 'output', 'error', 'seconds'}
    `ok` is the only field worth branching on; `output` is the ros2 command's
    console output, which is where a MoveIt failure explains itself.
    """

    def __init__(self, server_url: str = DEFAULT_SERVER_URL,
                 motion_timeout: int = MOTION_TIMEOUT,
                 query_timeout: int = QUERY_TIMEOUT):
        self.server_url = server_url
        self._motion = _proxy(server_url, motion_timeout)
        self._query = _proxy(server_url, query_timeout)
        self._connected = False

    # ------------------------------------------------------------ connection

    def connect(self) -> bool:
        try:
            self._query.ping()
            self._connected = True
            print('Connected to arm server at %s' % self.server_url)
            return True
        except Exception as exc:
            self._connected = False
            print('Error: could not reach the arm server at %s: %s'
                  % (self.server_url, exc))
            return False

    def is_connected(self) -> bool:
        return self._connected

    def disconnect(self):
        self._connected = False

    def _call(self, proxy, name: str, *args) -> Dict[str, Any]:
        try:
            return dict(getattr(proxy, name)(*args))
        except Exception as exc:
            self._connected = False
            return _failed(name, '%s: %s' % (type(exc).__name__, exc))

    # -------------------------------------------------------------- commands

    def enable_arm(self, timeout: int = 90, bridge: bool = True,
                   rviz: bool = False, port: str = '') -> Dict[str, Any]:
        """Start the ROS stack. Blocks until the nodes are up or it gives up."""
        return self._call(self._motion, 'enable_arm', timeout, bridge, rviz, port)

    def disable_arm(self, park: str = '') -> Dict[str, Any]:
        """Stop the ROS stack, optionally moving to `park` first."""
        return self._call(self._motion, 'disable_arm', park)

    def pick_can(self, x: float, y: float, z: float,
                 object: str = '') -> Dict[str, Any]:
        """Pick the object whose CENTRE is at (x, y, z) in base_link, and carry
        it. Does not place -- call place_can() for that."""
        return self._call(self._motion, 'pick_can', float(x), float(y),
                          float(z), object)

    def place_can(self) -> Dict[str, Any]:
        """Place whatever the gripper is carrying."""
        return self._call(self._motion, 'place_can')

    def is_holding(self, object: str = '') -> Dict[str, Any]:
        """Look through the wrist camera: is the object still in the jaws?

        Read `held`, and read it as THREE states -- True seen, False looked and
        it was not there, None the question could not be asked (no detector, a
        dark frame, an uncalibrated view). `ok` says only that the question got
        asked, so `ok` with `held` False is a working camera reporting an empty
        gripper.

        Means what it says at 'carry', which is where a pick leaves the arm.
        """
        return self._call(self._motion, 'is_holding', object)

    def move_to_state(self, name: str) -> Dict[str, Any]:
        """Move to a saved state -- see list_states()."""
        return self._call(self._motion, 'move_to_state', str(name))

    def reset_arm(self, state: str = 'ready',
                  force: bool = False) -> Dict[str, Any]:
        """Recover: clear the planning scene, open the gripper, go to `state`.

        What to reach for when a move fails instantly -- an aborted pick leaves
        the object in the scene and nothing can be planned out of a start state
        that is inside it. `force` drives out blind if MoveIt still will not
        plan; that move is NOT collision checked.
        """
        return self._call(self._motion, 'reset_arm', str(state), bool(force))

    def wave_arm(self, waves: int = 1, finish: str = '',
                 seconds: float = 0.0) -> Dict[str, Any]:
        """Wave hello, then stow the arm.

        A greeting gesture, planned and collision-checked like any other move,
        so the stack has to be enabled. `seconds` is how long ONE wave takes
        (default 3), so the pace holds whatever `waves` says.
        """
        return self._call(self._motion, 'wave_arm', int(waves), str(finish),
                          float(seconds))

    # --------------------------------------------------------------- queries

    def list_states(self) -> List[str]:
        try:
            return list(self._query.list_states())
        except Exception as exc:
            print('List states error: %s' % exc)
            return []

    def get_status(self) -> Optional[dict]:
        try:
            return dict(self._query.get_status())
        except Exception as exc:
            self._connected = False
            print('Get status error: %s' % exc)
            return None

    def tail_log(self, lines: int = 40) -> str:
        try:
            return str(self._query.tail_log(int(lines)))
        except Exception as exc:
            return 'Tail log error: %s' % exc

    def stop(self) -> Dict[str, Any]:
        """Abort the motion in flight. Safe to call from another thread while a
        pick is blocking -- that is what the short-timeout proxy is for."""
        return self._call(self._query, 'stop')

    def ping(self) -> Optional[str]:
        try:
            result = str(self._query.ping())
            self._connected = True
            return result
        except Exception as exc:
            self._connected = False
            print('Ping error: %s' % exc)
            return None


# ------------------------------------------------------------------ printing


def show(result: Dict[str, Any], verbose: bool = False) -> bool:
    """Print a result dict the way a person wants to read it."""
    mark = 'OK ' if result.get('ok') else 'FAIL'
    line = '%s %s' % (mark, result.get('command') or '?')
    if result.get('seconds'):
        line += ' (%.1fs)' % result['seconds']
    print(line)
    if result.get('error'):
        print('     %s' % result['error'])
    output = result.get('output') or ''
    if output and (verbose or not result.get('ok')):
        print('     --- output ---')
        for out_line in output.strip().splitlines():
            print('     %s' % out_line)
    return bool(result.get('ok'))


def show_held(result: Dict[str, Any]) -> bool:
    """Print an is_holding() result. Three answers, printed as three."""
    held = result.get('held')
    word = {True: 'HELD', False: 'EMPTY'}.get(held, 'UNKNOWN')
    print('%-8s %s' % (word, result.get('reason') or result.get('error') or ''))
    # Only the unknowns need explaining further; held and empty have said it.
    if held is None and not result.get('ok'):
        show(result)
    return bool(result.get('ok'))


def show_status(status: Optional[dict]):
    if status is None:
        return
    print('arm enabled : %s%s'
          % (status['arm_enabled'],
             ' (pid %d, up %.0fs)' % (status['launch_pid'],
                                      status['launch_uptime'])
             if status['arm_enabled'] else ''))
    print('busy        : %s' % (('%s, %.0fs' % (status['running'],
                                                status['running_for']))
                                if status['busy'] else 'no'))
    print('states      : %s' % ', '.join(status.get('states') or ['(unknown)']))
    print('workspace   : %s (ROS %s)' % (status['workspace'],
                                         status['ros_distro']))
    print('launch log  : %s' % status['launch_log'])
    last = status.get('last')
    if last:
        print('last command: %s %s%s'
              % (last['command'], 'ok' if last['ok'] else 'FAILED',
                 '' if last['ok'] else ' -- %s' % last['error']))


# ------------------------------------------------------------ interactive CLI


INTERACTIVE_HELP = """Commands:
  enable [--sim]        start the ROS stack (--sim: no servo bridge)
  disable [state]       stop the stack, optionally parking at `state` first
  pick X Y Z [object]   pick the object centred at X Y Z and carry it
  place                 place what the gripper is carrying
  state NAME            move to a saved state
  reset [state] [force] clear the scene, let go, go home (after a failed pick)
  wave [N]              wave hello N times, then stow
  held [object]         look: is the object still in the jaws?
  states                list the saved states
  status                enabled? busy? what ran last?
  log [N]               tail the launch log
  stop                  abort the motion in flight
  ping                  check the server
  help, quit"""


def interactive(client: ArmClient):
    print('DOFBOT Arm interactive client')
    print('Server: %s' % client.server_url)
    print(INTERACTIVE_HELP)
    print()
    if not client.connect():
        return 1

    while True:
        try:
            parts = input('>>> ').strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not parts:
            continue
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd in ('quit', 'exit'):
                return 0
            elif cmd == 'help':
                print(INTERACTIVE_HELP)
            elif cmd == 'ping':
                print(client.ping())
            elif cmd == 'status':
                show_status(client.get_status())
            elif cmd == 'states':
                print(', '.join(client.list_states()) or '(none)')
            elif cmd == 'log':
                print(client.tail_log(int(args[0]) if args else 40))
            elif cmd == 'stop':
                show(client.stop())
            elif cmd == 'held':
                show_held(client.is_holding(args[0] if args else ''))
            elif cmd == 'enable':
                print('Starting the stack, this takes a few seconds...')
                show(client.enable_arm(bridge='--sim' not in args))
            elif cmd == 'disable':
                show(client.disable_arm(args[0] if args else ''))
            elif cmd == 'place':
                show(client.place_can())
            elif cmd == 'wave':
                show(client.wave_arm(int(args[0]) if args else 1))
            elif cmd == 'reset':
                state = args[0] if args and args[0] != 'force' else 'ready'
                show(client.reset_arm(state, 'force' in args))
            elif cmd == 'state':
                if len(args) != 1:
                    print('Usage: state NAME (%s)'
                          % ', '.join(client.list_states()))
                else:
                    show(client.move_to_state(args[0]))
            elif cmd == 'pick':
                if len(args) not in (3, 4):
                    print('Usage: pick X Y Z [object]   '
                          '(metres, base_link, object CENTRE)')
                else:
                    show(client.pick_can(float(args[0]), float(args[1]),
                                         float(args[2]),
                                         args[3] if len(args) == 4 else ''))
            else:
                print('Unknown command: %s' % cmd)
        except KeyboardInterrupt:
            # Ctrl-C during a blocking move: tell the server to abort, so the
            # arm does not carry on executing a command nobody is waiting for.
            print('\nInterrupted -- asking the server to stop...')
            show(client.stop())
        except ValueError as exc:
            print('Bad argument: %s' % exc)


def build_parser():
    parser = argparse.ArgumentParser(
        prog='arm_client', description=__doc__.split('\n\n')[0])
    parser.add_argument('--url', default=DEFAULT_SERVER_URL,
                        help='server address (default: %(default)s)')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='print command output even when it succeeds')
    parser.add_argument('-i', '--interactive', action='store_true',
                        help='interactive prompt')
    sub = parser.add_subparsers(dest='cmd')

    enable = sub.add_parser('enable', help='start the ROS stack')
    enable.add_argument('--sim', action='store_true',
                        help='simulation only -- no moveit_bridge, no servos')
    enable.add_argument('--rviz', action='store_true',
                        help='also start RViz on the robot')
    enable.add_argument('--timeout', type=int, default=90)

    disable = sub.add_parser('disable', help='stop the ROS stack')
    disable.add_argument('--park', default='',
                         help='move to this state before shutting down')

    pick = sub.add_parser('pick', help='pick an object and carry it')
    pick.add_argument('x', type=float, help='object CENTRE x in base_link, m')
    pick.add_argument('y', type=float)
    pick.add_argument('z', type=float)
    pick.add_argument('--object', default='', help='catalogue entry')

    sub.add_parser('place', help='place what the gripper is carrying')

    state = sub.add_parser('state', help='move to a saved state')
    state.add_argument('name')

    reset = sub.add_parser('reset', help='recover after a failed pick')
    reset.add_argument('state', nargs='?', default='ready',
                       help='where to leave the arm (default: %(default)s)')
    reset.add_argument('--force', action='store_true',
                       help='drive out blind if MoveIt will not plan from '
                            'where the arm is -- NOT collision checked')

    wave = sub.add_parser('wave', help='wave hello, then stow')
    wave.add_argument('waves', nargs='?', type=int, default=1,
                      help='back-and-forth swings (default: %(default)s)')
    wave.add_argument('--finish', default='',
                      help='state to stow at afterwards (default: init)')
    wave.add_argument('--seconds', type=float, default=0.0,
                      help='how long ONE wave takes (default: 3)')

    held = sub.add_parser('held',
                          help='look: is the object still in the jaws?')
    held.add_argument('object', nargs='?', default='',
                      help='catalogue entry to look for')

    sub.add_parser('states', help='list the saved states')
    sub.add_parser('status', help='server and stack status')
    sub.add_parser('stop', help='abort the motion in flight')
    sub.add_parser('ping', help='check the server')

    log = sub.add_parser('log', help='tail the launch log')
    log.add_argument('lines', nargs='?', type=int, default=40)
    return parser


def main(argv=None):
    parser = build_parser()
    cli = parser.parse_args(argv)
    client = ArmClient(cli.url)

    if cli.interactive:
        return interactive(client)
    if cli.cmd is None:
        if not client.connect():
            return 1
        show_status(client.get_status())
        return 0

    if cli.cmd == 'enable':
        return 0 if show(client.enable_arm(timeout=cli.timeout,
                                           bridge=not cli.sim,
                                           rviz=cli.rviz), cli.verbose) else 1
    if cli.cmd == 'disable':
        return 0 if show(client.disable_arm(cli.park), cli.verbose) else 1
    if cli.cmd == 'pick':
        return 0 if show(client.pick_can(cli.x, cli.y, cli.z, cli.object),
                         cli.verbose) else 1
    if cli.cmd == 'place':
        return 0 if show(client.place_can(), cli.verbose) else 1
    if cli.cmd == 'state':
        return 0 if show(client.move_to_state(cli.name), cli.verbose) else 1
    if cli.cmd == 'reset':
        return 0 if show(client.reset_arm(cli.state, cli.force),
                         cli.verbose) else 1
    if cli.cmd == 'wave':
        return 0 if show(client.wave_arm(cli.waves, cli.finish, cli.seconds),
                         cli.verbose) else 1
    if cli.cmd == 'held':
        return 0 if show_held(client.is_holding(cli.object)) else 1
    if cli.cmd == 'stop':
        return 0 if show(client.stop(), cli.verbose) else 1
    if cli.cmd == 'states':
        states = client.list_states()
        print('\n'.join(states) if states else '(none)')
        return 0 if states else 1
    if cli.cmd == 'log':
        print(client.tail_log(cli.lines))
        return 0
    if cli.cmd == 'ping':
        reply = client.ping()
        print(reply or 'no reply')
        return 0 if reply == 'pong' else 1

    status = client.get_status()
    show_status(status)
    return 0 if status else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nInterrupted. The server may still be running the command; '
              '`arm_client.py stop` aborts it.')
        sys.exit(130)
