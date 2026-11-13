"""
Simulated conveyor station PLC, exposed over OPC UA.

Runs a 50 ms cycle like a PLC task: read the AMR's request bits and heartbeat,
advance the station's own sequence, write the output bits. Point
station_bridge at a real PLC's OPC UA endpoint instead and nothing on the ROS
side changes.
"""

import argparse
import asyncio
import os

from asyncua import Server, ua

NAMESPACE = 'urn:amr-station-handshake:station'
SCENARIOS = ('normal', 'never_ready', 'fault_during_transfer', 'crash_during_transfer')
CYCLE_S = 0.05

FAULT_CONVEYOR_BLOCKED = 1
FAULT_HEARTBEAT_LOST = 2
FAULT_AMR_LEFT = 3


async def run(args):
    server = Server()
    await server.init()
    endpoint = f'opc.tcp://{args.host}:{args.port}/station/'
    server.set_endpoint(endpoint)
    server.set_server_name('Simulated conveyor station PLC')
    idx = await server.register_namespace(NAMESPACE)
    station = await server.nodes.objects.add_object(ua.NodeId('Station', idx), 'Station')

    async def signal(name, value, varianttype):
        node = await station.add_variable(
            ua.NodeId(f'Station.{name}', idx), name, value, varianttype)
        await node.set_writable()
        return node

    boolean, int16 = ua.VariantType.Boolean, ua.VariantType.Int16
    dock_request = await signal('AmrDockRequest', False, boolean)
    in_position = await signal('AmrInPosition', False, boolean)
    heartbeat = await signal('AmrHeartbeat', 0, int16)
    ready = await signal('StationReady', False, boolean)
    transfer_active = await signal('TransferActive', False, boolean)
    transfer_done = await signal('TransferDone', False, boolean)
    fault = await signal('Fault', False, boolean)
    fault_code = await signal('FaultCode', 0, int16)

    loop = asyncio.get_running_loop()
    watchdog_s = args.watchdog_ms / 1000.0
    halfway = args.transfer_time / 2
    phase, code, due = 'idle', 0, 0.0
    last_beat, beat_seen = None, loop.time()

    print(f'station_sim: {endpoint} scenario={args.scenario}', flush=True)
    async with server:
        while True:
            request = await dock_request.read_value()
            docked = await in_position.read_value()
            beat = await heartbeat.read_value()
            now = loop.time()
            if beat != last_beat:
                last_beat, beat_seen = beat, now

            if not request:
                phase, code = 'idle', 0     # dropping the request also acknowledges a fault
            elif phase == 'fault':
                pass                        # latched until the request drops
            elif now - beat_seen > watchdog_s:
                phase, code = 'fault', FAULT_HEARTBEAT_LOST
            elif phase == 'idle':
                phase, due = 'preparing', now + args.ready_delay
            elif phase == 'preparing' and now >= due and args.scenario != 'never_ready':
                phase = 'ready'
            elif phase == 'ready' and docked:
                phase, due = 'transferring', now + args.transfer_time
            elif phase == 'transferring':
                if not docked:
                    phase, code = 'fault', FAULT_AMR_LEFT   # docs/SPEC.md, rule R10
                elif now >= due - halfway and args.scenario == 'fault_during_transfer':
                    phase, code = 'fault', FAULT_CONVEYOR_BLOCKED
                elif now >= due - halfway and args.scenario == 'crash_during_transfer':
                    print('station_sim: crashing mid-transfer', flush=True)
                    os._exit(1)
                elif now >= due:
                    phase = 'done'

            await ready.write_value(phase in ('ready', 'transferring', 'done'))
            await transfer_active.write_value(phase == 'transferring')
            await transfer_done.write_value(phase == 'done')
            await fault.write_value(phase == 'fault')
            await fault_code.write_value(code, int16)

            await asyncio.sleep(CYCLE_S)


def main():
    parser = argparse.ArgumentParser(description='Simulated conveyor station PLC over OPC UA.')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=4860)
    parser.add_argument('--scenario', choices=SCENARIOS, default='normal')
    parser.add_argument('--ready-delay', type=float, default=0.5)
    parser.add_argument('--transfer-time', type=float, default=2.0)
    parser.add_argument('--watchdog-ms', type=int, default=1000)
    # launch_ros appends --ros-args to every executable it starts.
    args, _ = parser.parse_known_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
