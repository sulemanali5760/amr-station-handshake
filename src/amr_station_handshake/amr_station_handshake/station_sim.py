"""Simulated conveyor station PLC, exposed over OPC UA.

Runs a 50 ms cycle like a PLC task: read the AMR's request bits, advance the
station's own sequence, write the output bits. Point station_bridge at a real
PLC's OPC UA endpoint instead and nothing on the ROS side changes.
"""

import argparse
import asyncio

from asyncua import Server, ua

NAMESPACE = 'urn:amr-station-handshake:station'
SCENARIOS = ('normal', 'never_ready', 'fault_during_transfer')
CYCLE_S = 0.05


async def run(host: str, port: int, scenario: str, ready_delay: float, transfer_time: float):
    server = Server()
    await server.init()
    endpoint = f'opc.tcp://{host}:{port}/station/'
    server.set_endpoint(endpoint)
    server.set_server_name('Simulated conveyor station PLC')
    idx = await server.register_namespace(NAMESPACE)
    station = await server.nodes.objects.add_object(ua.NodeId('Station', idx), 'Station')

    async def signal(name, value, varianttype=ua.VariantType.Boolean):
        node = await station.add_variable(
            ua.NodeId(f'Station.{name}', idx), name, value, varianttype)
        await node.set_writable()
        return node

    dock_request = await signal('AmrDockRequest', False)
    in_position = await signal('AmrInPosition', False)
    ready = await signal('StationReady', False)
    transfer_active = await signal('TransferActive', False)
    transfer_done = await signal('TransferDone', False)
    fault = await signal('Fault', False)
    fault_code = await signal('FaultCode', 0, ua.VariantType.Int32)

    loop = asyncio.get_running_loop()
    phase = 'idle'
    due = 0.0

    print(f'station_sim: {endpoint} scenario={scenario}', flush=True)
    async with server:
        while True:
            request = await dock_request.read_value()
            docked = await in_position.read_value()
            now = loop.time()

            if not request:
                phase = 'idle'                  # dropping the request also acknowledges a fault
            elif phase == 'idle':
                phase, due = 'preparing', now + ready_delay
            elif phase == 'preparing' and now >= due and scenario != 'never_ready':
                phase = 'ready'
            elif phase == 'ready' and docked:
                phase, due = 'transferring', now + transfer_time
            elif phase == 'transferring':
                if scenario == 'fault_during_transfer' and now >= due - transfer_time / 2:
                    phase = 'fault'
                elif now >= due:
                    phase = 'done'

            await ready.write_value(phase in ('ready', 'transferring', 'done'))
            await transfer_active.write_value(phase == 'transferring')
            await transfer_done.write_value(phase == 'done')
            await fault.write_value(phase == 'fault')
            await fault_code.write_value(
                ua.Variant(1 if phase == 'fault' else 0, ua.VariantType.Int32))

            await asyncio.sleep(CYCLE_S)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=4860)
    parser.add_argument('--scenario', choices=SCENARIOS, default='normal')
    parser.add_argument('--ready-delay', type=float, default=0.5)
    parser.add_argument('--transfer-time', type=float, default=2.0)
    # launch_ros appends --ros-args to every executable it starts.
    args, _ = parser.parse_known_args()
    try:
        asyncio.run(run(args.host, args.port, args.scenario, args.ready_delay, args.transfer_time))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
