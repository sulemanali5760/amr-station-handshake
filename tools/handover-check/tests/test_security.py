"""The tester reaches a station that only accepts encrypted sessions with a login, and is refused without."""
import asyncio
import socket

import pytest
from asyncua import Server, ua
from asyncua.crypto import cert_gen
from asyncua.server.user_managers import User, UserManager, UserRole
from cryptography.x509.oid import ExtendedKeyUsageOID

from handover_check.checks import SIGNALS, SIM_MAP, Rig
from handover_check.security import Security

ENDPOINT = "opc.tcp://127.0.0.1:48499/station/"
USER, PASSWORD = "commissioning", "s3cret"


class _OneUser(UserManager):
    def get_user(self, iserver, username=None, password=None, certificate=None):
        return User(role=UserRole.User, name=username) if (username, password) == (USER, PASSWORD) else None


async def _secure_station(pki, ready, stop):
    """The eight signals on a server that wants Basic256Sha256 + SignAndEncrypt and a user login."""
    cert, key = pki / "server_cert.der", pki / "server_key.pem"
    await cert_gen.setup_self_signed_certificate(key, cert, "urn:test:station", socket.gethostname(),
                                                 [ExtendedKeyUsageOID.SERVER_AUTH], {"commonName": "station"})
    server = Server(user_manager=_OneUser())
    await server.init()
    server.set_endpoint(ENDPOINT)
    await server.set_application_uri("urn:test:station")
    await server.load_certificate(str(cert))
    await server.load_private_key(str(key))
    server.set_security_policy([ua.SecurityPolicyType.Basic256Sha256_SignAndEncrypt])
    server.set_identity_tokens([ua.UserNameIdentityToken])
    idx = await server.register_namespace(SIM_MAP["namespace_uri"])
    station = await server.nodes.objects.add_object(ua.NodeId("Station", idx), "Station")
    for name in SIGNALS:
        vt = ua.VariantType.Int16 if name in ("AmrHeartbeat", "FaultCode") else ua.VariantType.Boolean
        node = await station.add_variable(ua.NodeId(f"Station.{name}", idx), name, ua.Variant(0 if vt == ua.VariantType.Int16 else False, vt))
        await node.set_writable()
    async with server:
        ready.set()
        await stop.wait()


def _enter(tmp_path, security):
    async def run():
        ready, stop = asyncio.Event(), asyncio.Event()
        task = asyncio.create_task(_secure_station(tmp_path, ready, stop))
        await asyncio.wait_for(ready.wait(), 20)
        try:
            async with Rig(ENDPOINT, SIM_MAP, period=0.03, security=security) as rig:
                return await rig.read()
        finally:
            stop.set()
            await task
    return asyncio.run(run())


def _secure(tmp_path, password=PASSWORD):
    return Security("Basic256Sha256", "SignAndEncrypt", str(tmp_path / "client"),
                    str(tmp_path / "server_cert.der"), USER, password)


def test_secured_station_is_reached_with_certificate_and_login(tmp_path):
    assert _enter(tmp_path, _secure(tmp_path)) is not None
    assert (tmp_path / "client" / "client_cert.der").exists()


def test_secured_station_refuses_a_plain_anonymous_session(tmp_path):
    with pytest.raises(Exception, match="No matching endpoints"):
        _enter(tmp_path, None)


def test_secured_station_refuses_a_wrong_password(tmp_path):
    with pytest.raises(Exception, match="BadUserAccessDenied"):
        _enter(tmp_path, _secure(tmp_path, password="wrong"))
