import random
import subprocess
import sys
from pathlib import Path

from rover_pilot_remastered import protocol


def test_crc_and_main_repo_anchor(vectors):
    assert protocol.crc16(b'123456789') == vectors['crc_check']['crc'] == 0x29B1
    # PC_HEARTBEAT from the telemetry repo's C reference implementation: same framing.
    assert protocol.encode_packet(0x81, 0, b'\x01\x00').hex() == '030181020103e02300'


def test_commands_match_the_shared_vectors(vectors):
    decoder = protocol.PacketDecoder()
    decoder.feed(b'\x00')
    for vector in vectors['commands']:
        payload = protocol.drive_command_payload(vector['estop'], vector['targets_rpm'])
        assert protocol.encode_packet(protocol.TYPE_DRIVE_COMMAND, vector['seq'], payload).hex() == vector['wire']
        (packet,) = decoder.feed(bytes.fromhex(vector['wire']))
        estop, targets = protocol.parse_drive_command(packet.payload)
        assert estop == vector['estop'] and list(targets) == vector['targets_rpm']
    assert decoder.bad == 0


def test_status_matches_the_shared_vector(vectors):
    status = vectors['status']
    (packet,) = protocol.PacketDecoder().feed(b'\x00' + bytes.fromhex(status['wire']))
    parsed = protocol.parse_drive_status(packet.payload)
    fields = status['fields']
    assert parsed.state == fields['state'] and parsed.state_name == 'DRIVING'
    assert list(parsed.pwm) == fields['pwm'] and parsed.max_wheel_rpm == fields['max_wheel_rpm']
    assert list(parsed.firmware_version) == fields['firmware_version']


def test_noise_and_split_chunks_resynchronise(vectors):
    wires = [bytes.fromhex(v['wire']) for v in vectors['commands']]
    stream = b'drive\r\n\x13\x00' + b''.join(wires) + b'\x05\x05\x05\x00' + wires[1]
    decoder, packets, rng, index = protocol.PacketDecoder(), [], random.Random(1), 0
    while index < len(stream):
        size = rng.randrange(1, 7)
        packets += decoder.feed(stream[index:index + size])
        index += size
    assert len(packets) == len(wires) + 1
    assert decoder.bad == 1


def test_targets_are_clamped_and_never_hit_the_sentinel():
    estop, targets = protocol.parse_drive_command(protocol.drive_command_payload(False, (1e9, -1e9, 0.04, -0.06)))
    assert targets == (3276.7, -3276.7, 0.0, -0.1)


def test_reserved_flag_bits_are_rejected_like_the_firmware():
    assert protocol.parse_drive_command(protocol.drive_command_payload(True, (0.0,) * 4)) == (True, (0.0,) * 4)
    assert protocol.parse_drive_command(b'\x02' + bytes(8)) is None
    assert protocol.parse_drive_command(bytes(8)) is None


def test_vector_files_are_up_to_date():
    root = Path(__file__).resolve().parents[4]
    script = root / 'tools' / 'gen_test_vectors.py'
    if script.exists():  # only inside the full repository checkout
        result = subprocess.run([sys.executable, str(script), '--check'], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
