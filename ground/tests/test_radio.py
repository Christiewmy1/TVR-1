"""Tests for parsing the dongle's receive lines (ground_station/radio.py)."""

import pytest

from ground_station.packet import PacketError
from ground_station.radio import parse_rx_line

PACKET = "$TVR,1,0,0,0,0,0,0,7400*1137"  # 28 chars


def test_rcv_line_gives_payload_rssi_and_snr():
    frame = parse_rx_line(f"+RCV=1,28,{PACKET},-45,9.5\r\n")
    assert frame.payload == PACKET
    assert frame.rssi_dbm == -45
    assert frame.snr_db == 9.5


def test_negative_snr():
    assert parse_rx_line(f"+RCV=1,28,{PACKET},-110,-3.5").snr_db == -3.5


def test_bare_packet_line_has_no_radio_info():
    frame = parse_rx_line(PACKET + "\n")
    assert frame.payload == PACKET
    assert frame.rssi_dbm is None and frame.snr_db is None


@pytest.mark.parametrize("line", ["+OK", "+READY", "+ERR=4", "", "   \r\n"])
def test_module_replies_and_blank_lines_are_not_data(line):
    assert parse_rx_line(line) is None


@pytest.mark.parametrize("line", [
    f"+RCV=1,abc,{PACKET},-45,9.5",     # length not a number
    f"+RCV=1,40,{PACKET},-45,9.5",      # length longer than the line
    f"+RCV=1,20,{PACKET},-45,9.5",      # length cuts the packet short
    f"+RCV=1,28,{PACKET}",              # no RSSI/SNR (line cut off)
    f"+RCV=1,28,{PACKET},-45",          # no SNR
    f"+RCV=1,28,{PACKET},-45,9.5,7",    # extra field
    f"+RCV=1,28,{PACKET},loud,9.5",     # RSSI not a number
    "+RCV=1",                           # header only
])
def test_malformed_rcv_lines_are_rx_format(line):
    with pytest.raises(PacketError) as err:
        parse_rx_line(line)
    assert err.value.reason == "rx_format"
