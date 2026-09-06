"""Тесты кодека протокола v2.

Проверяется три группы свойств:

1. Разметка структур совпадает с прошивкой (размеры и порядок полей).
2. Кадрирование восстанавливается после повреждения потока - именно этого
   не умел протокол v1, где один потерянный байт сдвигал поток навсегда.
3. Кодек ROS-ноды и стендовый кодек ``arduino/tools/rtk_link.py`` дают
   одинаковые байты. Модули намеренно продублированы, потому что стендовые
   скрипты обязаны работать без установленного ROS, и эта проверка не даёт
   копиям разойтись.
"""

import struct
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
DRIVER_SRC = REPO_ROOT / "src" / "rtk2026_driver"
ARDUINO_TOOLS = REPO_ROOT / "protocol"

if str(DRIVER_SRC) not in sys.path:
    sys.path.insert(0, str(DRIVER_SRC))


from rtk2026_driver.protocol import (
    COMMAND_FLAG_REQUEST_PID_DEBUG,
    CONTROL_MODE_WHEEL_SETPOINT,
    FRAME_SYNC1,
    FRAME_SYNC2,
    GAINS_REPORT_STRUCT,
    GAINS_SOURCE_EEPROM,
    MSG_CMD_RESET,
    MSG_CMD_VELOCITY,
    MSG_CMD_WHEEL_PWM,
    MSG_CMD_WHEEL_SETPOINT,
    MSG_GAINS_REPORT,
    MSG_GET_GAINS,
    MSG_PID_DEBUG,
    MSG_SAVE_GAINS,
    MSG_SET_GAINS,
    MSG_SONAR_SAMPLE,
    MSG_STATS,
    MSG_TELEMETRY,
    PID_DEBUG_STRUCT,
    RESET_ODOMETRY,
    RESET_PID,
    SET_GAINS_STRUCT,
    SONAR_SAMPLE_STRUCT,
    STATS_STRUCT,
    TELEMETRY_STRUCT,
    VELOCITY_STRUCT,
    WHEEL_LEFT,
    WHEEL_PWM_STRUCT,
    WHEEL_RIGHT,
    WHEEL_SETPOINT_STRUCT,
    FrameDecoder,
    SequenceTracker,
    build_frame,
    crc16_ccitt,
    decode_gains_report,
    decode_pid_debug,
    decode_sonar_sample,
    decode_stats,
    decode_telemetry,
    pack_get_gains,
    pack_reset_command,
    pack_save_gains,
    pack_set_gains,
    pack_velocity_command,
    pack_wheel_pwm_command,
    pack_wheel_setpoint_command,
)


def _telemetry_payload(
    seq=7,
    mcu_time_ms=123456,
    dt_us=20000,
    left_encoder_delta=120,
    right_encoder_delta=-95,
    left_encoder_total=100000,
    right_encoder_total=-90000,
    left_wheel_rps=1.5,
    right_wheel_rps=-1.25,
    left_setpoint_rps=1.75,
    right_setpoint_rps=-1.0,
    left_pwm=150,
    right_pwm=-170,
    odom_x_m=1.25,
    odom_y_m=-0.50,
    odom_heading_rad=0.75,
    current_linear_mps=0.42,
    current_angular_rps=-0.18,
    sonar_distance_cm=42,
    flags=0,
    mode=0,
):
    return TELEMETRY_STRUCT.pack(
        seq,
        mcu_time_ms,
        dt_us,
        left_encoder_delta,
        right_encoder_delta,
        left_encoder_total,
        right_encoder_total,
        left_wheel_rps,
        right_wheel_rps,
        left_setpoint_rps,
        right_setpoint_rps,
        left_pwm,
        right_pwm,
        odom_x_m,
        odom_y_m,
        odom_heading_rad,
        current_linear_mps,
        current_angular_rps,
        sonar_distance_cm,
        flags,
        mode,
    )


def _decode_all(decoder, data):
    return list(decoder.feed(data))


def test_struct_sizes_match_firmware():
    # Те же числа закреплены static_assert в arduino/include/control_protocol.h.
    assert VELOCITY_STRUCT.size == 9
    assert WHEEL_PWM_STRUCT.size == 5
    assert WHEEL_SETPOINT_STRUCT.size == 9
    assert SET_GAINS_STRUCT.size == 21
    assert GAINS_REPORT_STRUCT.size == 22
    assert TELEMETRY_STRUCT.size == 66
    assert PID_DEBUG_STRUCT.size == 66
    assert STATS_STRUCT.size == 49
    assert SONAR_SAMPLE_STRUCT.size == 9


def test_crc16_matches_reference_vector():
    # Стандартный вектор CRC16-CCITT (init 0xFFFF). Прошивка использует ту же
    # реализацию, поэтому расхождение здесь означало бы, что каждый кадр
    # отвергается как повреждённый.
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_build_frame_layout():
    frame = build_frame(MSG_CMD_RESET, b"\x03")

    assert frame[0] == FRAME_SYNC1
    assert frame[1] == FRAME_SYNC2
    assert frame[2] == MSG_CMD_RESET
    assert frame[3] == 1
    assert frame[4] == 0x03
    assert len(frame) == 1 + 6

    # CRC покрывает msg_id, len и payload, но не sync-байты.
    expected_crc = crc16_ccitt(bytes((MSG_CMD_RESET, 1, 0x03)))
    assert struct.unpack("<H", frame[5:7])[0] == expected_crc


def test_pack_velocity_command_roundtrip():
    frame = pack_velocity_command(0.25, -0.50, flags=1)
    decoder = FrameDecoder()

    messages = _decode_all(decoder, frame)

    assert len(messages) == 1
    message_id, payload = messages[0]

    assert message_id == MSG_CMD_VELOCITY

    linear_mps, angular_rps, flags = VELOCITY_STRUCT.unpack(payload)
    assert linear_mps == pytest.approx(0.25)
    assert angular_rps == pytest.approx(-0.50)
    assert flags == 1


def test_pack_reset_command_carries_mask():
    frame = pack_reset_command(RESET_ODOMETRY | RESET_PID)
    decoder = FrameDecoder()

    (message_id, payload), = _decode_all(decoder, frame)

    assert message_id == MSG_CMD_RESET
    assert payload == bytes((RESET_ODOMETRY | RESET_PID,))


def test_decode_telemetry_fields():
    frame = build_frame(MSG_TELEMETRY, _telemetry_payload())
    decoder = FrameDecoder()

    (message_id, payload), = _decode_all(decoder, frame)
    assert message_id == MSG_TELEMETRY

    telemetry = decode_telemetry(payload)

    assert telemetry.seq == 7
    assert telemetry.mcu_time_ms == 123456
    assert telemetry.dt_us == 20000
    assert telemetry.dt_s == pytest.approx(0.02)
    assert telemetry.left_encoder_delta == 120
    assert telemetry.right_encoder_delta == -95
    assert telemetry.left_encoder_total == 100000
    assert telemetry.right_encoder_total == -90000
    assert telemetry.left_wheel_rps == pytest.approx(1.5)
    assert telemetry.right_wheel_rps == pytest.approx(-1.25)
    assert telemetry.left_setpoint_rps == pytest.approx(1.75)
    assert telemetry.right_setpoint_rps == pytest.approx(-1.0)
    assert telemetry.left_pwm == 150
    assert telemetry.right_pwm == -170
    assert telemetry.odom_x_m == pytest.approx(1.25)
    assert telemetry.odom_y_m == pytest.approx(-0.50)
    assert telemetry.odom_heading_rad == pytest.approx(0.75)
    assert telemetry.current_linear_mps == pytest.approx(0.42)
    assert telemetry.current_angular_rps == pytest.approx(-0.18)
    assert telemetry.sonar_distance_cm == 42


def test_decode_sonar_sample_fields():
    payload = SONAR_SAMPLE_STRUCT.pack(123, 456789, 4, 875)

    sample = decode_sonar_sample(payload)

    assert sample.seq == 123
    assert sample.mcu_time_ms == 456789
    assert sample.sensor_index == 4
    assert sample.distance_mm == 875


def test_telemetry_flag_properties():
    payload = _telemetry_payload(flags=0x02 | 0x08 | 0x10)
    telemetry = decode_telemetry(payload)

    assert telemetry.command_timeout is True
    assert telemetry.pwm_saturated is True
    assert telemetry.cycle_overrun is True


def test_encoder_totals_survive_packet_loss():
    """Накопленные счётчики не зависят от того, дошли ли промежуточные пакеты.

    Сумма дельт на хосте после потери пакета занизила бы пройденный путь,
    а разность накопленных счётчиков остаётся верной.
    """

    first = decode_telemetry(
        _telemetry_payload(seq=1, left_encoder_total=1000, left_encoder_delta=10)
    )
    # Пакеты 2..9 потеряны, за них колесо накрутило ещё 800 отсчётов.
    tenth = decode_telemetry(
        _telemetry_payload(seq=10, left_encoder_total=1800, left_encoder_delta=10)
    )

    assert tenth.left_encoder_total - first.left_encoder_total == 800
    # Сумма дошедших дельт дала бы только 20.
    assert first.left_encoder_delta + tenth.left_encoder_delta == 20


def test_decode_stats_fields():
    payload = STATS_STRUCT.pack(
        2, 60000, 3000, 19800, 25400, 20010, 900, 24800,
        3000, 1500, 4, 1, 9, 0, 2, 5300,
    )

    stats = decode_stats(payload)

    assert stats.protocol_version == 2
    assert stats.uptime_ms == 60000
    assert stats.control_cycles == 3000
    assert stats.dt_min_us == 19800
    assert stats.dt_max_us == 25400
    assert stats.dt_mean_us == 20010
    assert stats.cycle_duration_max_us == 900
    assert stats.sonar_block_max_us == 24800
    assert stats.tx_frames == 3000
    assert stats.rx_frames == 1500
    assert stats.tx_dropped == 4
    assert stats.rx_bad_crc == 1
    assert stats.rx_resync == 9
    assert stats.rx_bad_length == 0
    assert stats.overruns == 2
    assert stats.free_ram_bytes == 5300


def test_decoder_handles_split_frame():
    frame = build_frame(MSG_TELEMETRY, _telemetry_payload())
    decoder = FrameDecoder()

    # Порт не обязан отдавать кадр целиком, поэтому кормим по кускам.
    assert _decode_all(decoder, frame[:7]) == []
    assert _decode_all(decoder, frame[7:20]) == []

    messages = _decode_all(decoder, frame[20:])
    assert len(messages) == 1
    assert decode_telemetry(messages[0][1]).seq == 7


def test_decoder_returns_multiple_frames_from_one_chunk():
    stream = (
        build_frame(MSG_TELEMETRY, _telemetry_payload(seq=1))
        + build_frame(MSG_TELEMETRY, _telemetry_payload(seq=2))
        + build_frame(MSG_TELEMETRY, _telemetry_payload(seq=3))
    )

    decoder = FrameDecoder()
    messages = _decode_all(decoder, stream)

    assert [decode_telemetry(payload).seq for _, payload in messages] == [1, 2, 3]
    assert decoder.frame_count == 3
    assert decoder.bad_crc_count == 0
    assert decoder.resync_count == 0


def test_decoder_rejects_corrupted_payload():
    frame = bytearray(build_frame(MSG_TELEMETRY, _telemetry_payload()))
    frame[10] ^= 0xFF

    decoder = FrameDecoder()

    assert _decode_all(decoder, bytes(frame)) == []
    assert decoder.bad_crc_count == 1
    assert decoder.frame_count == 0


def test_decoder_resynchronises_after_lost_byte():
    """Главное свойство v2: потеря байта стоит конечное число кадров.

    Восстановление не мгновенное. Из-за потерянного байта поле длины
    повреждённого кадра оказывается сдвинуто, разборщик отсчитывает
    нагрузку не оттуда и заглатывает начало следующего кадра. Поэтому
    цена одного потерянного байта - до двух кадров.

    Избежать этого можно только байт-стаффингом, который удорожает
    кодек на обеих сторонах. Важно другое: в протоколе v1 такая потеря
    сдвигала поток навсегда, и нода продолжала молча публиковать
    правдоподобную, но неверную одометрию.
    """

    frames = [
        build_frame(MSG_TELEMETRY, _telemetry_payload(seq=index))
        for index in range(1, 5)
    ]

    # Выбрасываем байт из середины первого кадра.
    damaged = frames[0][:12] + frames[0][13:] + b"".join(frames[1:])

    decoder = FrameDecoder()
    messages = _decode_all(decoder, damaged)

    recovered = [decode_telemetry(payload).seq for _, payload in messages]

    # Первый кадр потерян обязательно, второй - возможно.
    # Всё, что после, обязано разобраться корректно.
    assert recovered, "разборщик не восстановил синхронизацию"
    assert recovered == list(range(recovered[0], 5))
    assert recovered[0] <= 3, (
        f"восстановление заняло слишком много кадров: первый принятый {recovered[0]}"
    )


def test_decoder_recovers_after_leading_junk():
    frame = build_frame(MSG_TELEMETRY, _telemetry_payload(seq=5))
    junk = bytes((0x00, 0xAA, 0x13, 0xFF, 0xAA, 0xAA))

    decoder = FrameDecoder()
    messages = _decode_all(decoder, junk + frame)

    assert len(messages) == 1
    assert decode_telemetry(messages[0][1]).seq == 5
    assert decoder.resync_count > 0


def test_decoder_rejects_oversized_length():
    decoder = FrameDecoder(max_payload_bytes=32)

    # Кадр объявляет 200 байт нагрузки при пределе 32.
    assert _decode_all(decoder, bytes((FRAME_SYNC1, FRAME_SYNC2, 0x01, 200))) == []
    assert decoder.bad_length_count == 1

    # После отбраковки разборщик готов принять корректный кадр.
    frame = build_frame(MSG_CMD_RESET, b"\x01")
    assert len(_decode_all(decoder, frame)) == 1


def test_sequence_tracker_counts_losses():
    tracker = SequenceTracker()

    assert tracker.update(10) == 0
    assert tracker.update(11) == 0
    # Пропущены 12 и 13.
    assert tracker.update(14) == 2

    assert tracker.received == 3
    assert tracker.lost == 2
    assert tracker.loss_ratio == pytest.approx(2.0 / 5.0)


def test_sequence_tracker_handles_wraparound():
    tracker = SequenceTracker()

    tracker.update(0xFFFE)
    assert tracker.update(0xFFFF) == 0
    # Счётчик прошивки переполняется, потерь при этом нет.
    assert tracker.update(0x0000) == 0
    assert tracker.lost == 0


def test_sequence_tracker_ignores_backward_jump():
    """Перезапуск MCU не должен выглядеть как потеря десятков тысяч пакетов."""

    tracker = SequenceTracker()

    tracker.update(5000)
    assert tracker.update(3) == 0
    assert tracker.lost == 0
    assert tracker.reordered == 1


def test_ros_and_bench_codecs_agree():
    """ROS-кодек и стендовый кодек обязаны давать байт в байт одно и то же."""

    if str(ARDUINO_TOOLS) not in sys.path:
        sys.path.insert(0, str(ARDUINO_TOOLS))

    import rtk_link

    assert rtk_link.TELEMETRY_STRUCT.format == TELEMETRY_STRUCT.format
    assert rtk_link.STATS_STRUCT.format == STATS_STRUCT.format
    assert rtk_link.VELOCITY_STRUCT.format == VELOCITY_STRUCT.format
    assert rtk_link.SONAR_SAMPLE_STRUCT.format == SONAR_SAMPLE_STRUCT.format

    assert rtk_link.crc16_ccitt(b"123456789") == crc16_ccitt(b"123456789")

    assert rtk_link.pack_velocity_command(0.3, -0.2) == pack_velocity_command(0.3, -0.2)
    assert rtk_link.pack_reset_command(0x07) == pack_reset_command(0x07)

    assert rtk_link.MSG_TELEMETRY == MSG_TELEMETRY
    assert rtk_link.MSG_STATS == MSG_STATS
    assert rtk_link.MSG_SONAR_SAMPLE == MSG_SONAR_SAMPLE

    sonar_payload = SONAR_SAMPLE_STRUCT.pack(12, 3456, 2, -1)
    assert rtk_link.decode_sonar_sample(sonar_payload).distance_mm == -1

    # Кадр, собранный одним кодеком, должен разбираться другим.
    frame = build_frame(MSG_TELEMETRY, _telemetry_payload(seq=99))
    bench_decoder = rtk_link.FrameDecoder()
    (_, payload), = list(bench_decoder.feed(frame))
    assert rtk_link.decode_telemetry(payload).seq == 99


def test_pack_wheel_setpoint_command_roundtrip():
    """Уставка колёс минует кинематику: правое крутится, левое стоит."""

    frame = pack_wheel_setpoint_command(
        0.0, 2.5, flags=COMMAND_FLAG_REQUEST_PID_DEBUG
    )
    decoder = FrameDecoder()

    (message_id, payload), = _decode_all(decoder, frame)

    assert message_id == MSG_CMD_WHEEL_SETPOINT

    left_rps, right_rps, flags = WHEEL_SETPOINT_STRUCT.unpack(payload)
    assert left_rps == pytest.approx(0.0)
    assert right_rps == pytest.approx(2.5)
    assert flags == COMMAND_FLAG_REQUEST_PID_DEBUG


def test_pack_wheel_pwm_command_roundtrip():
    frame = pack_wheel_pwm_command(-120, 200)
    decoder = FrameDecoder()

    (message_id, payload), = _decode_all(decoder, frame)

    assert message_id == MSG_CMD_WHEEL_PWM

    left_pwm, right_pwm, flags = WHEEL_PWM_STRUCT.unpack(payload)
    assert left_pwm == -120
    assert right_pwm == 200
    assert flags == 0


def test_pack_set_gains_roundtrip():
    frame = pack_set_gains(WHEEL_RIGHT, 1.5, 0.25, 0.01, 30.0, 12.5)
    decoder = FrameDecoder()

    (message_id, payload), = _decode_all(decoder, frame)

    assert message_id == MSG_SET_GAINS

    wheel, kp, ki, kd, k_static, k_velocity = SET_GAINS_STRUCT.unpack(payload)
    assert wheel == WHEEL_RIGHT
    assert kp == pytest.approx(1.5)
    assert ki == pytest.approx(0.25)
    assert kd == pytest.approx(0.01)
    assert k_static == pytest.approx(30.0)
    assert k_velocity == pytest.approx(12.5)


def test_pack_set_gains_rejects_unknown_wheel():
    with pytest.raises(ValueError):
        pack_set_gains(7, 1.0, 0.0, 0.0, 0.0, 0.0)


def test_gain_commands_without_payload():
    for frame, expected_id in (
        (pack_save_gains(), MSG_SAVE_GAINS),
        (pack_get_gains(), MSG_GET_GAINS),
    ):
        decoder = FrameDecoder()
        (message_id, payload), = _decode_all(decoder, frame)

        assert message_id == expected_id
        assert payload == b""


def test_decode_gains_report():
    payload = GAINS_REPORT_STRUCT.pack(
        WHEEL_LEFT, 1.25, 0.5, 0.0, 28.0, 11.0, GAINS_SOURCE_EEPROM
    )

    report = decode_gains_report(payload)

    assert report.wheel == WHEEL_LEFT
    assert report.wheel_name == "left"
    assert report.kp == pytest.approx(1.25)
    assert report.k_static == pytest.approx(28.0)
    assert report.k_velocity == pytest.approx(11.0)
    assert report.is_persisted is True
    assert report.gains.ki == pytest.approx(0.5)


def test_gains_report_flags_unsaved_gains():
    """source отличает записанные коэффициенты от живущих только в RAM."""

    payload = GAINS_REPORT_STRUCT.pack(
        WHEEL_RIGHT, 1.0, 0.0, 0.0, 0.0, 0.0, 0
    )

    report = decode_gains_report(payload)

    assert report.wheel_name == "right"
    assert report.is_persisted is False


def test_decode_pid_debug_splits_wheels():
    left = (2.0, 1.6, 0.4, 0.8, 0.3, 40.0, 1.4, 41.4)
    right = (-2.0, -1.9, -0.1, -0.2, -0.05, -40.0, -0.35, -40.35)

    payload = PID_DEBUG_STRUCT.pack(99, *left, *right)
    debug = decode_pid_debug(payload)

    assert debug.seq == 99
    assert debug.left.setpoint_rps == pytest.approx(2.0)
    assert debug.left.measured_rps == pytest.approx(1.6)
    assert debug.left.feedforward == pytest.approx(40.0)
    assert debug.right.setpoint_rps == pytest.approx(-2.0)
    assert debug.right.output_pwm == pytest.approx(-40.35)


def test_pid_debug_recovers_derivative():
    """D отдельным полем не передаётся и восстанавливается как остаток."""

    # pid_output = proportional + integral_term + derivative
    left = (2.0, 1.6, 0.4, 0.80, 0.30, 40.0, 1.40, 41.40)
    payload = PID_DEBUG_STRUCT.pack(1, *left, *((0.0,) * 8))

    debug = decode_pid_debug(payload)

    assert debug.left.derivative == pytest.approx(1.40 - 0.80 - 0.30)


def test_pid_debug_feedforward_share():
    left = (2.0, 2.0, 0.0, 0.0, 0.0, 40.0, 0.0, 50.0)
    payload = PID_DEBUG_STRUCT.pack(1, *left, *((0.0,) * 8))

    debug = decode_pid_debug(payload)

    # Хорошо настроенный контур держит долю feedforward высокой:
    # ПИД правит остаток, а не тянет весь сигнал.
    assert debug.left.feedforward_share == pytest.approx(0.8)


def test_pid_debug_feedforward_share_survives_zero_output():
    left = (0.0,) * 8
    payload = PID_DEBUG_STRUCT.pack(1, *left, *((0.0,) * 8))

    assert decode_pid_debug(payload).left.feedforward_share == 0.0


def test_telemetry_carries_control_mode():
    payload = _telemetry_payload(mode=CONTROL_MODE_WHEEL_SETPOINT)

    assert decode_telemetry(payload).mode == CONTROL_MODE_WHEEL_SETPOINT


def test_pid_debug_frame_fits_firmware_tx_buffer():
    """Кадр отладки обязан помещаться в tx_buffer прошивки (80 байт)."""

    frame = build_frame(MSG_PID_DEBUG, bytes(PID_DEBUG_STRUCT.size))

    assert len(frame) == 72
    assert len(frame) <= 80


def test_gains_report_arrives_as_separate_frame():
    """Отчёт о коэффициентах не должен путаться с телеметрией в потоке."""

    stream = (
        build_frame(MSG_TELEMETRY, _telemetry_payload(seq=1))
        + build_frame(
            MSG_GAINS_REPORT,
            GAINS_REPORT_STRUCT.pack(WHEEL_LEFT, 1.0, 0.1, 0.0, 25.0, 10.0, 1),
        )
        + build_frame(MSG_TELEMETRY, _telemetry_payload(seq=2))
    )

    decoder = FrameDecoder()
    messages = _decode_all(decoder, stream)

    assert [message_id for message_id, _ in messages] == [
        MSG_TELEMETRY,
        MSG_GAINS_REPORT,
        MSG_TELEMETRY,
    ]
    assert decode_gains_report(messages[1][1]).k_static == pytest.approx(25.0)
