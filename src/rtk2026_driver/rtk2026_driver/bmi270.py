"""Драйвер BMI270 по I2C без внешних зависимостей.

Модуль не импортирует ROS: чип можно проверить обычным python-скриптом на
самой Pi, не поднимая стек. Ровно та же логика, по которой
:mod:`rtk2026_driver.protocol` отделён от ноды.

Обмен идёт через ``/dev/i2c-N`` и ioctl ``I2C_SLAVE``. Библиотека smbus
намеренно не используется: она ограничивает блочную передачу 32 байтами,
а конфигурацию чипа надо залить одним потоком в 8192 байта.

Почему вообще нужна заливка конфигурации
----------------------------------------

BMI270 после включения не измеряет ничего. Внутри стоит микроконтроллер
без собственной прошивки, и её обязан загрузить хост: 8 КБ бинарного
образа из BMI270_SensorAPI. Без этого чип отвечает на CHIP_ID, но
INTERNAL_STATUS никогда не станет ``init_ok``, а данные останутся нулями.

Регистры и значения сверены с
https://github.com/boschsensortec/BMI270_SensorAPI (bmi2_defs.h).
"""

from __future__ import annotations

import fcntl
import os
import struct
import time

# ioctl для выбора адреса ведомого на шине.
I2C_SLAVE = 0x0703

# Адреса регистров BMI270.
REG_CHIP_ID = 0x00
REG_INTERNAL_STATUS = 0x21
REG_ACC_DATA = 0x0C
REG_ACC_CONF = 0x40
REG_ACC_RANGE = 0x41
REG_GYR_CONF = 0x42
REG_GYR_RANGE = 0x43
REG_INIT_CTRL = 0x59
REG_INIT_ADDR_0 = 0x5B
REG_INIT_ADDR_1 = 0x5C
REG_INIT_DATA = 0x5E
REG_PWR_CONF = 0x7C
REG_PWR_CTRL = 0x7D
REG_CMD = 0x7E

CHIP_ID_BMI270 = 0x24
INIT_OK = 0x01
SOFT_RESET_CMD = 0xB6

# Биты PWR_CTRL.
PWR_CTRL_ACC_EN = 0x04
PWR_CTRL_GYR_EN = 0x02

I2C_ADDRESS_PRIMARY = 0x68
I2C_ADDRESS_SECONDARY = 0x69

# Коды ODR в полях acc_odr и gyr_odr. Значение регистра равно коду.
ODR_CODES = {
    25: 0x06,
    50: 0x07,
    100: 0x08,
    200: 0x09,
    400: 0x0A,
    800: 0x0B,
}

# Коды диапазона акселерометра: значение регистра -> предел в g.
ACCEL_RANGE_CODES = {2: 0x00, 4: 0x01, 8: 0x02, 16: 0x03}

# Коды диапазона гироскопа. Порядок обратный: чем больше код, тем уже
# диапазон и тем мельче цена младшего разряда.
GYRO_RANGE_CODES = {2000: 0x00, 1000: 0x01, 500: 0x02, 250: 0x03, 125: 0x04}

# Фильтры из BMI270_SensorAPI: acc_bwp = NORMAL_AVG4, gyr_bwp = NORMAL_MODE.
ACC_BWP_NORMAL_AVG4 = 0x02
GYR_BWP_NORMAL_MODE = 0x02

STANDARD_GRAVITY = 9.80665
DEGREES_TO_RADIANS = 0.017453292519943295

# Размер порции при заливке конфигурации. Ограничение здесь не от шины, а
# от здравого смысла: одна неудачная транзакция должна стоить недорого.
CONFIG_CHUNK_BYTES = 128

# Пауза после снятия adv_power_save, до первой записи INIT_CTRL.
POWER_SAVE_SETTLE_SEC = 0.001

# Пауза после INIT_CTRL=1. Datasheet обещает готовность за 20 мс,
# SensorAPI ждёт 150 мс; берём его значение как заведомо достаточное.
INIT_COMPLETE_WAIT_SEC = 0.15


class Bmi270Error(RuntimeError):
    """Чип не отвечает, не опознан или не прошёл инициализацию."""


class Bmi270:
    """BMI270 на шине I2C в режиме опроса.

    Прерывания не используются: на роботе к датчику подведены шесть
    проводов без линий INT, и данные забираются по таймеру ноды. Для
    одометрии этого достаточно - гироскоп всё равно интегрируется на
    частоте EKF, а не по каждому отсчёту.
    """

    def __init__(
        self,
        bus: int = 1,
        address: int = I2C_ADDRESS_PRIMARY,
        accel_range_g: int = 4,
        gyro_range_dps: int = 1000,
        output_data_rate_hz: int = 100,
    ) -> None:
        """Сохранить параметры обмена и проверить их допустимость.

        :param bus: номер шины, ``1`` соответствует ``/dev/i2c-1``.
        :param address: адрес чипа, 0x68 или 0x69 в зависимости от SDO.
        :param accel_range_g: предел акселерометра в g.
        :param gyro_range_dps: предел гироскопа в градусах в секунду.
        :param output_data_rate_hz: частота выдачи данных самим чипом.
        :raises ValueError: если запрошено значение вне таблиц чипа.
        """

        if accel_range_g not in ACCEL_RANGE_CODES:
            raise ValueError(
                f"accel_range_g={accel_range_g} нет в {sorted(ACCEL_RANGE_CODES)}"
            )

        if gyro_range_dps not in GYRO_RANGE_CODES:
            raise ValueError(
                f"gyro_range_dps={gyro_range_dps} нет в {sorted(GYRO_RANGE_CODES)}"
            )

        if output_data_rate_hz not in ODR_CODES:
            raise ValueError(
                f"output_data_rate_hz={output_data_rate_hz} нет в {sorted(ODR_CODES)}"
            )

        self._bus = int(bus)
        self._address = int(address)
        self._accel_range_g = int(accel_range_g)
        self._gyro_range_dps = int(gyro_range_dps)
        self._output_data_rate_hz = int(output_data_rate_hz)

        self._fd: int | None = None

        # Цена младшего разряда. Оба датчика отдают знаковые 16 бит на
        # полный диапазон в обе стороны, отсюда деление на 2^15.
        self.accel_scale_mps2 = (self._accel_range_g / 32768.0) * STANDARD_GRAVITY
        self.gyro_scale_rps = (self._gyro_range_dps / 32768.0) * DEGREES_TO_RADIANS

    # ------------------------------------------------------------------
    # Низкий уровень
    # ------------------------------------------------------------------

    def _write_register(self, register: int, value: int) -> None:
        """Записать один байт в регистр."""

        os.write(self._fd, bytes((register & 0xFF, value & 0xFF)))

    def _read_registers(self, register: int, length: int) -> bytes:
        """Прочитать подряд ``length`` байт начиная с регистра.

        Адрес регистра и чтение идут двумя транзакциями без repeated
        start. На шине единственный ведущий, поэтому вклиниться между
        ними некому.
        """

        os.write(self._fd, bytes((register & 0xFF,)))
        return os.read(self._fd, length)

    def _read_register(self, register: int) -> int:
        """Прочитать один байт регистра."""

        return self._read_registers(register, 1)[0]

    # ------------------------------------------------------------------
    # Запуск
    # ------------------------------------------------------------------

    def open(self, config_blob: bytes) -> None:
        """Открыть шину, опознать чип и залить в него конфигурацию.

        :param config_blob: 8192 байта образа из BMI270_SensorAPI.
        :raises Bmi270Error: чип не опознан или не поднялся.
        :raises ValueError: образ конфигурации неверного размера.
        """

        if len(config_blob) != 8192:
            raise ValueError(
                f"конфигурация BMI270 должна быть 8192 байта, получено {len(config_blob)}"
            )

        self._fd = os.open(f"/dev/i2c-{self._bus}", os.O_RDWR)
        fcntl.ioctl(self._fd, I2C_SLAVE, self._address)

        # Сброс приводит чип в известное состояние независимо от того,
        # что с ним делали до запуска ноды.
        self._write_register(REG_CMD, SOFT_RESET_CMD)
        time.sleep(0.005)

        chip_id = self._read_register(REG_CHIP_ID)

        if chip_id != CHIP_ID_BMI270:
            raise Bmi270Error(
                f"по адресу 0x{self._address:02X} чип 0x{chip_id:02X}, "
                f"ожидался BMI270 0x{CHIP_ID_BMI270:02X}"
            )

        self._upload_configuration(config_blob)
        self._configure_sensors()

    def _upload_configuration(self, config_blob: bytes) -> None:
        """Залить образ прошивки чипа и дождаться init_ok."""

        # Энергосбережение обязано быть выключено на время заливки:
        # спящий чип не примет поток.
        self._write_register(REG_PWR_CONF, 0x00)
        time.sleep(POWER_SAVE_SETTLE_SEC)

        self._write_register(REG_INIT_CTRL, 0x00)

        for offset in range(0, len(config_blob), CONFIG_CHUNK_BYTES):
            chunk = config_blob[offset : offset + CONFIG_CHUNK_BYTES]

            # Адрес задаётся в словах по два байта: младшие четыре бита
            # в INIT_ADDR_0, остальные в INIT_ADDR_1.
            word_index = offset // 2
            self._write_register(REG_INIT_ADDR_0, word_index & 0x0F)
            self._write_register(REG_INIT_ADDR_1, word_index >> 4)

            os.write(self._fd, bytes((REG_INIT_DATA,)) + chunk)

        self._write_register(REG_INIT_CTRL, 0x01)
        time.sleep(INIT_COMPLETE_WAIT_SEC)

        status = self._read_register(REG_INTERNAL_STATUS) & 0x0F

        if status != INIT_OK:
            raise Bmi270Error(
                f"INTERNAL_STATUS=0x{status:02X}, ожидался init_ok "
                f"0x{INIT_OK:02X}: конфигурация не принята"
            )

    def _configure_sensors(self) -> None:
        """Включить оба датчика и задать частоту, полосу и диапазоны."""

        odr_code = ODR_CODES[self._output_data_rate_hz]

        # ACC_CONF: odr в битах 3:0, bwp в 6:4, бит 7 - режим фильтра.
        acc_conf = odr_code | (ACC_BWP_NORMAL_AVG4 << 4) | (1 << 7)

        # GYR_CONF: odr в 3:0, bwp в 5:4, бит 6 - noise performance,
        # бит 7 - filter performance. Оба выставлены в performance:
        # шум гироскопа здесь напрямую становится дрейфом курса.
        gyr_conf = odr_code | (GYR_BWP_NORMAL_MODE << 4) | (1 << 6) | (1 << 7)

        self._write_register(REG_PWR_CTRL, PWR_CTRL_ACC_EN | PWR_CTRL_GYR_EN)
        self._write_register(REG_ACC_CONF, acc_conf)
        self._write_register(REG_ACC_RANGE, ACCEL_RANGE_CODES[self._accel_range_g])
        self._write_register(REG_GYR_CONF, gyr_conf)
        self._write_register(REG_GYR_RANGE, GYRO_RANGE_CODES[self._gyro_range_dps])

        # adv_power_save оставляем выключенным, fifo_self_wakeup включаем -
        # то же, что делает SensorAPI в режиме постоянных измерений.
        self._write_register(REG_PWR_CONF, 0x02)

        # Первые отсчёты после включения датчиков недостоверны.
        time.sleep(0.05)

    # ------------------------------------------------------------------
    # Работа
    # ------------------------------------------------------------------

    def read(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Прочитать одно измерение.

        Акселерометр лежит с 0x0C, гироскоп с 0x12, подряд, поэтому оба
        забираются одним чтением двенадцати байт: это вдвое меньше
        транзакций и оба датчика оказываются с одного момента времени.

        :returns: ``(accel_mps2, gyro_rps)``, обе тройки в осях датчика.
        """

        raw = self._read_registers(REG_ACC_DATA, 12)
        values = struct.unpack("<6h", raw)

        accel = tuple(value * self.accel_scale_mps2 for value in values[0:3])
        gyro = tuple(value * self.gyro_scale_rps for value in values[3:6])

        return accel, gyro

    def close(self) -> None:
        """Закрыть шину. Повторный вызов безопасен."""

        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
