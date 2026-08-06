#!/usr/bin/env python3
"""Графики стендовых инструментов на PyQtGraph.

Почему PyQtGraph, а не matplotlib
---------------------------------

Настройка идёт итерациями «поменял коэффициент - посмотрел отклик», и график
обновляется десятки раз в секунду. PyQtGraph рассчитан именно на это и держит
порядка 70 кадров в секунду против примерно 35 у matplotlib.

Библиотека необязательна. Если её нет, инструменты продолжают работать
и печатают метрики с однострочным графиком в терминале: настройка возможна
и по одним числам, просто менее наглядно.

Устройство окна
---------------

Каждое окно - обычный виджет с вертикальным макетом: сверху полотно графиков,
снизу текстовый блок. Разделение принципиально: если положить текст внутрь
``GraphicsLayoutWidget``, длинные строки растягивают колонку макета и
выдавливают графики за границы окна, из-за чего пропадают заголовки и легенды.

Отображение
-----------

Окна требуют графической подсистемы. При запуске из контейнера на Raspberry Pi
по SSH нужен проброс дисплея, иначе следует пользоваться терминальным выводом
и выгрузкой в CSV.
"""

from __future__ import annotations

#: Признак доступности графической подсистемы. Проверяется один раз при импорте.
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

    PLOTTING_AVAILABLE = True
    PLOTTING_ERROR = ""
except Exception as exception:  # pragma: no cover - зависит от окружения
    pg = None
    QtCore = None
    QtGui = None
    QtWidgets = None
    PLOTTING_AVAILABLE = False
    PLOTTING_ERROR = str(exception)


UNAVAILABLE_HINT = (
    "Графики недоступны: {reason}\n"
    "Установка: pip install pyqtgraph PyQt5\n"
    "Инструмент продолжит работу, показывая метрики в терминале."
)


# Палитры под светлый и тёмный фон. Одни и те же цвета на обоих фонах
# читаются плохо: то, что различимо на чёрном, на белом выцветает.
_PALETTES = {
    "light": {
        "background": "#ffffff",
        "foreground": "#1c1c1e",
        "panel": "#f7f7f8",
        "panel_text": "#1c1c1e",
        "panel_border": "#d0d0d5",
        "grid_alpha": 0.18,
        "setpoint": "#b26a00",
        "measured": "#0b5fa5",
        "pwm": "#5a5a5f",
        "feedforward": "#1f7a3d",
        "proportional": "#b32540",
        "integral": "#6a30b0",
        "series": ["#0b5fa5", "#1f7a3d", "#b26a00", "#6a30b0"],
    },
    "dark": {
        "background": "#1c1c1e",
        "foreground": "#f2f2f7",
        "panel": "#2c2c2e",
        "panel_text": "#f2f2f7",
        "panel_border": "#3a3a3c",
        "grid_alpha": 0.25,
        "setpoint": "#e8a33d",
        "measured": "#3d9be8",
        "pwm": "#8e8e93",
        "feedforward": "#5ac36a",
        "proportional": "#e85d75",
        "integral": "#a56ae8",
        "series": ["#3d9be8", "#5ac36a", "#e8a33d", "#a56ae8"],
    },
}

#: Действующая палитра. По умолчанию светлая: инструменты чаще смотрят днём
#: и рядом с терминалом на светлом фоне.
_palette = dict(_PALETTES["light"])



def set_theme(name: str) -> None:
    """Выбрать тему графиков.

    Вызывать до создания первого окна: pyqtgraph читает фон при построении.
    """

    if name not in _PALETTES:
        raise ValueError(f"неизвестная тема: {name!r}")

    _palette.clear()
    _palette.update(_PALETTES[name])

    if PLOTTING_AVAILABLE:
        pg.setConfigOption("background", _palette["background"])
        pg.setConfigOption("foreground", _palette["foreground"])




def theme_choices() -> list[str]:
    return sorted(_PALETTES)




def require_plotting(quiet: bool = False) -> bool:
    """Проверить доступность графиков и, при отсутствии, объяснить причину."""

    if PLOTTING_AVAILABLE:
        return True

    if not quiet:
        print(UNAVAILABLE_HINT.format(reason=PLOTTING_ERROR))

    return False



def _application():
    """Вернуть работающее приложение Qt, создав его при необходимости."""

    application = QtWidgets.QApplication.instance()
    if application is None:
        application = QtWidgets.QApplication([])

    pg.setConfigOptions(antialias=True)
    pg.setConfigOption("background", _palette["background"])
    pg.setConfigOption("foreground", _palette["foreground"])
    return application


def _monospace_font() -> "QtGui.QFont":
    """Моноширинный шрифт: числа в колонках должны стоять ровно."""

    font = QtGui.QFont()
    font.setStyleHint(QtGui.QFont.TypeWriter)
    font.setFamily("Menlo")
    font.setPointSize(11)
    return font


if PLOTTING_AVAILABLE:

    class _Panel(QtWidgets.QWidget):
        """Окно инструмента: полотно графиков сверху, текстовый блок снизу.

        Текст живёт в отдельном виджете, а не внутри графического макета.
        Иначе длинные строки метрик растягивают колонку и выдавливают графики
        за пределы окна вместе с заголовками и легендами.

        Виджет также пробрасывает нажатия клавиш наружу: это позволяет
        управлять роботом прямо из окна панели.
        """

        def __init__(
            self,
            title: str,
            size: tuple[int, int],
            text_lines: int = 9,
            key_handler=None,
        ) -> None:
            super().__init__()

            self.on_key = key_handler
            self.setWindowTitle(title)
            self.resize(*size)

            layout = QtWidgets.QVBoxLayout(self)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(8)

            self.graphics = pg.GraphicsLayoutWidget()
            layout.addWidget(self.graphics, stretch=1)

            self.text = QtWidgets.QPlainTextEdit()
            self.text.setReadOnly(True)
            self.text.setFont(_monospace_font())
            self.text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
            self.text.setFrameShape(QtWidgets.QFrame.StyledPanel)

            # Высота блока фиксируется по числу строк: иначе он то съедает
            # графики, то схлопывается в полоску при смене содержимого.
            row_height = QtGui.QFontMetrics(self.text.font()).lineSpacing()
            self.text.setFixedHeight(row_height * text_lines + 16)

            self.text.setStyleSheet(
                f"QPlainTextEdit {{"
                f" background-color: {_palette['panel']};"
                f" color: {_palette['panel_text']};"
                f" border: 1px solid {_palette['panel_border']};"
                f" }}"
            )
            layout.addWidget(self.text, stretch=0)

            self.setStyleSheet(f"background-color: {_palette['background']};")
            self.setFocusPolicy(QtCore.Qt.StrongFocus)

        def set_text(self, text: str) -> None:
            # Позиция прокрутки сохраняется: панель обновляется двадцать раз
            # в секунду, и прыжки к началу мешали бы читать.
            scroll = self.text.verticalScrollBar().value()
            self.text.setPlainText(text)
            self.text.verticalScrollBar().setValue(scroll)

        def keyPressEvent(self, event) -> None:
            # Автоповтор пропускаем: удержание отслеживается по нажатию
            # и отпусканию.
            if self.on_key is not None and not event.isAutoRepeat():
                self.on_key(event.key(), True)
            super().keyPressEvent(event)

        def keyReleaseEvent(self, event) -> None:
            if self.on_key is not None and not event.isAutoRepeat():
                self.on_key(event.key(), False)
            super().keyReleaseEvent(event)

else:  # pragma: no cover - зависит от окружения
    _Panel = None


def _prepare_plot(plot, left_label: str, bottom_label: str = "", legend: bool = False):
    """Единое оформление осей, сетки и легенды."""

    plot.setLabel("left", left_label)
    if bottom_label:
        plot.setLabel("bottom", bottom_label)

    plot.showGrid(x=True, y=True, alpha=_palette["grid_alpha"])

    if legend:
        # Легенда ставится внутри области построения со сдвигом от левого
        # верхнего угла: привязка к правому краю уезжала за границу окна
        # при изменении его размера.
        plot.addLegend(offset=(12, 12), labelTextColor=_palette["foreground"])

    return plot



class StepResponseWindow:
    """Окно ступенчатого отклика: скорость, PWM и составляющие регулятора.

    Окно живёт между ступеньками, поэтому можно менять коэффициент и сразу
    сравнивать новый отклик с предыдущим на том же масштабе.
    """

    def __init__(self, title: str = "Ступенчатый отклик") -> None:
        self._application = _application()
        self._panel = _Panel(title, (1150, 900), text_lines=11)

        graphics = self._panel.graphics

        self._speed_plot = _prepare_plot(
            graphics.addPlot(row=0, col=0, title="Скорость колеса"),
            "об/с",
            legend=True,
        )
        self._pwm_plot = _prepare_plot(
            graphics.addPlot(row=1, col=0, title="Команда PWM"), "PWM"
        )
        self._pwm_plot.setXLink(self._speed_plot)

        self._terms_plot = _prepare_plot(
            graphics.addPlot(row=2, col=0, title="Составляющие выхода регулятора"),
            "PWM",
            "время, с",
            legend=True,
        )
        self._terms_plot.setXLink(self._speed_plot)

        dashed = QtCore.Qt.DashLine
        dotted = QtCore.Qt.DotLine

        self._setpoint_curve = self._speed_plot.plot(
            pen=pg.mkPen(_palette["setpoint"], width=2, style=dashed), name="уставка"
        )
        self._measured_curve = self._speed_plot.plot(
            pen=pg.mkPen(_palette["measured"], width=2), name="измерено"
        )
        # Предыдущий отклик показывается блёклым: без него сравнивать итерации
        # настройки приходилось бы по памяти.
        self._previous_curve = self._speed_plot.plot(
            pen=pg.mkPen(_palette["measured"], width=1, style=dotted),
            name="предыдущий",
        )

        self._pwm_curve = self._pwm_plot.plot(pen=pg.mkPen(_palette["pwm"], width=2))

        self._feedforward_curve = self._terms_plot.plot(
            pen=pg.mkPen(_palette["feedforward"], width=2), name="feedforward"
        )
        self._proportional_curve = self._terms_plot.plot(
            pen=pg.mkPen(_palette["proportional"], width=2), name="P"
        )
        self._integral_curve = self._terms_plot.plot(
            pen=pg.mkPen(_palette["integral"], width=2), name="I"
        )

        self._previous_measured: tuple[list[float], list[float]] | None = None
        self._panel.show()


    def update(self, times, setpoints, measured, pwms, terms, metrics_text: str) -> None:
        """Отрисовать новую ступеньку.

        :param terms: словарь с ключами feedforward, proportional, integral
            или пустой, если отладочные кадры не запрашивались.
        """

        if self._previous_measured is not None:
            self._previous_curve.setData(*self._previous_measured)

        self._setpoint_curve.setData(times, setpoints)
        self._measured_curve.setData(times, measured)
        self._pwm_curve.setData(times, pwms)

        if terms:
            self._feedforward_curve.setData(times, terms.get("feedforward", []))
            self._proportional_curve.setData(times, terms.get("proportional", []))
            self._integral_curve.setData(times, terms.get("integral", []))
            self._terms_plot.setVisible(True)
        else:
            self._terms_plot.setVisible(False)

        self._panel.set_text(metrics_text)
        self._previous_measured = (list(times), list(measured))

        self.pump()



    def pump(self, seconds: float = 0.05) -> None:
        """Дать Qt обработать события, чтобы окно оставалось отзывчивым."""

        deadline = QtCore.QTime.currentTime().addMSecs(int(seconds * 1000))
        while QtCore.QTime.currentTime() < deadline:
            self._application.processEvents()


    @property
    def closed(self) -> bool:
        return not self._panel.isVisible()


    def wait_until_closed(self) -> None:
        """Держать окно открытым, пока пользователь его не закроет."""

        while not self.closed:
            self.pump(seconds=0.05)



    def close(self) -> None:
        self._panel.close()





class IdentificationWindow:
    """Окно идентификации: точки скорость-PWM и подогнанная прямая."""

    def __init__(self, title: str = "Идентификация мотора") -> None:
        self._application = _application()
        self._panel = _Panel(title, (1050, 760), text_lines=8)

        self._plot = _prepare_plot(
            self._panel.graphics.addPlot(row=0, col=0),
            "PWM",
            "скорость, об/с",
            legend=True,
        )

        self._colors = list(_palette["series"])
        self._next_color = 0
        self._panel.show()


    def add_series(self, label: str, speeds, pwms, k_static: float, k_velocity: float) -> None:
        """Добавить измеренные точки и прямую, подогнанную по ним.

        Расхождение точек с прямой видно сразу: если мотор нелинеен,
        feedforward одной прямой его не опишет, и остаток придётся добирать
        интегралом.
        """

        color = self._colors[self._next_color % len(self._colors)]
        self._next_color += 1

        self._plot.plot(
            speeds,
            pwms,
            pen=None,
            symbol="o",
            symbolSize=8,
            symbolBrush=color,
            symbolPen=None,
            name=f"{label}: измерено",
        )

        if k_velocity > 1e-9 and speeds:
            line_speeds = [0.0, max(speeds) * 1.05]
            line_pwms = [k_static + k_velocity * s for s in line_speeds]
            self._plot.plot(
                line_speeds,
                line_pwms,
                pen=pg.mkPen(color, width=2, style=QtCore.Qt.DashLine),
                name=f"{label}: ks={k_static:.1f} kv={k_velocity:.2f}",
            )



    def set_summary(self, text: str) -> None:
        self._panel.set_text(text)



    def show_blocking(self) -> None:
        """Показать окно и ждать, пока пользователь его не закроет."""

        while self._panel.isVisible():
            self._application.processEvents()
            QtCore.QThread.msleep(20)





class LiveMonitorWindow:
    """Живая панель состояния робота.

    Показывает то, что нельзя увидеть в отдельном ступенчатом отклике: как
    ведут себя оба колеса одновременно, насколько расходятся уставка
    и измерение в движении, и не деградирует ли связь.
    """

    def __init__(
        self,
        window_s: float = 20.0,
        title: str = "RTK2026: состояние",
        key_handler=None,
    ) -> None:
        self._application = _application()
        self._window_s = window_s
        self._panel = _Panel(title, (1250, 940), text_lines=10, key_handler=key_handler)

        graphics = self._panel.graphics

        self._speed_plot = _prepare_plot(
            graphics.addPlot(row=0, col=0, title="Скорости колёс"), "об/с", legend=True
        )
        self._pwm_plot = _prepare_plot(
            graphics.addPlot(row=1, col=0, title="Команды PWM"), "PWM", legend=True
        )
        self._pwm_plot.setXLink(self._speed_plot)

        self._error_plot = _prepare_plot(
            graphics.addPlot(row=2, col=0, title="Ошибка слежения"),
            "об/с",
            "время, с",
            legend=True,
        )
        self._error_plot.setXLink(self._speed_plot)

        dashed = QtCore.Qt.DashLine
        self._curves = {
            "left_setpoint": self._speed_plot.plot(
                pen=pg.mkPen(_palette["setpoint"], width=2, style=dashed),
                name="левое: уставка",
            ),
            "left_measured": self._speed_plot.plot(
                pen=pg.mkPen(_palette["measured"], width=2), name="левое: измерено"
            ),
            "right_setpoint": self._speed_plot.plot(
                pen=pg.mkPen(_palette["feedforward"], width=2, style=dashed),
                name="правое: уставка",
            ),
            "right_measured": self._speed_plot.plot(
                pen=pg.mkPen(_palette["integral"], width=2), name="правое: измерено"
            ),
            "left_pwm": self._pwm_plot.plot(
                pen=pg.mkPen(_palette["measured"], width=2), name="левое"
            ),
            "right_pwm": self._pwm_plot.plot(
                pen=pg.mkPen(_palette["integral"], width=2), name="правое"
            ),
            "left_error": self._error_plot.plot(
                pen=pg.mkPen(_palette["measured"], width=2), name="левое"
            ),
            "right_error": self._error_plot.plot(
                pen=pg.mkPen(_palette["integral"], width=2), name="правое"
            ),
        }

        self._panel.show()

    @property
    def closed(self) -> bool:
        return not self._panel.isVisible()


    def update(self, series: dict[str, list[float]], status_text: str) -> None:
        times = series["time"]

        for key, curve in self._curves.items():
            curve.setData(times, series[key])

        if times:
            latest = times[-1]
            self._speed_plot.setXRange(
                max(0.0, latest - self._window_s), max(self._window_s, latest)
            )

        self._panel.set_text(status_text)
        self._application.processEvents()



    def close(self) -> None:
        self._panel.close()





class TrajectoryWindow:
    """Окно маршрутного теста: идеальный путь против энкодерного.

    Ошибки, невидимые на графике одного колеса, здесь проявляются сразу.
    Неверная колея ``kTrackWidthM`` даёт систематический недоворот или
    переворот, а неверный радиус колеса - масштабную ошибку по всему пути.
    Оба дефекта на графике скорости выглядят нормально.
    """

    def __init__(self, title: str = "Маршрутный тест") -> None:
        self._application = _application()
        self._panel = _Panel(title, (1150, 950), text_lines=12)

        graphics = self._panel.graphics

        self._path_plot = _prepare_plot(
            graphics.addPlot(row=0, col=0, title="Траектория"),
            "y, м",
            "x, м",
            legend=True,
        )
        # Масштаб осей одинаков, иначе окружность выглядит эллипсом
        # и восьмёрка теряет смысл.
        self._path_plot.setAspectLocked(True)

        self._heading_plot = _prepare_plot(
            graphics.addPlot(row=1, col=0, title="Курс"),
            "рад",
            "время, с",
            legend=True,
        )

        dashed = QtCore.Qt.DashLine

        self._ideal_curve = self._path_plot.plot(
            pen=pg.mkPen(_palette["setpoint"], width=2, style=dashed),
            name="идеальный путь",
        )
        self._actual_curve = self._path_plot.plot(
            pen=pg.mkPen(_palette["measured"], width=2), name="по энкодерам"
        )
        self._start_marker = self._path_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=11,
            symbolBrush=_palette["feedforward"], symbolPen=None, name="старт",
        )
        self._finish_marker = self._path_plot.plot(
            [], [], pen=None, symbol="x", symbolSize=13,
            symbolPen=pg.mkPen(_palette["proportional"], width=3), name="финиш",
        )

        self._ideal_heading = self._heading_plot.plot(
            pen=pg.mkPen(_palette["setpoint"], width=2, style=dashed), name="идеальный"
        )
        self._actual_heading = self._heading_plot.plot(
            pen=pg.mkPen(_palette["measured"], width=2), name="по энкодерам"
        )

        self._panel.show()


    def update(
        self,
        ideal_xy: tuple[list[float], list[float]],
        actual_xy: tuple[list[float], list[float]],
        times: list[float],
        ideal_heading: list[float],
        actual_heading: list[float],
        summary: str,
    ) -> None:
        self._ideal_curve.setData(*ideal_xy)
        self._actual_curve.setData(*actual_xy)

        if actual_xy[0]:
            self._start_marker.setData([actual_xy[0][0]], [actual_xy[1][0]])
            self._finish_marker.setData([actual_xy[0][-1]], [actual_xy[1][-1]])

        self._ideal_heading.setData(times, ideal_heading)
        self._actual_heading.setData(times, actual_heading)

        self._panel.set_text(summary)
        self._application.processEvents()


    @property
    def closed(self) -> bool:
        return not self._panel.isVisible()


    def show_blocking(self) -> None:
        """Показать окно и ждать, пока пользователь его не закроет."""

        while self._panel.isVisible():
            self._application.processEvents()
            QtCore.QThread.msleep(20)



    def close(self) -> None:
        self._panel.close()


