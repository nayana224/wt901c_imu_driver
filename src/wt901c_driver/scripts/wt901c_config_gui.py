#!/usr/bin/env python3

import os
import shutil
import sys
from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import QProcess, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)


BAUDRATES = ["4800", "9600", "19200", "38400", "57600", "115200", "230400"]
OUTPUT_RATES = ["0.2", "0.5", "1", "2", "5", "10", "20", "50", "100", "200"]
BANDWIDTHS = ["256", "188", "98", "42", "20", "10", "5"]
OUTPUT_NAMES = [
    "time",
    "accel",
    "gyro",
    "angle",
    "mag",
    "port",
    "pressure",
    "gps",
    "velocity",
    "quaternion",
    "gsa",
]


class ConfiguratorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("WT901C Configurator")
        self.resize(880, 760)

        self.process: Optional[QProcess] = None
        self.pending_callback: Optional[Callable[[str], None]] = None
        self.pending_baud_after_write: Optional[str] = None
        self.current_values: Dict[str, str] = {}

        self.cli_path = self._find_cli()
        self._build_ui()

    def _find_cli(self) -> str:
        sibling = os.path.join(os.path.dirname(os.path.realpath(sys.argv[0])), "wt901c_config")
        if os.path.isfile(sibling) and os.access(sibling, os.X_OK):
            return sibling
        discovered = shutil.which("wt901c_config")
        if discovered:
            return discovered
        return sibling

    def _build_ui(self) -> None:
        central = QWidget(self)
        root = QVBoxLayout(central)
        self.setCentralWidget(central)

        warning = QLabel(
            "Important: stop imu_driver before using this app. "
            "The driver and configurator must not own the same serial port at the same time."
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("font-weight: 600;")
        root.addWidget(warning)

        root.addWidget(self._build_connection_group())
        root.addWidget(self._build_current_group())
        root.addWidget(self._build_settings_group())
        root.addWidget(self._build_calibration_group())

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText("Command output and errors appear here.")
        root.addWidget(self.log, 1)

        self.statusBar().showMessage(f"CLI: {self.cli_path}")

    def _build_connection_group(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QGridLayout(group)

        self.port_edit = QLineEdit("/dev/ttyUSB0")
        self.current_baud_combo = QComboBox()
        self.current_baud_combo.addItems(BAUDRATES)
        self.current_baud_combo.setCurrentText("115200")

        self.detect_button = QPushButton("Detect Baud")
        self.read_button = QPushButton("Read Current Settings")
        self.detect_button.clicked.connect(self.detect_baud)
        self.read_button.clicked.connect(self.read_status)

        self.connection_label = QLabel("Not read")

        layout.addWidget(QLabel("Serial port"), 0, 0)
        layout.addWidget(self.port_edit, 0, 1)
        layout.addWidget(QLabel("Current baud"), 0, 2)
        layout.addWidget(self.current_baud_combo, 0, 3)
        layout.addWidget(self.detect_button, 1, 0)
        layout.addWidget(self.read_button, 1, 1)
        layout.addWidget(QLabel("State"), 1, 2)
        layout.addWidget(self.connection_label, 1, 3)
        return group

    def _build_current_group(self) -> QGroupBox:
        group = QGroupBox("Current Sensor Settings")
        layout = QFormLayout(group)
        self.current_labels: Dict[str, QLabel] = {}
        fields = [
            ("baudrate", "Baudrate"),
            ("output_rate_hz", "Output rate"),
            ("output_content", "Output content"),
            ("output_content_mask", "Output mask"),
            ("bandwidth_hz", "Bandwidth"),
            ("installation_direction", "Installation"),
            ("led", "LED"),
            ("algorithm", "Algorithm"),
            ("power_on_output", "Power-on output"),
            ("version", "Version"),
        ]
        for key, title in fields:
            label = QLabel("-")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self.current_labels[key] = label
            layout.addRow(title, label)
        return group

    def _build_settings_group(self) -> QGroupBox:
        group = QGroupBox("Change Settings")
        layout = QGridLayout(group)

        self.new_baud_combo = QComboBox()
        self.new_baud_combo.addItems(BAUDRATES)
        self.new_baud_combo.setCurrentText("115200")
        baud_button = QPushButton("Apply Baudrate")
        baud_button.clicked.connect(self.apply_baudrate)

        self.rate_combo = QComboBox()
        self.rate_combo.addItems(OUTPUT_RATES)
        self.rate_combo.setCurrentText("100")
        rate_button = QPushButton("Apply Output Rate")
        rate_button.clicked.connect(
            lambda: self.apply_simple("output-rate", self.rate_combo.currentText())
        )

        self.bandwidth_combo = QComboBox()
        self.bandwidth_combo.addItems(BANDWIDTHS)
        self.bandwidth_combo.setCurrentText("20")
        bandwidth_button = QPushButton("Apply Bandwidth")
        bandwidth_button.clicked.connect(
            lambda: self.apply_simple("bandwidth", self.bandwidth_combo.currentText())
        )

        self.install_combo = QComboBox()
        self.install_combo.addItems(["horizontal", "vertical"])
        install_button = QPushButton("Apply Installation")
        install_button.clicked.connect(
            lambda: self.apply_simple("install-direction", self.install_combo.currentText())
        )

        self.led_combo = QComboBox()
        self.led_combo.addItems(["on", "off"])
        led_button = QPushButton("Apply LED")
        led_button.clicked.connect(lambda: self.apply_simple("led", self.led_combo.currentText()))

        layout.addWidget(QLabel("Baudrate"), 0, 0)
        layout.addWidget(self.new_baud_combo, 0, 1)
        layout.addWidget(baud_button, 0, 2)
        layout.addWidget(QLabel("Output rate (Hz)"), 1, 0)
        layout.addWidget(self.rate_combo, 1, 1)
        layout.addWidget(rate_button, 1, 2)
        layout.addWidget(QLabel("Bandwidth (Hz)"), 2, 0)
        layout.addWidget(self.bandwidth_combo, 2, 1)
        layout.addWidget(bandwidth_button, 2, 2)
        layout.addWidget(QLabel("Installation"), 3, 0)
        layout.addWidget(self.install_combo, 3, 1)
        layout.addWidget(install_button, 3, 2)
        layout.addWidget(QLabel("LED"), 4, 0)
        layout.addWidget(self.led_combo, 4, 1)
        layout.addWidget(led_button, 4, 2)

        output_box = QGroupBox("Output Packets")
        output_layout = QGridLayout(output_box)
        self.output_checks: Dict[str, QCheckBox] = {}
        default_enabled = {"accel", "gyro", "angle", "mag"}
        for index, name in enumerate(OUTPUT_NAMES):
            checkbox = QCheckBox(name)
            checkbox.setChecked(name in default_enabled)
            self.output_checks[name] = checkbox
            output_layout.addWidget(checkbox, index // 4, index % 4)
        output_button = QPushButton("Apply Output Packets")
        output_button.clicked.connect(self.apply_output_content)
        output_layout.addWidget(output_button, 3, 0, 1, 4)

        layout.addWidget(output_box, 5, 0, 1, 3)
        return group

    def _build_calibration_group(self) -> QGroupBox:
        group = QGroupBox("Calibration / Reference")
        layout = QHBoxLayout(group)

        heading_button = QPushButton("Set Heading Zero")
        mag_start_button = QPushButton("Start Magnetic Calibration")
        mag_stop_button = QPushButton("Stop && Save Magnetic Calibration")

        heading_button.clicked.connect(self.apply_heading_zero)
        mag_start_button.clicked.connect(lambda: self.apply_mag_calibration("start"))
        mag_stop_button.clicked.connect(lambda: self.apply_mag_calibration("stop"))

        layout.addWidget(heading_button)
        layout.addWidget(mag_start_button)
        layout.addWidget(mag_stop_button)
        return group

    def _base_args(self) -> List[str]:
        return [
            "--port",
            self.port_edit.text().strip(),
            "--baudrate",
            self.current_baud_combo.currentText(),
        ]

    def _run_cli(
        self,
        command_args: List[str],
        callback: Optional[Callable[[str], None]] = None,
    ) -> None:
        if self.process is not None:
            QMessageBox.information(self, "WT901C", "A command is already running.")
            return
        if not os.path.exists(self.cli_path):
            QMessageBox.critical(
                self,
                "WT901C",
                "wt901c_config executable was not found. Rebuild and source the workspace.",
            )
            return

        args = self._base_args() + command_args
        self.log.appendPlainText("\n$ " + self.cli_path + " " + " ".join(args))
        self.pending_callback = callback
        self.process = QProcess(self)
        self.process.setProgram(self.cli_path)
        self.process.setArguments(args)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.finished.connect(self._process_finished)
        self.process.start()
        self._set_busy(True)

    def _process_finished(self, exit_code: int, _status: QProcess.ExitStatus) -> None:
        assert self.process is not None
        output = bytes(self.process.readAll()).decode(errors="replace")
        self.log.appendPlainText(output.rstrip())
        callback = self.pending_callback
        self.pending_callback = None
        self.process.deleteLater()
        self.process = None
        self._set_busy(False)

        if exit_code != 0:
            self.connection_label.setText("Command failed")
            QMessageBox.warning(self, "WT901C", "Command failed. See the log for details.")
            self.pending_baud_after_write = None
            return

        if self.pending_baud_after_write is not None:
            self.current_baud_combo.setCurrentText(self.pending_baud_after_write)
            self.pending_baud_after_write = None

        if callback is not None:
            callback(output)

    def _set_busy(self, busy: bool) -> None:
        self.detect_button.setEnabled(not busy)
        self.read_button.setEnabled(not busy)
        self.port_edit.setEnabled(not busy)
        self.current_baud_combo.setEnabled(not busy)
        self.statusBar().showMessage("Command running..." if busy else f"CLI: {self.cli_path}")

    @staticmethod
    def _parse_key_values(output: str) -> Dict[str, str]:
        values: Dict[str, str] = {}
        for raw_line in output.splitlines():
            line = raw_line.strip()
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
        return values

    def detect_baud(self) -> None:
        self.connection_label.setText("Probing...")
        self._run_cli(["detect-baud"], self._detected_baud)

    def _detected_baud(self, output: str) -> None:
        values = self._parse_key_values(output)
        detected = values.get("detected_baudrate")
        if not detected:
            self.connection_label.setText("Not detected")
            return
        self.current_baud_combo.setCurrentText(detected)
        self.connection_label.setText(f"Connected @ {detected}")
        self.read_status()

    def read_status(self) -> None:
        self.connection_label.setText("Reading...")
        self._run_cli(["status"], self._status_read)

    def _status_read(self, output: str) -> None:
        values = self._parse_key_values(output)
        self.current_values = values
        for key, label in self.current_labels.items():
            label.setText(values.get(key, "-"))

        baud = values.get("baudrate")
        if baud in BAUDRATES:
            self.current_baud_combo.setCurrentText(baud)
            self.new_baud_combo.setCurrentText(baud)
        rate = values.get("output_rate_hz")
        if rate in OUTPUT_RATES:
            self.rate_combo.setCurrentText(rate)
        bandwidth = values.get("bandwidth_hz")
        if bandwidth in BANDWIDTHS:
            self.bandwidth_combo.setCurrentText(bandwidth)
        installation = values.get("installation_direction")
        if installation in {"horizontal", "vertical"}:
            self.install_combo.setCurrentText(installation)
        led = values.get("led")
        if led in {"on", "off"}:
            self.led_combo.setCurrentText(led)

        enabled = set(filter(None, values.get("output_content", "").split(",")))
        if enabled:
            for name, checkbox in self.output_checks.items():
                checkbox.setChecked(name in enabled)

        self.connection_label.setText(
            f"Connected @ {self.current_baud_combo.currentText()}"
        )

    def _confirm(self, title: str, message: str) -> bool:
        return (
            QMessageBox.question(self, title, message, QMessageBox.Yes | QMessageBox.No)
            == QMessageBox.Yes
        )

    def apply_baudrate(self) -> None:
        new_baud = self.new_baud_combo.currentText()
        current_baud = self.current_baud_combo.currentText()
        if new_baud == current_baud:
            QMessageBox.information(self, "WT901C", "The selected baudrate is already in use.")
            return
        if not self._confirm(
            "Change baudrate",
            f"Change the sensor baudrate from {current_baud} to {new_baud}?",
        ):
            return
        self.pending_baud_after_write = new_baud
        self._run_cli(["baudrate", new_baud], lambda _output: self.read_status())

    def apply_simple(self, command: str, value: str) -> None:
        self._run_cli([command, value], lambda _output: self.read_status())

    def apply_output_content(self) -> None:
        enabled = [name for name, checkbox in self.output_checks.items() if checkbox.isChecked()]
        if not enabled:
            QMessageBox.warning(self, "WT901C", "Enable at least one output packet.")
            return
        if "accel" not in enabled or "gyro" not in enabled:
            if not self._confirm(
                "Output packets",
                "accel or gyro is disabled. imu/data may stop publishing. Continue?",
            ):
                return
        self._run_cli(
            ["output-content", ",".join(enabled)],
            lambda _output: self.read_status(),
        )

    def apply_heading_zero(self) -> None:
        if not self._confirm(
            "Set heading zero",
            "Use the sensor's current heading as zero and save the setting?",
        ):
            return
        self._run_cli(["heading-zero"], lambda _output: self.read_status())

    def apply_mag_calibration(self, mode: str) -> None:
        if mode == "start":
            message = (
                "Start spherical magnetic calibration? Rotate the sensor through multiple "
                "orientations, then press Stop & Save."
            )
        else:
            message = "Stop magnetic calibration and save the result?"
        if not self._confirm("Magnetic calibration", message):
            return
        self._run_cli(["mag-calibration", mode])


def main() -> int:
    app = QApplication(sys.argv)
    window = ConfiguratorWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
