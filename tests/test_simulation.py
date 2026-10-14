"""Offline simulator lifecycle and isolation checks. Run with system Python."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import unittest
from PyQt5.QtTest import QTest
import main


class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = main.QApplication.instance() or main.QApplication([])

    def setUp(self):
        self.window = main.MainWindow()

    def tearDown(self):
        if self.window.simulating:
            self.window.toggle_simulation(False)
        self.window.pages[0].timer.stop()
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()

    def test_offline_animation_and_command_isolation(self):
        w = self.window
        calls = []
        w.device._exec = lambda *args: calls.append(args)
        w.sim_button.click()
        QTest.qWait(350)
        p = w.pages[0]
        self.assertGreaterEqual(w.sim_frame, 3)
        self.assertGreater(len(p.map.fcu_path), 1)
        self.assertGreater(len(p.map.algo_path), 1)
        self.assertTrue(p.last.simulated)
        self.assertFalse(w.device.connected)
        self.assertFalse(p.rgb.image.pixmap().isNull())
        self.assertFalse(p.ir.image.pixmap().isNull())
        self.assertFalse(p.sensor_btn.isEnabled())
        w.start_roscore()
        w.toggle_connection()
        w.stop_all_sensors()
        self.assertEqual(calls, [])

    def test_restore_map_and_repeat(self):
        w = self.window
        m = w.pages[0].map
        original = main.np.zeros((300, 400, 3), main.np.uint8)
        m.map_image = original
        m.fcu_path = [(20, 30)]
        path = m.fcu_path
        for _ in range(2):
            w.sim_button.click()
            w.sim_button.click()
            self.assertFalse(w.simulating)
            self.assertFalse(w.sim_timer.isActive())
            self.assertIs(m.map_image, original)
            self.assertIs(m.fcu_path, path)
            self.assertFalse(w.pages[0].last.simulated)
            self.assertTrue(w.pages[0].sensor_btn.isEnabled())
            self.assertIsNone(w.pages[0].rgb.image.pixmap())

    def test_connected_board_prevents_simulation(self):
        w = self.window
        w.device.connected = True
        w.sim_button.click()
        self.assertFalse(w.simulating)
        self.assertFalse(w.sim_button.isChecked())
        self.assertFalse(w.sim_timer.isActive())


if __name__ == "__main__":
    unittest.main()
