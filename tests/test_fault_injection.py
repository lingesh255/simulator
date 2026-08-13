from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from gui.fault_injection import FaultInjectionPanel


def test_fault_targets_toggle_on_row_click(qtbot):
    app = QApplication.instance() or QApplication([])
    panel = FaultInjectionPanel()
    panel.update_active_sysids([1, 2, 3])

    item = panel.drone_list.item(0)
    assert item.checkState() == Qt.Unchecked

    panel.drone_list.itemClicked.emit(item)

    assert item.checkState() == Qt.Checked
    assert panel._checked_sysids() == [1]
