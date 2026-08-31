#ifndef THEME_H
#define THEME_H

#include <QApplication>
#include <QColor>
#include <QPalette>
#include <QString>
#include <QStyleFactory>

namespace theme
{

inline QColor bgBase()        { return QColor("#16191D"); }
inline QColor bgSurface()     { return QColor("#1E2227"); }
inline QColor bgRaised()      { return QColor("#262B31"); }
inline QColor bgInput()       { return QColor("#12151A"); }
inline QColor border()        { return QColor("#333A42"); }
inline QColor borderStrong()  { return QColor("#454E58"); }
inline QColor textPrimary()   { return QColor("#E3E7EC"); }
inline QColor textDim()       { return QColor("#9AA4B0"); }
inline QColor textDisabled()  { return QColor("#5C6570"); }
inline QColor accent()        { return QColor("#4C9AFF"); }
inline QColor accentDim()     { return QColor("#2A5A9E"); }

inline QColor plotCanvas()      { return QColor("#12151A"); }
inline QColor plotGrid()        { return QColor("#2A3138"); }
inline QColor plotPosition()    { return QColor("#E8EDF2"); }
inline QColor plotGoal()        { return QColor("#4C9AFF"); }
inline QColor plotTorque()      { return QColor("#FFA94D"); }
inline QColor plotSpeed()       { return QColor("#51CF66"); }
inline QColor plotCurrent()     { return QColor("#22D3EE"); }
inline QColor plotTemperature() { return QColor("#FFD43B"); }
inline QColor plotVoltage()     { return QColor("#C77DFF"); }
inline QColor plotLimit()       { return QColor("#FF6B9D"); }

inline QColor statusReleased() { return QColor("#3FB950"); }
inline QColor statusEngaged()  { return QColor("#E0A030"); }
inline QColor statusMixed()    { return QColor("#E05252"); }
inline QColor statusUnknown()  { return QColor("#6B7280"); }

inline QPalette darkPalette()
{
    QPalette p;

    p.setColor(QPalette::Window, bgSurface());
    p.setColor(QPalette::WindowText, textPrimary());
    p.setColor(QPalette::Base, bgInput());
    p.setColor(QPalette::AlternateBase, bgSurface());
    p.setColor(QPalette::Text, textPrimary());
    p.setColor(QPalette::Button, bgRaised());
    p.setColor(QPalette::ButtonText, textPrimary());
    p.setColor(QPalette::BrightText, QColor("#FFFFFF"));
    p.setColor(QPalette::ToolTipBase, bgRaised());
    p.setColor(QPalette::ToolTipText, textPrimary());
    p.setColor(QPalette::PlaceholderText, textDisabled());
    p.setColor(QPalette::Highlight, accent());
    p.setColor(QPalette::HighlightedText, QColor("#0B0E12"));
    p.setColor(QPalette::Link, accent());
    p.setColor(QPalette::LinkVisited, QColor("#B197FC"));
    p.setColor(QPalette::Light, borderStrong());
    p.setColor(QPalette::Midlight, border());
    p.setColor(QPalette::Mid, border());
    p.setColor(QPalette::Dark, bgBase());
    p.setColor(QPalette::Shadow, QColor("#0B0E12"));

    p.setColor(QPalette::Disabled, QPalette::WindowText, textDisabled());
    p.setColor(QPalette::Disabled, QPalette::Text, textDisabled());
    p.setColor(QPalette::Disabled, QPalette::ButtonText, textDisabled());
    p.setColor(QPalette::Disabled, QPalette::Base, bgBase());
    p.setColor(QPalette::Disabled, QPalette::Button, bgSurface());
    p.setColor(QPalette::Disabled, QPalette::Highlight, QColor("#2E353D"));
    p.setColor(QPalette::Disabled, QPalette::HighlightedText, textDisabled());

    return p;
}

inline QString darkStyleSheet()
{
    return QString(R"QSS(
QMainWindow, QDialog {
    background: #1E2227;
}

QWidget:disabled {
    color: #5C6570;
}

QToolTip {
    background: #262B31;
    color: #E3E7EC;
    border: 1px solid #454E58;
    padding: 4px 6px;
}

QMenuBar {
    background: #16191D;
    color: #E3E7EC;
    border-bottom: 1px solid #333A42;
}
QMenuBar::item {
    padding: 4px 10px;
    background: transparent;
}
QMenuBar::item:selected {
    background: #262B31;
}

QStatusBar {
    background: #16191D;
    color: #9AA4B0;
    border-top: 1px solid #333A42;
}

QTabWidget::pane {
    background: #1E2227;
    border: 1px solid #333A42;
    border-radius: 4px;
    top: -1px;
}
QTabBar {
    background: transparent;
}
QTabBar::tab {
    background: #16191D;
    color: #9AA4B0;
    border: 1px solid #333A42;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    padding: 7px 16px;
    margin-right: 2px;
}
QTabBar::tab:hover {
    background: #22262B;
    color: #E3E7EC;
}
QTabBar::tab:selected {
    background: #1E2227;
    color: #E3E7EC;
    border-bottom: 2px solid #4C9AFF;
}

QGroupBox {
    background: #1E2227;
    border: 1px solid #333A42;
    border-radius: 5px;
    margin-top: 10px;
    padding-top: 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0px 5px;
    color: #9AA4B0;
    background: #1E2227;
}

QLabel {
    background: transparent;
    color: #E3E7EC;
}
QLabel:disabled {
    color: #5C6570;
}

QLineEdit, QPlainTextEdit, QTextEdit {
    background: #12151A;
    color: #E3E7EC;
    border: 1px solid #333A42;
    border-radius: 4px;
    padding: 4px 6px;
    selection-background-color: #4C9AFF;
    selection-color: #0B0E12;
}
QLineEdit:focus, QPlainTextEdit:focus {
    border: 1px solid #4C9AFF;
}
QLineEdit:disabled, QPlainTextEdit:disabled {
    background: #16191D;
    color: #5C6570;
    border: 1px solid #262B31;
}
QLineEdit:read-only {
    background: #16191D;
    color: #9AA4B0;
}

QPushButton {
    background: #262B31;
    color: #E3E7EC;
    border: 1px solid #3C444D;
    border-radius: 4px;
    padding: 5px 14px;
    min-height: 18px;
}
QPushButton:hover {
    background: #2F353D;
    border: 1px solid #4C5663;
}
QPushButton:pressed {
    background: #1A1E23;
}
QPushButton:focus {
    border: 1px solid #4C9AFF;
}
QPushButton:disabled {
    background: #1B1F24;
    color: #5C6570;
    border: 1px solid #262B31;
}

QComboBox {
    background: #262B31;
    color: #E3E7EC;
    border: 1px solid #3C444D;
    border-radius: 4px;
    padding: 4px 6px;
    min-height: 18px;
}
QComboBox:hover {
    border: 1px solid #4C5663;
}
QComboBox:focus {
    border: 1px solid #4C9AFF;
}
QComboBox:disabled {
    background: #1B1F24;
    color: #5C6570;
    border: 1px solid #262B31;
}
QComboBox::drop-down {
    subcontrol-origin: padding;
    subcontrol-position: center right;
    width: 18px;
    border-left: 1px solid #3C444D;
}
QComboBox QAbstractItemView {
    background: #1E2227;
    color: #E3E7EC;
    border: 1px solid #454E58;
    selection-background-color: #4C9AFF;
    selection-color: #0B0E12;
    outline: none;
}

QCheckBox, QRadioButton {
    background: transparent;
    color: #E3E7EC;
    spacing: 7px;
    padding: 1px;
}
QCheckBox:disabled, QRadioButton:disabled {
    color: #5C6570;
}
QCheckBox::indicator, QRadioButton::indicator {
    width: 14px;
    height: 14px;
    background: #12151A;
    border: 1px solid #4C5663;
}
QCheckBox::indicator {
    border-radius: 3px;
}
QRadioButton::indicator {
    border-radius: 8px;
}
QCheckBox::indicator:hover, QRadioButton::indicator:hover {
    border: 1px solid #4C9AFF;
}
QCheckBox::indicator:checked, QRadioButton::indicator:checked {
    background: #4C9AFF;
    border: 1px solid #4C9AFF;
}
QCheckBox::indicator:disabled, QRadioButton::indicator:disabled {
    background: #16191D;
    border: 1px solid #2E353D;
}
QCheckBox::indicator:checked:disabled, QRadioButton::indicator:checked:disabled {
    background: #3A4550;
    border: 1px solid #3A4550;
}

QSlider::groove:horizontal {
    background: #12151A;
    border: 1px solid #2E353D;
    height: 5px;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #4C9AFF;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #D7DEE6;
    border: 1px solid #6B7684;
    width: 13px;
    margin: -5px 0;
    border-radius: 7px;
}
QSlider::handle:horizontal:hover {
    background: #FFFFFF;
}
QSlider::groove:horizontal:disabled {
    background: #16191D;
}
QSlider::sub-page:horizontal:disabled {
    background: #333A42;
}
QSlider::handle:horizontal:disabled {
    background: #3A4550;
    border: 1px solid #333A42;
}

QTableView {
    background: #12151A;
    alternate-background-color: #171B20;
    color: #E3E7EC;
    gridline-color: #262C33;
    border: 1px solid #333A42;
    border-radius: 4px;
    selection-background-color: #2A5A9E;
    selection-color: #FFFFFF;
    outline: none;
}
QTableView::item {
    padding: 3px 5px;
    border: none;
}
QTableView::item:selected {
    background: #2A5A9E;
    color: #FFFFFF;
}
QTableView:disabled {
    background: #16191D;
    color: #5C6570;
    gridline-color: #21262C;
    border: 1px solid #262B31;
}

QHeaderView {
    background: transparent;
    border: none;
}
QHeaderView::section {
    background: #262B31;
    color: #9AA4B0;
    padding: 5px 6px;
    border: none;
    border-right: 1px solid #333A42;
    border-bottom: 1px solid #333A42;
}
QHeaderView::section:hover {
    background: #2F353D;
    color: #E3E7EC;
}
QHeaderView::section:disabled {
    background: #1B1F24;
    color: #5C6570;
}
QTableCornerButton::section {
    background: #262B31;
    border: none;
}

QScrollBar:vertical {
    background: #16191D;
    width: 11px;
    margin: 0px;
    border: none;
}
QScrollBar::handle:vertical {
    background: #3C444D;
    min-height: 26px;
    border-radius: 5px;
}
QScrollBar::handle:vertical:hover {
    background: #4C5663;
}
QScrollBar:horizontal {
    background: #16191D;
    height: 11px;
    margin: 0px;
    border: none;
}
QScrollBar::handle:horizontal {
    background: #3C444D;
    min-width: 26px;
    border-radius: 5px;
}
QScrollBar::handle:horizontal:hover {
    background: #4C5663;
}
QScrollBar::add-line, QScrollBar::sub-line {
    height: 0px;
    width: 0px;
    background: none;
    border: none;
}
QScrollBar::add-page, QScrollBar::sub-page {
    background: none;
}

QMessageBox {
    background: #1E2227;
}
QMessageBox QLabel {
    color: #E3E7EC;
}
)QSS");
}

inline void applyDarkTheme(QApplication &app)
{
    app.setStyle(QStyleFactory::create("Fusion"));
    app.setPalette(darkPalette());
    app.setStyleSheet(darkStyleSheet());
}

}

#endif
