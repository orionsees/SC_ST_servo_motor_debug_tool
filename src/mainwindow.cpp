#include "mainwindow.h"
#include "theme.h"
#include "ui_mainwindow.h"
#include <QSerialPort>
#include <QAbstractItemView>
#include <QTimer>
#include <QTableView>
#include <QScrollBar>
#include <QStandardItemModel>
#include <QtCore/QDebug>
#include <QLineEdit>
#include <QIntValidator>
#include <QRegExpValidator>
#include <QDir>
#include <QMessageBox>
#include <QThread>
#include <QComboBox>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QPainter>
#include <QPixmap>
#include <QIcon>
#include <QSignalBlocker>
#include <QGuiApplication>
#include <QScopeGuard>
#include <cmath>

namespace
{

using feetech_servo::TORQUE_ENABLE_ADDRESS;
using feetech_servo::decodeSignMagnitude;
using feetech_servo::encodeSignMagnitude;
using feetech_servo::isSignMagnitude;
using feetech_servo::seriesName;

constexpr qint64 CONTROL_FOLLOW_RESUME_MS = 1500;

// Pause between telemetry samples. The bus is free during it, which leaves
// room for the interactive commands the user is actually waiting on.
constexpr int STATUS_POLL_GAP_MS = 10;
// How often to look again when there is nothing to sample -- no port, no
// servo, or the debug tab is not the one on screen.
constexpr int STATUS_IDLE_RETRY_MS = 200;

// Wait cursor for the user-initiated operations that run long enough to
// notice. The periodic polls deliberately do not use it: at their cadence the
// cursor would never settle.
struct BusyCursor
{
    BusyCursor() { QGuiApplication::setOverrideCursor(Qt::WaitCursor); }
    ~BusyCursor() { QGuiApplication::restoreOverrideCursor(); }
};

}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("ServoBench");

    graph_timer_ = new QTimer(this);
    connect(graph_timer_, &QTimer::timeout, this, &MainWindow::onGraphTimerTimeout);
    graph_timer_->start(30);

    // Every servo transaction runs here, so a blocking read never stalls the
    // window. The bus owns the port; the UI only posts work to it.
    bus_thread_ = new QThread(this);
    rebuildBus(false);

    setupComSettings();
    setupServoLists();

    setupServoControl();
    setupAutoDebug();
    setupDataAnalysis();

    setupProgramming();
    setupCalibration();
    styleGraphLegend();
    updateFunctionalityGate();

    setIntRangeLineEdit(ui->upLimitLineEdit, 0, 1200);
    setIntRangeLineEdit(ui->downLimitLineEdit, 0, 1200);

    ui->graphWidget->setToolTip(
        "Wheel: zoom the time axis about the cursor\n"
        "Shift + wheel: zoom the value axes\n"
        "Drag: pan\n"
        "Double-click: reset the view");

    requestNextStatus();
}

MainWindow::~MainWindow()
{
    // Shut the bus down on its own thread, and wait for it. quit() is not a
    // queued event -- it sets the loop's exit flag directly, so a merely posted
    // close can be dropped without ever running, leaving the port or the
    // socket to be torn down on this thread instead. Their notifiers belong to
    // the bus thread, so that has to be avoided.
    QMetaObject::invokeMethod(bus_, [this]{
        // A remote port belongs to the server, which keeps serving after this
        // window closes; letting go of the link is all that is ours to do.
        if(!bus_->isRemote())
            bus_->close();
        bus_->disconnectTransport();
    }, Qt::BlockingQueuedConnection);
    bus_thread_->quit();
    bus_thread_->wait();
    delete bus_;

    delete ui;
    delete graph_timer_;
    delete servo_list_model_;
    delete prog_mem_model_;
    delete port_search_timer_;
    delete link_status_timer_;
    delete auto_debug_timer_;
    delete prog_timer_;
}

void MainWindow::setupComSettings()
{
    QStringList baudRates = {"1000000", "500000", "250000", "256000", "128000", "115200", "76800", "57600", "38400", "19200", "9600", "4800"};
    ui->BaudComboBox->addItems(baudRates);

    QStringList parity = {"NONE", "ODD", "EVEN"};
    ui->ParityComboBox->addItems(parity);

    setIntRangeLineEdit(ui->timeoutLineEdit, 0, 10000);

    ui->LinkComboBox->addItems(QStringList() << "Serial" << "Network");
    setIntRangeLineEdit(ui->netPortLineEdit, 1, 65535);
    // A token is a credential, and this panel is the part of the window most
    // likely to be on screen while someone is demonstrating a robot.
    ui->tokenLineEdit->setEchoMode(QLineEdit::Password);
    connect(ui->LinkComboBox, &QComboBox::currentTextChanged, this, &MainWindow::onLinkModeChanged);
    updateLinkFields();
    updateLinkStatus();

    port_refresh_clock_.start();
    onPortSearchTimerTimeout();
    connect(ui->ComOpenButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    port_search_timer_ = new QTimer(this);
    connect(port_search_timer_, &QTimer::timeout, this, &MainWindow::onPortSearchTimerTimeout);
    port_search_timer_->start(500);

    // Only the latency reading moves on its own, so this is slow on purpose.
    link_status_timer_ = new QTimer(this);
    connect(link_status_timer_, &QTimer::timeout, this, &MainWindow::onLinkStatusTimerTimeout);
    link_status_timer_->start(1000);
}

void MainWindow::setupServoLists()
{
    connect(ui->SearchButton, &QPushButton::clicked, this, &MainWindow::onSearchButtonClicked);

    ui->ServoSearchText->setText("Stop");
    servo_list_model_ = new QStandardItemModel(0, 2);
    ui->ServoListView->setModel(servo_list_model_);
    clearServoList();

    connect(ui->torqueAllButton, &QPushButton::clicked, this, &MainWindow::onTorqueAllButtonClicked);
    updateTorqueAllButton();

    torque_state_timer_ = new QTimer(this);
    connect(torque_state_timer_, &QTimer::timeout, this, &MainWindow::onTorqueStateTimerTimeout);
    torque_state_timer_->start(1000);
    connect(ui->ServoListView->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onServoListSelection);
}

void MainWindow::setupServoControl()
{
    connect(ui->writeRadioButton, &QRadioButton::toggled, this, &MainWindow::onModeRadioButtonsToggled);
    connect(ui->syncWriteRadioButton, &QRadioButton::toggled, this, &MainWindow::onModeRadioButtonsToggled);
    connect(ui->regWriteRadioButton, &QRadioButton::toggled, this, &MainWindow::onModeRadioButtonsToggled);

    connect(ui->goalSlider, &QSlider::valueChanged, this, &MainWindow::onGoalSliderValueChanged);

    setIntRangeLineEdit(ui->accLineEdit, 0, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->speedLineEdit, 0, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->goalLineEdit, 0, 4095);
    setIntRangeLineEdit(ui->timeLineEdit, 0, std::numeric_limits<int>::max());

    connect(ui->setPushButton, &QPushButton::clicked, this, &MainWindow::onSetBuggonClicked);
    connect(ui->setPushButton, &QPushButton::pressed, this, &MainWindow::noteControlInteraction);
    connect(ui->torqueEnableCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onTorqueEnableCheckBoxStateChanged);
    connect(ui->actionPushButton, &QPushButton::clicked, this, &MainWindow::onActionButtonClicked);

    control_touch_.start();
    connect(ui->goalLineEdit, &QLineEdit::textEdited, this, &MainWindow::noteControlInteraction);
    connect(ui->angleLineEdit, &QLineEdit::textEdited, this, &MainWindow::noteControlInteraction);
    setDoubleLineEdit(ui->angleLineEdit);
    connect(ui->angleSetButton, &QPushButton::clicked, this, &MainWindow::onAngleSetButtonClicked);
    connect(ui->angleSetButton, &QPushButton::pressed, this, &MainWindow::noteControlInteraction);
    connect(ui->angleLineEdit, &QLineEdit::textChanged, this, &MainWindow::onAngleInputChanged);
    connect(ui->angleUnitComboBox, &QComboBox::currentTextChanged, this, &MainWindow::onAngleUnitChanged);
    updateAngleInputFromCounts(ui->goalSlider->value());
}

void MainWindow::setupAutoDebug()
{
    setIntRangeLineEdit(ui->startLineEdit, 0, 4095);
    setIntRangeLineEdit(ui->endLineEdit, 0, 4095);
    setIntRangeLineEdit(ui->sweepLineEdit, 0, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->stepSizeLineEdit, 1, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->stepDelayLineEdit, 1, std::numeric_limits<int>::max());

    // Reads as guidance rather than as another control.
    QFont hint_font = ui->autoDebugHintLabel->font();
    hint_font.setPointSizeF(qMax(7.5, hint_font.pointSizeF() - 1.0));
    ui->autoDebugHintLabel->setFont(hint_font);
    ui->autoDebugHintLabel->setStyleSheet(
        QString("color: %1;").arg(theme::textDim().name()));

    connect(ui->sweepButton, &QPushButton::clicked, this, &MainWindow::onSweepButtonClicked);
    connect(ui->stepButton, &QPushButton::clicked, this, &MainWindow::onStepButtonClicked);

    auto_debug_timer_ = new QTimer(this);
    connect(auto_debug_timer_, &QTimer::timeout, this, &MainWindow::onAutoDebugTimerTimeout);
}

void MainWindow::setupDataAnalysis()
{
    connect(ui->exportPushButton, &QPushButton::clicked, this, &MainWindow::onExportButtonClicked);
    connect(ui->clearPushButton, &QPushButton::clicked, this, &MainWindow::onClearButtonClicked);
    setIntRangeLineEdit(ui->recTimeLineEdit, 0, std::numeric_limits<int>::max());

    data_analysis_timer_ = new QTimer(this);
    connect(data_analysis_timer_, &QTimer::timeout, this, &MainWindow::onDataAnalysisTimerTimeout);
    data_analysis_timer_->start(50);
}

void MainWindow::styleGraphLegend()
{
    const std::vector<std::pair<QCheckBox*, QColor>> legend = {
        {ui->posCheckBox,     theme::plotPosition()},
        {ui->goalCheckBox,    theme::plotGoal()},
        {ui->torqueCheckBox,  theme::plotTorque()},
        {ui->speedCheckBox,   theme::plotSpeed()},
        {ui->currentCheckBox, theme::plotCurrent()},
        {ui->tempCheckBox,    theme::plotTemperature()},
        {ui->voltageCheckBox, theme::plotVoltage()},
    };

    for (const auto &entry : legend)
    {
        const QString c = entry.second.name();
        entry.first->setStyleSheet(QString(
            "QCheckBox::indicator { width: 13px; height: 13px; border-radius: 3px;"
            " border: 1px solid %1; background: transparent; }"
            "QCheckBox::indicator:checked { background: %1; border: 1px solid %1; }"
            "QCheckBox::indicator:disabled { border: 1px solid #3A4550; background: transparent; }"
            "QCheckBox::indicator:checked:disabled { background: #3A4550; border: 1px solid #3A4550; }")
            .arg(c));
    }
}

void MainWindow::setupProgramming()
{
    prog_mem_model_ = new QStandardItemModel(0, 5);
    ui->memoryTableView->setModel(prog_mem_model_);
    clearProgMemTable();

    connect(ui->memoryTableView->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onMemoryTableSelection);
    connect(ui->memSetButton, &QPushButton::clicked, this, &MainWindow::onMemSetButtonClicked);
    connect(ui->memSaveButton, &QPushButton::clicked, this, &MainWindow::onMemSaveClicked);
    connect(ui->memLoadButton, &QPushButton::clicked, this, &MainWindow::onMemLoadClicked);
    connect(ui->calibrationMidButton, &QPushButton::clicked, this, &MainWindow::onCalibrationMidButtonClicked);

    prog_timer_ = new QTimer(this);
    connect(prog_timer_, &QTimer::timeout, this, &MainWindow::onProgTimerTimeout);
    prog_timer_->start(50);
}

void MainWindow::setupCalibration()
{
    calib_model_ = new QStandardItemModel(0, 6, this);
    calib_model_->setHorizontalHeaderLabels(
        QStringList() << "ID" << "Joint Name" << "drive_mode" << "homing_offset" << "range_min" << "range_max");
    ui->calibTableView->setModel(calib_model_);
    ui->calibTableView->verticalHeader()->setVisible(false);
    ui->calibTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->calibTableView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->calibTableView->setColumnWidth(0, 50);
    ui->calibTableView->setSelectionBehavior(QAbstractItemView::SelectItems);

    connect(ui->calibRecheckButton, &QPushButton::clicked, this, &MainWindow::onCalibRecheckClicked);
    connect(ui->calibApplyNamesButton, &QPushButton::clicked, this, &MainWindow::onCalibApplyNamesClicked);
    connect(ui->calibNamesLineEdit, &QLineEdit::returnPressed, this, &MainWindow::onCalibApplyNamesClicked);
    connect(ui->calibHomeButton, &QPushButton::clicked, this, &MainWindow::onCalibHomeClicked);
    connect(ui->calibRecordButton, &QPushButton::clicked, this, &MainWindow::onCalibRecordClicked);
    connect(ui->calibExportButton, &QPushButton::clicked, this, &MainWindow::onCalibExportClicked);
    connect(ui->tabWidget, &QTabWidget::currentChanged, this, &MainWindow::onTabChanged);

    calib_timer_ = new QTimer(this);
    connect(calib_timer_, &QTimer::timeout, this, &MainWindow::onCalibTimerTimeout);
    calib_timer_->start(50);

    setCalibrationEnabled(false);
}

void MainWindow::setCalibrationEnabled(bool enabled)
{
    ui->calibHomeButton->setEnabled(enabled);
    ui->calibRecordButton->setEnabled(enabled);
    ui->calibExportButton->setEnabled(enabled);
    ui->calibTableView->setEnabled(enabled);
    ui->calibNamesLineEdit->setEnabled(enabled);
    ui->calibApplyNamesButton->setEnabled(enabled);
}

void MainWindow::updateCalibrationGate()
{
    if(calib_recording_)
        return;

    QString message;
    bool open = false;

    if(!bus_open_)
    {
        message = "Serial port is not open.";
    }
    else if(is_searching_)
    {
        message = "A servo search is still running.";
    }
    else if(id_list_.empty())
    {
        message = "No servos detected. Run Search on the left first.";
    }
    else
    {
        const std::vector<uint8_t> ids = id_list_;
        const feetech_servo::ModelSeries series = select_servo_.model_;
        const std::vector<int> values = runOnBus([this, ids, series]{
            std::vector<int> out;
            out.reserve(ids.size());
            for(uint8_t id : ids)
                out.push_back(bus_->readByte(id, series, TORQUE_ENABLE_ADDRESS));
            return out;
        });

        if(values.size() != ids.size())
            return;

        QStringList torque_on;
        QStringList silent;
        for(std::size_t i = 0; i < ids.size(); i++)
        {
            if(values[i] < 0)
            {
                silent << QString::number(ids[i]);
            }
            else if(values[i] != 0)
            {
                torque_on << QString::number(ids[i]);
            }
        }

        if(!silent.isEmpty())
        {
            message = QString("No response from ID %1.").arg(silent.join(", "));
        }
        else if(!torque_on.isEmpty())
        {
            message = QString("Torque is still enabled on ID %1. Use Torque Off (All) to release it.")
                          .arg(torque_on.join(", "));
        }
        else
        {
            message = QString("Torque disabled on %1 servo(s). Calibration is available.")
                          .arg(static_cast<int>(id_list_.size()));
            open = true;
        }
    }

    ui->calibStatusLabel->setText(message);

    if(open != calib_gate_open_)
    {
        calib_gate_open_ = open;
        setCalibrationEnabled(open);
        if(open)
        {
            populateCalibrationTable();
        }
    }
    else if(open && calib_model_->rowCount() != static_cast<int>(id_list_.size()))
    {
        populateCalibrationTable();
    }
}

void MainWindow::populateCalibrationTable()
{
    std::map<int, QString> previous_names;
    std::map<int, QString> previous_drive;
    for(int row = 0; row < calib_model_->rowCount(); row++)
    {
        const int id = calib_model_->item(row, 0)->text().toInt();
        previous_names[id] = calib_model_->item(row, 1)->text();
        previous_drive[id] = calib_model_->item(row, 2)->text();
    }

    calib_model_->removeRows(0, calib_model_->rowCount());
    calib_series_.clear();
    calib_min_.clear();
    calib_max_.clear();

    std::vector<uint8_t> ids = id_list_;
    std::sort(ids.begin(), ids.end());

    const std::vector<int> model_numbers = runOnBus([this, ids]{
        std::vector<int> out;
        out.reserve(ids.size());
        for(uint8_t id : ids)
            out.push_back(bus_->readModelNumber(id));
        return out;
    });

    if(model_numbers.size() != ids.size())
        return;

    for(std::size_t i = 0; i < ids.size(); i++)
    {
        const uint8_t id = ids[i];
        const feetech_servo::ModelSeries series =
            feetech_servo::getModelSeries(feetech_servo::getModelType(model_numbers[i]));
        calib_series_.push_back(series);
        calib_min_.push_back(0);
        calib_max_.push_back(0);

        QList<QStandardItem*> row;
        auto *id_item = new QStandardItem(QString::number(id));
        id_item->setFlags(id_item->flags() & ~Qt::ItemIsEditable);
        auto *name_item = new QStandardItem(previous_names.count(id) ? previous_names[id] : QString());
        auto *drive_item = new QStandardItem(previous_drive.count(id) ? previous_drive[id] : QString("0"));
        auto *offset_item = new QStandardItem(QString("-"));
        offset_item->setFlags(offset_item->flags() & ~Qt::ItemIsEditable);
        auto *min_item = new QStandardItem(QString("-"));
        auto *max_item = new QStandardItem(QString("-"));

        row << id_item << name_item << drive_item << offset_item << min_item << max_item;
        calib_model_->appendRow(row);
    }
}

void MainWindow::ensureCalibVectors()
{
    const std::size_t rows = static_cast<std::size_t>(calib_model_->rowCount());
    calib_series_.resize(rows, select_servo_.model_);
    calib_min_.resize(rows, 0);
    calib_max_.resize(rows, 0);
}

bool MainWindow::writeCalibWord(uint8_t id, feetech_servo::ModelSeries series, const QString &reg_name, int value)
{
    const feetech_servo::MemoryConfig *config = feetech_servo::findMemConfig(series, reg_name);
    if(config == nullptr)
    {
        return false;
    }

    return runOnBus([this, id, series, config, value]{
        return bus_->writeRegister(id, series, *config, value).ok;
    });
}

// Present Position for every row of the calibration table, in one trip to the
// bus. Rows that did not answer come back as -1.
std::vector<int> MainWindow::readCalibPositions()
{
    ensureCalibVectors();

    std::vector<uint8_t> ids;
    std::vector<feetech_servo::ModelSeries> series;
    for(int row = 0; row < calib_model_->rowCount(); row++)
    {
        ids.push_back(static_cast<uint8_t>(calib_model_->item(row, 0)->text().toInt()));
        series.push_back(calib_series_[row]);
    }

    return runOnBus([this, ids, series]{
        std::vector<int> out;
        out.reserve(ids.size());
        for(std::size_t i = 0; i < ids.size(); i++)
            out.push_back(bus_->readPosition(ids[i], series[i]).value_or(-1));
        return out;
    });
}

std::optional<int> MainWindow::readCalibPosition(uint8_t id, feetech_servo::ModelSeries series)
{
    return runOnBus([this, id, series]{ return bus_->readPosition(id, series); });
}

void MainWindow::onCalibRecheckClicked()
{
    updateCalibrationGate();
}

void MainWindow::onCalibApplyNamesClicked()
{
    const int rows = calib_model_->rowCount();
    if(rows == 0)
    {
        QMessageBox::warning(this, "Apply Names", "No servos in the table yet. Run Search first.");
        return;
    }

    QStringList names;
    for(const QString &token : ui->calibNamesLineEdit->text().split(QRegExp("[,;\n\t]")))
    {
        const QString name = token.trimmed();
        if(!name.isEmpty())
        {
            names << name;
        }
    }

    if(names.isEmpty())
    {
        QMessageBox::warning(this, "Apply Names",
            "No joint names found. Separate them with commas, in ascending servo ID order.");
        return;
    }

    const int applied = std::min(rows, static_cast<int>(names.size()));
    for(int row = 0; row < applied; row++)
    {
        calib_model_->item(row, 1)->setText(names[row]);
    }

    if(names.size() == rows)
    {
        ui->calibStatusLabel->setText(QString("Applied %1 joint name(s) in servo ID order.").arg(rows));
    }
    else
    {
        ui->calibStatusLabel->setText(QString("Applied %1 of %2 row(s).").arg(applied).arg(rows));
        QMessageBox::warning(this, "Apply Names",
            QString("%1 name(s) given for %2 detected servo(s).\n\n"
                    "The first %3 row(s) were filled in ID order. "
                    "Check the list and apply again if that is not what you wanted.")
                .arg(names.size()).arg(rows).arg(applied));
    }
}

void MainWindow::onTabChanged(int index)
{
    if(index == ui->tabWidget->indexOf(ui->CalibrationTab))
    {
        updateCalibrationGate();
    }
}

void MainWindow::onCalibTimerTimeout()
{
    if(ui->tabWidget->currentIndex() != ui->tabWidget->indexOf(ui->CalibrationTab))
        return;

    if(pollsSuspended())
        return;

    if(calib_recording_)
    {
        ensureCalibVectors();

        // Sample every joint in one visit to the bus, so the readings across
        // the arm belong to as close to the same instant as the bus allows.
        const std::vector<int> positions = readCalibPositions();

        for(int row = 0; row < static_cast<int>(positions.size()); row++)
        {
            if(positions[row] < 0)
                continue;

            calib_min_[row] = std::min(calib_min_[row], positions[row]);
            calib_max_[row] = std::max(calib_max_[row], positions[row]);
            calib_model_->item(row, 4)->setText(QString::number(calib_min_[row]));
            calib_model_->item(row, 5)->setText(QString::number(calib_max_[row]));
        }
        return;
    }

    calib_gate_tick_++;
    if(calib_gate_tick_ >= 20)
    {
        calib_gate_tick_ = 0;
        updateCalibrationGate();
    }
}

void MainWindow::onCalibHomeClicked()
{
    const int rows = calib_model_->rowCount();
    if(rows == 0)
        return;

    ensureCalibVectors();

    std::vector<uint8_t> ids;
    for(int row = 0; row < rows; row++)
        ids.push_back(static_cast<uint8_t>(calib_model_->item(row, 0)->text().toInt()));

    const std::vector<int> before = readCalibPositions();

    if(before.size() != ids.size())
        return;

    QStringList affected;
    QStringList current_positions;
    for(int row = 0; row < rows; row++)
    {
        affected << QString::number(ids[row]);
        current_positions << QString("    ID %1: %2")
                                 .arg(ids[row])
                                 .arg(before[row] >= 0 ? QString::number(before[row]) : QString("no reply"));
    }

    const QStringList steps = buildSetHomeConfirmations(rows, affected.join(", "), current_positions.join("\n"));

    if(!confirmDangerousAction("Set Home", steps))
        return;

    const std::vector<feetech_servo::ModelSeries> series = calib_series_;

    // The whole sweep is one visit to the bus. Nothing else can slip a
    // transaction in between a servo's offset write and its verification.
    std::vector<HomeRowResult> results;
    {
    BusyCursor busy;
    results = runOnBus([this, ids, series]{
        std::vector<HomeRowResult> out;
        out.reserve(ids.size());

        for(std::size_t i = 0; i < ids.size(); i++)
        {
            const uint8_t id = ids[i];
            const feetech_servo::ModelSeries s = series[i];
            const int max_res = feetech_servo::countsPerRev(s) - 1;
            const int half_turn = max_res / 2;

            HomeRowResult row;
            row.half_turn = half_turn;

            auto write = [&](const char *name, int value) {
                const feetech_servo::MemoryConfig *config = feetech_servo::findMemConfig(s, name);
                return config && bus_->writeRegister(id, s, *config, value).ok;
            };
            auto position = [&]() { return bus_->readPosition(id, s).value_or(-1); };

            bool ok = write("Position Offset Value", 0);
            ok = write("Min Position Limit", 0) && ok;
            ok = write("Max Position Limit", max_res) && ok;

            const int raw = position();
            if(!ok || raw < 0)
            {
                out.push_back(row);
                continue;
            }

            row.homing_offset = half_turn - raw;
            if(!write("Position Offset Value", row.homing_offset))
            {
                out.push_back(row);
                continue;
            }

            row.written = true;
            row.corrected = position();
            out.push_back(row);
        }
        return out;
    });
    }

    if(results.size() != ids.size())
    {
        QMessageBox::warning(this, "Set Home", "The calibration write did not complete.");
        return;
    }

    QStringList failed;
    QStringList mismatched;

    for(int row = 0; row < rows; row++)
    {
        const HomeRowResult &r = results[row];
        if(!r.written)
        {
            failed << QString::number(ids[row]);
            continue;
        }

        if(r.corrected < 0 || std::abs(r.corrected - r.half_turn) > 2)
        {
            mismatched << QString("ID %1 reads %2, expected %3")
                              .arg(ids[row])
                              .arg(r.corrected >= 0 ? QString::number(r.corrected) : QString("no reply"))
                              .arg(r.half_turn);
        }

        calib_model_->item(row, 3)->setText(QString::number(r.homing_offset));
        calib_min_[row] = r.half_turn;
        calib_max_[row] = r.half_turn;
        calib_model_->item(row, 4)->setText(QString::number(r.half_turn));
        calib_model_->item(row, 5)->setText(QString::number(r.half_turn));
    }

    if(!failed.isEmpty())
    {
        QMessageBox::warning(this, "Set Home",
            QString("Could not write calibration to ID %1.").arg(failed.join(", ")));
    }
    else if(!mismatched.isEmpty())
    {
        QMessageBox::warning(this, "Set Home",
            QString("Offset written, but the servo does not report the midpoint:\n\n%1\n\n"
                    "The firmware may apply the offset with the opposite sign.")
                .arg(mismatched.join("\n")));
    }
    else
    {
        ui->calibStatusLabel->setText(QString("Home set on %1 servo(s). Now record the range of motion.").arg(rows));
    }
}

void MainWindow::onCalibRecordClicked()
{
    if(calib_recording_)
    {
        calib_recording_ = false;
        ui->calibRecordButton->setText("2. Start Recording Range");
        ui->calibHomeButton->setEnabled(true);
        ui->calibExportButton->setEnabled(true);
        ui->calibStatusLabel->setText("Recording stopped. Check the ranges, then export.");
        return;
    }

    const int rows = calib_model_->rowCount();
    if(rows == 0)
        return;

    ensureCalibVectors();

    for(int row = 0; row < rows; row++)
    {
        const uint8_t id = static_cast<uint8_t>(calib_model_->item(row, 0)->text().toInt());
        const auto position = readCalibPosition(id, calib_series_[row]);
        if(!position.has_value())
        {
            QMessageBox::warning(this, "Record Range",
                QString("No response from ID %1.").arg(id));
            return;
        }
        calib_min_[row] = *position;
        calib_max_[row] = *position;
    }

    calib_recording_ = true;
    ui->calibRecordButton->setText("Stop Recording");
    ui->calibHomeButton->setEnabled(false);
    ui->calibExportButton->setEnabled(false);
    ui->calibStatusLabel->setText("Recording. Move every joint slowly through its full range.");
}

QVector<JointCalibration> MainWindow::collectCalibrationJoints()
{
    QVector<JointCalibration> joints;
    for(int row = 0; row < calib_model_->rowCount(); row++)
    {
        JointCalibration joint;
        joint.name = calib_model_->item(row, 1)->text().trimmed();
        joint.id = calib_model_->item(row, 0)->text().toInt();
        joint.drive_mode = calib_model_->item(row, 2)->text().toInt();
        joint.homing_offset = calib_model_->item(row, 3)->text().toInt();
        joint.range_min = calib_model_->item(row, 4)->text().toInt();
        joint.range_max = calib_model_->item(row, 5)->text().toInt();
        joints << joint;
    }
    return joints;
}

void MainWindow::onCalibExportClicked()
{
    const QVector<JointCalibration> joints = collectCalibrationJoints();
    const QStringList problems = validateCalibrationJoints(joints);
    if(!problems.isEmpty())
    {
        QMessageBox::warning(this, "Export JSON",
            QString("Fix these before exporting:\n\n%1").arg(problems.join("\n")));
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this, "Export calibration",
        QDir::homePath() + "/calibration.json", "JSON files (*.json)");
    if(path.isEmpty())
        return;

    QFile file(path);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this, "Export JSON", QString("Could not write %1.").arg(path));
        return;
    }

    QTextStream stream(&file);
    stream << buildCalibrationJson(joints);
    file.close();

    ui->calibStatusLabel->setText(QString("Exported %1 joint(s) to %2").arg(joints.size()).arg(path));
}

void MainWindow::setEnableComSettings(bool state)
{
    ui->ComComboBox->setEnabled(state);
    ui->BaudComboBox->setEnabled(state);
    ui->ParityComboBox->setEnabled(state);
    ui->timeoutLineEdit->setEnabled(state);
}

void MainWindow::clearServoList()
{
    servo_list_model_->clear();
    servo_list_model_->setHorizontalHeaderLabels(QStringList() << "ID" << "Module");

    ui->ServoListView->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->ServoListView->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
    ui->ServoListView->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    ui->ServoListView->horizontalScrollBar()->setDisabled(true);
    ui->ServoListView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->ServoListView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->ServoListView->setColumnWidth(0, 66);
    ui->ServoListView->verticalHeader()->setVisible(false);
    ui->ServoListView->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::appendServoList(const int id, const QString &name)
{
    servo_list_model_->appendRow(QList<QStandardItem*>() << new QStandardItem(QString::number(id)) << new QStandardItem(name));
}

void MainWindow::clearProgMemTable()
{
    prog_mem_model_->clear();
    prog_mem_model_->setHorizontalHeaderLabels(QStringList() << "Address" << "Memory" << "Value" << "Area" << "R/W");
    ui->memoryTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->memoryTableView->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
    ui->memoryTableView->verticalHeader()->setVisible(false);
    ui->memoryTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    ui->memoryTableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->memoryTableView->setColumnWidth(0, 84);
    ui->memoryTableView->setColumnWidth(2, 70);
    ui->memoryTableView->setColumnWidth(3, 70);
    ui->memoryTableView->setColumnWidth(4, 70);
    ui->memoryTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::updatePorgMemTable()
{
    clearProgMemTable();
    auto mem_config = feetech_servo::getMemConfig(select_servo_.model_);

    for(auto &item : mem_config)
    {
        auto &[address, name, size, default_value, dir_bit, is_eprom, is_readonly, min_val, max_val] = item;
        QString area = is_eprom ? "EPROM" : "SRAM";
        QString rw = is_readonly ? "R" : "R/W";
        QList<QStandardItem*> rowList;

        rowList << new QStandardItem(QString::number(address)) << new QStandardItem(name) << new QStandardItem(QString::number(default_value)) << new QStandardItem(area) << new QStandardItem(rw);

        const QBrush ink(is_readonly ? theme::textDisabled() : theme::textPrimary());
        for (QStandardItem *cell : rowList)
        {
            cell->setForeground(ink);
            cell->setToolTip(is_readonly
                ? QString("Address %1 (%2) is read-only").arg(address).arg(name)
                : QString("Address %1 (%2) is writable in %3").arg(address).arg(name).arg(area));
        }
        prog_mem_model_->appendRow(rowList);
    }
}

void MainWindow::setIntRangeLineEdit(QLineEdit *edit, int min, int max)
{
    edit->setValidator(new QIntValidator(min, max, edit));
}

void MainWindow::setIntLineEdit(QLineEdit *edit)
{
    edit->setValidator(new QRegExpValidator(QRegExp("-?\\d*"), edit));
}

void MainWindow::setDoubleLineEdit(QLineEdit *edit)
{
    edit->setValidator(new QRegExpValidator(QRegExp("-?\\d*\\.?\\d*"), edit));
}

int MainWindow::countsPerRev() const
{
    return feetech_servo::countsPerRev(select_servo_.model_);
}

int MainWindow::centerCount() const
{
    return countsPerRev() / 2;
}

double MainWindow::countsToRadians(int count) const
{
    return (count - centerCount()) * (2.0 * M_PI / countsPerRev());
}

double MainWindow::countsToDegrees(int count) const
{
    return (count - centerCount()) * (360.0 / countsPerRev());
}

int MainWindow::angleToCounts(double angle, bool is_radians) const
{
    const double per_count = is_radians ? (2.0 * M_PI / countsPerRev()) : (360.0 / countsPerRev());
    return centerCount() + static_cast<int>(std::lround(angle / per_count));
}

bool MainWindow::isAngleInputRadians() const
{
    return ui->angleUnitComboBox->currentText() == "rad";
}

void MainWindow::updateAngleInputFromCounts(int count)
{
    const bool is_radians = isAngleInputRadians();
    const double angle = is_radians ? countsToRadians(count) : countsToDegrees(count);
    ui->angleLineEdit->setText(QString::number(angle, 'f', is_radians ? 3 : 1));
}

void MainWindow::updateAnglePreview()
{
    const int counts = angleToCounts(ui->angleLineEdit->text().toDouble(), isAngleInputRadians());
    const int max_count = countsPerRev() - 1;

    if(counts < 0 || counts > max_count)
    {
        ui->angleCountLabel->setText(QString("= %1 !").arg(counts));
    }
    else
    {
        ui->angleCountLabel->setText(QString("= %1").arg(counts));
    }
}

void MainWindow::selectServoSeries(feetech_servo::ModelSeries series)
{
    // The wire endianness this series needs is applied per transaction on the
    // bus thread, so there is nothing to set here.
    select_servo_.model_ = series;
    updatePorgMemTable();
}

bool MainWindow::writeMemValue(const feetech_servo::MemoryConfig &config, int value)
{
    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;

    is_mem_writing_ = true;
    const feetech_servo::RegisterWriteResult result = runOnBus([this, id, series, config, value]{
        return bus_->writeRegister(id, series, config, value);
    });
    is_mem_writing_ = false;

    // Writing the ID register moved the servo, so follow it to its new address.
    // Guarded because a default-built result carries -1, which would silently
    // deselect the servo rather than leaving it where it was.
    if(result.effective_id >= 0)
        select_servo_.id_ = result.effective_id;

    return result.ok;
}

bool MainWindow::confirmDangerousAction(const QString &title, const QStringList &steps)
{
    // Each message box runs its own event loop, so the periodic polls would
    // otherwise keep firing underneath it -- and updateCalibrationGate can
    // rebuild the calibration table, which is exactly the thing the caller has
    // already captured row indices from and is asking the user about.
    prompt_open_ = true;
    const auto close_prompt = qScopeGuard([this]{ prompt_open_ = false; });

    for(int i = 0; i < steps.size(); i++)
    {
        const auto answer = QMessageBox::warning(this,
            QString("%1 - confirmation %2 of %3").arg(title).arg(i + 1).arg(steps.size()),
            steps[i],
            QMessageBox::Cancel | QMessageBox::Ok, QMessageBox::Cancel);
        if(answer != QMessageBox::Ok)
        {
            return false;
        }
    }
    return true;
}

void MainWindow::setProgStatus(const QString &text)
{
    ui->progStatusLabel->setText(text);
}

void MainWindow::writePos(int pos, int time, int speed, int acc)
{
    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;
    postToBus([this, id, series, pos, time, speed, acc]{
        bus_->writePos(id, series, pos, time, speed, acc);
    });
}

void MainWindow::syncWritePos(int pos, int time, int speed, int acc)
{
    const std::vector<uint8_t> ids = id_list_;
    const feetech_servo::ModelSeries series = select_servo_.model_;
    postToBus([this, ids, series, pos, time, speed, acc]{
        bus_->syncWritePos(ids, series, pos, time, speed, acc);
    });
}

void MainWindow::regWritePos(int pos, int time, int speed, int acc)
{
    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;
    postToBus([this, id, series, pos, time, speed, acc]{
        bus_->regWritePos(id, series, pos, time, speed, acc);
    });
}

void MainWindow::onPortSearchTimerTimeout()
{
    // Rebuilding the list under an open dropdown would snatch it away mid
    // choice, so wait until it is closed.
    if(ui->ComComboBox->view()->isVisible())
        return;

    QVector<feetech_servo::PortInfo> ports;

    if(bus_->isRemote())
    {
        // The ports that matter are the robot's, and each look costs a round
        // trip, so this is paced rather than run at the local cadence.
        if(!link_up_ || pollsSuspended())
            return;
        if(!port_refresh_due_ && port_refresh_clock_.elapsed() < REMOTE_PORT_REFRESH_MS)
            return;

        port_refresh_due_ = false;
        port_refresh_clock_.restart();
        ports = runOnBus([this]{ return bus_->listPorts(); });
    }
    else
    {
        ports = feetech_servo::localPorts();
    }

    QStringList names;
    for(const feetech_servo::PortInfo &p : ports)
        names << p.name;

    // Rebuilding the list resets the selection, and doing that every tick lost
    // what made a port impossible to choose. Only touch the list when the set
    // of ports has actually changed, and put the selection back afterwards.
    if(names == com_port_names_)
        return;
    com_port_names_ = names;

    const QString selected = ui->ComComboBox->currentData().toString();

    QSignalBlocker blocker(ui->ComComboBox);
    ui->ComComboBox->clear();
    for(const feetech_servo::PortInfo &p : ports)
    {
        // The port name is what gets opened; the description is what lets you
        // tell one adapter from another.
        const QString label = p.description.isEmpty()
                            ? p.name
                            : QString("%1 - %2").arg(p.name, p.description);
        ui->ComComboBox->addItem(label, p.name);
    }

    int restore = ui->ComComboBox->findData(selected);
    if(restore < 0 && bus_->isRemote())
    {
        // Nothing was chosen yet, so start on the port the server was told to
        // hold. On a robot that is settled when the server is started, and it
        // is the one the user almost certainly wants.
        //
        // The server may name it either way round -- "/dev/ttyUSB0" is what a
        // shell user types, "ttyUSB0" is what the port list carries -- so the
        // match is on the bare name.
        auto *remote = static_cast<servobench_net::RemoteServoBus *>(bus_);
        const QString wanted = remote->serverDevice().section('/', -1);
        for(int i = 0; i < ui->ComComboBox->count() && restore < 0; i++)
        {
            if(ui->ComComboBox->itemData(i).toString().section('/', -1) == wanted)
                restore = i;
        }
    }
    if(restore >= 0)
        ui->ComComboBox->setCurrentIndex(restore);
}

bool MainWindow::isNetworkMode() const
{
    return ui->LinkComboBox->currentIndex() == 1;
}

void MainWindow::updateLinkFields()
{
    ui->netSettingsWidget->setVisible(isNetworkMode());

    // Everything here describes where the bus is, so it is settled only while
    // a port is actually open on it. Keying this off the link being up instead
    // would strand a window that reached a server but could not open a port on
    // it: no way to correct the host, and no way back to a local port either.
    ui->LinkComboBox->setEnabled(!bus_open_);
    ui->hostLineEdit->setEnabled(!bus_open_);
    ui->netPortLineEdit->setEnabled(!bus_open_);
    ui->tokenLineEdit->setEnabled(!bus_open_);
}

void MainWindow::updateLinkStatus()
{
    if(!bus_->isRemote())
    {
        ui->linkStatusLabel->clear();
        return;
    }

    if(!link_up_)
    {
        ui->linkStatusLabel->setText("Not connected.");
        return;
    }

    ui->linkStatusLabel->setText(QString("Connected to %1 - %2 ms round trip")
                                     .arg(ui->hostLineEdit->text().trimmed())
                                     .arg(bus_->latencyMs()));
}

void MainWindow::rebuildBus(bool remote)
{
    const bool was_polling = status_pending_;

    if(bus_ != nullptr)
    {
        // Stopping the thread is what makes the swap safe: nothing can be
        // reading bus_ across it. Deleting the old bus then drops the calls
        // still queued for it, rather than letting them run against the new
        // one once the thread restarts.
        QMetaObject::invokeMethod(bus_, [this]{
            if(!bus_->isRemote())
                bus_->close();
            bus_->disconnectTransport();
        }, Qt::BlockingQueuedConnection);
        bus_thread_->quit();
        bus_thread_->wait();
        delete bus_;
    }

    bus_ = remote ? static_cast<feetech_servo::IServoBus *>(new servobench_net::RemoteServoBus())
                  : static_cast<feetech_servo::IServoBus *>(new feetech_servo::ServoBus());
    bus_->moveToThread(bus_thread_);

    connect(bus_, &feetech_servo::IServoBus::statusReady, this, &MainWindow::onStatusReady);
    connect(bus_, &feetech_servo::IServoBus::openedChanged, this, &MainWindow::onBusOpenedChanged);
    connect(bus_, &feetech_servo::IServoBus::transportLost, this, &MainWindow::onTransportLost);
    connect(bus_, &feetech_servo::IServoBus::scanProgress, this, &MainWindow::onScanProgress);
    connect(bus_, &feetech_servo::IServoBus::scanFound, this, &MainWindow::onScanFound);
    connect(bus_, &feetech_servo::IServoBus::scanFinished, this, &MainWindow::onScanFinished);

    bus_thread_->start();

    bus_open_ = false;
    // A serial bus is already where its servos are; a network one has a
    // connection still to make.
    link_up_ = !remote;
    status_pending_ = false;
    status_clock_aligned_ = false;
    port_refresh_due_ = true;

    // The telemetry chain is each sample asking for the next, so a sample
    // dropped with the old bus would end it. Start it again.
    if(was_polling)
        QTimer::singleShot(0, this, &MainWindow::requestNextStatus);
}

bool MainWindow::reachLink(QString *error)
{
    if(!bus_->isRemote())
    {
        link_up_ = true;
        return true;
    }

    const QString host = ui->hostLineEdit->text().trimmed();
    if(host.isEmpty())
    {
        *error = "Enter the machine running \"servobench-cli serve\".";
        return false;
    }

    const auto net_port = static_cast<quint16>(ui->netPortLineEdit->text().toUInt());
    const QString token = ui->tokenLineEdit->text();

    BusyCursor busy;
    QString reason;
    const bool reached = runOnBus([this, host, net_port, token, &reason]{
        return bus_->connectTransport(host, net_port, token, &reason);
    });

    link_up_ = reached;
    if(!reached)
    {
        *error = reason;
        return false;
    }

    status_clock_aligned_ = false;
    port_refresh_due_ = true;
    return true;
}

void MainWindow::releaseLink()
{
    if(is_searching_)
    {
        bus_->abortScan();
        is_searching_ = false;
        ui->SearchButton->setText("Search");
    }

    runOnBus([this]{
        bus_->close();
        bus_->disconnectTransport();
        return true;
    });

    link_up_ = !bus_->isRemote();
    bus_open_ = false;
    status_clock_aligned_ = false;
    port_refresh_due_ = true;

    ui->ComOpenButton->setText("Open");
    setEnableComSettings(true);
    select_servo_.id_ = -1;
    clearServoList();
    id_list_.clear();
    torque_state_.clear();
    fault_state_.clear();
    updateFunctionalityGate();
    updateTorqueAllButton();
    updateLinkFields();
    updateLinkStatus();
}

void MainWindow::onLinkModeChanged()
{
    if(bus_open_ || link_up_)
        releaseLink();

    rebuildBus(isNetworkMode());

    // The dropdown was listing the other machine's ports.
    com_port_names_.clear();
    ui->ComComboBox->clear();

    updateLinkFields();
    updateLinkStatus();
    onPortSearchTimerTimeout();
}

void MainWindow::onLinkStatusTimerTimeout()
{
    if(bus_->isRemote() && link_up_)
        updateLinkStatus();
}

void MainWindow::onTransportLost(const QString &reason)
{
    if(!link_up_)
        return;

    link_up_ = false;
    bus_open_ = false;
    status_clock_aligned_ = false;

    if(is_searching_)
    {
        is_searching_ = false;
        ui->SearchButton->setText("Search");
    }

    ui->ComOpenButton->setText("Open");
    setEnableComSettings(true);
    select_servo_.id_ = -1;
    clearServoList();
    id_list_.clear();
    torque_state_.clear();
    fault_state_.clear();
    updateFunctionalityGate();
    updateTorqueAllButton();
    updateLinkFields();

    // Deliberately not a dialog. The link can drop while the user is watching
    // the plot rather than this panel, and a modal box would stop every timer
    // in the window behind it -- including the ones that would notice the link
    // coming back.
    ui->linkStatusLabel->setText(reason);
}

void MainWindow::onConnectButtonClicked()
{
    if(bus_open_)
    {
        releaseLink();
        return;
    }

    // In network mode there is no port to choose until the machine holding it
    // has been reached. Doing that here rather than behind a second button
    // keeps this one meaning a single thing: make the bus usable.
    //
    // Done on every press, not only the first: connectTransport lets go of any
    // link it already has, so pressing Open again after editing the host or
    // the token does what it looks like it does.
    if(bus_->isRemote())
    {
        QString error;
        if(!reachLink(&error))
        {
            ui->linkStatusLabel->setText(error);
            QMessageBox::warning(this, "Connect", error);
            return;
        }

        updateLinkFields();
        updateLinkStatus();
        // The dropdown was empty, or listing this machine's ports.
        onPortSearchTimerTimeout();
    }

    // The visible text carries the device description too, so the port name
    // comes from the item data rather than from what is on screen.
    const QString port = ui->ComComboBox->currentData().toString();
    if(port.isEmpty())
    {
        QMessageBox::warning(this, "Open", bus_->isRemote()
            ? "The server reports no serial ports."
            : "No serial port selected.");
        return;
    }

    const int baud = ui->BaudComboBox->currentText().toInt();
    uint8_t pidx = ui->ParityComboBox->currentIndex();
    QSerialPort::Parity p = QSerialPort::Parity::NoParity;
    if(pidx == 1)
        p = QSerialPort::Parity::OddParity;
    if(pidx == 2)
        p = QSerialPort::Parity::EvenParity;
    const int timeout = ui->timeoutLineEdit->text().toUInt();

    const bool opened = runOnBus([this, port, baud, p, timeout]{
        return bus_->open(port, baud, p, timeout);
    });

    if(opened)
    {
        ui->ComOpenButton->setText("Close");
        setEnableComSettings(false);
    }
    else
    {
        ui->ComOpenButton->setText("Open");
        QMessageBox::warning(this, "Open", bus_->isRemote()
            ? QString("The server could not open %1.").arg(port)
            : QString("Could not open %1.").arg(port));
    }
    updateLinkFields();
}

// The bus emits this from inside open()/close(), so it is queued to this
// thread ahead of the reply runOnBus is waiting on. By the time an open() or
// close() call returns, bus_open_ has already caught up.
//
// On a remote bus it also arrives unasked, when the server's port opens or
// closes for a reason of its own, so the panel follows the bus rather than
// only following the button that was pressed here.
void MainWindow::onBusOpenedChanged(bool is_open)
{
    bus_open_ = is_open;

    ui->ComOpenButton->setText(is_open ? "Close" : "Open");
    setEnableComSettings(!is_open);
    updateFunctionalityGate();
    updateLinkFields();
}

void MainWindow::onSearchButtonClicked()
{
    if(!bus_open_)
        return;

    if(is_searching_)
    {
        // The bus thread is inside the sweep and could not service a queued
        // call until it ended, so this is the one bus call made straight from
        // here rather than posted.
        bus_->abortScan();
        ui->ServoSearchText->setText("Stopping...");
        return;
    }

    is_searching_ = true;
    ui->SearchButton->setText("Stop");
    clearServoList();
    id_list_.clear();
    torque_state_.clear();
    fault_state_.clear();
    updateFunctionalityGate();

    // One operation rather than a ping per address. Locally that only saves
    // the round trip through this thread; over a network it is the difference
    // between one request and five hundred.
    postToBus([this]{
        // The IDs on the bus may be different servos this time round.
        bus_->invalidateModeCaches();
        bus_->startScan(0, 0xfd);
    });
}

void MainWindow::onScanProgress(int id)
{
    ui->ServoSearchText->setText(QString("Ping ID:%1 Servo...").arg(id));
}

void MainWindow::onScanFound(int id, int model_number)
{
    const QString name = feetech_servo::getModelType(model_number);
    appendServoList(id, name);
    id_list_.push_back(static_cast<uint8_t>(id));
    select_servo_.id_ = id;
    selectServoSeries(feetech_servo::getModelSeries(name));
}

void MainWindow::onScanFinished(bool)
{
    is_searching_ = false;
    ui->SearchButton->setText("Search");
    ui->ServoSearchText->setText("Stop");
    refreshTorqueStates();
    updateFunctionalityGate();
}

void MainWindow::onServoListSelection()
{
    QModelIndex selectedRows = ui->ServoListView->selectionModel()->selectedRows()[0];
    std::size_t row = selectedRows.row();
    auto index = servo_list_model_->index(row, 0);
    select_servo_.id_ = servo_list_model_->data(index).toInt();
    index = servo_list_model_->index(row, 1);
    const feetech_servo::ModelSeries series = feetech_servo::getModelSeries(servo_list_model_->data(index).toString());
    selectServoSeries(series);

    if (bus_open_ && !is_searching_)
    {
        const auto position = readCalibPosition((uint8_t)select_servo_.id_, series);
        if (position.has_value())
        {
            syncControlToPosition(*position);
        }
    }
}

bool MainWindow::controlFollowEnabled() const
{
    return isServoValidNow()
        && !ui->goalSlider->isSliderDown()
        && !ui->goalLineEdit->hasFocus()
        && !ui->angleLineEdit->hasFocus()
        && !ui->setPushButton->isDown()
        && !ui->angleSetButton->isDown()
        && control_touch_.elapsed() > CONTROL_FOLLOW_RESUME_MS;
}

void MainWindow::syncControlToPosition(int pos)
{
    const int max_count = countsPerRev() - 1;
    if (pos < 0 || pos > max_count)
        return;

    if (ui->goalSlider->value() != pos)
    {
        QSignalBlocker blocker(ui->goalSlider);
        ui->goalSlider->setValue(pos);
    }
    if (!ui->goalLineEdit->hasFocus() && ui->goalLineEdit->text().toInt() != pos)
    {
        ui->goalLineEdit->setText(QString::number(pos));
    }
    if (!ui->angleLineEdit->hasFocus())
    {
        updateAngleInputFromCounts(pos);
    }
}

void MainWindow::onGoalSliderValueChanged()
{
    noteControlInteraction();
    int goal = ui->goalSlider->value();
    ui->goalLineEdit->setText(QString::number(goal));
    updateAngleInputFromCounts(goal);

    if(!isServoValidNow())
        return;

    if(mode_ == MODE_REG_WRITE)
    {
        regWritePos(goal, 0, 0, 0);
    }
    else if(mode_ == MODE_SYNC_WRITE)
    {
        syncWritePos(goal, 0, 0, 0);
    }
    else if(mode_ == MODE_WRITE)
    {
        writePos(goal, 0, 0, 0);
    }
}

void MainWindow::onSetBuggonClicked()
{
    noteControlInteraction();
    int goal = ui->goalLineEdit->text().toUInt();
    int speed = ui->speedLineEdit->text().toUInt();
    int acc = ui->accLineEdit->text().toUInt();
    int time = ui->timeLineEdit->text().toUInt();
    {
        QSignalBlocker blocker(ui->goalSlider);
        ui->goalSlider->setValue(goal);
    }
    updateAngleInputFromCounts(goal);

    if(!isServoValidNow())
        return;

    if(mode_ == MODE_REG_WRITE)
    {
        regWritePos(goal, time, speed, acc);
    }
    else if(mode_ == MODE_SYNC_WRITE)
    {
        syncWritePos(goal, time, speed, acc);
    }
    else if(mode_ == MODE_WRITE)
    {
        writePos(goal, time, speed, acc);
    }
}

void MainWindow::onTorqueEnableCheckBoxStateChanged()
{
    if(!isServoValidNow())
        return;

    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;
    const bool on = ui->torqueEnableCheckBox->isChecked();
    postToBus([this, id, series, on]{ bus_->enableTorque(id, series, on); });
}

int MainWindow::aggregateTorqueState() const
{
    int on = 0;
    int off = 0;
    int unknown = 0;
    for (uint8_t id : id_list_)
    {
        const auto it = torque_state_.find((int)id);
        if (it == torque_state_.end() || it->second < 0)
            unknown++;
        else if (it->second != 0)
            on++;
        else
            off++;
    }
    if (on + off == 0)
        return -1;
    if (unknown > 0 || (on > 0 && off > 0))
        return 2;
    return on > 0 ? 1 : 0;
}

void MainWindow::updateTorqueAllButton()
{
    QStringList on_ids;
    QStringList off_ids;
    QStringList silent_ids;
    for (uint8_t id : id_list_)
    {
        const auto it = torque_state_.find((int)id);
        if (it == torque_state_.end() || it->second < 0)
            silent_ids << QString::number(id);
        else if (it->second != 0)
            on_ids << QString::number(id);
        else
            off_ids << QString::number(id);
    }

    const int state = aggregateTorqueState();
    QColor c;
    QString tip;
    if (state == 0)
    {
        c = theme::statusReleased();
        tip = QString("Torque disabled on all %1 servo(s). Safe to move by hand.").arg(off_ids.size());
    }
    else if (state == 1)
    {
        c = theme::statusEngaged();
        tip = QString("Torque enabled on all %1 servo(s). They are holding position.").arg(on_ids.size());
    }
    else if (state == 2)
    {
        c = theme::statusMixed();
        QStringList parts;
        if (!on_ids.isEmpty())
            parts << QString("enabled on ID %1").arg(on_ids.join(", "));
        if (!off_ids.isEmpty())
            parts << QString("disabled on ID %1").arg(off_ids.join(", "));
        if (!silent_ids.isEmpty())
            parts << QString("no reply from ID %1").arg(silent_ids.join(", "));
        tip = "Inconsistent: " + parts.join("; ");
    }
    else
    {
        c = theme::statusUnknown();
        tip = "Torque state unknown. Open the port and run Search.";
    }

    all_torque_on_ = (state != 0);
    ui->torqueAllButton->setText(all_torque_on_ ? "Torque Off (All)" : "Torque On (All)");
    ui->torqueAllButton->setToolTip(tip);
    ui->torqueAllButton->setStyleSheet(QString(
        "QPushButton { background: rgba(%1,%2,%3,40); border: 1px solid rgba(%1,%2,%3,180);"
        " border-radius: 4px; padding: 5px 14px; min-height: 18px; color: %4; }"
        "QPushButton:hover { background: rgba(%1,%2,%3,72); border: 1px solid rgba(%1,%2,%3,235); }"
        "QPushButton:pressed { background: rgba(%1,%2,%3,24); }"
        "QPushButton:disabled { background: #1B1F24; border: 1px solid #262B31; color: #5C6570; }")
        .arg(c.red()).arg(c.green()).arg(c.blue()).arg(c.lighter(135).name()));
}

void MainWindow::updateServoListTorqueColors()
{
    for (int row = 0; row < servo_list_model_->rowCount(); row++)
    {
        QStandardItem *id_item = servo_list_model_->item(row, 0);
        if (id_item == nullptr)
            continue;
        const int id = id_item->text().toInt();

        QColor c = theme::statusUnknown();
        QString tip = QString("ID %1: torque state unknown").arg(id);
        const auto it = torque_state_.find(id);
        if (it != torque_state_.end() && it->second >= 0)
        {
            const bool on = it->second != 0;
            c = on ? theme::statusEngaged() : theme::statusReleased();
            tip = QString("ID %1: torque %2").arg(id).arg(on ? "ENABLED, holding position" : "disabled, free to move");
        }

        // A servo reporting a fault outranks whatever its torque state is:
        // that is the thing worth noticing about the row.
        const auto fault_it = fault_state_.find(id);
        if (fault_it != fault_state_.end() && fault_it->second > 0)
        {
            // Decode against this row's own series, not the selected servo's:
            // the potentiometer servos leave bits 1 and 3 undefined where the
            // magnetic-encoder ones use them for sensor and current faults.
            QStandardItem *model_item = servo_list_model_->item(row, 1);
            const feetech_servo::ModelSeries row_series =
                model_item ? feetech_servo::getModelSeries(model_item->text())
                           : select_servo_.model_;

            const QStringList faults =
                feetech_servo::decodeServoStatus(row_series, fault_it->second);
            c = theme::statusMixed();
            tip = QString("ID %1: %2\n\n(Servo Status register = %3)")
                      .arg(id).arg(faults.join(", ")).arg(fault_it->second);
        }

        QPixmap chip(11, 11);
        chip.fill(Qt::transparent);
        QPainter chip_painter(&chip);
        chip_painter.setRenderHint(QPainter::Antialiasing, true);
        chip_painter.setBrush(QBrush(c));
        chip_painter.setPen(QPen(c.darker(160)));
        chip_painter.drawEllipse(QRectF(0.5, 0.5, 10.0, 10.0));
        chip_painter.end();
        id_item->setIcon(QIcon(chip));

        QColor bg = c;
        bg.setAlpha(52);
        for (int col = 0; col < servo_list_model_->columnCount(); col++)
        {
            QStandardItem *item = servo_list_model_->item(row, col);
            if (item == nullptr)
                continue;
            item->setForeground(QBrush(c));
            item->setBackground(QBrush(bg));
            item->setToolTip(tip);
        }
    }
}

void MainWindow::updateFunctionalityGate()
{
    const bool ready = bus_open_ && !is_searching_ && !id_list_.empty();

    ui->groupBox_3->setEnabled(ready);
    ui->groupBox_4->setEnabled(ready);
    ui->groupBox_6->setEnabled(ready);
    ui->torqueAllButton->setEnabled(ready);
    ui->memoryTableView->setEnabled(ready);
    ui->memSetLineEdit->setEnabled(ready);
    ui->memSetButton->setEnabled(ready);
    ui->memSaveButton->setEnabled(ready);
    ui->memLoadButton->setEnabled(ready);
    ui->calibrationGroup->setEnabled(ready);

    if (!ready && !is_searching_)
    {
        if (!bus_open_)
            ui->ServoSearchText->setText("Port closed");
        else if (id_list_.empty())
            ui->ServoSearchText->setText("No servos detected");
    }
}

void MainWindow::refreshTorqueStates()
{
    if (!bus_open_ || is_searching_)
    {
        torque_state_.clear();
        fault_state_.clear();
        updateTorqueAllButton();
        updateServoListTorqueColors();
        return;
    }

    const std::vector<uint8_t> ids = id_list_;
    const feetech_servo::ModelSeries series = select_servo_.model_;

    // Torque state and fault state for every servo in one visit. Both are one
    // byte, so folding the status read in here costs a round trip per servo
    // per second and means a faulted joint shows up without being selected.
    const std::vector<std::pair<int, int>> values = runOnBus([this, ids, series]{
        std::vector<std::pair<int, int>> out;
        out.reserve(ids.size());
        for(uint8_t id : ids)
        {
            const int torque = bus_->readByte(id, series, TORQUE_ENABLE_ADDRESS);
            const int status = bus_->readByte(id, series, feetech_servo::SERVO_STATUS_ADDRESS);
            out.emplace_back(torque, status);
        }
        return out;
    });

    if(values.size() != ids.size())
        return;

    for (std::size_t i = 0; i < ids.size(); i++)
    {
        torque_state_[(int)ids[i]] = values[i].first;
        fault_state_[(int)ids[i]] = values[i].second;
    }

    updateTorqueAllButton();
    updateServoListTorqueColors();
    updateFunctionalityGate();
}

void MainWindow::onTorqueStateTimerTimeout()
{
    if (!bus_open_ || is_searching_ || id_list_.empty() || pollsSuspended())
        return;
    refreshTorqueStates();
}

void MainWindow::onTorqueAllButtonClicked()
{
    const QString title = all_torque_on_ ? "Torque Off" : "Torque On";

    if(!bus_open_)
    {
        QMessageBox::warning(this, title, "Serial port is not open.");
        return;
    }

    if(is_searching_)
    {
        QMessageBox::warning(this, title, "A servo search is still running.");
        return;
    }

    if(id_list_.empty())
    {
        QMessageBox::warning(this, title, "No servos detected. Run Search first.");
        return;
    }

    const uint8_t value = all_torque_on_ ? 0 : 1;
    const std::vector<uint8_t> ids = id_list_;
    const feetech_servo::ModelSeries series = select_servo_.model_;

    // Write to every servo first, then verify, so they all release together
    // rather than one at a time down the length of the arm.
    std::vector<int> read_back;
    {
        BusyCursor busy;
        read_back = runOnBus([this, ids, series, value]{
            for(uint8_t id : ids)
                bus_->writeByte(id, series, TORQUE_ENABLE_ADDRESS, value);

            std::vector<int> out;
            out.reserve(ids.size());
            for(uint8_t id : ids)
                out.push_back(bus_->readByte(id, series, TORQUE_ENABLE_ADDRESS));
            return out;
        });
    }

    if(read_back.size() != ids.size())
        return;

    QStringList failed;
    for(std::size_t i = 0; i < ids.size(); i++)
    {
        if(read_back[i] < 0 || (read_back[i] != 0) != (value != 0))
        {
            failed << QString::number(ids[i]);
        }
    }

    const int total = static_cast<int>(id_list_.size());
    const bool all_failed = failed.size() == total;

    if(!all_failed)
    {
        all_torque_on_ = (value != 0);
        updateTorqueAllButton();

        QSignalBlocker blocker(ui->torqueEnableCheckBox);
        ui->torqueEnableCheckBox->setChecked(all_torque_on_);
    }

    refreshTorqueStates();

    const QString state = (value != 0) ? "on" : "off";
    if(failed.isEmpty())
    {
        ui->ServoSearchText->setText(QString("Torque %1: %2 servo(s)").arg(state).arg(total));
    }
    else
    {
        ui->ServoSearchText->setText(QString("Torque %1: %2/%3 failed").arg(state).arg(failed.size()).arg(total));
        QMessageBox::warning(this, title,
            QString("ID %1 did not report Torque Enable = %2.\n\nThose servos may still be in the previous state.")
                .arg(failed.join(", ")).arg(value));
    }
}

void MainWindow::onModeRadioButtonsToggled(bool checked)
{
    if(checked)
    {
        if(ui->writeRadioButton->isChecked())
        {
            mode_ = MODE_WRITE;
        }
        else if(ui->syncWriteRadioButton->isChecked())
        {
            mode_ = MODE_SYNC_WRITE;
        }
        else if(ui->regWriteRadioButton->isChecked())
        {
            mode_ = MODE_REG_WRITE;
        }

        ui->actionPushButton->setEnabled(mode_ == MODE_REG_WRITE);
    }
}

void MainWindow::onActionButtonClicked()
{
    if(!isServoValidNow())
        return;

    if(mode_ == MODE_REG_WRITE)
    {
        const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
        postToBus([this, id]{ bus_->regWriteAction(id); });
    }
}

void MainWindow::onAngleInputChanged()
{
    updateAnglePreview();
}

void MainWindow::onAngleUnitChanged()
{
    updateAngleInputFromCounts(ui->goalLineEdit->text().toInt());
    updateAnglePreview();
}

void MainWindow::onAngleSetButtonClicked()
{
    noteControlInteraction();
    const bool is_radians = isAngleInputRadians();
    const double angle = ui->angleLineEdit->text().toDouble();
    const int goal = angleToCounts(angle, is_radians);
    const int max_count = countsPerRev() - 1;

    if(goal < 0 || goal > max_count)
    {
        QMessageBox::warning(this, "Set Angle",
            QString("%1 %2 maps to position %3, which is outside the encoder range 0..%4.")
                .arg(angle).arg(is_radians ? "rad" : "deg").arg(goal).arg(max_count));
        return;
    }

    ui->goalLineEdit->setText(QString::number(goal));
    {
        QSignalBlocker blocker(ui->goalSlider);
        ui->goalSlider->setValue(goal);
    }

    if(!isServoValidNow())
        return;

    const int speed = ui->speedLineEdit->text().toUInt();
    const int acc = ui->accLineEdit->text().toUInt();
    const int time = ui->timeLineEdit->text().toUInt();

    if(mode_ == MODE_REG_WRITE)
    {
        regWritePos(goal, time, speed, acc);
    }
    else if(mode_ == MODE_SYNC_WRITE)
    {
        syncWritePos(goal, time, speed, acc);
    }
    else if(mode_ == MODE_WRITE)
    {
        writePos(goal, time, speed, acc);
    }
}

void MainWindow::onSweepButtonClicked()
{
    if(!isServoValidNow())
        return;

    if(sweep_running_)
    {
        sweep_running_ = false;
        ui->sweepButton->setText("Sweep");
        ui->stepButton->setEnabled(true);
        auto_debug_timer_->stop();
    }
    else
    {
        sweep_running_ = true;
        ui->sweepButton->setText("Stop");
        ui->stepButton->setEnabled(false);
        latest_auto_debug_goal_ = ui->startLineEdit->text().toUInt();

        writePos(latest_auto_debug_goal_, 0, 0, 0);
        auto_debug_timer_->start(ui->sweepLineEdit->text().toUInt());
    }
}

void MainWindow::onStepButtonClicked()
{
    if(!isServoValidNow())
        return;

    if(step_running_)
    {
        step_running_ = false;
        ui->stepButton->setText("Step");
        ui->sweepButton->setEnabled(true);
        auto_debug_timer_->stop();
    }
    else
    {
        step_running_ = true;
        step_increase_ = true;
        ui->stepButton->setText("Stop");
        ui->sweepButton->setEnabled(false);
        latest_auto_debug_goal_ = ui->startLineEdit->text().toUInt();
        writePos(latest_auto_debug_goal_, 0, 0, 0);
        auto_debug_timer_->start(ui->stepDelayLineEdit->text().toUInt());
    }
}

void MainWindow::onAutoDebugTimerTimeout()
{
    if(!isServoValidNow())
    {
        auto_debug_timer_->stop();
        sweep_running_ = false;
        step_running_ = false;
        ui->sweepButton->setText("Sweep");
        ui->sweepButton->setEnabled(true);
        ui->stepButton->setText("Step");
        ui->stepButton->setEnabled(true);
        return;
    }

    if(sweep_running_)
    {
        int start = ui->startLineEdit->text().toUInt();
        int end = ui->endLineEdit->text().toUInt();
        if(latest_auto_debug_goal_ == start)
        {
            latest_auto_debug_goal_ = end;
        }
        else
        {
            latest_auto_debug_goal_ = start;
        }
        writePos(latest_auto_debug_goal_, 0, 0, 0);
    }
    else if(step_running_)
    {
        int start = ui->startLineEdit->text().toUInt();
        int end = ui->endLineEdit->text().toUInt();
        int step = ui->stepSizeLineEdit->text().toUInt();

        latest_auto_debug_goal_ += step_increase_ ? step : -step;

        if(end < latest_auto_debug_goal_)
        {
            latest_auto_debug_goal_ = end;
            step_increase_ = false;
        }
        else if(latest_auto_debug_goal_ < start)
        {
            latest_auto_debug_goal_ = start;
            step_increase_ = true;
        }

        writePos(latest_auto_debug_goal_, 0, 0, 0);
    }
    else
    {
        auto_debug_timer_->stop();
    }
}

void MainWindow::onExportButtonClicked()
{
    if(!isServoValidNow())
        return;

    if(is_recording_)
    {
        is_recording_ = false;
        ui->exportPushButton->setText("Export");
        ui->clearPushButton->setEnabled(true);

        if(!record_section_data_.isEmpty())
        {
            auto record_file_ = new QFile(record_file_name_);

            if(!record_file_->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
            {
                qDebug() << "file open error: " << record_file_name_;
                return;
            }

            auto record_stream_ = new QTextStream(record_file_);
            *record_stream_ << record_section_data_;
            record_section_data_.clear();

            record_file_->close();
        }
        ui->recSizeLineEdit->setText(QString::number(record_data_count_));
    }
    else
    {
        is_recording_ = true;
        ui->exportPushButton->setText("Stop");
        ui->clearPushButton->setEnabled(false);
        record_section_data_.clear();
        record_data_count_ = 0;
        file_write_interval_ = ui->recTimeLineEdit->text().toUInt();
        if(file_write_interval_ == 0)
            file_write_interval_ = 1;

        record_file_name_ = ui->recFileNameLineEdit->text();

#if defined(Q_OS_LINUX)
        QString home_path = QDir::homePath();
        if(record_file_name_.startsWith("~"))
        {
            record_file_name_.replace("~", home_path);
        }
#endif

        auto record_file_ = new QFile(record_file_name_);
        if(!record_file_->open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qDebug() << "file open error: " << record_file_name_;
            return;
        }
        auto record_stream_ = new QTextStream(record_file_);
        *record_stream_ << "No,Pos,Gol,Ft,V,C,T,Vol\n";
        record_file_->close();
    }
}

void MainWindow::onClearButtonClicked()
{
    record_data_count_ = 0;
    record_section_data_.clear();
    ui->recSizeLineEdit->setText(QString::number(record_data_count_));
}

void MainWindow::onDataAnalysisTimerTimeout()
{
    if(!isServoValidNow())
        return;

    if(!is_recording_)
        return;

    record_data_count_++;
    QString line = QString("%1,%2,%3,%4,%5,%6,%7,%8,END\n").arg(record_data_count_).arg(latest_status_.pos).arg(latest_status_.goal).arg(latest_status_.torque).arg(latest_status_.speed).arg(latest_status_.current).arg(latest_status_.temp).arg(latest_status_.voltage);
    record_section_data_ += line;
    const size_t section_size = 20 * file_write_interval_;

    if(record_data_count_%section_size == 0)
    {
        auto record_file_ = new QFile(record_file_name_);

        if(!record_file_->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
        {
            qDebug() << "file open error: " << record_file_name_;
            return;
        }

        auto record_stream_ = new QTextStream(record_file_);
        *record_stream_ << record_section_data_;
        record_section_data_.clear();

        record_file_->close();

        ui->recSizeLineEdit->setText(QString::number(record_data_count_));
    }
}

void MainWindow::onProgTimerTimeout()
{
    if(ui->tabWidget->currentIndex() != 1)
        return;

    if(!isServoValidNow())
        return;

    if(is_mem_writing_ || pollsSuspended())
        return;

    int firstVisibleRow = ui->memoryTableView->indexAt(ui->memoryTableView->viewport()->rect().topLeft()).row();
    int lastVisibleRow = ui->memoryTableView->indexAt(ui->memoryTableView->viewport()->rect().bottomLeft()).row();
    if (firstVisibleRow != -1 && lastVisibleRow != -1)
    {
        const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
        const feetech_servo::ModelSeries series = select_servo_.model_;
        const int first = firstVisibleRow;
        const int last = lastVisibleRow;

        const std::vector<int> values = runOnBus([this, id, series, first, last]{
            const auto &mem_config = feetech_servo::getMemConfig(series);
            std::vector<int> out;
            out.reserve(last - first + 1);
            for(int i = first; i <= last; i++)
                out.push_back(bus_->readRegister(id, series, mem_config[i]));
            return out;
        });

        if(values.size() != static_cast<std::size_t>(last - first + 1))
            return;

        for(int i = first; i <= last; i++)
        {
            ui->memoryTableView->model()->setData(ui->memoryTableView->model()->index(i, 2),
                                                  QString::number(values[i - first]));
        }
    }
}

void MainWindow::onMemoryTableSelection()
{
    QModelIndex selectedRows = ui->memoryTableView->selectionModel()->selectedRows().first();
    std::size_t row = selectedRows.row();
    auto index = prog_mem_model_->index(row, 1);
    ui->memLabel->setText(prog_mem_model_->data(index).toString());
    ui->memSetLineEdit->setText(prog_mem_model_->data(prog_mem_model_->index(row, 2)).toString());
    auto mem_config = feetech_servo::getMemConfig(select_servo_.model_);
    bool is_readonly = mem_config[row].is_readonly;
    if(is_readonly)
    {
        ui->memSetLineEdit->setEnabled(false);
        ui->memSetButton->setEnabled(false);
    }
    else
    {
        ui->memSetLineEdit->setEnabled(true);
        ui->memSetButton->setEnabled(true);
    }
}

int MainWindow::readMemValue(const feetech_servo::MemoryConfig &config)
{
    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;
    return runOnBus([this, id, series, config]{ return bus_->readRegister(id, series, config); });
}

void MainWindow::onMemSaveClicked()
{
    if (!isServoValidNow())
    {
        QMessageBox::warning(this, "Save Registers", "No servo selected.");
        return;
    }

    const auto &mem_config = feetech_servo::getMemConfig(select_servo_.model_);
    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;

    // One trip to the bus for the model number and the whole register map,
    // rather than a round trip per row.
    struct SnapshotRead { int model_number = -1; std::vector<int> values; };
    SnapshotRead read;
    {
        BusyCursor busy;
        is_mem_writing_ = true;
        read = runOnBus([this, id, series]{
            SnapshotRead out;
            out.model_number = bus_->readModelNumber(id);
            const auto &configs = feetech_servo::getMemConfig(series);
            out.values.reserve(configs.size());
            for (const auto &config : configs)
                out.values.push_back(bus_->readRegister(id, series, config));
            return out;
        });
        is_mem_writing_ = false;
    }

    if(read.values.size() != mem_config.size())
    {
        QMessageBox::warning(this, "Save Registers", "The register read did not complete.");
        return;
    }

    RegisterSnapshot snapshot;
    snapshot.id = select_servo_.id_;
    snapshot.series = seriesName(select_servo_.model_);
    snapshot.model = feetech_servo::getModelType(read.model_number);

    QStringList unread;
    for (std::size_t i = 0; i < mem_config.size(); i++)
    {
        const auto &config = mem_config[i];
        if (read.values[i] < 0)
        {
            unread << QString::number(config.address);
            continue;
        }
        RegisterEntry e;
        e.address = config.address;
        e.name = config.name;
        e.writable = !config.is_readonly;
        e.value = read.values[i];
        snapshot.registers.append(e);
    }

    if (snapshot.registers.isEmpty())
    {
        QMessageBox::warning(this, "Save Registers", "No registers could be read from the servo.");
        return;
    }

    const QString path = QFileDialog::getSaveFileName(this, "Save registers",
        QDir::homePath() + QString("/servo%1_registers.json").arg(select_servo_.id_),
        "JSON files (*.json)");
    if (path.isEmpty())
        return;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QMessageBox::warning(this, "Save Registers", QString("Could not write %1.").arg(path));
        return;
    }
    file.write(buildRegisterSnapshotJson(snapshot));
    file.close();

    QString message = QString("Saved %1 register(s) from ID %2 to %3")
                          .arg(snapshot.registers.size()).arg(snapshot.id).arg(path);
    if (!unread.isEmpty())
        message += QString(" (no reply for address %1)").arg(unread.join(", "));
    setProgStatus(message);
}

void MainWindow::onMemLoadClicked()
{
    if (!isServoValidNow())
    {
        QMessageBox::warning(this, "Load Registers", "No servo selected.");
        return;
    }

    const QString path = QFileDialog::getOpenFileName(this, "Load registers",
        QDir::homePath(), "JSON files (*.json)");
    if (path.isEmpty())
        return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::warning(this, "Load Registers", QString("Could not read %1.").arg(path));
        return;
    }
    const QByteArray data = file.readAll();
    file.close();

    RegisterSnapshot snapshot;
    QString error;
    if (!parseRegisterSnapshotJson(data, &snapshot, &error))
    {
        QMessageBox::warning(this, "Load Registers", QString("%1\n\n%2").arg(path).arg(error));
        return;
    }

    const QString target_series = seriesName(select_servo_.model_);
    if (snapshot.series != target_series)
    {
        QMessageBox::warning(this, "Load Registers",
            QString("This file was saved from a %1 servo but ID %2 is %3.\n\n"
                    "Register addresses differ between series, so loading it would write "
                    "the wrong registers. Refusing.")
                .arg(snapshot.series).arg(select_servo_.id_).arg(target_series));
        return;
    }

    const auto &mem_config = feetech_servo::getMemConfig(select_servo_.model_);
    QVector<QPair<const feetech_servo::MemoryConfig*, int>> planned;
    QStringList skipped_readonly;
    QStringList skipped_unknown;
    bool skipped_id = false;

    for (const RegisterEntry &e : snapshot.registers)
    {
        const feetech_servo::MemoryConfig *config = nullptr;
        for (const auto &item : mem_config)
        {
            if (item.address == e.address)
            {
                config = &item;
                break;
            }
        }
        if (config == nullptr)
        {
            skipped_unknown << QString::number(e.address);
            continue;
        }
        if (config->is_readonly)
        {
            skipped_readonly << QString::number(e.address);
            continue;
        }
        if (config->address == 5)
        {
            skipped_id = true;
            continue;
        }
        planned.append(qMakePair(config, e.value));
    }

    if (planned.isEmpty())
    {
        QMessageBox::warning(this, "Load Registers", "Nothing in this file is writable to the servo.");
        return;
    }

    int eprom_count = 0;
    for (const auto &p : planned)
        if (p.first->is_eprom)
            eprom_count++;

    QStringList steps;
    steps << QString("About to write %1 register(s) to servo ID %2 from:\n%3\n\n"
                     "%4 of them are EPROM registers, so those changes are permanent.\n\n"
                     "Continue?")
                 .arg(planned.size()).arg(select_servo_.id_).arg(path).arg(eprom_count);
    steps << QString("This replaces the servo's current configuration and cannot be undone.\n\n"
                     "Position Offset Value, Min/Max Position Limit, PID gains, torque limits and "
                     "protection settings will all be overwritten by the values in the file. "
                     "Save the current registers first if you might want them back.\n\n"
                     "Continue?");
    steps << QString("Final confirmation.\n\n"
                     "Target: ID %1 (%2)\nSource file series: %3\nRegisters to write: %4\n\n"
                     "The servo ID register is never written by a load, so ID %1 stays as it is.\n\n"
                     "Commit these writes?")
                 .arg(select_servo_.id_).arg(target_series).arg(snapshot.series).arg(planned.size());

    if (!confirmDangerousAction("Load Registers", steps))
        return;

    // One visit to the bus for the whole file: the servo is reconfigured as a
    // single uninterrupted run rather than register by register.
    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;
    std::vector<QPair<const feetech_servo::MemoryConfig*, int>> writes(planned.begin(), planned.end());

    std::vector<bool> ok;
    {
        BusyCursor busy;
        is_mem_writing_ = true;
        ok = runOnBus([this, id, series, writes]{
            std::vector<bool> out;
            out.reserve(writes.size());
            for (const auto &w : writes)
                out.push_back(bus_->writeRegister(id, series, *w.first, w.second).ok);
            return out;
        });
        is_mem_writing_ = false;
    }

    if (ok.size() != static_cast<std::size_t>(planned.size()))
    {
        QMessageBox::warning(this, "Load Registers", "The register write did not complete.");
        return;
    }

    QStringList failed;
    for (int i = 0; i < planned.size(); i++)
    {
        if (!ok[i])
            failed << QString::number(planned[i].first->address);
    }

    QStringList notes;
    notes << QString("wrote %1").arg(planned.size() - failed.size());
    if (!failed.isEmpty())
        notes << QString("failed %1").arg(failed.size());
    if (!skipped_readonly.isEmpty())
        notes << QString("skipped %1 read-only").arg(skipped_readonly.size());
    if (!skipped_unknown.isEmpty())
        notes << QString("skipped %1 unknown").arg(skipped_unknown.size());
    if (skipped_id)
        notes << "skipped ID";
    setProgStatus("Load: " + notes.join(", "));

    if (!failed.isEmpty())
    {
        QMessageBox::warning(this, "Load Registers",
            QString("These addresses did not verify after writing: %1").arg(failed.join(", ")));
    }
    refreshTorqueStates();
}

void MainWindow::onMemSetButtonClicked()
{
    if(!isServoValidNow())
    {
        setProgStatus("No servo selected");
        return;
    }

    auto selectedRows = ui->memoryTableView->selectionModel()->selectedRows();
    if(selectedRows.isEmpty())
    {
        return;
    }

    auto mem_config = feetech_servo::getMemConfig(select_servo_.model_);
    auto &config = mem_config[selectedRows.first().row()];
    if(config.is_readonly)
    {
        return;
    }

    const int value = ui->memSetLineEdit->text().toInt();
    const QString area = config.is_eprom ? "EPROM" : "SRAM";

    if(writeMemValue(config, value))
    {
        setProgStatus(QString("[%1] %2 = %3 saved to %4").arg(config.address).arg(config.name).arg(value).arg(area));
    }
    else
    {
        setProgStatus(QString("[%1] %2: write failed").arg(config.address).arg(config.name));
        QMessageBox::warning(this, "Write failed",
            QString("Address %1 (%2) still does not hold %3 after the write.\n\n"
                    "The servo either did not respond or rejected the value "
                    "(out of range, or outside the %4 area).")
                .arg(config.address).arg(config.name).arg(value).arg(area));
    }
}

void MainWindow::onCalibrationMidButtonClicked()
{
    if(!isServoValidNow())
    {
        setProgStatus("No servo selected");
        return;
    }

    const feetech_servo::MemoryConfig *offset = feetech_servo::findMemConfig(select_servo_.model_, "Position Offset Value");
    if(!feetech_servo::supportsMidpointCalibration(select_servo_.model_) || offset == nullptr)
    {
        QMessageBox::warning(this, "Set Midpoint",
            "This servo series has no Position Offset Value register.");
        return;
    }

    const auto answer = QMessageBox::question(this, "Set Midpoint",
        QString("Store the current position of servo ID %1 as the midpoint (2048)?\n\n"
                "This overwrites Position Offset Value (address %2) in EPROM, "
                "so it survives a power cycle. The servo does not move.")
            .arg(select_servo_.id_).arg(offset->address),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
    if(answer != QMessageBox::Ok)
    {
        return;
    }

    const uint8_t id = static_cast<uint8_t>(select_servo_.id_);
    const feetech_servo::ModelSeries series = select_servo_.model_;

    is_mem_writing_ = true;
    const feetech_servo::MidpointResult result = runOnBus([this, id, series, offset]{
        return bus_->setMidpoint(id, series, *offset);
    });
    is_mem_writing_ = false;

    if(!result.acked || result.after < 0)
    {
        setProgStatus("Set midpoint: no response");
        QMessageBox::warning(this, "Set Midpoint",
            "No response from the servo. The midpoint was not changed.");
        return;
    }

    const int decoded = decodeSignMagnitude(result.after, offset->dir_bit);
    setProgStatus(QString("Midpoint set: offset = %1").arg(decoded));

    if(result.before >= 0 && result.after == result.before)
    {
        QMessageBox::warning(this, "Set Midpoint",
            QString("Position Offset Value is unchanged (%1).\n\n"
                    "Either the servo was already centred here, or the firmware "
                    "rejected the calibration command.").arg(decoded));
    }
}

void MainWindow::onGraphTimerTimeout()
{
    ui->graphWidget->up_limit = ui->upLimitLineEdit->text().toUInt();
    ui->graphWidget->down_limit = ui->downLimitLineEdit->text().toUInt();

    if(is_searching_ || !bus_open_ || select_servo_.id_ < 0)
        return;

    ui->positionLabel->setText(QString::number(latest_status_.pos));
    if (controlFollowEnabled())
    {
        syncControlToPosition(latest_status_.pos);
    }
    ui->angleRadLabel->setText(QString::number(countsToRadians(latest_status_.pos), 'f', 3) + " rad");
    ui->angleDegLabel->setText(QString::number(countsToDegrees(latest_status_.pos), 'f', 1) + QString::fromUtf8("\u00B0"));
    ui->torqueLabel->setText(QString::number(latest_status_.torque));
    ui->speedLabel->setText(QString::number(latest_status_.speed));
    ui->currentLabel->setText(QString::number(latest_status_.current));
    ui->temperatureLabel->setText(QString::number(latest_status_.temp));
    ui->voltageLabel->setText(QString::number(0.1*latest_status_.voltage, 'f', 1) + QString("V"));
    ui->movingLabel->setText(QString::number(latest_status_.move));
    ui->goalLabel->setText(QString::number(latest_status_.goal));

    ui->graphWidget->pos_visible = ui->posCheckBox->isChecked();
    ui->graphWidget->goal_visible = ui->goalCheckBox->isChecked();
    ui->graphWidget->torque_visible = ui->torqueCheckBox->isChecked();
    ui->graphWidget->speed_visible = ui->speedCheckBox->isChecked();
    ui->graphWidget->current_visible = ui->currentCheckBox->isChecked();
    ui->graphWidget->temp_visible = ui->tempCheckBox->isChecked();
    ui->graphWidget->voltage_visible = ui->voltageCheckBox->isChecked();
}

// Asks the bus for the next telemetry sample. Only one is ever outstanding:
// the next is asked for when the last one lands, so a slow servo slows the
// sampling rate instead of building up a backlog of stale requests.
void MainWindow::requestNextStatus()
{
    if(status_pending_)
        return;

    if(ui->tabWidget->currentIndex() != 0 || !isServoValidNow())
    {
        // Nothing to sample yet. Idle until there is.
        QTimer::singleShot(STATUS_IDLE_RETRY_MS, this, &MainWindow::requestNextStatus);
        return;
    }

    status_pending_ = true;
    const int id = select_servo_.id_;
    const int series = static_cast<int>(select_servo_.model_);
    postToBus([this, id, series]{ bus_->pollStatus(id, series); });
}

void MainWindow::onStatusReady(const feetech_servo::ServoStatus &status)
{
    status_pending_ = false;

    // A sample can take several round trips, and the user may have picked a
    // different servo while it was in flight. Attributing one servo's position
    // to another would drag the goal slider -- and then a Set -- to a position
    // belonging to a different joint.
    if(status.id == select_servo_.id_)
    {
        latest_status_ = status;

        // Line the sampling machine's clock up with the plot's, once per link,
        // then draw every sample at the moment it was actually taken. Stamping
        // them on arrival instead would draw the link's jitter as though the
        // servo had moved.
        qint64 sample_ms = -1;
        if(status.t_ms >= 0)
        {
            if(!status_clock_aligned_)
            {
                status_clock_offset_ = ui->graphWidget->elapsed() - status.t_ms;
                status_clock_aligned_ = true;
            }
            sample_ms = status.t_ms + status_clock_offset_;
        }

        ui->graphWidget->append_data(latest_status_.pos, latest_status_.goal, latest_status_.torque,
                                     latest_status_.speed, latest_status_.current, latest_status_.temp,
                                     latest_status_.voltage, sample_ms);
    }

    QTimer::singleShot(STATUS_POLL_GAP_MS, this, &MainWindow::requestNextStatus);
}
