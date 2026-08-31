#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QMainWindow>
#include <QPointer>
#include <QSerialPort>
#include <QStandardItemModel>
#include <QStringList>
#include <QTableView>
#include <QThread>
#include "servo/scserial.h"
#include "servo/servo_bus.h"
#include "calibration_json.h"
#include "register_snapshot.h"
#include "confirm_steps.h"
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

// How long each processEvents slice waits while draining a bus reply that
// arrived after the event loop was already told to quit.
constexpr int BUS_DRAIN_SLICE_MS = 5;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    void setupComSettings();
    void setupServoLists();
    void setupServoControl();
    void setupAutoDebug();
    void setupDataAnalysis();
    void setupProgramming();
    void styleGraphLegend();
    void setupCalibration();

    void setEnableComSettings(bool state);
    void clearServoList();
    void appendServoList(const int id, const QString &name);
    void clearProgMemTable();
    void updatePorgMemTable();
    void setIntRangeLineEdit(QLineEdit *edit, int min, int max);
    void setIntLineEdit(QLineEdit *edit);
    void setDoubleLineEdit(QLineEdit *edit);

    int countsPerRev() const;
    int centerCount() const;
    double countsToRadians(int count) const;
    double countsToDegrees(int count) const;
    int angleToCounts(double angle, bool is_radians) const;
    bool isAngleInputRadians() const;
    void updateAngleInputFromCounts(int count);
    void updateAnglePreview();
    void syncControlToPosition(int pos);
    bool controlFollowEnabled() const;
    void noteControlInteraction() { control_touch_.restart(); }

    bool isServoValidNow() const { return !(is_searching_ || !bus_open_ || select_servo_.id_ < 0); }

    // Runs fn on the bus thread and waits for its result, while letting the
    // window repaint and its timers run, so a slow servo transaction does not
    // freeze the UI. User input is held back for the duration, so a multi-step
    // EPROM write cannot be interrupted half way through.
    //
    // fn runs on the other thread: it may use bus_ and its own by-value
    // captures, and nothing else. Reading bus_ from there is safe only because
    // it is never reassigned after the constructor.
    template<typename F>
    auto runOnBus(F &&fn) -> decltype(fn())
    {
        using Result = decltype(fn());

        // Heap-allocated so the result outlives fn no matter which thread
        // finishes with it last.
        struct Shared { Result value{}; bool done = false; };
        auto shared = std::make_shared<Shared>();

        QEventLoop loop;
        QPointer<QEventLoop> loop_ptr(&loop);

        QMetaObject::invokeMethod(bus_, [shared, loop_ptr, f = std::forward<F>(fn)]() mutable {
            Result produced = f();
            // Hop back to the UI thread to publish the result and end the wait.
            QMetaObject::invokeMethod(qApp, [shared, loop_ptr, produced]() {
                shared->value = produced;
                shared->done = true;
                if(loop_ptr)
                    loop_ptr->quit();
            }, Qt::QueuedConnection);
        }, Qt::QueuedConnection);

        const bool was_busy = bus_busy_;
        bus_busy_ = true;
        loop.exec(QEventLoop::ExcludeUserInputEvents);

        // exec() returns -1 immediately once the application has started
        // quitting -- QCoreApplication::exit() exits every nested loop on this
        // thread -- so the reply may not have arrived yet. The bus thread is
        // still running at this point (it is stopped in ~MainWindow), so keep
        // pumping until it answers rather than handing back a default-built
        // result that callers would read as real data.
        while(!shared->done)
        {
            QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents, BUS_DRAIN_SLICE_MS);
            if(!shared->done)
                QThread::msleep(1);
        }

        bus_busy_ = was_busy;

        return shared->value;
    }

    // Queues fn on the bus thread and returns straight away. For commands
    // whose result nothing waits on, such as a position write.
    template<typename F>
    void postToBus(F &&fn)
    {
        QMetaObject::invokeMethod(bus_, std::forward<F>(fn), Qt::QueuedConnection);
    }

    void selectServoSeries(feetech_servo::ModelSeries series);
    void requestNextStatus();
    bool writeMemValue(const feetech_servo::MemoryConfig &config, int value);
    int readMemValue(const feetech_servo::MemoryConfig &config);
    void setProgStatus(const QString &text);
    bool confirmDangerousAction(const QString &title, const QStringList &steps);
    void updateTorqueAllButton();
    void refreshTorqueStates();
    void updateFunctionalityGate();
    void updateServoListTorqueColors();
    int aggregateTorqueState() const;

    void updateCalibrationGate();
    void setCalibrationEnabled(bool enabled);
    void populateCalibrationTable();
    void ensureCalibVectors();
    bool writeCalibWord(uint8_t id, feetech_servo::ModelSeries series, const QString &reg_name, int value);
    std::optional<int> readCalibPosition(uint8_t id, feetech_servo::ModelSeries series);
    std::vector<int> readCalibPositions();

    // Outcome of homing one row of the calibration table.
    struct HomeRowResult
    {
        bool written = false;
        int homing_offset = 0;
        int half_turn = 0;
        int corrected = -1;
    };
    QVector<JointCalibration> collectCalibrationJoints();

    void writePos(int pos, int time, int speed, int acc);
    void syncWritePos(int pos, int time, int speed, int acc);
    void regWritePos(int pos, int time, int speed, int acc);

private slots:
    void onPortSearchTimerTimeout();
    void onConnectButtonClicked();

    void onSearchButtonClicked();
    void onSearchTimerTimeout();
    void onServoListSelection();
    void onTorqueAllButtonClicked();
    void onTorqueStateTimerTimeout();

    void onGoalSliderValueChanged();
    void onSetBuggonClicked();
    void onTorqueEnableCheckBoxStateChanged();
    void onModeRadioButtonsToggled(bool checked);
    void onActionButtonClicked();
    void onAngleSetButtonClicked();
    void onAngleInputChanged();
    void onAngleUnitChanged();

    void onSweepButtonClicked();
    void onStepButtonClicked();
    void onAutoDebugTimerTimeout();

    void onExportButtonClicked();
    void onClearButtonClicked();
    void onDataAnalysisTimerTimeout();

    void onProgTimerTimeout();
    void onMemoryTableSelection();
    void onMemSetButtonClicked();
    void onMemSaveClicked();
    void onMemLoadClicked();

    void onCalibRecheckClicked();
    void onCalibApplyNamesClicked();
    void onCalibHomeClicked();
    void onCalibRecordClicked();
    void onCalibExportClicked();
    void onCalibTimerTimeout();
    void onTabChanged(int index);
    void onCalibrationMidButtonClicked();

    void onGraphTimerTimeout();
    void onStatusReady(const feetech_servo::ServoStatus &status);
    void onBusOpenedChanged(bool is_open);

private:
    Ui::MainWindow *ui;
    QTimer *graph_timer_;
    QThread *bus_thread_;
    feetech_servo::ServoBus *bus_;
    QStandardItemModel *servo_list_model_;
    QStandardItemModel *prog_mem_model_;
    QTimer *port_search_timer_;
    QTimer *search_timer_;
    QTimer *auto_debug_timer_;
    QTimer *prog_timer_;
    QTimer *data_analysis_timer_;
    QTimer *calib_timer_;
    QTimer *torque_state_timer_;
    QStandardItemModel *calib_model_;

    // Mirrors the bus thread's port state, so the UI can ask "is the port
    // open" without reaching across to the QSerialPort itself.
    bool bus_open_ = false;
    // Set while runOnBus is waiting. The periodic polls skip their turn rather
    // than nest another wait inside it.
    bool bus_busy_ = false;
    // Set while a modal confirmation is up. That dialog runs its own event
    // loop, so the periodic polls have to stand down for the same reason.
    bool prompt_open_ = false;
    // True when no periodic poll should touch the bus or the models.
    bool pollsSuspended() const { return bus_busy_ || prompt_open_; }
    // Set between asking for a telemetry sample and it arriving, so only one
    // request is ever outstanding and a slow bus paces itself.
    bool status_pending_ = false;

    // Port names currently in the COM dropdown, so it is only rebuilt when the
    // set actually changes rather than on every refresh tick.
    QStringList com_port_names_;

    bool is_searching_ = false;
    std::vector<uint8_t> id_list_;
    int search_id_ = 0;
    struct
    {
        feetech_servo::ModelSeries model_ = feetech_servo::ModelSeries::STS;
        int id_ = -1;
    }select_servo_;
    enum Mode
    {
        MODE_WRITE,
        MODE_SYNC_WRITE,
        MODE_REG_WRITE
    };
    Mode mode_ = MODE_WRITE;
    bool sweep_running_ = false;
    bool step_running_ = false;
    int latest_auto_debug_goal_ = 0;
    bool step_increase_ = true;
    bool is_mem_writing_ = false;
    bool all_torque_on_ = true;
    std::map<int, int> torque_state_;
    // Servo Status register per ID, as last polled. -1 means the servo did not
    // answer; 0 means it answered and reports no fault.
    std::map<int, int> fault_state_;
    QElapsedTimer control_touch_;
    bool calib_gate_open_ = false;
    bool calib_recording_ = false;
    int calib_gate_tick_ = 0;
    std::vector<feetech_servo::ModelSeries> calib_series_;
    std::vector<int> calib_min_;
    std::vector<int> calib_max_;

    bool is_recording_ = false;
    int file_write_interval_ = 0;
    size_t record_data_count_ = 0;
    QString record_file_name_;
    QString record_section_data_;

    feetech_servo::ServoStatus latest_status_;
};
#endif
