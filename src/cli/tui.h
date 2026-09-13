#ifndef CLI_TUI_H
#define CLI_TUI_H

#include <QElapsedTimer>
#include <QMap>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

#include <functional>

#include "calibration_json.h"
#include "cli/plot.h"
#include "cli/session.h"
#include "cli/term.h"

class QSocketNotifier;
class QTimer;

namespace cli
{

// The window, in a terminal: the same three tabs, the same servo list and
// connection panel, the same gating, driven from the keyboard.
class Tui : public QObject
{
    Q_OBJECT

public:
    explicit Tui(const ConnectionOptions &options, QObject *parent = nullptr);
    ~Tui() override;

    // Sets everything up, runs the event loop, and puts the terminal back.
    int run();

private:
    enum class Tab
    {
        Debug,
        Programming,
        Calibration
    };

    enum class Overlay
    {
        None,
        Help,
        Prompt,
        Confirm,
        Menu
    };

    struct MenuItem
    {
        QChar key;
        QString label;
        std::function<void()> action;
    };

    struct CalibRow
    {
        int id = 0;
        QString name;
        int drive_mode = 0;
        QString homing_offset = "-";
        QString range_min = "-";
        QString range_max = "-";
        ModelSeries series = ModelSeries::STS;
        int min_seen = 0;
        int max_seen = 0;
    };

    // --- lifecycle ---
    void setupTimers();
    void shutdown();

    // --- input ---
    void onStdinReady();
    void handleKey(const term::Event &event);
    void handleGlobalKey(const term::Event &event);
    void handleDebugKey(const term::Event &event);
    void handleProgrammingKey(const term::Event &event);
    void handleCalibrationKey(const term::Event &event);
    void handleMouse(const term::Event &event);
    void handleOverlayKey(const term::Event &event);

    // --- overlays ---
    void ask(const QString &title, const QString &initial, std::function<void(const QString &)> done);
    void confirm(const QString &title, const QStringList &steps, std::function<void()> accepted);
    void menu(const QString &title, const QVector<MenuItem> &items);
    void closeOverlay();

    // --- polling ---
    void onTelemetryTick();
    void onSearchTick();
    void onTorqueTick();
    void onProgrammingTick();
    void onCalibrationTick();
    void onAutoDebugTick();
    void onRecordTick();
    void render();

    // --- actions ---
    void toggleConnection();
    void startSearch();
    void stopSearch();
    void selectServoAt(int index);
    void stepSelection(int delta);
    void toggleTorqueAll();
    void toggleTorqueSelected();
    void refreshTorqueStates();
    void sendGoal(int goal);
    void setGoalFromAngle(double angle);
    void toggleSweep();
    void toggleStep();
    void toggleRecording();
    void writeSelectedRegister(const QString &text);
    void saveRegisterSnapshot(const QString &path);
    void loadRegisterSnapshot(const QString &path);
    void setMidpoint();
    void updateCalibrationGate(bool force);
    void populateCalibrationTable();
    void applyJointNames(const QString &text);
    void calibrationSetHome();
    void toggleRangeRecording();
    void exportCalibration(const QString &path);

    // --- drawing ---
    QStringList renderLeftPanel(int height) const;
    QStringList renderDebug(int height, int width);
    QStringList renderProgramming(int height, int width);
    QStringList renderCalibration(int height, int width) const;
    QStringList renderOverlay(int height, int width) const;
    QString renderTitle(int width) const;
    QString renderTabs(int width) const;
    QString renderHints(int width) const;

    // --- helpers ---
    bool servoSelected() const;
    int selectedId() const;
    bool ready() const;
    bool pollsSuspended() const;
    ModelSeries selectedSeries() const;
    int countsPerRev() const;
    int maxCount() const;
    QString servoColor(int id) const;
    QString torqueSummary() const;
    void setMessage(const QString &text, bool warning = false);
    const std::vector<MemoryConfig> &registers() const;
    void appendSample(const ServoStatus &status);
    // True when the goal controls should follow the measured position, as the
    // window's do once the user has left them alone for a moment.
    bool controlFollowEnabled() const;
    void noteControlInteraction() { control_touch_.restart(); }

    Session session_;
    term::Screen screen_;
    term::KeyReader keys_;
    QSocketNotifier *stdin_notifier_ = nullptr;

    QTimer *render_timer_ = nullptr;
    QTimer *telemetry_timer_ = nullptr;
    QTimer *search_timer_ = nullptr;
    QTimer *torque_timer_ = nullptr;
    QTimer *programming_timer_ = nullptr;
    QTimer *calibration_timer_ = nullptr;
    QTimer *auto_debug_timer_ = nullptr;
    QTimer *record_timer_ = nullptr;

    ConnectionOptions options_;
    Tab tab_ = Tab::Debug;
    bool quitting_ = false;

    // --- servo list ---
    int selected_index_ = -1;
    bool searching_ = false;
    int search_id_ = 0;
    QMap<int, int> torque_state_;
    QMap<int, int> fault_state_;
    bool all_torque_on_ = true;

    // --- telemetry ---
    ServoStatus latest_;
    Ring<int> pos_buf_{300};
    Ring<int> goal_buf_{300};
    Ring<int> torque_buf_{300};
    Ring<int> speed_buf_{300};
    Ring<int> current_buf_{300};
    Ring<int> temp_buf_{300};
    Ring<int> voltage_buf_{300};
    Ring<qint64> time_buf_{300};
    QElapsedTimer clock_;
    // When the current bus transaction started, so the title can say "working"
    // for the ones worth waiting on rather than flickering on every poll.
    QElapsedTimer busy_since_;
    bool series_visible_[7] = {true, true, false, false, false, false, false};
    PlotView view_;
    int up_limit_ = 0;
    int down_limit_ = 0;
    int cursor_cell_ = -1;
    QStringList readout_;
    QStringList readout_colors_;
    bool dragging_ = false;
    int drag_x_ = 0;
    int drag_y_ = 0;
    // Plot area geometry from the last frame, so a mouse position can be
    // turned into a zoom anchor without guessing the layout.
    int plot_left_ = 0;
    int plot_top_ = 0;
    int plot_cols_ = 0;
    int plot_rows_ = 0;

    // --- control ---
    WriteMode mode_ = WriteMode::Write;
    int goal_ = 2048;
    int speed_ = 0;
    int acc_ = 0;
    int time_ = 0;
    bool angle_in_radians_ = false;
    bool torque_selected_on_ = true;
    QElapsedTimer control_touch_;

    // --- auto debug ---
    int auto_start_ = 0;
    int auto_end_ = 4095;
    int auto_hold_ms_ = 2500;
    int auto_step_ = 10;
    int auto_delay_ms_ = 10;
    bool sweep_running_ = false;
    bool step_running_ = false;
    bool step_increasing_ = true;
    int auto_goal_ = 0;

    // --- recording ---
    Recorder recorder_;
    QString record_path_ = "~/record.txt";
    int record_interval_ = 30;

    // --- programming ---
    int register_row_ = 0;
    int register_scroll_ = 0;
    int register_visible_ = 10;
    QVector<std::optional<int>> register_values_;
    QString prog_status_ = "-";

    // --- calibration ---
    QVector<CalibRow> calib_rows_;
    int calib_row_ = 0;
    bool calib_gate_open_ = false;
    bool calib_recording_ = false;
    int calib_gate_tick_ = 0;
    QString calib_status_ = "Serial port is not open.";

    // --- overlay ---
    Overlay overlay_ = Overlay::None;
    QString overlay_title_;
    QString prompt_buffer_;
    int prompt_cursor_ = 0;
    // A prompt opens with the current value in it and that value selected, so
    // typing replaces it and an arrow key or backspace keeps it to be edited.
    bool prompt_fresh_ = false;
    std::function<void(const QString &)> prompt_done_;
    QStringList confirm_steps_;
    int confirm_index_ = 0;
    std::function<void()> confirm_accepted_;
    QVector<MenuItem> menu_items_;

    QString message_;
    bool message_warning_ = false;
};

}

#endif
