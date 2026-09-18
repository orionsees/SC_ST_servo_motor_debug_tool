#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QElapsedTimer>
#include <QMainWindow>
#include <QObject>
#include <QPoint>
#include <QRect>
#include <QWidget>

// Placement of the three notes on the plot footer: axis ranges left and
// centre, view state right. Kept out of paintEvent because "do these collide"
// is easy to get subtly wrong -- summing the widths is not enough, since the
// centre note is centred and can come within a pixel of the right-aligned one
// while the total still appears to fit.
namespace graph_footer
{

struct Layout
{
    bool draw_centre = false;
    int centre_x = 0;
    bool draw_hint = false;
    bool hint_is_long = false;
    int hint_x = 0;
    int hint_w = 0;
};

// The left note is always drawn at plot_left. Returns where the other two go,
// dropping the hint to its short form and then the centre note entirely, in
// that order, as the width runs out.
inline Layout place(int plot_left, int plot_right, int left_w,
                    int centre_w, int hint_long_w, int hint_short_w, int gap)
{
    Layout out;
    const int left_end = plot_left + left_w;
    const int mid = plot_left + (plot_right - plot_left) / 2;

    if (left_end + gap <= plot_right - hint_long_w)
    {
        out.draw_hint = true;
        out.hint_is_long = true;
        out.hint_w = hint_long_w;
    }
    else if (left_end + gap <= plot_right - hint_short_w)
    {
        out.draw_hint = true;
        out.hint_is_long = false;
        out.hint_w = hint_short_w;
    }
    out.hint_x = plot_right - out.hint_w;

    const int hint_start = out.draw_hint ? out.hint_x : plot_right;
    const int centre_x = mid - centre_w / 2;
    if (centre_x >= left_end + gap && centre_x + centre_w + gap <= hint_start)
    {
        out.draw_centre = true;
        out.centre_x = centre_x;
    }
    return out;
}

}

class GraphWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GraphWidget(QWidget *parent = nullptr);

    template<typename T>
    class RingBuffer {
    public:
        RingBuffer(std::size_t size = 300)
            : buffer(size), capacity(size), head(0), tail(0), count(0) {}

        void push(const T& value) {
            if (count == capacity) {
                tail = (tail + 1) % capacity;
            } else {
                count++;
            }

            buffer[head] = value;
            head = (head + 1) % capacity;
        }

        bool pop(T& value) {
            if (is_empty()) {
                return false;
            }

            value = buffer[tail];
            tail = (tail + 1) % capacity;
            count--;
            return true;
        }

        T at(std::size_t index) const {
            if (index >= count) {
                throw std::out_of_range("Index out of range");
            }
            return buffer[(tail + index) % capacity];
        }

        bool is_empty() const {
            return count == 0;
        }

        std::size_t size() const {
            return count;
        }

        std::size_t max_size() const {
            return capacity;
        }

        void resize(std::size_t size){
            buffer.resize(size);
        }

        void clear() {
            head = 0;
            tail = 0;
            count = 0;
        }

    private:
        std::vector<T> buffer;
        std::size_t capacity;
        std::size_t head;
        std::size_t tail;
        std::size_t count;
    };

    void reset_data();
    // sample_ms is when the sample was taken, on this widget's own clock.
    // Pass -1 to stamp it on arrival instead, which is only right when the
    // two are the same thing -- they are not once the samples have crossed a
    // network, where arrival time carries the link's jitter rather than the
    // servo's motion.
    void append_data(int pos, int goal, int torque, int speed, int current, int temp, int voltage,
                     qint64 sample_ms = -1);

    // Reading of the same clock the timestamps are measured against, so a
    // caller can line an outside clock up with it.
    qint64 elapsed() const { return clock_.elapsed(); }

    bool pos_visible = true;
    bool goal_visible = true;
    bool torque_visible = false;
    bool speed_visible = false;
    bool current_visible = false;
    bool temp_visible = false;
    bool voltage_visible = false;

    int down_limit = 0;
    int up_limit = 0;

    int sample_count() const { return (int)pos_buf_.size(); }

    // Back to showing everything: full time span, full value range.
    void resetView();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void leaveEvent(QEvent *event) override;

private slots:
    void onTimeout();

private:
    // The view is held as a magnification plus a pixel pan for each axis,
    // rather than as an explicit value range, so that zooming about the cursor
    // and dragging to pan compose without either having to know about the
    // other. Tick labels are recovered by inverting the mapping at the plot
    // edges, which keeps them honest whatever the view is doing.
    double x_scale_ = 1.0;
    double y_scale_ = 1.0;
    double pan_x_ = 0.0;
    double pan_y_ = 0.0;

    bool viewIsDefault() const;
    void zoomAxis(double &scale, double &pan, double cursor_pos, double anchor, double factor);
    void clampPan();

    QTimer *timer_;
    double phase_ = 0;
    QElapsedTimer clock_;
    QPoint cursor_;
    bool cursor_inside_ = false;
    bool dragging_ = false;
    QPoint drag_last_;
    // Last plot rect painted, so the event handlers can clamp against it.
    QRect plot_rect_;

    RingBuffer<int> pos_buf_;
    RingBuffer<int> goal_buf_;
    RingBuffer<int> torque_buf_;
    RingBuffer<int> speed_buf_;
    RingBuffer<int> current_buf_;
    RingBuffer<int> temp_buf_;
    RingBuffer<int> voltage_buf_;
    RingBuffer<qint64> time_buf_;
};

#endif
