#ifndef CLI_PLOT_H
#define CLI_PLOT_H

#include <QString>
#include <QStringList>
#include <QVector>

// The GUI's telemetry plot, drawn with braille cells instead of a painter.
// The view maths -- what the axes span, how zoom anchors, how far a pan may
// go -- is the same as graphwidget.cpp, so the two show the same picture of
// the same samples.
namespace cli
{

template<typename T>
class Ring
{
public:
    explicit Ring(int capacity = 300)
        : buf_(capacity)
        , capacity_(capacity)
    {
    }

    void push(const T &value)
    {
        buf_[head_] = value;
        head_ = (head_ + 1) % capacity_;
        if(count_ < capacity_)
            count_++;
    }

    void clear()
    {
        head_ = 0;
        count_ = 0;
    }

    T at(int index) const
    {
        const int tail = (head_ - count_ + capacity_) % capacity_;
        return buf_[(tail + index) % capacity_];
    }

    int size() const { return count_; }
    int capacity() const { return capacity_; }

private:
    QVector<T> buf_;
    int capacity_;
    int head_ = 0;
    int count_ = 0;
};

struct PlotSeries
{
    QString name;
    QString color;
    bool visible = false;
    const Ring<int> *data = nullptr;
    bool left_axis = false;
    double gain = 1.0;
    char mark = '*';                   // used when braille is off
};

// Magnification plus a pan for each axis, exactly as the GUI holds it, so
// zooming about a point and dragging compose without either knowing about the
// other.
struct PlotView
{
    double x_scale = 1.0;
    double y_scale = 1.0;
    double pan_x = 0.0;
    double pan_y = 0.0;

    bool isDefault() const;
    void reset();

    // width and height are the plot area in braille sub-pixels.
    void zoomTime(double cursor_x, double width, double factor);
    void zoomValues(double cursor_y, double height, double factor);
    void panBy(double dx, double dy, double width, double height);
    void clamp(double width, double height);
};

struct PlotInput
{
    QVector<PlotSeries> series;
    const Ring<qint64> *times = nullptr;
    qint64 now_ms = 0;
    int up_limit = 0;
    int down_limit = 0;
    // Column the cursor sits on, in plot-area cells, or -1 for no crosshair.
    int cursor_cell = -1;
};

struct PlotOutput
{
    QStringList lines;
    // Values under the crosshair, ready to print beside the plot. Empty when
    // there is no crosshair or no sample there.
    QStringList readout;
    QStringList readout_colors;
};

// Renders the plot into exactly `rows` lines of `cols` columns: the frame and
// traces, an axis-label row, and a footer describing the axes and the view.
PlotOutput renderPlot(int rows, int cols, const PlotInput &input, const PlotView &view);

// Plot-area geometry, so the caller can turn a mouse position into a zoom
// anchor and a crosshair column.
struct PlotGeometry
{
    int left_cell = 0;                 // first cell of the plot area
    int area_cols = 0;
    int area_rows = 0;
};

PlotGeometry plotGeometry(int rows, int cols);

}

#endif
