#include "graphwidget.h"
#include "theme.h"
#include <QFontMetrics>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>
#include <cmath>
#include <vector>

namespace
{

struct SeriesSpec
{
    const char *name;
    QColor color;
    bool visible;
    GraphWidget::RingBuffer<int> *buf;
    bool left_axis;
    double gain;
};

const int LEFT_AXIS_MAX = 4095;
const int RIGHT_AXIS_SPAN = 1000;

double mapRange(double v, double i0, double i1, double o0, double o1)
{
    if (i1 == i0)
        return o0;
    return (v - i0) * (o1 - o0) / (i1 - i0) + o0;
}

}

GraphWidget::GraphWidget(QWidget *parent) :
    QWidget(parent)
{
    setMouseTracking(true);
    clock_.start();
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &GraphWidget::onTimeout);
    timer_->start(30);
}

bool GraphWidget::viewIsDefault() const
{
    return qFuzzyCompare(x_scale_, 1.0) && qFuzzyCompare(y_scale_, 1.0)
        && qFuzzyIsNull(pan_x_) && qFuzzyIsNull(pan_y_);
}

void GraphWidget::resetView()
{
    x_scale_ = 1.0;
    y_scale_ = 1.0;
    pan_x_ = 0.0;
    pan_y_ = 0.0;
    update();
}

// Keeps the traces reachable. Without this a drag can put the data completely
// outside the plot, leaving a blank grid and no clue which way to drag back.
//
// The rule is that the *centre* of the content stays inside the plot, rather
// than merely some edge of it overlapping. Bounding the edges is not enough:
// content whose right edge just touches plot.left() is legal by that rule and
// entirely invisible. Holding the centre also keeps mid-scale on screen --
// 2048 counts and zero speed/load -- which is where servo data actually sits.
void GraphWidget::clampPan()
{
    if (plot_rect_.isNull())
        return;

    const double w = plot_rect_.width();
    const double h = plot_rect_.height();

    // Time content spans w*x_scale_ and ends at plot.right()+pan_x_, so its
    // centre is at plot.right() - w*x_scale_/2 + pan_x_.
    pan_x_ = qBound(w * x_scale_ / 2.0 - w, pan_x_, w * x_scale_ / 2.0);

    // Value content is centred on the plot's middle, offset by pan_y_.
    pan_y_ = qBound(-h / 2.0, pan_y_, h / 2.0);
}

// Scales about the point under the cursor: whatever was there before the zoom
// is still there after it, which is what makes wheel-zoom feel like it is
// pointing at something rather than jumping.
void GraphWidget::zoomAxis(double &scale, double &pan, double cursor_pos, double anchor, double factor)
{
    const double MIN_SCALE = 1.0;
    const double MAX_SCALE = 60.0;

    const double target = qBound(MIN_SCALE, scale * factor, MAX_SCALE);
    if (qFuzzyCompare(target, scale))
        return;

    // Position under the cursor, in unscaled coordinates, has to be preserved:
    //   cursor = anchor + (base - anchor) * scale + pan
    const double base = anchor + (cursor_pos - pan - anchor) / scale;
    scale = target;
    pan = cursor_pos - anchor - (base - anchor) * scale;
}

void GraphWidget::wheelEvent(QWheelEvent *event)
{
    const int delta = event->angleDelta().y();
    if (delta == 0 || plot_rect_.isNull())
    {
        QWidget::wheelEvent(event);
        return;
    }

    const double factor = delta > 0 ? 1.15 : 1.0 / 1.15;
#if QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    const QPointF pos = event->position();
#else
    const QPointF pos = event->posF();
#endif

    // Time by default, values with shift held: the two axes carry different
    // units, so zooming them together is rarely what is wanted.
    if (event->modifiers() & Qt::ShiftModifier)
        zoomAxis(y_scale_, pan_y_, pos.y(), (plot_rect_.top() + plot_rect_.bottom()) / 2.0, factor);
    else
        zoomAxis(x_scale_, pan_x_, pos.x(), plot_rect_.right(), factor);

    clampPan();
    update();
    event->accept();
}

void GraphWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        dragging_ = true;
        drag_last_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void GraphWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && dragging_)
    {
        dragging_ = false;
        unsetCursor();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

void GraphWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        resetView();
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

void GraphWidget::mouseMoveEvent(QMouseEvent *event)
{
    cursor_ = event->pos();
    cursor_inside_ = true;

    if (dragging_)
    {
        pan_x_ += event->pos().x() - drag_last_.x();
        pan_y_ += event->pos().y() - drag_last_.y();
        drag_last_ = event->pos();
        clampPan();
    }

    update();
    QWidget::mouseMoveEvent(event);
}

void GraphWidget::leaveEvent(QEvent *event)
{
    cursor_inside_ = false;
    update();
    QWidget::leaveEvent(event);
}

void GraphWidget::paintEvent(QPaintEvent *event)
{
    (void) event;
    QPainter painter(this);
    painter.fillRect(rect(), theme::plotCanvas());

    QFont label_font = font();
    label_font.setPointSizeF(qMax(7.0, font().pointSizeF() - 1.5));
    painter.setFont(label_font);
    const QFontMetrics fm(label_font);
    const int text_mid = (fm.ascent() - fm.descent()) / 2;

    const int left_pad = fm.horizontalAdvance("4095") + 16;
    const int right_pad = fm.horizontalAdvance("-1000") + 16;
    const int top_pad = 12;
    const int bottom_pad = fm.height() * 2 + 16;

    const QRect plot(left_pad, top_pad,
                     width() - left_pad - right_pad,
                     height() - top_pad - bottom_pad);
    if (plot.width() < 60 || plot.height() < 50)
        return;

    plot_rect_ = plot;

    const std::size_t capacity = pos_buf_.max_size();
    const std::size_t filled = pos_buf_.size();
    const std::size_t offset = capacity - filled;

    // At rest the whole buffer fills the plot with the newest sample at the
    // right edge, and the value axes show their full range. Zoom magnifies
    // about a fixed anchor -- the right edge for time, the vertical centre for
    // values -- and pan then slides the result.
    const double y_centre = (plot.top() + plot.bottom()) / 2.0;

    auto axis_x = [&](double slot) {
        const double base = mapRange(slot, 0, (double)capacity,
                                     plot.right() - plot.width(), plot.right());
        return plot.right() + (base - plot.right()) * x_scale_ + pan_x_;
    };
    auto left_axis_y = [&](double v) {
        const double base = mapRange(v, 0, LEFT_AXIS_MAX, plot.bottom(), plot.top());
        return y_centre + (base - y_centre) * y_scale_ + pan_y_;
    };
    auto left_value_at = [&](double y) {
        const double base = y_centre + (y - pan_y_ - y_centre) / y_scale_;
        return mapRange(base, plot.bottom(), plot.top(), 0, LEFT_AXIS_MAX);
    };
    auto right_axis_y = [&](double v) {
        const double base = mapRange(v, -RIGHT_AXIS_SPAN, RIGHT_AXIS_SPAN, plot.bottom(), plot.top());
        return y_centre + (base - y_centre) * y_scale_ + pan_y_;
    };
    auto right_value_at = [&](double y) {
        const double base = y_centre + (y - pan_y_ - y_centre) / y_scale_;
        return mapRange(base, plot.bottom(), plot.top(), -RIGHT_AXIS_SPAN, RIGHT_AXIS_SPAN);
    };

    // What the plot edges actually correspond to right now. Everything the
    // axes label is derived from these, so the numbers cannot drift from the
    // picture however the view has been zoomed or dragged.
    const double left_lo = left_value_at(plot.bottom());
    const double left_hi = left_value_at(plot.top());
    const double right_lo = right_value_at(plot.bottom());
    const double right_hi = right_value_at(plot.top());

    const int rows = 10;
    QPen grid_pen;
    grid_pen.setWidthF(1.0);
    grid_pen.setColor(theme::plotGrid());
    painter.setPen(grid_pen);
    painter.setRenderHint(QPainter::Antialiasing, false);

    for (int i = 0; i <= rows; i++)
    {
        const int y = plot.top() + i * (plot.height() - 1) / rows;
        painter.drawLine(plot.left(), y, plot.right(), y);
    }

    const int cols = 6;
    std::vector<double> tick_slots;
    for (int i = 0; i <= cols; i++)
        tick_slots.push_back((double)(capacity - 1) * i / cols);
    for (double slot : tick_slots)
    {
        const int x = (int)axis_x(slot);
        if (x < plot.left() || x > plot.right())
            continue;
        painter.drawLine(x, plot.top(), x, plot.bottom());
    }

    QPen frame_pen(theme::border());
    frame_pen.setWidthF(1.0);
    painter.setPen(frame_pen);
    painter.drawRect(plot);

    const int zero_y = (int)right_axis_y(0);
    QPen zero_pen(theme::borderStrong());
    zero_pen.setWidthF(1.0);
    painter.setPen(zero_pen);
    painter.drawLine(plot.left(), zero_y, plot.right(), zero_y);

    // Ticks are spread across whatever range is actually on screen.
    const int AXIS_TICKS = 5;
    for (int i = 0; i < AXIS_TICKS; i++)
    {
        const double t = (double)i / (AXIS_TICKS - 1);

        const double lv = left_lo + (left_hi - left_lo) * t;
        const int ly = (int)left_axis_y(lv);
        const QString ls = QString::number(qRound(lv));
        painter.setPen(theme::border());
        painter.drawLine(plot.left() - 4, ly, plot.left(), ly);
        painter.setPen(i == AXIS_TICKS / 2 ? theme::textPrimary() : theme::textDim());
        painter.drawText(plot.left() - 9 - fm.horizontalAdvance(ls), ly + text_mid, ls);

        const double rv = right_lo + (right_hi - right_lo) * t;
        const int ry = (int)right_axis_y(rv);
        const QString rs = QString::number(qRound(rv));
        painter.setPen(theme::border());
        painter.drawLine(plot.right(), ry, plot.right() + 4, ry);
        painter.setPen(theme::textDim());
        painter.drawText(plot.right() + 9, ry + text_mid, rs);
    }

    const qint64 now = clock_.elapsed();
    for (double slot : tick_slots)
    {
        const int x = (int)axis_x(slot);
        if (x < plot.left() || x > plot.right())
            continue;
        const long long d = (long long)slot - (long long)offset;
        QString text = "-";
        if (filled > 1 && d >= 0 && d < (long long)filled)
        {
            const double age = (now - time_buf_.at((std::size_t)d)) / 1000.0;
            text = QString::number(-age, 'f', 1) + "s";
        }
        const int tw = fm.horizontalAdvance(text);
        int tx = x - tw / 2;
        tx = qBound(plot.left(), tx, plot.right() - tw);
        painter.drawText(tx, plot.bottom() + 6 + fm.ascent(), text);
    }

    std::vector<SeriesSpec> series = {
        {"Position",    theme::plotPosition(),    pos_visible,     &pos_buf_,     true,  1.0},
        {"Goal",        theme::plotGoal(),        goal_visible,    &goal_buf_,    true,  1.0},
        {"Torque",      theme::plotTorque(),      torque_visible,  &torque_buf_,  false, 1.0},
        {"Speed",       theme::plotSpeed(),       speed_visible,   &speed_buf_,   false, 0.2},
        {"Current",     theme::plotCurrent(),     current_visible, &current_buf_, false, 1.0},
        {"Temperature", theme::plotTemperature(), temp_visible,    &temp_buf_,    false, 1.0},
        {"Voltage",     theme::plotVoltage(),     voltage_visible, &voltage_buf_, false, 1.0},
    };

    painter.save();
    painter.setClipRect(plot.adjusted(1, 1, 0, 0));
    painter.setRenderHint(QPainter::Antialiasing, true);

    for (const SeriesSpec &s : series)
    {
        if (!s.visible || s.buf->size() < 2)
            continue;
        QPen pen(s.color);
        pen.setWidthF(1.6);
        painter.setPen(pen);
        QPainterPath path;
        for (std::size_t i = 0; i < s.buf->size(); i++)
        {
            const double x = axis_x((double)(offset + i));
            const double y = s.left_axis ? left_axis_y(s.gain * s.buf->at(i))
                                         : right_axis_y(s.gain * s.buf->at(i));
            if (i == 0)
                path.moveTo(x, y);
            else
                path.lineTo(x, y);
        }
        painter.drawPath(path);
    }

    QPen limit_pen(theme::plotLimit());
    limit_pen.setStyle(Qt::DashLine);
    limit_pen.setWidthF(1.2);
    painter.setPen(limit_pen);
    for (int limit : {up_limit, down_limit})
    {
        if (limit == 0)
            continue;
        const int y1 = (int)right_axis_y(limit);
        const int y2 = (int)right_axis_y(-limit);
        painter.drawLine(plot.left(), y1, plot.right(), y1);
        painter.drawLine(plot.left(), y2, plot.right(), y2);
    }
    painter.restore();

    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(theme::textDisabled());
    const int footer_y = plot.bottom() + 10 + fm.height() + fm.ascent();
    // Three notes share one line: axis ranges left and centre, view state
    // right. They are only drawn if they actually fit -- at narrow widths the
    // longest hint is dropped first, then the centre note, rather than letting
    // them overlap into an unreadable smear.
    const QString left_note = QString("left axis: counts %1-%2")
                                  .arg(qRound(left_lo)).arg(qRound(left_hi));
    const QString right_note = QString("right axis: %1 to %2 (speed x0.2)")
                                   .arg(qRound(right_lo)).arg(qRound(right_hi));

    // At rest say how to drive the plot; once moved, say where it is and how
    // to get back. The short forms are used when the long ones will not fit.
    const bool at_rest = viewIsDefault();
    const QString hint_long = at_rest
        ? QString("wheel: zoom time  ·  shift+wheel: zoom values  ·  drag: pan")
        : QString("x%1  y%2  ·  double-click to reset").arg(x_scale_, 0, 'f', 1).arg(y_scale_, 0, 'f', 1);
    const QString hint_short = at_rest
        ? QString("wheel: zoom  ·  drag: pan")
        : QString("x%1  y%2").arg(x_scale_, 0, 'f', 1).arg(y_scale_, 0, 'f', 1);

    const graph_footer::Layout footer = graph_footer::place(
        plot.left(), plot.right(),
        fm.horizontalAdvance(left_note),
        fm.horizontalAdvance(right_note),
        fm.horizontalAdvance(hint_long),
        fm.horizontalAdvance(hint_short),
        14);

    painter.drawText(plot.left(), footer_y, left_note);
    if (footer.draw_centre)
        painter.drawText(footer.centre_x, footer_y, right_note);
    if (footer.draw_hint)
        painter.drawText(footer.hint_x, footer_y, footer.hint_is_long ? hint_long : hint_short);

    if (!cursor_inside_ || !plot.contains(cursor_) || filled < 1)
        return;

    double best_dx = 1e9;
    long long best_d = -1;
    int best_x = 0;
    for (std::size_t i = 0; i < filled; i++)
    {
        const double x = axis_x((double)(offset + i));
        const double dx = std::fabs(x - cursor_.x());
        if (dx < best_dx)
        {
            best_dx = dx;
            best_d = (long long)i;
            best_x = (int)x;
        }
    }
    if (best_d < 0)
        return;

    QPen cross_pen(theme::borderStrong());
    cross_pen.setStyle(Qt::DotLine);
    painter.setPen(cross_pen);
    painter.drawLine(best_x, plot.top(), best_x, plot.bottom());

    QStringList rows_text;
    std::vector<QColor> rows_color;
    const double age = (now - time_buf_.at((std::size_t)best_d)) / 1000.0;
    rows_text << QString("t  -%1s").arg(age, 0, 'f', 2);
    rows_color.push_back(theme::textDim());
    for (const SeriesSpec &s : series)
    {
        if (!s.visible)
            continue;
        rows_text << QString("%1  %2").arg(s.name).arg(s.buf->at((std::size_t)best_d));
        rows_color.push_back(s.color);
    }

    int box_w = 0;
    for (const QString &r : rows_text)
        box_w = qMax(box_w, fm.horizontalAdvance(r));
    box_w += 26;
    const int line_h = fm.height() + 2;
    const int box_h = line_h * rows_text.size() + 10;

    int bx = best_x + 12;
    if (bx + box_w > plot.right())
        bx = best_x - 12 - box_w;
    bx = qBound(plot.left(), bx, qMax(plot.left(), plot.right() - box_w));
    int by = qBound(plot.top(), cursor_.y() - box_h / 2, qMax(plot.top(), plot.bottom() - box_h));

    QColor panel = theme::bgRaised();
    panel.setAlpha(238);
    painter.setPen(QPen(theme::borderStrong()));
    painter.setBrush(panel);
    painter.drawRoundedRect(QRect(bx, by, box_w, box_h), 4, 4);
    painter.setBrush(Qt::NoBrush);

    for (int i = 0; i < rows_text.size(); i++)
    {
        const int ty = by + 6 + i * line_h + fm.ascent();
        if (i > 0)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(rows_color[i]);
            painter.drawRect(bx + 8, ty - fm.ascent() / 2 - 3, 7, 7);
            painter.setBrush(Qt::NoBrush);
        }
        painter.setPen(i == 0 ? theme::textDim() : theme::textPrimary());
        painter.drawText(bx + 20, ty, rows_text[i]);
    }
}

void GraphWidget::onTimeout() {
    update();
}

void GraphWidget::reset_data()
{
    pos_buf_.clear();
    goal_buf_.clear();
    torque_buf_.clear();
    speed_buf_.clear();
    current_buf_.clear();
    temp_buf_.clear();
    voltage_buf_.clear();
    time_buf_.clear();
}

void GraphWidget::append_data(int pos, int goal, int torque, int speed, int current, int temp, int voltage)
{
    pos_buf_.push(pos);
    goal_buf_.push(goal);
    torque_buf_.push(torque);
    speed_buf_.push(speed);
    current_buf_.push(current);
    temp_buf_.push(temp);
    voltage_buf_.push(voltage);
    time_buf_.push(clock_.elapsed());
}
