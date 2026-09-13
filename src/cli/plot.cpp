#include "cli/plot.h"

#include "cli/term.h"

#include <QtGlobal>
#include <cmath>

namespace cli
{

namespace
{

// The two value axes, shared with the GUI: counts on the left, a signed span
// on the right for everything else.
const int LEFT_AXIS_MAX = 4095;
const int RIGHT_AXIS_SPAN = 1000;

const int LEFT_GUTTER = 6;
// One column for the frame, one space, then up to five characters of label
// ("-1000" is the widest the right axis prints).
const int RIGHT_GUTTER = 7;

// Braille cells are two sub-pixels wide and four tall.
const int SUB_X = 2;
const int SUB_Y = 4;

const double MIN_SCALE = 1.0;
const double MAX_SCALE = 60.0;

const QString GRID_COLOR = "#2A3138";
const QString FRAME_COLOR = "#3A4550";
const QString LABEL_COLOR = "#9AA4B0";
const QString LIMIT_COLOR = "#FF6B9D";

double mapRange(double v, double i0, double i1, double o0, double o1)
{
    if(i1 == i0)
        return o0;
    return (v - i0) * (o1 - o0) / (i1 - i0) + o0;
}

// One cell of the plot area: the braille dots set in it, plus the colour of
// whichever series set the last of them. A terminal cell can only carry one
// colour, so overlapping traces resolve in draw order.
struct Cell
{
    quint8 dots = 0;
    int series = -1;
    QChar glyph;                       // background glyph when no dots are set
    QString glyph_color;
};

class Canvas
{
public:
    Canvas(int cols, int rows)
        : cols_(cols)
        , rows_(rows)
        , cells_(cols * rows)
    {
    }

    int width() const { return cols_ * SUB_X; }
    int height() const { return rows_ * SUB_Y; }

    void dot(int px, int py, int series)
    {
        if(px < 0 || py < 0 || px >= width() || py >= height())
            return;

        Cell &cell = cells_[(py / SUB_Y) * cols_ + (px / SUB_X)];
        cell.dots |= bitFor(px % SUB_X, py % SUB_Y);
        cell.series = series;
    }

    // Bresenham, so a trace stays connected however steep the step between
    // two samples is.
    void line(int x0, int y0, int x1, int y1, int series)
    {
        // A segment with both ends off the same edge cannot cross the plot, so
        // it is dropped before it costs anything. Zoomed right in, a value far
        // off screen would otherwise be walked pixel by pixel to get there.
        if((x0 < 0 && x1 < 0) || (x0 >= width() && x1 >= width())
           || (y0 < 0 && y1 < 0) || (y0 >= height() && y1 >= height()))
            return;

        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        // Whatever is left still costs a step per pixel of its bounding box,
        // which a steep segment can make much larger than the plot.
        int guard = qMin((dx - dy) + 4, 4 * (width() + height()));
        while(guard-- > 0)
        {
            dot(x0, y0, series);
            if(x0 == x1 && y0 == y1)
                break;
            const int e2 = 2 * err;
            if(e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }
            if(e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    void background(int cell_x, int cell_y, QChar glyph, const QString &color)
    {
        if(cell_x < 0 || cell_y < 0 || cell_x >= cols_ || cell_y >= rows_)
            return;
        Cell &cell = cells_[cell_y * cols_ + cell_x];
        cell.glyph = glyph;
        cell.glyph_color = color;
    }

    const Cell &at(int cell_x, int cell_y) const { return cells_[cell_y * cols_ + cell_x]; }

private:
    static quint8 bitFor(int x, int y)
    {
        // Braille dot numbering: the first three rows are bits 0-2 in the left
        // column and 3-5 in the right, with the fourth row on bits 6 and 7.
        static const quint8 map[4][2] = {
            {0x01, 0x08},
            {0x02, 0x10},
            {0x04, 0x20},
            {0x40, 0x80},
        };
        return map[y][x];
    }

    int cols_;
    int rows_;
    QVector<Cell> cells_;
};

}

bool PlotView::isDefault() const
{
    return qFuzzyCompare(x_scale, 1.0) && qFuzzyCompare(y_scale, 1.0)
        && qFuzzyIsNull(pan_x) && qFuzzyIsNull(pan_y);
}

void PlotView::reset()
{
    x_scale = 1.0;
    y_scale = 1.0;
    pan_x = 0.0;
    pan_y = 0.0;
}

namespace
{

// Scales about a fixed point: whatever was under the cursor before the zoom is
// still under it afterwards.
void zoomAxis(double &scale, double &pan, double cursor_pos, double anchor, double factor)
{
    const double target = qBound(MIN_SCALE, scale * factor, MAX_SCALE);
    if(qFuzzyCompare(target, scale))
        return;

    const double base = anchor + (cursor_pos - pan - anchor) / scale;
    scale = target;
    pan = cursor_pos - anchor - (base - anchor) * scale;
}

}

void PlotView::zoomTime(double cursor_x, double width, double factor)
{
    zoomAxis(x_scale, pan_x, cursor_x, width - 1, factor);
    clamp(width, width > 0 ? width : 1);
}

void PlotView::zoomValues(double cursor_y, double height, double factor)
{
    zoomAxis(y_scale, pan_y, cursor_y, (height - 1) / 2.0, factor);
    clamp(height > 0 ? height : 1, height);
}

void PlotView::panBy(double dx, double dy, double width, double height)
{
    pan_x += dx;
    pan_y += dy;
    clamp(width, height);
}

// Keeps the traces reachable: the centre of the content stays inside the plot,
// rather than merely some edge of it overlapping. Bounding the edges is not
// enough -- content whose right edge just touches the left of the plot is
// legal by that rule and entirely invisible.
void PlotView::clamp(double width, double height)
{
    if(width <= 0 || height <= 0)
        return;

    pan_x = qBound(width * x_scale / 2.0 - width, pan_x, width * x_scale / 2.0);
    pan_y = qBound(-height / 2.0, pan_y, height / 2.0);
}

PlotGeometry plotGeometry(int rows, int cols)
{
    PlotGeometry geometry;
    geometry.left_cell = LEFT_GUTTER;
    geometry.area_cols = cols - LEFT_GUTTER - RIGHT_GUTTER;
    // One row of time labels and one footer row sit under the frame.
    geometry.area_rows = rows - 2;
    return geometry;
}

PlotOutput renderPlot(int rows, int cols, const PlotInput &input, const PlotView &view)
{
    PlotOutput out;

    const PlotGeometry geometry = plotGeometry(rows, cols);
    if(geometry.area_cols < 16 || geometry.area_rows < 4)
    {
        for(int i = 0; i < rows; i++)
            out.lines << term::Line(cols).color(LABEL_COLOR).text("plot: window too small").str();
        return out;
    }

    Canvas canvas(geometry.area_cols, geometry.area_rows);
    const double width = canvas.width();
    const double height = canvas.height();
    const double right = width - 1;
    const double bottom = height - 1;
    const double y_centre = bottom / 2.0;

    // Capacity, not sample count, sets the time span: a half-full buffer draws
    // on the right half of the plot and fills leftwards, the same as the GUI.
    int capacity = 300;
    int filled = 0;
    for(const PlotSeries &s : input.series)
    {
        if(s.data != nullptr)
        {
            capacity = s.data->capacity();
            filled = qMax(filled, s.data->size());
        }
    }
    const int offset = capacity - filled;

    auto axis_x = [&](double slot) {
        const double base = mapRange(slot, 0, capacity, right - width, right);
        return right + (base - right) * view.x_scale + view.pan_x;
    };
    auto left_axis_y = [&](double v) {
        const double base = mapRange(v, 0, LEFT_AXIS_MAX, bottom, 0);
        return y_centre + (base - y_centre) * view.y_scale + view.pan_y;
    };
    auto left_value_at = [&](double y) {
        const double base = y_centre + (y - view.pan_y - y_centre) / view.y_scale;
        return mapRange(base, bottom, 0, 0, LEFT_AXIS_MAX);
    };
    auto right_axis_y = [&](double v) {
        const double base = mapRange(v, -RIGHT_AXIS_SPAN, RIGHT_AXIS_SPAN, bottom, 0);
        return y_centre + (base - y_centre) * view.y_scale + view.pan_y;
    };
    auto right_value_at = [&](double y) {
        const double base = y_centre + (y - view.pan_y - y_centre) / view.y_scale;
        return mapRange(base, bottom, 0, -RIGHT_AXIS_SPAN, RIGHT_AXIS_SPAN);
    };

    // What the plot edges actually correspond to right now. Every label is
    // derived from these, so the numbers cannot drift from the picture
    // however the view has been zoomed or dragged.
    const double left_lo = left_value_at(bottom);
    const double left_hi = left_value_at(0);
    const double right_lo = right_value_at(bottom);
    const double right_hi = right_value_at(0);

    // --- background: zero line, limits, time ticks ---

    const int zero_row = static_cast<int>(right_axis_y(0)) / SUB_Y;
    for(int x = 0; x < geometry.area_cols; x++)
        canvas.background(x, zero_row, term::unicode ? QChar(0x2500) : QChar('-'), GRID_COLOR);

    for(int limit : {input.up_limit, input.down_limit})
    {
        if(limit == 0)
            continue;
        for(int sign : {1, -1})
        {
            const int row = static_cast<int>(right_axis_y(sign * limit)) / SUB_Y;
            if(row < 0 || row >= geometry.area_rows)
                continue;
            for(int x = 0; x < geometry.area_cols; x += 2)
                canvas.background(x, row, term::unicode ? QChar(0x2504) : QChar('-'), LIMIT_COLOR);
        }
    }

    const int TIME_TICKS = 6;
    QVector<double> tick_slots;
    for(int i = 0; i <= TIME_TICKS; i++)
        tick_slots << static_cast<double>(capacity - 1) * i / TIME_TICKS;

    for(double slot : tick_slots)
    {
        const int cell = static_cast<int>(axis_x(slot)) / SUB_X;
        if(cell <= 0 || cell >= geometry.area_cols)
            continue;
        for(int y = 0; y < geometry.area_rows; y++)
        {
            if(canvas.at(cell, y).glyph.isNull())
                canvas.background(cell, y, term::unicode ? QChar(0x2502) : QChar(':'), GRID_COLOR);
        }
    }

    if(input.cursor_cell >= 0 && input.cursor_cell < geometry.area_cols)
    {
        for(int y = 0; y < geometry.area_rows; y++)
            canvas.background(input.cursor_cell, y, term::unicode ? QChar(0x2506) : QChar('|'), FRAME_COLOR);
    }

    // --- traces ---

    for(int s = 0; s < input.series.size(); s++)
    {
        const PlotSeries &series = input.series[s];
        if(!series.visible || series.data == nullptr || series.data->size() < 2)
            continue;

        int prev_x = 0;
        int prev_y = 0;
        for(int i = 0; i < series.data->size(); i++)
        {
            const double value = series.gain * series.data->at(i);
            const double x = axis_x(offset + i);
            const double y = series.left_axis ? left_axis_y(value) : right_axis_y(value);

            const int px = static_cast<int>(std::lround(x));
            const int py = static_cast<int>(std::lround(y));
            if(i > 0)
                canvas.line(prev_x, prev_y, px, py, s);
            prev_x = px;
            prev_y = py;
        }
    }

    // --- compose the rows ---

    QVector<double> label_rows;
    const int AXIS_TICKS = 5;
    for(int i = 0; i < AXIS_TICKS; i++)
        label_rows << static_cast<double>(i) / (AXIS_TICKS - 1);

    for(int y = 0; y < geometry.area_rows; y++)
    {
        term::Line line(cols);

        // Left axis labels land on the rows nearest the tick values.
        QString left_label;
        QString right_label;
        for(double t : label_rows)
        {
            const double lv = left_lo + (left_hi - left_lo) * t;
            if(static_cast<int>(left_axis_y(lv)) / SUB_Y == y)
                left_label = QString::number(qRound(lv));

            const double rv = right_lo + (right_hi - right_lo) * t;
            if(static_cast<int>(right_axis_y(rv)) / SUB_Y == y)
                right_label = QString::number(qRound(rv));
        }

        line.color(LABEL_COLOR);
        line.text(left_label.rightJustified(LEFT_GUTTER - 1, ' ').right(LEFT_GUTTER - 1));
        line.color(FRAME_COLOR).text(term::unicode ? QString(QChar(0x2502)) : QString("|"));

        int current = -2;
        for(int x = 0; x < geometry.area_cols; x++)
        {
            const Cell &cell = canvas.at(x, y);
            if(cell.dots != 0)
            {
                if(current != cell.series)
                {
                    line.color(input.series[cell.series].color);
                    current = cell.series;
                }
                line.text(term::unicode
                              ? QString(QChar(0x2800 + cell.dots))
                              : QString(QChar(input.series[cell.series].mark)));
            }
            else if(!cell.glyph.isNull())
            {
                line.color(cell.glyph_color);
                current = -2;
                line.text(QString(cell.glyph));
            }
            else
            {
                line.text(" ");
            }
        }

        line.color(FRAME_COLOR).text(term::unicode ? QString(QChar(0x2502)) : QString("|"));
        line.color(LABEL_COLOR).text(" " + right_label);
        out.lines << line.str();
    }

    // --- time labels ---

    QString axis(geometry.area_cols, QChar(' '));
    for(double slot : tick_slots)
    {
        const int cell = static_cast<int>(axis_x(slot)) / SUB_X;
        if(cell < 0 || cell >= geometry.area_cols)
            continue;

        const int index = static_cast<int>(slot) - offset;
        QString text = "-";
        if(input.times != nullptr && filled > 1 && index >= 0 && index < filled)
        {
            const double age = (input.now_ms - input.times->at(index)) / 1000.0;
            text = QString("-%1s").arg(age, 0, 'f', 1);
        }

        int at = qBound(0, cell - text.size() / 2, geometry.area_cols - text.size());
        if(at + text.size() <= geometry.area_cols)
            axis.replace(at, text.size(), text);
    }

    out.lines << term::Line(cols).pad(LEFT_GUTTER).color(LABEL_COLOR).text(axis).str();

    // --- footer ---

    const QString left_note = QString("left: counts %1-%2").arg(qRound(left_lo)).arg(qRound(left_hi));
    const QString right_note = QString("right: %1 to %2 (speed x0.2)").arg(qRound(right_lo)).arg(qRound(right_hi));
    const QString hint = view.isDefault()
        ? QString("+/- zoom time   </> zoom values   h/l pan   0 reset")
        : QString("x%1 y%2  ·  0 resets").arg(view.x_scale, 0, 'f', 1).arg(view.y_scale, 0, 'f', 1);

    term::Line footer(cols);
    footer.pad(LEFT_GUTTER).color(LABEL_COLOR).text(left_note);
    footer.text("   ").text(right_note);
    if(footer.room() > hint.size() + 2)
        footer.color("#5C6570").right(hint + " ");
    out.lines << footer.str();

    // --- crosshair readout ---

    if(input.cursor_cell >= 0 && filled > 0)
    {
        // The sample nearest the cursor column, found the same way the GUI
        // finds it: by distance along the time axis.
        int best = -1;
        double best_dx = 1e9;
        for(int i = 0; i < filled; i++)
        {
            const double dx = std::fabs(axis_x(offset + i) / SUB_X - input.cursor_cell);
            if(dx < best_dx)
            {
                best_dx = dx;
                best = i;
            }
        }

        if(best >= 0)
        {
            if(input.times != nullptr)
            {
                const double age = (input.now_ms - input.times->at(best)) / 1000.0;
                out.readout << QString("t  -%1s").arg(age, 0, 'f', 2);
                out.readout_colors << LABEL_COLOR;
            }
            for(const PlotSeries &series : input.series)
            {
                if(!series.visible || series.data == nullptr || best >= series.data->size())
                    continue;
                out.readout << QString("%1 %2").arg(series.name, QString::number(series.data->at(best)));
                out.readout_colors << series.color;
            }
        }
    }

    while(out.lines.size() < rows)
        out.lines << term::Line(cols).str();
    while(out.lines.size() > rows)
        out.lines.removeLast();

    return out;
}

}
