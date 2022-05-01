#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <vector>
#include <QRectF>
#include <QLabel>
#include <QTimer>
#include <queue>
#include <QWidget>

using std::pair;
using std::map;
using std::vector;
using std::set;
using std::queue;
using point = pair<qreal, qreal>;

enum class WAY_TYPE {
    START, END
};

class radar_emulator_widget : public QWidget {
    Q_OBJECT

    ~radar_emulator_widget();

public:
    radar_emulator_widget(QWidget* parent = nullptr);
    void paintEvent(QPaintEvent*) override;
    void set_label(QLabel* label);
    void set_max_w(qreal w);
    void set_max_h(qreal h);
    void set_point_size(qreal size);

private:
    QRectF scaled_coordinates(qreal x, qreal y, qreal w, qreal h);
    qreal scale(qreal coord, qreal max_src, qreal max_scaled);

private Q_SLOTS:
    void update_aerodrome();

private:
    QLabel* label;
    QTimer* update_timer;
    qreal MAX_W;
    qreal MAX_H;
    qreal POINT_SIZE;
    size_t planes_amount = 5;
    vector<pair<size_t, point>> planes_departure;
    vector<pair<size_t, point>> planes_arrival;
    vector<queue<size_t>> taxiway;

private:
    static map<point, point> DEPARTURES;
    static map<point, vector<point>> DEPARTURES_VARIADIC;
    static vector<point> SPAWN;
    static map<point, pair<size_t, WAY_TYPE>> TAXIWAY_END_POINTS;
};
