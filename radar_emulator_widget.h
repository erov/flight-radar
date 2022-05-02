#pragma once

#include <cstddef>
#include <deque>
#include <map>
#include <set>
#include <vector>
#include <QRectF>
#include <QLabel>
#include <QTimer>
#include <queue>
#include <QWidget>
#include <unordered_map>

using std::pair;
using std::map;
using std::unordered_map;
using std::vector;
using std::set;
using std::queue;
using std::deque;
using point = pair<qreal, qreal>;

enum class WAY_TYPE {
    START, END, IGNORE
};

struct point_hasher {
    size_t operator()(const point& p) const;
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

public Q_SLOTS:
    void set_plane_number(int value);
    void set_speed(int boost);

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
    size_t plane_number = 2;
    size_t special_equipment_id = 0;
    int special_equipment_delta = 0;
    vector<pair<size_t, point>> planes_departure;
    vector<pair<size_t, deque<point>>> planes_arrival;
    unordered_map<size_t, vector<size_t>> waiting_arrival;
    vector<queue<size_t>> taxiway;

private:
    static vector<point> SPECIAL_EQUIPMENT;
    static unordered_map<point, point, point_hasher> DEPARTURES;
    static unordered_map<point, vector<point>, point_hasher> DEPARTURES_VARIADIC;
    static vector<point> SPAWN;
    static unordered_map<point, pair<size_t, WAY_TYPE>, point_hasher> TAXIWAY_END_POINTS;
};
