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
using point = pair<qreal, qreal>;  // sorry about that


namespace detail {

enum class taxiway_endpoints_t {
    START, END, IGNORE
};

struct point_hasher {
    size_t operator()(const point& p) const;
};

} // namespace detail


class radar_emulator_widget : public QWidget {
    Q_OBJECT

    ~radar_emulator_widget();

public:
    radar_emulator_widget(QWidget* parent = nullptr);
    void paintEvent(QPaintEvent*) override;
    void set_label(QLabel* label);
    void set_point_size(size_t size);

public Q_SLOTS:
    void set_plane_number(int value);
    void set_speed(int boost);

private:
    QRectF scaled_coordinates(qreal x, qreal y, qreal w, qreal h);
    qreal scale(qreal coord, qreal max_src, qreal max_scaled);

private Q_SLOTS:
    void update_aerodrome();

private:
    QLabel* painting_label;
    QTimer* timer;
    size_t point_pixel_size{21};
    size_t plane_number{2};
    size_t helper_position{0};
    int helper_position_delta{0};
    vector<pair<size_t, point>> departure_aircrafts;
    vector<pair<size_t, deque<point>>> arrival_aircrafts;
    unordered_map<size_t, vector<size_t>> arival_waiting_aircrafts;
    vector<queue<size_t>> taxiway_que;

public:
    constexpr static qreal maximum_w{static_cast<qreal>(1920)};
    constexpr static qreal maximum_h{static_cast<qreal>(1080)};

private:
    constexpr static int STANDART_SPEED = 1'000;
    static vector<point> HELPER_TRAJECTORY;
    static unordered_map<point, point, detail::point_hasher> DEPARTURES_TRAJECTORY;
    static unordered_map<point, vector<point>, detail::point_hasher> DEPARTURES_VARIADIC_TRAJECTORY;
    static vector<point> SPAWNPOINT;
    static unordered_map<point, pair<size_t, detail::taxiway_endpoints_t>, detail::point_hasher> TAXIWAY_ENDPOINTS;
};
