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
using point_t = pair<qreal, qreal>;  // sorry about that


namespace detail {

enum class taxiway_endpoints_t {
    START, END, IGNORE
};

} // namespace detail


class radar_emulator_widget : public QWidget {
    Q_OBJECT

    ~radar_emulator_widget();

public:
    radar_emulator_widget(QWidget* parent = nullptr);
    void paintEvent(QPaintEvent*) override;
    void set_label(QLabel* label);

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
    size_t plane_number{2};
    size_t helper_position{0};
    int helper_position_delta{0};
    vector<pair<size_t, size_t>> departure_aircrafts;
    vector<pair<size_t, deque<size_t>>> arrival_aircrafts;
    unordered_map<size_t, vector<size_t>> arival_waiting_aircrafts;
    vector<queue<size_t>> taxiway_que;

public:
    constexpr static qreal maximum_w{static_cast<qreal>(1920)};
    constexpr static qreal maximum_h{static_cast<qreal>(1080)};
    constexpr static size_t point_pixel_size{21};
    static vector<point_t> POINT_BY_ID;

private:
    constexpr static int STANDART_SPEED = 1'000;
    static vector<size_t> HELPER_TRAJECTORY;
    static unordered_map<size_t, size_t> DEPARTURES_TRAJECTORY;
    static unordered_map<size_t, vector<size_t>> DEPARTURES_VARIADIC_TRAJECTORY;
    static vector<size_t> SPAWNPOINT;
    static unordered_map<size_t, pair<size_t, detail::taxiway_endpoints_t>> TAXIWAY_ENDPOINTS;
};

