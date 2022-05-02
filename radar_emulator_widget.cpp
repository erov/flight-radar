#include "radar_emulator_widget.h"

#include <algorithm>
#include <QImage>
#include <QPainter>
#include <unordered_set>

using std::unordered_set;


namespace detail {

constexpr inline point FAKE_POINT = point(radar_emulator_widget::maximum_w * 2 + 1, radar_emulator_widget::maximum_h);
constexpr inline point FAIL_POINT = point(radar_emulator_widget::maximum_w * 2 + 2, radar_emulator_widget::maximum_h);
constexpr inline point SKIP_POINT = point(radar_emulator_widget::maximum_w * 2 + 3, radar_emulator_widget::maximum_h);

size_t point_hasher::operator()(const point& p) const {
    return p.first * radar_emulator_widget::maximum_w * 10 + p.second;
}

} // namespace detail


using detail::taxiway_endpoints_t;

radar_emulator_widget::radar_emulator_widget(QWidget* parent)
    : QWidget(parent),
      painting_label(nullptr),
      timer(new QTimer(this)),
      taxiway_que(vector<queue<size_t>>(11))
{
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(update_aerodrome()));
    timer->start(STANDART_SPEED);
}

radar_emulator_widget::~radar_emulator_widget() {
    delete timer;
}


void radar_emulator_widget::update_aerodrome() {

    assert(plane_number <= SPAWNPOINT.size());

    unordered_set<size_t> non_free_ids;
    unordered_set<point, detail::point_hasher> non_free_points;
    vector<pair<size_t, point>> next_departures;
    vector<pair<size_t, deque<point>>> next_arrivals;

    auto fix_non_free = [&](size_t id, point& step) -> void {
        non_free_ids.insert(id);
        non_free_points.insert(step);
    };

    auto make_step_departure = [&](size_t id, point& step) -> void {
        next_departures.push_back({id, step});
        fix_non_free(id, step);
    };

    auto make_step_arrival = [&](size_t id, deque<point>& steps) -> void {
        if (steps.empty()) {
            return;
        }
        fix_non_free(id, steps.front());
        next_arrivals.push_back({id, std::move(steps)});
    };

    auto in_ith_queue = [&](size_t que_id, size_t plane_id) -> bool {
        queue<size_t> que_copy = taxiway_que[que_id];
        while (!que_copy.empty()) {
            if (que_copy.front() == plane_id) {
                return true;
            }
            que_copy.pop();
        }
        return false;
    };

    auto compute_queues = [&](size_t id, point& current_point, point& step_point,
            taxiway_endpoints_t START = taxiway_endpoints_t::START,
            taxiway_endpoints_t END = taxiway_endpoints_t::END,
            bool last_one = false) -> point {

        if (TAXIWAY_ENDPOINTS.count(current_point)) {
            point result = detail::SKIP_POINT;
            auto [way_id, point_type] = TAXIWAY_ENDPOINTS[current_point];

            if (point_type == START && !in_ith_queue(way_id, id)) {
                taxiway_que[way_id].push(id);
            }

            if (taxiway_que[way_id].front() == id) {
                if (!last_one) {
                    result = step_point;
                }
                if (point_type == END) {
                    taxiway_que[way_id].pop();
                }
            } else {
                result = current_point;
            }
            return result;
        }
        return detail::FAIL_POINT;
    };



    helper_position += helper_position_delta;
    if (helper_position == 0 || helper_position == HELPER_TRAJECTORY.size()) {
        helper_position_delta = 0;
    }

    // helper starts moving with 0.04 frequency
    if (helper_position_delta == 0 && rand() % 25 == 0) {
        helper_position_delta = (helper_position == 0 ? 1 : -1);
    }


    for (size_t i = 0; i != departure_aircrafts.size(); ++i) {
        size_t id = departure_aircrafts[i].first;
        point current_point = departure_aircrafts[i].second;

        if (DEPARTURES_TRAJECTORY.count(current_point) || DEPARTURES_VARIADIC_TRAJECTORY.count(current_point)) {
            point step_point = DEPARTURES_TRAJECTORY.count(current_point)
                    ? DEPARTURES_TRAJECTORY[current_point]
                    : DEPARTURES_VARIADIC_TRAJECTORY[current_point][rand() % DEPARTURES_VARIADIC_TRAJECTORY[current_point].size()];

            if (non_free_points.count(step_point)) {
                make_step_departure(id, current_point);
            } else {
                point result = compute_queues(id, current_point, step_point);
                if (result == detail::FAIL_POINT) {
                    make_step_departure(id, step_point);
                } else {
                    if (result != detail::SKIP_POINT) {
                        make_step_departure(id, result);
                    }
                }
            }
        } else {
            point result = compute_queues(id, current_point, current_point, taxiway_endpoints_t::START, taxiway_endpoints_t::END, true);
            if (result != detail::FAIL_POINT && result != detail::SKIP_POINT) {
                make_step_departure(id, result);
            }
        }
    }

    departure_aircrafts.swap(next_departures);
    next_departures.clear();


    for (size_t i = 0; i != arrival_aircrafts.size(); ++i) {
        size_t id = arrival_aircrafts[i].first;
        deque<point>& all_steps = arrival_aircrafts[i].second;

        if (arival_waiting_aircrafts.count(id)) {
            next_arrivals.push_back({id, std::move(all_steps)});
            continue;
        }

        point current_point = all_steps.front();
        all_steps.pop_front();

        if (!all_steps.empty()) {
            point step_point = all_steps.front();

            if (non_free_points.count(step_point)) {
                all_steps.push_front(current_point);
                make_step_arrival(id, all_steps);

            } else {
                point result = compute_queues(id, current_point, step_point, taxiway_endpoints_t::IGNORE, taxiway_endpoints_t::START);
                if (result == detail::FAIL_POINT) {
                    make_step_arrival(id, all_steps);
                } else {
                    if (result != detail::SKIP_POINT) {
                        if (result == current_point) {
                            all_steps.push_front(current_point);
                        }
                        make_step_arrival(id, all_steps);
                    }
                }
            }
        } else {
            compute_queues(id, current_point, current_point, taxiway_endpoints_t::IGNORE, taxiway_endpoints_t::START, true);
        }
    }

    arrival_aircrafts.swap(next_arrivals);
    next_arrivals.clear();


    if (arival_waiting_aircrafts.empty()) {
        for (size_t i = departure_aircrafts.size() + arrival_aircrafts.size(); i < plane_number; ++i) {
            size_t id;
            for (;;) {
                id = rand() % SPAWNPOINT.size();
                if (!non_free_ids.count(id)) {
                    break;
                }
            }

            non_free_ids.insert(id);
            // flight is departure with 0.66 frequency
            if (rand() % 3 != 0) {
                departure_aircrafts.push_back({id, SPAWNPOINT[id]});
                continue;
            }

            vector<point> path;
            point temp = SPAWNPOINT[id];
            vector<size_t> chosen;
            while (temp != detail::FAKE_POINT) {
                path.push_back(temp);
                if (DEPARTURES_TRAJECTORY.count(temp)) {
                    temp = DEPARTURES_TRAJECTORY[temp];
                } else {
                    chosen.push_back(rand() % DEPARTURES_VARIADIC_TRAJECTORY[temp].size());
                    temp = DEPARTURES_VARIADIC_TRAJECTORY[temp][chosen.back()];
                }
            }
            path.push_back(detail::FAKE_POINT);
            reverse(path.begin(), path.end());

            arrival_aircrafts.push_back({id, {}});
            for (point iter : path) {
                arrival_aircrafts.back().second.push_back(iter);
            }

            if (id < 15) {
                arival_waiting_aircrafts[id] = {0, 9};
            } else if (id < 21) {
                arival_waiting_aircrafts[id] = {8, 1, 0};
            } else if (id < 29) {
                arival_waiting_aircrafts[id] = {7, 10, 1, 0};
            } else if (id < 40) {
                if (!chosen.empty() && chosen[0] == 0) {
                    arival_waiting_aircrafts[id] = {6, 10, 4, 0};
                } else {
                    arival_waiting_aircrafts[id] = {6, 10, 1, 0};
                }
            }
        }
    }

    vector<size_t> can_arrive;
    for (auto& [id, taxiways_needed] : arival_waiting_aircrafts) {
        bool ok = true;
        for (size_t i : taxiways_needed) {
            ok &= taxiway_que[i].empty();
        }
        if (ok) {
            can_arrive.push_back(id);
            for (size_t i : taxiways_needed) {
                taxiway_que[i].push(id);
            }
            break;
        }
    }

    for (size_t i : can_arrive) {
        arival_waiting_aircrafts.erase(i);
    }

    repaint();
}


void radar_emulator_widget::paintEvent(QPaintEvent* event = nullptr) {
    QPixmap pixmap(":/src/img/aerodrom-satellite.png");
    int w = width();
    int h = height();
    pixmap = pixmap.scaled(w, h);

    QPainter painter(&pixmap);
    const QRectF whole_picture = QRectF(0, 0, w, h);

    QPixmap green_point(":/src/img/point-sample.png");
    green_point = green_point.scaled(w, h);

    QPixmap yellow_point(":/src/img/yellow-point.png");
    yellow_point = yellow_point.scaled(w, h);

    if (HELPER_TRAJECTORY[helper_position] != detail::FAKE_POINT) {
        painter.drawPixmap(scaled_coordinates(
                               HELPER_TRAJECTORY[helper_position].first,
                               HELPER_TRAJECTORY[helper_position].second,
                               w,
                               h),
                           yellow_point,
                           whole_picture);
    }

    for (auto [id, point] : departure_aircrafts) {
        painter.drawPixmap(scaled_coordinates(point.first, point.second, w,  h), green_point, whole_picture);
    }

    QPixmap blue_point(":/src/img/point-sample-blue.png");
    blue_point = blue_point.scaled(w, h);

    for (size_t i = 0; i != arrival_aircrafts.size(); ++i) {
        deque<point>& points = arrival_aircrafts[i].second;
        point point = points.front();
        painter.drawPixmap(scaled_coordinates(point.first, point.second, w,  h), blue_point, whole_picture);
    }

    painting_label->setPixmap(pixmap);
}


void radar_emulator_widget::set_label(QLabel* label) {
    this->painting_label = label;
}

void radar_emulator_widget::set_point_size(size_t size) {
    this->point_pixel_size = size;
}


void radar_emulator_widget::set_plane_number(int value) {
    this->plane_number = static_cast<size_t>(value);
    update();
}

void radar_emulator_widget::set_speed(int boost) {
    this->timer->setInterval(STANDART_SPEED / boost);
    update();
}


QRectF radar_emulator_widget::scaled_coordinates(qreal x, qreal y, qreal w, qreal h) {
    return QRectF(scale(x - static_cast<qreal>(point_pixel_size) / 2, maximum_w, w), scale(y - static_cast<qreal>(point_pixel_size) / 2, maximum_h, h), w, h);
}

qreal radar_emulator_widget::scale(qreal coord, qreal max_src, qreal max_scaled) {
    return coord / max_src * max_scaled;
}


vector<point> radar_emulator_widget::HELPER_TRAJECTORY = {
    detail::FAKE_POINT, {1756, 454}, {1730, 455}, {1719, 477}, {1720, 505}, {1720, 532},
    {1720, 560}, {1720, 587}, {1721, 614}, {1721, 643}, {1722, 671}, {1723, 699},
    {1721, 728}, {1703, 748}, {1679, 755}, {1658, 765}, detail::FAKE_POINT
};

// that's taugh
unordered_map<point, point, detail::point_hasher> radar_emulator_widget::DEPARTURES_TRAJECTORY = {
    { {1750, 444}, {1731, 447} },
    { {1750, 420}, {1730, 423} },
    { {1712, 432}, {1713, 454} },
    { {1730, 423}, {1712, 432} },
    { {1731, 447}, {1713, 454} },
    { {1713, 454}, {1710, 482} },
    { {1710, 482}, {1711, 514} },
    { {1711, 514}, {1711, 548} },
    { {1710, 408}, {1712, 432} },
    { {1710, 385}, {1710, 408} },
    { {1732, 385}, {1710, 385} },
    { {1708, 364}, {1710, 385} },
    { {1707, 342}, {1708, 364} },
    { {1732, 344}, {1707, 342} },
    { {1707, 320}, {1707, 342} },
    { {1723, 300}, {1707, 320} },
    { {1725, 278}, {1723, 300} },
    { {1725, 256}, {1725, 278} },
    { {1746, 245}, {1725, 256} },
    { {1746, 269}, {1725, 278} },
    { {1746, 292}, {1723, 300} },
    { {1687, 316}, {1707, 320} },
    { {1687, 336}, {1707, 342} },
    { {1748, 324}, {1732, 344} },
    { {1687, 357}, {1708, 364} },
    { {1688, 378}, {1710, 385} },
    { {1748, 364}, {1732, 385} },
    { {1689, 398}, {1710, 408} },
    { {1688, 419}, {1710, 408} },
    { {1727, 324}, {1707, 342} },
    { {1727, 365}, {1710, 385} },
    { {1711, 548}, {1710, 558} },
    { {1540, 783}, {1560, 760} },
    { {1560, 760}, {1589, 761} },
    { {1564, 782}, {1589, 761} },
    { {1585, 782}, {1589, 761} },
    { {1589, 761}, {1621, 760} },
    { {1607, 782}, {1621, 760} },
    { {1621, 760}, {1651, 759} },
    { {1629, 782}, {1651, 759} },
    { {1651, 782}, {1651, 759} },
    { {1651, 759}, {1664, 742} },
    { {1664, 742}, {1704, 737} },
    { {1704, 737}, {1711, 702} },
    { {1711, 702}, {1710, 694} },
    { {1710, 694}, {1712, 660} },
    { {1712, 660}, {1712, 633} },
    { {1712, 633}, {1711, 608} },
    { {1711, 608}, {1711, 601} },
    { {1711, 601}, {1706, 578} },
    { {1246, 751}, {1261, 771} },
    { {1261, 771}, {1288, 772} },
    { {1274, 751}, {1288, 772} },
    { {1288, 772}, {1317, 770} },
    { {1304, 750}, {1317, 770} },
    { {1317, 770}, {1352, 769} },
    { {1334, 749}, {1352, 769} },
    { {1352, 769}, {1390, 771} },
    { {1367, 749}, {1390, 771} },
    { {1390, 794}, {1390, 771} },
    { {1390, 771}, {1415, 778} },
    { {1415, 778}, {1441, 767} },
    { {1462, 775}, {1441, 767} },
    { {1488, 775}, {1471, 753} },
    { {1441, 767}, {1442, 736} },
    { {1471, 753}, {1442, 736} },
    { {1442, 736}, {1439, 705} },
    { {1439, 705}, {1442, 698} },
    { {1442, 698}, {1468, 683} },
    { {1468, 683}, {1515, 684} },
    { {1515, 684}, {1555, 683} },
    { {1555, 683}, {1595, 683} },
    { {1595, 683}, {1640, 683} },
    { {1640, 683}, {1680, 684} },
    { {1680, 684}, {1690, 682} },
    { {1690, 682}, {1712, 660} },
    { {1220, 825}, {1201, 810} },
    { {1201, 810}, {1202, 771} },
    { {1219, 797}, {1202, 771} },
    { {1215, 749}, {1202, 771} },
    { {1202, 771}, {1166, 766} },
    { {1182, 785}, {1166, 766} },
    { {1183, 750}, {1166, 766} },
    { {1166, 766}, {1134, 766} },
    { {1150, 749}, {1134, 766} },
    { {1151, 784}, {1134, 766} },
    { {1134, 766}, {1105, 765} },
    { {1120, 749}, {1105, 765} },
    { {1121, 785}, {1105, 765} },
    { {1105, 765}, {1076, 767} },
    { {1092, 750}, {1076, 767} },
    { {1093, 785}, {1076, 767} },
    { {1076, 767}, {1071, 735} },
    { {1071, 735}, {1072, 710} },
    { {1072, 710}, {1075, 702} },
    { {1075, 702}, {1071, 684} },
    { {1106, 682}, {1143, 683} },
    { {1143, 683}, {1183, 683} },
    { {1183, 683}, {1215, 684} },
    { {1215, 684}, {1249, 683} },
    { {1249, 683}, {1288, 683} },
    { {1288, 683}, {1327, 683} },
    { {1327, 683}, {1366, 683} },
    { {1366, 683}, {1403, 683} },
    { {1403, 683}, {1437, 682} },
    { {1437, 682}, {1468, 683} },
    { {1045, 683}, {1016, 683} },
    { {1016, 683}, {987, 685} },
    { {987, 685}, {953, 684} },
    { {953, 684}, {918, 685} },
    { {918, 685}, {877, 685} },
    { {877, 685}, {838, 687} },
    { {838, 687}, {798, 686} },
    { {798, 686}, {758, 686} },
    { {758, 686}, {723, 684} },
    { {723, 684}, {714, 679} },
    { {714, 679}, {684, 667} },
    { {684, 667}, {671, 640} },
    { {671, 640}, {658, 610} },
    { {658, 610}, {667, 599} },

    /* departure from right */
    { {667, 599}, {691, 581} },
    { {691, 581}, {719, 580} },
    { {719, 580}, {752, 580} },
    { {752, 580}, {786, 579} },
    { {786, 579}, {819, 580} },
    { {819, 580}, {851, 581} },
    { {851, 581}, {898, 580} },
    { {898, 580}, {945, 580} },
    { {945, 580}, {992, 579} },
    { {992, 579}, {1037, 580} },
    { {1037, 580}, {1086, 580} },
    { {1086, 580}, {1154, 579} },
    { {1154, 579}, {1222, 580} },
    { {1222, 580}, {1293, 579} },
    { {1293, 579}, {1362, 579} },
    { {1362, 579}, {1431, 580} },
    { {1431, 580}, detail::FAKE_POINT },

    /* departure from left */
    { {1710, 558}, {1706, 578} },
    { {1706, 578}, {1677, 579} },
    { {1677, 579}, {1650, 580} },
    { {1650, 580}, {1617, 580} },
    { {1617, 580}, {1583, 580} },
    { {1583, 580}, {1551, 580} },
    { {1551, 580}, {1518, 580} },
    { {1518, 580}, {1471, 580} },
    { {1471, 580}, {1424, 580} },
    { {1424, 580}, {1378, 580} },
    { {1378, 580}, {1331, 580} },
    { {1331, 580}, {1248, 580} },
    { {1248, 580}, {1214, 580} },
    { {1214, 580}, {1144, 580} },
    { {1144, 580}, {1074, 580} },
    { {1074, 580}, {1005, 580} },
    { {1005, 580}, {936, 580} },
    { {936, 580}, detail::FAKE_POINT }



};

unordered_map<point, vector<point>, detail::point_hasher> radar_emulator_widget::DEPARTURES_VARIADIC_TRAJECTORY = {
    { {1071, 684}, { {1045, 683}, {1106, 682} } }
};

vector<point> radar_emulator_widget::SPAWNPOINT = {
    {1750, 444}, // 1
    {1750, 420}, // 2
    {1748, 364}, // 3
    {1727, 365}, // 4
    {1748, 324}, // 5
    {1727, 324}, // 6
    {1746, 292}, // 7
    {1746, 269}, // 8
    {1746, 245}, // 9
    {1687, 316}, // 10
    {1687, 336}, // 11
    {1687, 357}, // 12
    {1688, 378}, // 13
    {1689, 398}, // 14
    {1688, 419}, // 15
    {1651, 782}, // 16
    {1629, 782}, // 17
    {1607, 782}, // 18
    {1585, 782}, // 19
    {1564, 782}, // 20
    {1540, 783}, // 21
    {1488, 775}, // 22
    {1462, 775}, // 23
    {1390, 794}, // 24
    {1367, 749}, // 25
    {1334, 749}, // 26
    {1304, 750}, // 27
    {1274, 751}, // 28
    {1246, 751}, // 29
    {1215, 749}, // 30
    {1183, 750}, // 31
    {1150, 749}, // 32
    {1120, 749}, // 33
    {1092, 750}, // 34
    {1093, 785}, // 35
    {1121, 785}, // 36
    {1151, 784}, // 37
    {1182, 785}, // 38
    {1219, 797}, // 39
    {1220, 825}, // 40

};

unordered_map<point, pair<size_t, taxiway_endpoints_t>, detail::point_hasher> radar_emulator_widget::TAXIWAY_ENDPOINTS = {
    { {1750, 444}, {9, taxiway_endpoints_t::START} }, // 1
    { {1750, 420}, {9, taxiway_endpoints_t::START} }, // 2
    { {1748, 364}, {9, taxiway_endpoints_t::START} }, // 3
    { {1727, 365}, {9, taxiway_endpoints_t::START} }, // 4
    { {1748, 324}, {9, taxiway_endpoints_t::START} }, // 5
    { {1727, 324}, {9, taxiway_endpoints_t::START} }, // 6
    { {1746, 292}, {9, taxiway_endpoints_t::START} }, // 7
    { {1746, 269}, {9, taxiway_endpoints_t::START} }, // 8
    { {1746, 245}, {9, taxiway_endpoints_t::START} }, // 9
    { {1687, 316}, {9, taxiway_endpoints_t::START} }, // 10
    { {1687, 336}, {9, taxiway_endpoints_t::START} }, // 11
    { {1687, 357}, {9, taxiway_endpoints_t::START} }, // 12
    { {1688, 378}, {9, taxiway_endpoints_t::START} }, // 13
    { {1689, 398}, {9, taxiway_endpoints_t::START} }, // 14
    { {1688, 419}, {9, taxiway_endpoints_t::START} }, // 15
    { {1710, 558}, {9, taxiway_endpoints_t::END} },
    { {1711, 548}, {0, taxiway_endpoints_t::START} },
    { detail::FAKE_POINT, {0, taxiway_endpoints_t::END} },
    { {1651, 782}, {8, taxiway_endpoints_t::START} }, // 16
    { {1629, 782}, {8, taxiway_endpoints_t::START} }, // 17
    { {1607, 782}, {8, taxiway_endpoints_t::START} }, // 18
    { {1585, 782}, {8, taxiway_endpoints_t::START} }, // 19
    { {1564, 782}, {8, taxiway_endpoints_t::START} }, // 20
    { {1540, 783}, {8, taxiway_endpoints_t::START} }, // 21
    { {1710, 694}, {8, taxiway_endpoints_t::END} },
    { {1711, 702}, {1, taxiway_endpoints_t::START} },
    { {1711, 601}, {1, taxiway_endpoints_t::END} },
    { {1711, 608}, {0, taxiway_endpoints_t::START} },
    { {1488, 775}, {7, taxiway_endpoints_t::START} }, // 22
    { {1462, 775}, {7, taxiway_endpoints_t::START} }, // 23
    { {1390, 794}, {7, taxiway_endpoints_t::START} }, // 24
    { {1367, 749}, {7, taxiway_endpoints_t::START} }, // 25
    { {1334, 749}, {7, taxiway_endpoints_t::START} }, // 26
    { {1304, 750}, {7, taxiway_endpoints_t::START} }, // 27
    { {1274, 751}, {7, taxiway_endpoints_t::START} }, // 28
    { {1246, 751}, {7, taxiway_endpoints_t::START} }, // 29
    { {1442, 698}, {7, taxiway_endpoints_t::END} },
    { {1439, 705}, {10, taxiway_endpoints_t::START} },
    { {1690, 682}, {10, taxiway_endpoints_t::END} },
    { {1680, 684}, {1, taxiway_endpoints_t::START} },
    { {1215, 749}, {6, taxiway_endpoints_t::START} }, // 30
    { {1183, 750}, {6, taxiway_endpoints_t::START} }, // 31
    { {1150, 749}, {6, taxiway_endpoints_t::START} }, // 32
    { {1120, 749}, {6, taxiway_endpoints_t::START} }, // 33
    { {1092, 750}, {6, taxiway_endpoints_t::START} }, // 34
    { {1093, 785}, {6, taxiway_endpoints_t::START} }, // 35
    { {1121, 785}, {6, taxiway_endpoints_t::START} }, // 36
    { {1151, 784}, {6, taxiway_endpoints_t::START} }, // 37
    { {1182, 785}, {6, taxiway_endpoints_t::START} }, // 38
    { {1219, 797}, {6, taxiway_endpoints_t::START} }, // 39
    { {1220, 825}, {6, taxiway_endpoints_t::START} }, // 40
    { {1075, 702}, {6, taxiway_endpoints_t::END} },
    { {1072, 710}, {10, taxiway_endpoints_t::START} },
    { {723, 684}, {4, taxiway_endpoints_t::START} },
    { {714, 679}, {10, taxiway_endpoints_t::END} },
    { {658, 610}, {0, taxiway_endpoints_t::START} },
    { {667, 599}, {4, taxiway_endpoints_t::END} },
    { {1354, 593}, {2, taxiway_endpoints_t::END} },
    { {1360, 599}, {0, taxiway_endpoints_t::START} },
    { {1451, 680}, {10, taxiway_endpoints_t::END} },
    { {1461, 683}, {2, taxiway_endpoints_t::START} }
};
