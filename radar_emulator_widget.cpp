#include "radar_emulator_widget.h"

#include <algorithm>
#include <QImage>
#include <QPainter>
#include <unordered_set>

using std::unordered_set;


namespace detail {

inline size_t FAKE_POINT = radar_emulator_widget::POINT_BY_ID.size() - 3;
inline size_t FAIL_POINT = radar_emulator_widget::POINT_BY_ID.size() - 2;
inline size_t SKIP_POINT = radar_emulator_widget::POINT_BY_ID.size() - 1;

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
    unordered_set<size_t> non_free_points;
    vector<pair<size_t, size_t>> next_departures;
    vector<pair<size_t, deque<size_t>>> next_arrivals;

    auto fix_non_free = [&](size_t id, size_t step) -> void {
        non_free_ids.insert(id);
        non_free_points.insert(step);
    };

    auto make_step_departure = [&](size_t id, size_t step) -> void {
        next_departures.push_back({id, step});
        fix_non_free(id, step);
    };

    auto make_step_arrival = [&](size_t id, deque<size_t>& steps) -> void {
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

    auto compute_queues = [&](size_t id, size_t current_point, size_t step_point,
            taxiway_endpoints_t START = taxiway_endpoints_t::START,
            taxiway_endpoints_t END = taxiway_endpoints_t::END,
            bool last_one = false) -> size_t {

        if (TAXIWAY_ENDPOINTS.count(current_point)) {
            size_t result = detail::SKIP_POINT;
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
        size_t current_point = departure_aircrafts[i].second;

        if (DEPARTURES_TRAJECTORY.count(current_point) || DEPARTURES_VARIADIC_TRAJECTORY.count(current_point)) {
            size_t step_point = DEPARTURES_TRAJECTORY.count(current_point)
                    ? DEPARTURES_TRAJECTORY[current_point]
                    : DEPARTURES_VARIADIC_TRAJECTORY[current_point][rand() % DEPARTURES_VARIADIC_TRAJECTORY[current_point].size()];

            if (non_free_points.count(step_point)) {
                make_step_departure(id, current_point);
            } else {
                size_t result = compute_queues(id, current_point, step_point);
                if (result == detail::FAIL_POINT) {
                    make_step_departure(id, step_point);
                } else {
                    if (result != detail::SKIP_POINT) {
                        make_step_departure(id, result);
                    }
                }
            }
        } else {
            size_t result = compute_queues(id, current_point, current_point, taxiway_endpoints_t::START, taxiway_endpoints_t::END, true);
            if (result != detail::FAIL_POINT && result != detail::SKIP_POINT) {
                make_step_departure(id, result);
            }
        }
    }

    departure_aircrafts.swap(next_departures);
    next_departures.clear();


    for (size_t i = 0; i != arrival_aircrafts.size(); ++i) {
        size_t id = arrival_aircrafts[i].first;
        deque<size_t>& all_steps = arrival_aircrafts[i].second;

        if (arival_waiting_aircrafts.count(id)) {
            next_arrivals.push_back({id, std::move(all_steps)});
            continue;
        }

        size_t current_point = all_steps.front();
        all_steps.pop_front();

        if (!all_steps.empty()) {
            size_t step_point = all_steps.front();

            if (non_free_points.count(step_point)) {
                all_steps.push_front(current_point);
                make_step_arrival(id, all_steps);

            } else {
                size_t result = compute_queues(id, current_point, step_point, taxiway_endpoints_t::IGNORE, taxiway_endpoints_t::START);
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

            vector<size_t> path;
            size_t temp = SPAWNPOINT[id];
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
            for (size_t iter : path) {
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

    vector<size_t> all_points;
    if (helper_position != 0 && helper_position != HELPER_TRAJECTORY.size()) {
        all_points.push_back(HELPER_TRAJECTORY[helper_position]);
        point_t point = POINT_BY_ID[HELPER_TRAJECTORY[helper_position]];
        painter.drawPixmap(scaled_coordinates(point.first, point.second, w, h), yellow_point, whole_picture);
    }

    for (auto [id, point_id] : departure_aircrafts) {
        all_points.push_back(point_id);
        point_t point = POINT_BY_ID[point_id];
        painter.drawPixmap(scaled_coordinates(point.first, point.second, w,  h), green_point, whole_picture);
    }

    QPixmap blue_point(":/src/img/point-sample-blue.png");
    blue_point = blue_point.scaled(w, h);

    for (size_t i = 0; i != arrival_aircrafts.size(); ++i) {
        size_t point_id = arrival_aircrafts[i].second.front();
        all_points.push_back(point_id);
        point_t point = POINT_BY_ID[point_id];
        painter.drawPixmap(scaled_coordinates(point.first, point.second, w,  h), blue_point, whole_picture);
    }

    painting_label->setPixmap(pixmap);
}


void radar_emulator_widget::set_label(QLabel* label) {
    this->painting_label = label;
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
    return QRectF(scale(x - static_cast<qreal>(point_pixel_size) / 2, maximum_w, w),
                  scale(y - static_cast<qreal>(point_pixel_size) / 2, maximum_h, h),
                  w,
                  h);
}

qreal radar_emulator_widget::scale(qreal coord, qreal max_src, qreal max_scaled) {
    return coord / max_src * max_scaled;
}

vector<size_t> radar_emulator_widget::HELPER_TRAJECTORY = {
    detail::FAKE_POINT, 160, 161, 162, 163, 164,
    165, 166, 167, 168, 169, 170,
    171, 172, 173, 174, detail::FAKE_POINT
};


vector<size_t> radar_emulator_widget::SPAWNPOINT = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39
};


unordered_map<size_t, size_t> radar_emulator_widget::DEPARTURES_TRAJECTORY = {
    { 0, 40 },
    { 1, 41 },
    { 42, 43 },
    { 41, 42 },
    { 40, 43 },
    { 43, 44 },
    { 44, 45 },
    { 45, 46 },
    { 47, 42 },
    { 48, 47 },
    { 49, 48 },
    { 50, 48 },
    { 51, 50 },
    { 52, 51 },
    { 53, 51 },
    { 54, 53 },
    { 55, 54 },
    { 56, 55 },
    { 8, 56 },
    { 7, 55 },
    { 6, 54 },
    { 9, 53 },
    { 10, 51 },
    { 4, 52 },
    { 11, 50 },
    { 12, 48 },
    { 2, 49 },
    { 13, 47 },
    { 14, 47 },
    { 5, 51 },
    { 3, 48 },
    { 46, 57 },
    { 20, 58 },
    { 58, 59 },
    { 19, 59 },
    { 18, 59 },
    { 59, 60 },
    { 17, 60 },
    { 60, 61 },
    { 16, 61 },
    { 15, 61 },
    { 61, 62 },
    { 62, 63 },
    { 63, 64 },
    { 64, 65 },
    { 65, 66 },
    { 66, 67 },
    { 67, 68 },
    { 68, 69 },
    { 69, 70 },
    { 28, 71 },
    { 71, 72 },
    { 27, 72 },
    { 72, 73 },
    { 26, 73 },
    { 73, 74 },
    { 25, 74 },
    { 74, 75 },
    { 24, 75 },
    { 23, 75 },
    { 75, 76 },
    { 76, 77 },
    { 22, 77 },
    { 21, 78 },
    { 77, 79 },
    { 78, 79 },
    { 79, 80 },
    { 80, 81 },
    { 81, 82 },
    { 82, 83 },
    { 83, 84 },
    { 84, 85 },
    { 85, 86 },
    { 86, 87 },
    { 87, 88 },
    { 88, 66 },
    { 39, 89 },
    { 89, 90 },
    { 38, 90 },
    { 29, 90 },
    { 90, 91 },
    { 37, 91 },
    { 30, 91 },
    { 91, 92 },
    { 31, 92 },
    { 36, 92 },
    { 92, 93 },
    { 32, 93 },
    { 35, 93 },
    { 93, 94 },
    { 33, 94 },
    { 34, 94 },
    { 94, 95 },
    { 95, 96 },
    { 96, 97 },
    { 97, 98 },
    { 99, 100 },
    { 100, 101 },
    { 101, 102 },
    { 102, 103 },
    { 103, 104 },
    { 104, 105 },
    { 105, 106 },
    { 106, 107 },
    { 107, 108 },
    { 108, 82 },
    { 109, 110 },
    { 110, 111 },
    { 111, 112 },
    { 112, 113 },
    { 113, 114 },
    { 114, 115 },
    { 115, 116 },
    { 116, 117 },
    { 117, 118 },
    { 118, 119 },
    { 119, 120 },
    { 120, 121 },
    { 121, 122 },
    { 122, 123 },

    /* departure from right */
    { 123, 124 },
    { 124, 125 },
    { 125, 126 },
    { 126, 127 },
    { 127, 128 },
    { 128, 129 },
    { 129, 130 },
    { 130, 131 },
    { 131, 132 },
    { 132, 133 },
    { 133, 134 },
    { 134, 135 },
    { 135, 136 },
    { 136, 137 },
    { 137, 138 },
    { 138, 139 },
    { 139, detail::FAKE_POINT },

    /* departure from left */
    { 57, 70 },
    { 70, 140 },
    { 140, 141 },
    { 141, 142 },
    { 142, 143 },
    { 143, 144 },
    { 144, 145 },
    { 145, 146 },
    { 146, 147 },
    { 147, 148 },
    { 148, 149 },
    { 149, 150 },
    { 150, 151 },
    { 151, 152 },
    { 152, 153 },
    { 153, 154 },
    { 154, 155 },
    { 155, detail::FAKE_POINT }



};

unordered_map<size_t, vector<size_t>> radar_emulator_widget::DEPARTURES_VARIADIC_TRAJECTORY = {
    { 98, { 109, 99 } }
};


unordered_map<size_t, pair<size_t, taxiway_endpoints_t>> radar_emulator_widget::TAXIWAY_ENDPOINTS = {
    { 0, {9, taxiway_endpoints_t::START} },
    { 1, {9, taxiway_endpoints_t::START} },
    { 2, {9, taxiway_endpoints_t::START} },
    { 3, {9, taxiway_endpoints_t::START} },
    { 4, {9, taxiway_endpoints_t::START} },
    { 5, {9, taxiway_endpoints_t::START} },
    { 6, {9, taxiway_endpoints_t::START} },
    { 7, {9, taxiway_endpoints_t::START} },
    { 8, {9, taxiway_endpoints_t::START} },
    { 9, {9, taxiway_endpoints_t::START} },
    { 10, {9, taxiway_endpoints_t::START} },
    { 11, {9, taxiway_endpoints_t::START} },
    { 12, {9, taxiway_endpoints_t::START} },
    { 13, {9, taxiway_endpoints_t::START} },
    { 14, {9, taxiway_endpoints_t::START} },
    { 57, {9, taxiway_endpoints_t::END} },
    { 46, {0, taxiway_endpoints_t::START} },
    { detail::FAKE_POINT, {0, taxiway_endpoints_t::END} },
    { 15, {8, taxiway_endpoints_t::START} },
    { 16, {8, taxiway_endpoints_t::START} },
    { 17, {8, taxiway_endpoints_t::START} },
    { 18, {8, taxiway_endpoints_t::START} },
    { 19, {8, taxiway_endpoints_t::START} },
    { 20, {8, taxiway_endpoints_t::START} },
    { 65, {8, taxiway_endpoints_t::END} },
    { 64, {1, taxiway_endpoints_t::START} },
    { 69, {1, taxiway_endpoints_t::END} },
    { 68, {0, taxiway_endpoints_t::START} },
    { 21, {7, taxiway_endpoints_t::START} },
    { 22, {7, taxiway_endpoints_t::START} },
    { 23, {7, taxiway_endpoints_t::START} },
    { 24, {7, taxiway_endpoints_t::START} },
    { 25, {7, taxiway_endpoints_t::START} },
    { 26, {7, taxiway_endpoints_t::START} },
    { 27, {7, taxiway_endpoints_t::START} },
    { 28, {7, taxiway_endpoints_t::START} },
    { 81, {7, taxiway_endpoints_t::END} },
    { 80, {10, taxiway_endpoints_t::START} },
    { 88, {10, taxiway_endpoints_t::END} },
    { 87, {1, taxiway_endpoints_t::START} },
    { 29, {6, taxiway_endpoints_t::START} },
    { 30, {6, taxiway_endpoints_t::START} },
    { 31, {6, taxiway_endpoints_t::START} },
    { 32, {6, taxiway_endpoints_t::START} },
    { 33, {6, taxiway_endpoints_t::START} },
    { 34, {6, taxiway_endpoints_t::START} },
    { 35, {6, taxiway_endpoints_t::START} },
    { 36, {6, taxiway_endpoints_t::START} },
    { 37, {6, taxiway_endpoints_t::START} },
    { 38, {6, taxiway_endpoints_t::START} },
    { 39, {6, taxiway_endpoints_t::START} },
    { 97, {6, taxiway_endpoints_t::END} },
    { 96, {10, taxiway_endpoints_t::START} },
    { 118, {4, taxiway_endpoints_t::START} },
    { 119, {10, taxiway_endpoints_t::END} },
    { 122, {0, taxiway_endpoints_t::START} },
    { 123, {4, taxiway_endpoints_t::END} },
    { 156, {2, taxiway_endpoints_t::END} },
    { 157, {0, taxiway_endpoints_t::START} },
    { 158, {10, taxiway_endpoints_t::END} },
    { 159, {2, taxiway_endpoints_t::START} }
};


vector<point_t> radar_emulator_widget::POINT_BY_ID = {
   {1750, 444},
   {1750, 420},
   {1748, 364},
   {1727, 365},
   {1748, 324},
   {1727, 324},
   {1746, 292},
   {1746, 269},
   {1746, 245},
   {1687, 316},
   {1687, 336},
   {1687, 357},
   {1688, 378},
   {1689, 398},
   {1688, 419},
   {1651, 782},
   {1629, 782},
   {1607, 782},
   {1585, 782},
   {1564, 782},
   {1540, 783},
   {1488, 775},
   {1462, 775},
   {1390, 794},
   {1367, 749},
   {1334, 749},
   {1304, 750},
   {1274, 751},
   {1246, 751},
   {1215, 749},
   {1183, 750},
   {1150, 749},
   {1120, 749},
   {1092, 750},
   {1093, 785},
   {1121, 785},
   {1151, 784},
   {1182, 785},
   {1219, 797},
   {1220, 825},
   {1731, 447},
   {1730, 423},
   {1712, 432},
   {1713, 454},
   {1710, 482},
   {1711, 514},
   {1711, 548},
   {1710, 408},
   {1710, 385},
   {1732, 385},
   {1708, 364},
   {1707, 342},
   {1732, 344},
   {1707, 320},
   {1723, 300},
   {1725, 278},
   {1725, 256},
   {1710, 558},
   {1560, 760},
   {1589, 761},
   {1621, 760},
   {1651, 759},
   {1664, 742},
   {1704, 737},
   {1711, 702},
   {1710, 694},
   {1712, 660},
   {1712, 633},
   {1711, 608},
   {1711, 601},
   {1706, 578},
   {1261, 771},
   {1288, 772},
   {1317, 770},
   {1352, 769},
   {1390, 771},
   {1415, 778},
   {1441, 767},
   {1471, 753},
   {1442, 736},
   {1439, 705},
   {1442, 698},
   {1468, 683},
   {1515, 684},
   {1555, 683},
   {1595, 683},
   {1640, 683},
   {1680, 684},
   {1690, 682},
   {1201, 810},
   {1202, 771},
   {1166, 766},
   {1134, 766},
   {1105, 765},
   {1076, 767},
   {1071, 735},
   {1072, 710},
   {1075, 702},
   {1071, 684},
   {1106, 682},
   {1143, 683},
   {1183, 683},
   {1215, 684},
   {1249, 683},
   {1288, 683},
   {1327, 683},
   {1366, 683},
   {1403, 683},
   {1437, 682},
   {1045, 683},
   {1016, 683},
   {987, 685},
   {953, 684},
   {918, 685},
   {877, 685},
   {838, 687},
   {798, 686},
   {758, 686},
   {723, 684},
   {714, 679},
   {684, 667},
   {671, 640},
   {658, 610},
   {667, 599},
   {691, 581},
   {719, 580},
   {752, 580},
   {786, 579},
   {819, 580},
   {851, 581},
   {898, 580},
   {945, 580},
   {992, 579},
   {1037, 580},
   {1086, 580},
   {1154, 579},
   {1222, 580},
   {1293, 579},
   {1362, 579},
   {1431, 580},
   {1677, 579},
   {1650, 580},
   {1617, 580},
   {1583, 580},
   {1551, 580},
   {1518, 580},
   {1471, 580},
   {1424, 580},
   {1378, 580},
   {1331, 580},
   {1284, 580},
   {1214, 580},
   {1144, 580},
   {1074, 580},
   {1005, 580},
   {936, 580},
   {1354, 593},
   {1360, 599},
   {1451, 680},
   {1461, 683},
   {1756, 454},
   {1730, 455},
   {1719, 477},
   {1720, 505},
   {1720, 532},
   {1720, 560},
   {1720, 587},
   {1721, 614},
   {1721, 643},
   {1722, 671},
   {1723, 699},
   {1721, 728},
   {1703, 748},
   {1679, 755},
   {1658, 765},

    /* detail fake points */
    {radar_emulator_widget::maximum_w * 2 + 1, radar_emulator_widget::maximum_h},
    {radar_emulator_widget::maximum_w * 2 + 2, radar_emulator_widget::maximum_h},
    {radar_emulator_widget::maximum_w * 2 + 3, radar_emulator_widget::maximum_h}
};

