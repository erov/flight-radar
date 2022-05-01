#include "radar_emulator_widget.h"

#include <QImage>
#include <QPainter>
#include <iostream>

using std::pair;
using std::map;
using std::vector;
using std::set;
using point = pair<qreal, qreal>;

constexpr inline point FAKE_POINT = point(1920 * 4, 1080 * 4);

radar_emulator_widget::radar_emulator_widget(QWidget* parent)
    : QWidget(parent)
{
    update_timer = new QTimer(this);
    QObject::connect(update_timer, SIGNAL(timeout()), this, SLOT(update_aerodrome()));
    update_timer->start(1'000 / 10);

    taxiway.resize(11);
}

radar_emulator_widget::~radar_emulator_widget() {
    delete update_timer;
}

void radar_emulator_widget::update_aerodrome() {

    assert(planes_amount <= SPAWN.size());

    set<size_t> non_free_ids;
    set<point> non_free_points;

    vector<pair<size_t, point>> next;

    auto make_step = [&](size_t id, point& step) -> void {
        next.push_back({id, step});
        non_free_ids.insert(id);
        non_free_points.insert(step);
    };

    auto in_ith_queue = [&](size_t que_id, size_t plane_id) -> bool {
        queue<size_t> que_copy = taxiway[que_id];
        while (!que_copy.empty()) {
            if (que_copy.front() == plane_id) {
                return true;
            }
            que_copy.pop();
        }
        return false;
    };

    auto compute_queues = [&](size_t id, point& current_point, point& step_point, bool last_one = false) -> bool {
        if (TAXIWAY_END_POINTS.count(current_point)) {
            auto [way_id, point_type] = TAXIWAY_END_POINTS[current_point];
            if (point_type == WAY_TYPE::START && !in_ith_queue(way_id, id)) {
                taxiway[way_id].push(id);
            }
            if (taxiway[way_id].front() == id) {
                if (!last_one) {
                    make_step(id, step_point);
                }
                if (point_type == WAY_TYPE::END) {
                    taxiway[way_id].pop();
                }
            } else {
                make_step(id, current_point);
            }
            return true;
        }
        return false;
    };

    for (size_t i = 0; i != planes_departure.size(); ++i) {
        non_free_points.insert(planes_departure[i].second);
    }

    for (size_t i = 0; i != planes_departure.size(); ++i) {
        size_t id = planes_departure[i].first;
        point current_point = planes_departure[i].second;
        std::cerr << id << ' ' << current_point.first << ' ' << current_point.second << '\n';

        if (DEPARTURES.count(current_point) || DEPARTURES_VARIADIC.count(current_point)) {
            point step_point = DEPARTURES.count(current_point)
                    ? DEPARTURES[current_point]
                    : DEPARTURES_VARIADIC[current_point][rand() % DEPARTURES_VARIADIC[current_point].size()];

            if (non_free_points.count(step_point)) {
                make_step(id, current_point);
            } else {
                if (!compute_queues(id, current_point, step_point)) {
                    make_step(id, step_point);
                }
            }
        } else {
            compute_queues(id, current_point, current_point, true);
        }
    }

    planes_departure.swap(next);
    next.clear();

    for (size_t i = planes_departure.size(); i < planes_amount; ++i) {
        size_t id;
        while (true) {
            id = rand() % SPAWN.size();
            if (!non_free_ids.count(id)) {
                break;
            }
        }
        non_free_ids.insert(id);
        planes_departure.push_back({id, SPAWN[id]});
    }

//    for (size_t i = planes_departure.size() + planes_arrival.size(); i < planes_amount; ++i) {

//    }

    repaint();
}

void radar_emulator_widget::paintEvent(QPaintEvent* event = nullptr) {
    QPixmap pixmap(":/img/img/aerodrom-satellite.png");
    int w = width();
    int h = height();
    pixmap = pixmap.scaled(w, h);

    QPixmap point_sample(":/img/img/point-sample.png");
    point_sample = point_sample.scaled(w, h);

    QPainter painter(&pixmap);
    for (auto [id, point] : planes_departure) {
        painter.drawPixmap(scaled_coordinates(point.first, point.second, w,  h), point_sample, QRectF(0, 0, w, h));

    }

    label->setPixmap(pixmap);
}

void radar_emulator_widget::set_label(QLabel* label) {
    this->label = label;
}

void radar_emulator_widget::set_max_w(qreal w) {
    this->MAX_W = w;
}

void radar_emulator_widget::set_max_h(qreal h) {
    this->MAX_H = h;
}

void radar_emulator_widget::set_point_size(qreal size) {
    this->POINT_SIZE = size;
}

QRectF radar_emulator_widget::scaled_coordinates(qreal x, qreal y, qreal w, qreal h) {
    return QRectF(scale(x - POINT_SIZE / 2, MAX_W, w), scale(y - POINT_SIZE / 2, MAX_H, h), w, h);
}

qreal radar_emulator_widget::scale(qreal coord, qreal max_src, qreal max_scaled) {
    return coord / max_src * max_scaled;
}

map<point, point> radar_emulator_widget::DEPARTURES = {
    { {1750, 444}, {1731, 447} }, /* taxiway-9 */
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
    { {1688, 378}, {1732, 385} },
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
    { {1651, 782}, {1651, 759} }, // taxiway 8
    { {1651, 759}, {1664, 742} },
    { {1664, 742}, {1704, 737} },
    { {1704, 737}, {1711, 702} },
    { {1711, 702}, {1710, 694} }, // /taxiway 1
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
    { {1471, 753}, {1442, 736} }, // taxiway 7
    { {1442, 736}, {1439, 705} },
    { {1439, 705}, {1442, 698} }, // 10
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


    { {667, 599}, {691, 581} }, /* departure from right */
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
    { {1431, 580}, FAKE_POINT },




    { {1710, 558}, {1706, 578} }, /* departure from left */
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
    { {936, 580}, FAKE_POINT }
//    { {936, 580}, {813, 581} },
    //        { {813, 581}, {692, 582} },
    //        { {692, 582}, {571, 582} },
    //        { {571, 582}, {431, 583} },



};

map<point, vector<point>> radar_emulator_widget::DEPARTURES_VARIADIC = {
    { {1071, 684}, { {1045, 683}, {1106, 682} } }
};

vector<point> radar_emulator_widget::SPAWN = {
    {1750, 444},  // 1
    {1750, 420},  // 2
    {1748, 364},  // 3
    {1727, 365},  // 4
    {1748, 324},  // 5
    {1727, 324},  // 6
    {1746, 292},  // 7
    {1746, 269},  // 8
    {1746, 245},  // 9
    {1687, 316},  // 10
    {1687, 336},  // 11
    {1687, 357},  // 12
    {1688, 378},  // 13
    {1689, 398},  // 14
    {1688, 419},  // 15
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

map<point, pair<size_t, WAY_TYPE>> radar_emulator_widget::TAXIWAY_END_POINTS = {

//    { {1713, 454}, {9, WAY_TYPE::START} },
//    { {1711, 548}, {9, WAY_TYPE::END} },
//    { {1710, 558}, {0, WAY_TYPE::START} },
//    { FAKE_POINT, {0, WAY_TYPE::END} },
//    { {1651, 759}, {8, WAY_TYPE::START} },
//    { {1711, 702}, {8, WAY_TYPE::END} },
//    { {1710, 694}, {1, WAY_TYPE::START} },
//    { {1711, 608}, {1, WAY_TYPE::END} },
//    { {1711, 601}, {0, WAY_TYPE::START} }, // end same as 0
//    { {1442, 736}, {7, WAY_TYPE::START} },
//    { {1439, 705}, {7, WAY_TYPE::END} },
//    { {1442, 698}, {10, WAY_TYPE::START} },
//    { {1680, 684}, {10, WAY_TYPE::END} },
//    { {1690, 682}, {1, WAY_TYPE::START} } // end same as 1
    { {1713, 454}, {9, WAY_TYPE::START} },
    { {1710, 558}, {9, WAY_TYPE::END} },
    { {1711, 548}, {0, WAY_TYPE::START} },
    { FAKE_POINT, {0, WAY_TYPE::END} },
    { {1651, 759}, {8, WAY_TYPE::START} },
    { {1710, 694}, {8, WAY_TYPE::END} },
    { {1711, 702}, {1, WAY_TYPE::START} },
    { {1711, 601}, {1, WAY_TYPE::END} },
    { {1711, 608}, {0, WAY_TYPE::START} }, // end same as 0
    { {1442, 736}, {7, WAY_TYPE::START} },
    { {1442, 698}, {7, WAY_TYPE::END} },
    { {1439, 705}, {10, WAY_TYPE::START} },
    { {1690, 682}, {10, WAY_TYPE::END} },
    { {1680, 684}, {1, WAY_TYPE::START} },  // end same as 1
    { {1076, 767}, {6, WAY_TYPE::START} },
    { {1075, 702}, {6, WAY_TYPE::END} },
    { {1072, 710}, {10, WAY_TYPE::START} },
    { {714, 679}, {4, WAY_TYPE::START} },
    { {723, 684}, {10, WAY_TYPE::END} },
    { {658, 610}, {0, WAY_TYPE::START} },
    { {667, 599}, {4, WAY_TYPE::END} }

};
