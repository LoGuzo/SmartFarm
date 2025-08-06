#include "drawMap.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QGraphicsPathItem>
#include <QFont>
#include <QPen>
#include <QBrush>
#include <QPainterPath>
#include <QtDebug>
#include <algorithm>
#include <limits>

#define NOMINMAX
#undef min
#undef max
constexpr double sceneW = 910, sceneH = 450;
constexpr double mapW = 700, mapH = 400;
constexpr double mapInnerPad = 50;
constexpr double mapOriginX = sceneW - mapW;  // 210
constexpr double mapOriginY = 0;

// ------------------ 클래스별 색상 테이블 ------------------
static QMap<QString, QColor> classColor = {
    { "Angular Leafspot",      QColor(85, 107, 47) },    // 진녹/올리브
    { "Anthracnose Fruit Rot", QColor(230, 126, 34) },   // 오렌지
    { "Blossom Blight",        QColor(238, 130, 238) },  // 연분홍
    { "Gray Mold",             QColor(128, 128, 128) },  // 회색
    { "Leaf Spot",             QColor(139, 69, 19) },    // 진갈색
    { "Powdery Mildew Fruit",  QColor(203, 195, 227) },  // 연보라
    { "Powdery Mildew Leaf",   QColor(144, 238, 144) },  // 연녹색
    { "ripe",                  QColor(220, 30, 70) },    // 진빨강
};

DrawMap::DrawMap(QGraphicsView *view_, QObject *parent)
    : QObject(parent), view(view_)
{
    scene = new QGraphicsScene(this);
    scene->setSceneRect(0, 0, 910, 450);
    scene->setBackgroundBrush(QBrush(QColor("#0c1919"))); // 어두운 남색 톤
    if (view)
        view->setScene(scene);
}

void DrawMap::feedLine(const QByteArray &lineRaw) {
    QString line = QString::fromUtf8(lineRaw).trimmed();
    qDebug() << "[Debug]" << line;
    if (line == "SPLIT") {
        section = BoundsSection;
        boundsData.clear();
        return;
    }
    if (line == "END") {
        // 큐에 등록: 데이터가 있을 때만 각 함수 실행
        QMetaObject::invokeMethod(this, [this] {
            if (!mapData.isEmpty())
                drawMap();
            if (!boundsData.isEmpty())
                drawBounds();
        }, Qt::QueuedConnection);
        section = None;
        return;
    }
    if (line.isEmpty()) return;

    // 섹션 초기값이 설정되어 있지 않으면 MAP 데이터로 간주
    if (section != BoundsSection)
        section = MapSection;

    if (section == MapSection) processMapLine(line.split('|'));
    else if (section == BoundsSection) processBoundLine(line.split('|'));
}


bool DrawMap::receiveMapData(SSL *ssl)
{
    if (!ssl) {
        qDebug() << "[DrawMap] SSL connection is null";
        return false;
    }

    mapData.clear();
    boundsData.clear();
    section = MapSection;

    QString recvBuf;
    char buffer[1024];
    bool finished = false;

    while (!finished) {
        int n = SSL_read(ssl, buffer, sizeof(buffer) - 1);
        if (n <= 0) {
            // 서버 종료 또는 오류
            break;
        }

        buffer[n] = '\0';
        recvBuf += QString::fromUtf8(buffer);

        int idx;
        // 한 번에 여러 줄이 들어올 수 있으니, '\n' 단위로 처리
        while ((idx = recvBuf.indexOf('\n')) >= 0) {
            QString line = recvBuf.left(idx).trimmed();
            recvBuf = recvBuf.mid(idx + 1);

            feedLine(line.toUtf8());

            if (line == QLatin1String("END")) {
                finished = true;
                break;  // 내부 while 탈출
            }
        }
    }

    // 남은 줄(END 이후 잔여 데이터)이 있을 경우 처리
    if (!recvBuf.trimmed().isEmpty() && !finished) {
        feedLine(recvBuf.toUtf8());
    }

    section = None;
    return !mapData.isEmpty();
}


void DrawMap::processMapLine(const QStringList &p) {
    if (p.size() != 7 && p.size() != 8) return;
    bool ok;
    MapPoint pt;
    pt.ts = p[0].toLongLong(&ok);  if (!ok) return;
    pt.cx = p[1].toDouble(&ok);    if (!ok) return;
    pt.cy = p[2].toDouble(&ok);    if (!ok) return;
    pt.rx = p[3].toDouble(&ok);    if (!ok) return;
    pt.ry = p[4].toDouble(&ok);    if (!ok) return;
    pt.lx = p[5].toDouble(&ok);    if (!ok) return;
    pt.ly = p[6].toDouble(&ok);    if (!ok) return;
    pt.createdAt = (p.size() == 8) ? p[7] : QString();

    mapData << pt;
}



void DrawMap::processBoundLine(const QStringList &p) {
    if (p.size() != 3) return;
    bool ok;
    BoundEvt e;
    e.ts = p[0].toLongLong(&ok); if (!ok) return;
    e.cls = p[1];
    e.feat = p[2];
    boundsData << e;
}

QPointF DrawMap::scale(double x, double y) const {
    double inW = maxX - minX;
    double inH = maxY - minY;
    double effW = mapW - 2 * mapInnerPad;
    double effH = mapH - 2 * mapInnerPad;

    // X, Y축 → 둘 중 더 작은 배율(scaleFactor)만큼만 스케일
    double scaleX = effW / inW;
    double scaleY = effH / inH;
    double scaleFactor = std::min(scaleX, scaleY);

    double sx = (x - minX) * scaleFactor;
    double sy = (y - minY) * scaleFactor;

    // 나머지 여백을 중앙정렬 (좌우/위아래 패딩)
    double leftoverW = effW - inW * scaleFactor;
    double leftoverH = effH - inH * scaleFactor;
    sx += mapOriginX + mapInnerPad + leftoverW / 2.0;
    sy += mapOriginY + mapInnerPad + leftoverH / 2.0;

    return QPointF(sx, sy);

}

void DrawMap::drawBounds() {
    if (boundsData.isEmpty() || mapData.size() < 2) return;

    for (int bi = 1; bi < boundsData.size(); ++bi) {
        const auto &evt = boundsData[bi];
        if (!classColor.contains(evt.cls))
            continue;
        int i = 0;
        while (i < mapData.size() - 1 && evt.ts > mapData[i + 1].ts)
            ++i;
        if (i >= mapData.size() - 1) continue;

        const MapPoint &a = mapData[i];
        const MapPoint &b = mapData[i + 1];
        double ratio = (b.ts != a.ts) ? (evt.ts - a.ts) / double(b.ts - a.ts) : 0.0;

        double fx = 0, fy = 0;
        if (evt.feat == "0") { // 좌: 두 지점의 lx,ly를 보간
            fx = a.lx + (b.lx - a.lx) * ratio;
            fy = a.ly + (b.ly - a.ly) * ratio;
        } else if (evt.feat == "1") { // 우: 두 지점의 rx,ry를 보간
            fx = a.rx + (b.rx - a.rx) * ratio;
            fy = a.ry + (b.ry - a.ry) * ratio;
        } else {
            continue; // 혹시 feat 입력 잘못됐을 때 건너뜀
        }

        QPointF c = scale(fx, fy);
        QColor col = classColor.value(evt.cls, Qt::black);
        scene->addEllipse(c.x() - 7, c.y() - 7, 14, 14, QPen(Qt::black, 2), QBrush(col));
    }
}

void DrawMap::drawMap() {
    scene->clear();
    if (mapData.isEmpty()) return;

    // --- 좌표 범위 ---
    minX = minY = std::numeric_limits<double>::max();
    maxX = maxY = -std::numeric_limits<double>::max();
    for (const auto &pt : mapData) {
        // x좌표 최대/최소
        minX = std::min({minX, pt.cx, pt.lx, pt.rx});
        maxX = std::max({maxX, pt.cx, pt.lx, pt.rx});
        // y좌표 최대/최소
        minY = std::min({minY, pt.cy, pt.ly, pt.ry});
        maxY = std::max({maxY, pt.cy, pt.ly, pt.ry});
    }
    if (minX == maxX) { minX -= 0.5; maxX += 0.5; }
    if (minY == maxY) { minY -= 0.5; maxY += 0.5; }


    // --- 곡선/경로 ---
    QPainterPath path;
    QPointF firstPt = scale(mapData[0].cx, mapData[0].cy);
    path.moveTo(firstPt);
    for (int i = 1; i < mapData.size() - 2; ++i) {
        QPointF p0 = scale(mapData[i].cx, mapData[i].cy);
        QPointF p1 = scale(mapData[i + 1].cx, mapData[i + 1].cy);
        QPointF mid = (p0 + p1) / 2;
        path.quadTo(p0, mid);
    }
    if (mapData.size() >= 2) {
        QPointF last = scale(mapData.last().cx, mapData.last().cy);
        path.lineTo(last);
    }
    auto *pathItem = new QGraphicsPathItem(path);
    QPen pen(QColor("#00ffcc")); pen.setWidth(4); pen.setCapStyle(Qt::RoundCap);
    pathItem->setPen(pen);
    scene->addItem(pathItem);

    // --- 시작/끝점 마커/라벨 ---
    QPointF start = scale(mapData.first().cx, mapData.first().cy);
    QPointF end = scale(mapData.last().cx, mapData.last().cy);

    scene->addEllipse(start.x() - 10, start.y() - 10, 20, 20,
                      QPen(QColor("#00ffff"), 2), QBrush(QColor(0, 255, 255, 150)));
    QFont labelFont("Segoe UI", 12, QFont::Bold);
    auto *startTxt = scene->addText("START", labelFont);
    startTxt->setDefaultTextColor(QColor("#00ffff"));
    startTxt->setPos(start.x() - 24, start.y() - 28);

    scene->addEllipse(end.x() - 10, end.y() - 10, 20, 20,
                      QPen(QColor("#ff0066"), 2), QBrush(QColor(255, 0, 102, 150)));
    auto *endTxt = scene->addText("END", labelFont);
    endTxt->setDefaultTextColor(QColor("#ff0066"));
    endTxt->setPos(end.x() - 10, end.y() + 12);

    // --- 범례(좌상단) ---
    const int r = 8, lx = 22, ly = 20, spacing = 22;
    int idx = 0;
    QFont legendFont("Arial", 11, QFont::Bold);
    for (auto it = classColor.begin(); it != classColor.end(); ++it, ++idx) {
        QColor col = it.value();
        scene->addEllipse(lx, ly + idx * spacing, 2*r, 2*r, QPen(Qt::black, 2), QBrush(col));
        auto *txt = scene->addText(it.key(), legendFont);

        if(it.key() == "ripe")
            txt = scene->addText("Ripe", legendFont);

        txt->setDefaultTextColor(Qt::white);
        txt->setPos(lx + 2*r + 8, ly + idx*spacing - 4);
    }

    // --- 날짜(우하단, scene 기준) ---
    QString timeText;
    const MapPoint& last = mapData.last();
    if (!last.createdAt.isEmpty()) {
        int spaceIdx = last.createdAt.indexOf(' ');
        timeText = (spaceIdx > 0) ? last.createdAt.left(spaceIdx) : last.createdAt;
    } else
        timeText = QString::number(last.ts);
    QFont dateFont("Consolas", 14, QFont::Bold);
    auto *tsItem = scene->addText(timeText, dateFont);
    tsItem->setDefaultTextColor(Qt::darkGray);

    int marginRight = 24, marginBottom = 20, charPixelW = 11;
    double textX = sceneW - (timeText.size() * charPixelW) - marginRight;
    double textY = sceneH - marginBottom - 22;
    tsItem->setPos(textX, textY);

    // (디버그) 시작/끝점 콘솔 출력
    qDebug() << "[MAP] 시작점:" << mapData.first().cx << mapData.first().cy
             << "끝점:" << mapData.last().cx << mapData.last().cy;
}
