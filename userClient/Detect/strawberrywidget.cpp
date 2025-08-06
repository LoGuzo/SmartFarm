#include "strawberrywidget.h"
#include "ui_strawberrywidget.h"
#include <QDate>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include "clientUtil.h"
#include "network.h"
#include "drawmap.h"

StrawBerryWidget::StrawBerryWidget(QStackedWidget *stack, QWidget *parent)
    : DetectCoreWidget(stack, parent)
    , ui(new Ui::StrawBerryWidget)
{
    ui->setupUi(this);
    type = "strawberry_detected";
    tableWidget = ui->strawEventTable;
    tableWidget->SetDetectStr(type);
    tableWidget->setPageSize(4);
    myIndex = 3;
    setupLineChart();
    connect(ui->btnBackFromGrowth, &QPushButton::clicked, this, &StrawBerryWidget::showHomePage);

    setupMapView();
}

StrawBerryWidget::~StrawBerryWidget()
{
    delete ui;
}

void StrawBerryWidget::appendData(const QString &newDateStr,
                                  const QString &newCountsStr,
                                  const int &newCountsInt) {
    qDebug() << "[appendData] date =" << newDateStr
             << ", class =" << newCountsStr
             << ", count =" << newCountsInt;

    // 1) 문자열을 QDate로 변환
    QDate newDate = QDate::fromString(newDateStr, "yyyy-MM-dd");
    if (!newDate.isValid()) {
        qDebug() << "[appendData] Invalid QDate:" << newDateStr;
        return;
    }

    // 2) 삽입 (기존 같은 날짜가 있으면 덮어쓰기)
    data[newDate][newCountsStr] = newCountsInt;

    // 3) 사이즈가 7개 초과하면 맨 앞(가장 오래된) 제거
    while (data.size() > 7) {
        data.erase(data.begin());
    }

    // 4) 차트 갱신
    QMetaObject::invokeMethod(this, [=]() {
        updatePieChartFromTable();
        updateLineChartFromData(QString("ripe"));
    }, Qt::QueuedConnection);
}


void StrawBerryWidget::pageChangedIdx() {
    // 질병 파이차트 정리
    if (diseasePieChartView) {
        QMetaObject::invokeMethod(this, [=]() {
            diseasePieChartView->hide();
            diseasePieChartView->deleteLater();
            diseasePieChartView = nullptr;
        }, Qt::QueuedConnection);
    }
    // disease mode 상태 초기화
    isDiseaseMode = false;

    // 기본 처리
    DetectCoreWidget::pageChangedIdx();

    sendFile(type.c_str(), "WEEKLY");

    while (1) {
        char buffer[1024];
        int n = SSL_read(sock_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';

            if (strncmp(buffer, "END", 3) == 0) break;

            QString json = QString::fromUtf8(buffer);
            QStringList fields = json.trimmed().split('|', Qt::SkipEmptyParts);

            if (fields.size() >= 3) {
                QString dateStr = fields[0];
                QString categoryStr = fields[1];
                int count = fields[2].toInt();

                // step1: 데이터 누적만
                QMetaObject::invokeMethod(this, [=]() {
                    appendData(dateStr, categoryStr, count);
                }, Qt::QueuedConnection);
            }

        } else {
            qDebug() << "WeeklyData failed or no data.";
            break;
        }
    }

    // step2: 루프 끝나고 UI 갱신은 한 번만
    QMetaObject::invokeMethod(this, [=]() {
        refreshCharts();
    }, Qt::QueuedConnection);


    requestMapData();
}

void StrawBerryWidget::setupLineChart()
{
    ripeSeries = new QLineSeries();
    ripeSeries->setName("Ripe Count");

    QPen pen(QColor("#1de9b6")); // 밝은 민트색 강조
    pen.setWidthF(2.0);
    ripeSeries->setPen(pen);

    QChart *chart = new QChart();
    chart->addSeries(ripeSeries);
    chart->setTitle("Ripe Strawberry Count");
    chart->setTitleFont(QFont("Segoe UI", 20, QFont::Bold));
    chart->setTitleBrush(QBrush(QColor("#aef3c0")));
    chart->setBackgroundBrush(Qt::transparent);
    chart->legend()->hide();

    // X축 (QDateTimeAxis 사용)
    axisX = new QDateTimeAxis;
    axisX->setFormat("MM-dd");
    axisX->setTitleText("Date");
    axisX->setTitleBrush(QBrush(QColor("#ffffff")));
    axisX->setLabelsColor(QColor("#ffffff"));
    axisX->setGridLineColor(QColor("#225544"));
    chart->addAxis(axisX, Qt::AlignBottom);
    ripeSeries->attachAxis(axisX);

    // Y축
    axisY = new QValueAxis;
    axisY->setTitleText("Count");
    axisY->setTitleBrush(QBrush(QColor("#ffffff")));
    axisY->setLabelsColor(QColor("#ffffff"));
    axisY->setGridLineColor(QColor("#225544"));
    axisY->setRange(0, 10);
    chart->addAxis(axisY, Qt::AlignLeft);
    ripeSeries->attachAxis(axisY);

    ui->lineChartView->setChart(chart);
    ui->lineChartView->setRenderHint(QPainter::Antialiasing);
    ui->lineChartView->setStyleSheet("background-color: transparent; border: none;");
}



// 선택된 항목에 따라 라인차트 업데이트
void StrawBerryWidget::updateLineChartFromData(const QString& category)
{
    if (!ripeSeries || !axisX || !axisY) return;
    ripeSeries->clear();
    QChart* chart = ui->lineChartView->chart();

    QList<QDate> dates = data.keys();
    std::sort(dates.begin(), dates.end());
    int maxVal = 1;

    for (const QDate& date : dates) {
        int count = 0;
        if (category == "ripe" || category == "unripe") {
            count = data[date].value(category, 0);
        } else if (category == "disease") {
            // 전체 질병 합산
            const QMap<QString, int>& dailyMap = data[date];
            for (auto it = dailyMap.begin(); it != dailyMap.end(); ++it) {
                QString cls = it.key();
                if (cls != "ripe" && cls != "unripe") {
                    count += it.value();
                }
            }
        } else {
            // 실제 질병명인 경우 (예: Angular Leafspot)
            count = data[date].value(category, 0);
        }
        ripeSeries->append(date.startOfDay().toMSecsSinceEpoch(), count);
        maxVal = std::max(maxVal, count);
    }

    if (!dates.isEmpty()) {
        axisX->setRange(dates.first().startOfDay(), dates.last().addDays(1).startOfDay());
    }
    axisY->setRange(0, maxVal + 2);

    QString title;
    if (category == "ripe") title = "Ripe Strawberry Count";
    else if (category == "unripe") title = "Unripe Strawberry Count";
    else if (category == "disease") title = "Total Disease Count";
    else title = QString("%1 Count").arg(category);  // 병명 직접 반영!
    chart->setTitle(title);
}

void StrawBerryWidget::updateLineChartFromData() {
    updateLineChartFromData("ripe");
}

void StrawBerryWidget::onPieSliceClicked(QPieSlice* slice)
{
    QString category = slice->label();

    // ✅ label -> 내부 데이터 키로 매핑
    if (category == "Ripe") category = "ripe";
    else if (category == "Unripe") category = "unripe";
    else if (category == "Disease") category = "disease";

    if (category == "disease") {
        showDiseaseMode(true);
        QMetaObject::invokeMethod(this, [=]() {
            updateLineChartFromData(category);
        }, Qt::QueuedConnection);
    } else {
        updateLineChartFromData(category);
    }
}
void StrawBerryWidget::updatePieChartFromTable()
{
    QPieSeries* series = new QPieSeries();
    QMap<QString, int> eventCounts;
    for (auto outerIt = data.begin(); outerIt != data.end(); ++outerIt) {
        const QMap<QString, int>& dailyMap = outerIt.value();
        for (auto innerIt = dailyMap.begin(); innerIt != dailyMap.end(); ++innerIt) {
            const QString& event = innerIt.key();
            int count = innerIt.value();
            if (event == "ripe" || event == "unripe") {
                eventCounts[event] += count;
            } else {
                eventCounts["disease"] += count;
            }
        }
    }

    for (auto it = eventCounts.begin(); it != eventCounts.end(); ++it) {
        QString rawKey = it.key();
        QString labelText;  // ✅ 변경됨

        // ✅ 범례 텍스트를 첫 글자 대문자로 설정
        if (rawKey == "ripe") labelText = "Ripe";
        else if (rawKey == "unripe") labelText = "Unripe";
        else if (rawKey == "disease") labelText = "Disease";
        else labelText = rawKey;

        QPieSlice* slice = new QPieSlice(labelText, it.value());  // ✅ 변경됨
        if (rawKey == "ripe") slice->setBrush(QColor("#f28b82"));
        else if (rawKey == "unripe") slice->setBrush(QColor("#81c995"));
        else slice->setBrush(QColor("#9e9e9e"));
        slice->setPen(Qt::NoPen);
        slice->setLabelVisible(false);
        series->append(slice);
    }

    connect(series, &QPieSeries::clicked, this, [=](QPieSlice* slice) {
        QString category = slice->label();
        if (category == "Ripe") category = "ripe";            // ✅ 변경됨
        else if (category == "Unripe") category = "unripe";    // ✅ 변경됨
        else if (category == "Disease") category = "disease";  // ✅ 변경됨

        if (category == "disease") {
            showDiseaseMode(true);  // UI 조작
            QMetaObject::invokeMethod(this, [=]() {
                updateLineChartFromData(category);  // 안전한 비동기 호출
            }, Qt::QueuedConnection);
        } else {
            updateLineChartFromData(category);
        }
    });

    QChart* chart = new QChart();
    chart->addSeries(series);
    chart->setTitle("Strawberry Status");
    chart->setTitleBrush(QBrush(QColor("#ffffff"))); // ✅ 제목 색상 민트로

    chart->setBackgroundVisible(false);
    chart->setBackgroundBrush(Qt::NoBrush);
    chart->setPlotAreaBackgroundVisible(false);
    chart->setMargins(QMargins(0, 0, 0, 0));
    chart->legend()->setVisible(true);
    chart->legend()->setLabelColor(QColor("#ffffff"));     // ✅ 추가
    chart->legend()->setFont(QFont("Segoe UI", 11));       // ✅ 추가
    // ✅ 범례 네모 테두리 제거
    for (QLegendMarker* marker : chart->legend()->markers(series)) {
        marker->setLabelBrush(QBrush(QColor("#ffffff")));
        marker->setPen(QPen(QColor("#0d1e1e")));
    }

    series->setLabelsVisible(false);
    for (QPieSlice *slice : series->slices()) {
        slice->setPen(Qt::NoPen);
    }

    ui->pieChartView->setChart(chart);
    ui->pieChartView->setRenderHint(QPainter::Antialiasing);
    connect(series, &QPieSeries::clicked, this, &StrawBerryWidget::onPieSliceClicked);
}

void StrawBerryWidget::onDiseaseSliceClicked(QPieSlice* slice)
{
    if (!slice || !diseaseChart) return;
    QString label = slice->label();
    qDebug() << "[Disease Pie] slice clicked:" << label;
    updateLineChartFromData(label);
    tableWidget->setClassType(label.toStdString());
}

//전환 부분 애니메이션
void StrawBerryWidget::showDiseaseMode(bool enable)
{
    if (!ui) return;

    if (enable) {
        if (!isDiseaseMode) {
            // ✅ 원래 pieChartView 위치 저장
            QRect geom = ui->pieChartView->geometry();
            QPoint frameTopLeft = ui->pieChartView->parentWidget()->mapFromGlobal(
                ui->pieChartView->mapToGlobal(QPoint(0, 0))
                );
            originalPieGeometry = QRect(frameTopLeft, geom.size());
            isDiseaseMode = true;
        }

        // ✅ 질병 파이 차트 표시
        showDiseasePieChart();

        // ✅ pieChartView 축소 애니메이션
        QRect target = QRect(originalPieGeometry.x(), originalPieGeometry.y() + originalPieGeometry.height() * 0.6,
                             originalPieGeometry.width() * 0.4, originalPieGeometry.height() * 0.4);
        QPropertyAnimation* shrink = new QPropertyAnimation(ui->pieChartView, "geometry");
        shrink->setDuration(300);
        shrink->setStartValue(originalPieGeometry);
        shrink->setEndValue(target);

        // ✅ shrink 끝난 후에 질병 차트를 앞으로 올림
        connect(shrink, &QPropertyAnimation::finished, this, [=]() {
            if (diseasePieChartView) {
                QGraphicsOpacityEffect* effect = qobject_cast<QGraphicsOpacityEffect*>(diseasePieChartView->graphicsEffect());
                if (effect) {
                    QPropertyAnimation* fadeIn = new QPropertyAnimation(effect, "opacity");
                    fadeIn->setDuration(300);
                    fadeIn->setStartValue(0.0);
                    fadeIn->setEndValue(1.0);
                    fadeIn->start(QAbstractAnimation::DeleteWhenStopped);
                }
                diseasePieChartView->raise();
            }

            if (miniPieButton) {
                miniPieButton->raise();
            }
        });


        shrink->start(QAbstractAnimation::DeleteWhenStopped);

        // ✅ miniPie 클릭 처리용 투명 버튼
        if (!miniPieButton) {
            miniPieButton = new QPushButton(this);
            miniPieButton->setFlat(true);
            miniPieButton->setStyleSheet("background-color: transparent; border: none;");
            connect(miniPieButton, &QPushButton::clicked, this, [=]() {
                qDebug() << "⬅ mini pie clicked!";
                tableWidget->setClassType("");
                showDiseaseMode(false);
            });
        }

        // 버튼을 축소된 pie 위치에 맞게 설정
        QRect miniPieRect = QRect(
            ui->pieChartView->mapTo(this, target.topLeft()),
            target.size()
            );

        // 가운데 정렬된 버튼 크기
        int btnWidth = miniPieRect.width() * 0.6;
        int btnHeight = miniPieRect.height() * 0.6;
        int btnX = miniPieRect.x() + (miniPieRect.width() - btnWidth) / 2;
        int btnY = miniPieRect.y() + (miniPieRect.height() - btnHeight) / 2;

        miniPieButton->setGeometry(btnX, btnY, btnWidth, btnHeight);

        miniPieButton->raise();
        miniPieButton->show();

    } else {
        // ✅ 복원 애니메이션
        QPropertyAnimation* expand = new QPropertyAnimation(ui->pieChartView, "geometry");
        expand->setDuration(300);
        expand->setStartValue(ui->pieChartView->geometry());
        expand->setEndValue(originalPieGeometry);
        expand->start(QAbstractAnimation::DeleteWhenStopped);

        // ✅ 질병 파이 차트 제거
        if (diseasePieChartView) {
            diseasePieChartView->hide();
            diseasePieChartView->deleteLater();
            diseasePieChartView = nullptr;
        }

        // ✅ 투명 버튼 숨기기
        if (miniPieButton) {
            miniPieButton->hide();
        }

        isDiseaseMode = false;
    }
}

QMap<QString, QColor> diseaseColorMap = {
    { "Angular Leafspot",       QColor(85, 107, 47) },     // 진녹/올리브
    { "Anthracnose Fruit Rot",  QColor(230, 126, 34) },    // 오렌지
    { "Blossom Blight",         QColor(238, 130, 238) },   // 연분홍
    { "Gray Mold",              QColor(128, 128, 128) },   // 회색
    { "Leaf Spot",              QColor(139, 69, 19) },     // 진갈색
    { "Powdery Mildew Fruit",   QColor(203, 195, 227) },   // 연보라
    { "Powdery Mildew Leaf",    QColor(144, 238, 144) },   // 연녹색
    { "Ripe",                   QColor(220, 30, 70) },     // 진빨강 (← 안 써도 되지만 참고)
};
void StrawBerryWidget::showDiseasePieChart()
{
    if (diseasePieChartView) {
        qDebug() << "[INFO] diseasePieChartView already exists!";
        return;
    }

    QPieSeries* diseaseSeries = new QPieSeries();
    QMap<QString, int> diseaseCounts;

    for (auto outerIt = data.begin(); outerIt != data.end(); ++outerIt) {
        const QMap<QString, int>& dailyMap = outerIt.value();
        for (auto innerIt = dailyMap.begin(); innerIt != dailyMap.end(); ++innerIt) {
            QString cls = innerIt.key();
            if (cls != "ripe" && cls != "unripe") {
                diseaseCounts[cls] += innerIt.value();
            }
        }
    }

    for (auto it = diseaseCounts.begin(); it != diseaseCounts.end(); ++it) {
        QPieSlice* slice = new QPieSlice(it.key(), it.value());
        slice->setLabelVisible(true);
        // ✅ 색상 지정
        if (diseaseColorMap.contains(it.key())) {
            slice->setColor(diseaseColorMap[it.key()]);
        }
        diseaseSeries->append(slice);
        slice->setPen(Qt::NoPen); // ✅ append 이후에 호출!
    }


    connect(diseaseSeries, &QPieSeries::clicked, this, &StrawBerryWidget::onDiseaseSliceClicked);

    diseaseChart = new QChart();
    diseaseChart->addSeries(diseaseSeries);
    diseaseChart->setTitle("Disease Detail");
    diseaseChart->setTitleBrush(QBrush(QColor("#aef3c0"))); // ✅ 제목 색상 민트로

    diseaseChart->setBackgroundVisible(false);
    diseaseChart->legend()->setVisible(true);
    diseaseChart->legend()->setLabelColor(QColor("#aef3c0"));  // ✅ 글자색
    diseaseChart->legend()->setFont(QFont("Segoe UI", 11));    // ✅ 폰트 통일
    // ✅ 범례 네모 테두리 제거
    for (QLegendMarker* marker : diseaseChart->legend()->markers(diseaseSeries)) {
        marker->setLabelBrush(QBrush(QColor("#aef3c0"))); // 텍스트 민트
        marker->setPen(QPen(QColor("#0d1e1e")));          // 테두리 민트 → 거의 배경과 같음
    }


    diseasePieChartView = new QChartView(diseaseChart, this);
    diseasePieChartView->setRenderHint(QPainter::Antialiasing);
    diseasePieChartView->setStyleSheet("background-color: transparent; border: none;");

    QRect targetRect = QRect(
        ui->pieChartView->mapTo(this, QPoint(0, 0)),  // ← this 기준으로 좌표 변환
        ui->pieChartView->size()
        );
    diseasePieChartView->setGeometry(targetRect);
    // ✅ opacity 효과 추가
    QGraphicsOpacityEffect* effect = new QGraphicsOpacityEffect(diseasePieChartView);
    effect->setOpacity(0.0);  // 처음에는 완전히 투명
    diseasePieChartView->setGraphicsEffect(effect);

    diseasePieChartView->show();
}


bool StrawBerryWidget::eventFilter(QObject *obj, QEvent *event)
{
    if ((obj == ui->pieChartView || obj == diseasePieChartView) && isDiseaseMode) {
        if (event->type() == QEvent::MouseButtonPress) {
            if (!ui || !diseasePieChartView) return false;
            qDebug() << "✅ 복원 시작";
            showDiseaseMode(false);
            return true;
        }
    }
    return DetectCoreWidget::eventFilter(obj, event);
}

void StrawBerryWidget::setupMapView() {
    QVBoxLayout *layout = new QVBoxLayout(ui->graphicView);
    layout->setContentsMargins(0, 0, 0, 0);  // 여백 제거

    QGraphicsView *mapView = new QGraphicsView(ui->graphicView);
    // mapView->setFixedSize(640, 480);
    mapView->setStyleSheet(R"(
    border: 1px solid #33aa88;
    border-radius: 10px;
    padding: 8px;
    outline: none;
)");
    mapView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    mapView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    layout->addWidget(mapView);

    int x = this->width() - mapView->width() - 24;
    int y = this->height() - mapView->height() - 20;
    mapView->move(x, y);
    mapView->raise();

    drawMap = new DrawMap(mapView, this);
}

void StrawBerryWidget::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);

    if (!isDiseaseMode || !ui || !miniPieButton)
        return;

    // 💡 원래 pieChartView 위치 재계산
    QRect geom = ui->pieChartView->geometry();
    QPoint globalTopLeft = ui->pieChartView->mapTo(this, QPoint(0, 0));
    originalPieGeometry = QRect(globalTopLeft, geom.size());

    // 💡 축소 후 위치 재계산
    QRect target = QRect(
        originalPieGeometry.x(),
        originalPieGeometry.y() + originalPieGeometry.height() * 0.6,
        originalPieGeometry.width() * 0.4,
        originalPieGeometry.height() * 0.4
        );

    // 💡 pieChartView와 miniPieButton의 위치 다시 설정
    ui->pieChartView->setGeometry(target);
    miniPieButton->setGeometry(target);

    if (diseasePieChartView) {
        diseasePieChartView->setGeometry(target);  // 👈 같이 움직이도록 맞춰줌
    }
}

void StrawBerryWidget::refreshCharts()
{
    if (!this || !ui) return;  // null pointer 안전
    updatePieChartFromTable();
    updateLineChartFromData(QString("ripe"));
}

void StrawBerryWidget::requestMapData() {
    if (!drawMap) {
        qDebug() << "[StrawBerry] DrawMap object not initialized!";
        return;
    }
    // MAP 데이터 요청 트리거
    sendFile("1", "MAP");

    // 1. 줄 단위 robust 함수로 교체!
    bool success = drawMap->receiveMapData(sock_fd);
    if (success) {
        qDebug() << "[StrawBerry] Map data received and rendered successfully";
    } else {
        qDebug() << "[StrawBerry] Failed to receive map data";
    }
}
