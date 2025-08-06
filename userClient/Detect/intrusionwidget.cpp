#include "intrusionwidget.h"
#include "ui_intrusionwidget.h"
#include "clientUtil.h"
#include "network.h"
#include <QDateTime>
#include <QDebug>
#include <QtCharts/QBarSet>
#include <QtCharts/QBarSeries>
#include <QtCharts/QCategoryAxis>
#include <QtCharts/QValueAxis>
#include <QtCharts/QChart>
#include <QColor>
#include <QTimer>

IntrusionWidget::IntrusionWidget(QStackedWidget *stack, QWidget *parent)
    : DetectCoreWidget(stack, parent)
    , ui(new Ui::IntrusionWidget)
    , m_stack(stack)
{
    ui->setupUi(this);
    setupBarChart();
    setupBarInteraction();

    type = "intrusion_detected";
    tableWidget = ui->intrusionEventTable;
    tableWidget->SetDetectStr(type);
    tableWidget->setPageSize(10);
    myIndex = 2;

    connect(ui->btnBackFromIntrusion, &QPushButton::clicked, this, &IntrusionWidget::showHomePage);
    connect(ui->dateEdit, &QDateEdit::dateChanged, this, &IntrusionWidget::setDateAndUpdate);

    QTimer::singleShot(0, this, [=]() {
        ui->dateEdit->setDate(QDate::currentDate());
    });
}

IntrusionWidget::~IntrusionWidget()
{
    delete ui;
}

void IntrusionWidget::pageChangedIdx() {
    // 기본 처리
    DetectCoreWidget::pageChangedIdx();

    QTableWidget* realTable = tableWidget->findChild<QTableWidget*>();
    if (realTable && realTable->rowCount() > 0) {
        QString datetimeStr = realTable->item(0, 0)->text();  // 첫 번째 행의 첫 번째 열
        QDateTime ts = QDateTime::fromString(datetimeStr, "yyyy-MM-dd HH:mm:ss");
        if (ts.isValid()) {
            intrusionTimestamps.append(ts);
            setDateAndUpdate(ts.date());
        }
    }

}

void IntrusionWidget::setupBarChart()
{
    // 1. BarSet 생성 (색상/테두리 커스텀)
    barSet = new QBarSet("Intrusions");
    for (int i = 0; i < 24; ++i) {
        *barSet << 0;
    }
    barSet->setColor(QColor("#33aa88"));  // 메인 민트 컬러
    barSet->setBorderColor(QColor("#1de9b6")); // 엣지 민트
    barSet->setLabelBrush(QColor("#ffffff")); // 값 표시 색
    barSet->setLabelFont(QFont("Segoe UI", 10, QFont::Bold));

    // 2. BarSeries 설정
    barSeries = new QBarSeries();
    barSeries->append(barSet);
    barSeries->setBarWidth(0.9);
    barSeries->setLabelsVisible(true);  // 막대 위 숫자 표시

    // 3. Chart 설정
    barChart = new QChart();
    barChart->addSeries(barSeries);
    barChart->setTitle("Hourly Intrusion Events");
    barChart->setTitleFont(QFont("Segoe UI", 13));
    barChart->setTitleBrush(QBrush(QColor("#ffffff")));
    barChart->setBackgroundVisible(false);
    barChart->setBackgroundBrush(Qt::NoBrush);
    barChart->setPlotAreaBackgroundVisible(false);
    barChart->legend()->hide();

    // 4. X축: QCategoryAxis (2시간 간격 라벨)
    axisX = new QCategoryAxis();
    for (int i = 0; i < 24; i += 2) {
        axisX->append(QString("%1시").arg(i), i);
    }
    axisX->setRange(0, 23);
    axisX->setLabelsColor(QColor("#ffffff"));
    axisX->setTitleBrush(QBrush(QColor("#ffffff")));
    axisX->setTitleText("Hour");
    axisX->setTitleFont(QFont("Segoe UI", 11));
    axisX->setLabelsFont(QFont("Segoe UI", 9));

    QPen gridPen(QColor("#406060"));
    gridPen.setStyle(Qt::DashLine);
    axisX->setGridLinePen(gridPen);

    barChart->addAxis(axisX, Qt::AlignBottom);
    barSeries->attachAxis(axisX);

    // 5. Y축
    axisY = new QValueAxis();
    axisY->setRange(0, 5);
    axisY->setLabelFormat("%d");
    axisY->setTitleText("Count");
    axisY->setLabelsColor(QColor("#ffffff"));
    axisY->setTitleBrush(QBrush(QColor("#ffffff")));
    axisY->setTitleFont(QFont("Segoe UI", 11));
    axisY->setLabelsFont(QFont("Segoe UI", 9));
    axisY->setGridLinePen(gridPen);

    barChart->addAxis(axisY, Qt::AlignLeft);
    barSeries->attachAxis(axisY);

    // 6. ChartView
    ui->chartView->setChart(barChart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
}

void IntrusionWidget::addIntrusionEvent(const QDateTime& timestamp)
{
    if (!intrusionTimestamps.contains(timestamp)) {  // ✅ 중복 방지
        intrusionTimestamps.append(timestamp);
    }
    // 현재 선택된 날짜와 같으면 차트 갱신
    if (timestamp.date() == selectedDate) {
        setDateAndUpdate(selectedDate);
    }
}

void IntrusionWidget::setDateAndUpdate(const QDate& date)
{
    selectedDate = date;

    // ✅ Bar Chart 초기화
    barSet->remove(0, barSet->count());
    for (int i = 0; i < 24; ++i)
        *barSet << 0;

    axisY->setRange(0, 5);
    axisY->setTickCount(6);
    axisY->applyNiceNumbers();
    ui->chartView->update();

    // ✅ 서버에서 차트 데이터 요청 (테이블 X)
    requestIntrusionData(date.toString("yyyy-MM-dd"));
}

void IntrusionWidget::setupBarInteraction()
{
    connect(barSet, &QBarSet::clicked, this, [=](int index) {
        tableWidget->setDateHour(makeTimeRangeString(index));
    });
}

std::string IntrusionWidget::makeTimeRangeString(int index) {
    // 1) dateEdit에서 날짜를 꺼냅니다.
    QDate date = ui->dateEdit->date();

    // 2) 시작 시각 계산 (index 시 0분 0초)
    QDateTime start(date, QTime(index, 0, 0));

    // 3) 1시간 뒤 계산
    QDateTime end = start.addSecs(3600);

    // 4) 문자열 포맷
    const QString fmt = "yyyy-MM-dd HH:mm:ss";
    QString s = start.toString(fmt);
    QString e = end  .toString(fmt);

    // 5) std::string으로 변환해 반환
    return (s + "|" + e).toStdString();
}

void IntrusionWidget::requestIntrusionData(const QString& date)
{
    std::string payload = type + "|" + date.toStdString();
    sendFile(payload.c_str(), "BAR");

    while (1) {
        char buffer[1024];
        int n = SSL_read(sock_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';

            // 종료 조건
            if (strncmp(buffer, "END", 3) == 0) break;

            // 예: "10|3"
            QString line = QString::fromUtf8(buffer).trimmed();
            QStringList parts = line.split('|');
            if (parts.size() == 2) {
                bool ok1 = false, ok2 = false;
                int index = parts[0].toInt(&ok1);
                int count = parts[1].toInt(&ok2);

                qDebug()<<index<<count;
                if (ok1 && ok2 && index >= 0 && index < 24) {
                    // 실시간 업데이트 호출 (쓰레드 안전하게)
                    QMetaObject::invokeMethod(this, [=]() {
                        updateHourlyChart(index, count);
                    }, Qt::QueuedConnection);
                }
            }
        } else {
            qDebug() << "[Intrusion] BAR receive failed.";
            break;
        }
    }
}

void IntrusionWidget::updateHourlyChart(const int index, const int count)
{
    if (index >= 0 && index < barSet->count()) {
        barSet->replace(index, count);  // ✅ 올바른 방식

        // Y축 최대값 재계산
        int maxVal = 0;
        for (int i = 0; i < barSet->count(); ++i) {
            maxVal = std::max(maxVal, int(barSet->at(i)));
        }

        axisY->setRange(0, std::max(5, maxVal + 1));
        axisY->setTickCount(std::max(6, maxVal + 2));
        axisY->applyNiceNumbers();
        axisY->setLabelFormat("%d");

        ui->chartView->update();
    }

}
