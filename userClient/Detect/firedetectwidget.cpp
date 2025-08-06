#include "firedetectwidget.h"
#include "ui_firedetectwidget.h"
#include "sensorreceive.h"
FireDetectWidget* fireWidget = nullptr;

FireDetectWidget::FireDetectWidget(QStackedWidget *stack, QWidget *parent)
    : DetectCoreWidget(stack, parent)
    , ui(new Ui::FireDetectWidget)
{
    ui->setupUi(this);
    type = "fire_detected";
    tableWidget = ui->fireEventTable;
    tableWidget->SetDetectStr(type);
    tableWidget->setPageSize(9);
    myIndex = 1;
    setupChart();
    fireWidget = this; //포인터 등록

    // thermometer icon
    QPixmap tempIcon(":/icons/temperature_icon3.png");

    // // 🔁 랜덤 센서 시뮬레이션 (3초 간격)
    // QTimer* simTimer = new QTimer(this);
    // connect(simTimer, &QTimer::timeout, this, [=]() {
    //     double co2 = 400 + QRandomGenerator::global()->bounded(400);   // 400~800 ppm
    //     double temp = 20 + QRandomGenerator::global()->bounded(10);    // 20~30℃
    //     QString data = QString("%1 %2").arg(co2).arg(temp);
    //     onSensorDataReceived(data);
    // });
    // simTimer->start(3000);  // 3초마다

    connect(ui->btnBackFromFire, &QPushButton::clicked, this, &FireDetectWidget::showHomePage);
}
void FireDetectWidget::setupChart()
{
    // 1. CO₂ 선 그래프 스타일
    co2Series = new QLineSeries();
    co2Series->setName("CO₂ 농도 (ppm)");
    QPen pen(QColor("#1de9b6")); // 밝은 민트색 강조
    pen.setWidthF(2.0);
    co2Series->setPen(pen);

    // 2. Chart 설정
    QChart *chart = new QChart();
    chart->addSeries(co2Series);
    chart->setTitle("CO₂ LEVELS");
    chart->setTitleFont(QFont("Segoe UI", 20, QFont::Bold));
    chart->setTitleBrush(QBrush(QColor("#aef3c0")));
    chart->setBackgroundBrush(Qt::transparent);  // 투명
    chart->legend()->hide();

    // 3. X축 (시간)
    axisX = new QDateTimeAxis;
    axisX->setFormat("HH:mm");  // 더 정밀하게 보기
    axisX->setTitleText("Time");
    axisX->setTitleBrush(QBrush(QColor("#ffffff")));
    axisX->setLabelsColor(QColor("#ffffff"));
    axisX->setGridLineColor(QColor("#225544"));
    chart->addAxis(axisX, Qt::AlignBottom);
    co2Series->attachAxis(axisX);

    // 🔧 초기 X축 범위: 최근 1분 → 데이터 하나만 들어와도 보이게
    QDateTime now = QDateTime::currentDateTime();
    axisX->setRange(now.addSecs(-60), now);

    // 4. Y축 (농도)
    axisY = new QValueAxis;
    axisY->setTitleText("농도 (ppm)");
    axisY->setTitleBrush(QBrush(QColor("#ffffff")));
    axisY->setLabelsColor(QColor("#ffffff"));
    axisY->setGridLineColor(QColor("#225544"));
    axisY->setRange(300, 2000);  // 🔧 더 넓게 잡아서 보장
    chart->addAxis(axisY, Qt::AlignLeft);
    co2Series->attachAxis(axisY);

    // 5. ChartView 스타일
    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
    ui->chartView->setStyleSheet("background-color: transparent; border: none;");
}
void FireDetectWidget::onSensorDataReceived(const QString& data) {
    // 예: "738 23.4"
    QStringList parts = data.split(" ");
    if (parts.size() >= 2) {
        double co2 = parts[0].toDouble();
        double temp = parts[1].toDouble();

        // 최대값 비교
        if (co2 > maxCo2) {
            maxCo2 = co2;
            ui->label_co2MNum->setText(
                QString("<span style='font-size:50pt; font-weight:bold;'>%1</span><br>"
                        "<span style='font-size:20pt;'>ppm</span>").arg((int)maxCo2));
        }

        // 1. QLabel에 실시간 표시
        ui->label_co2Num->setText(
            QString("<span style='font-size:50pt; font-weight:bold;'>%1</span><br>"
                    "<span style='font-size:20pt;'>ppm</span>").arg((int)co2));
        ui->label_TempNum->setText(
            QString("<span style='font-size:50pt; font-weight:bold;'>%1</span><br>"
                    "<span style='font-size:20pt;'>℃</span>").arg(temp, 0, 'f', 1));

        // 2. 그래프에 추가
        QDateTime now = QDateTime::currentDateTime();
        co2Series->append(now.toMSecsSinceEpoch(), co2);

        // X축 시간 범위 업데이트
        axisX->setRange(now.addSecs(-600), now);  // 최근 10분만 보기
    }
}
void FireDetectWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);  // 기본 처리

    // 이미지 원본 로드
    QPixmap tempIcon(":/icons/temperature_icon3.png");

    // label의 현재 크기 가져오기
    QSize labelSize = ui->labelTempIcon->size();

    // 최대 크기 제한 (예: 100x100)
    int maxW = 150;
    int maxH = 150;

    int w = std::min(labelSize.width(), maxW);
    int h = std::min(labelSize.height(), maxH);

    // 비율 유지하면서 부드럽게 스케일
    ui->labelTempIcon->setPixmap(tempIcon.scaled(w, h, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void FireDetectWidget::onSensorDataReceivedWrapper() {
    QString data = QString("%1 %2").arg(sensorData.co2Value).arg(sensorData.tempValue);
    onSensorDataReceived(data);
}
void FireDetectWidget::showEvent(QShowEvent *event)
{
    QWidget::showEvent(event);  // 부모 클래스 기본 동작 먼저 수행

    // 온도 아이콘 강제 스케일링
    QPixmap tempIcon(":/icons/temperature_icon3.png");
    QSize labelSize = ui->labelTempIcon->size();
    int maxW = 150;
    int maxH = 150;
    int w = std::min(labelSize.width(), maxW);
    int h = std::min(labelSize.height(), maxH);
    ui->labelTempIcon->setPixmap(tempIcon.scaled(w, h, Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

FireDetectWidget::~FireDetectWidget()
{
    delete ui;
}
