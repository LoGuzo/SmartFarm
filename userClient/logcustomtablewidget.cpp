#include "logcustomtablewidget.h"
#include "ui_CustomTableWidget.h"
#include "LogSystemManager.h"
#include <QHeaderView>
#include <QStandardItem>
#include <QPixmap>

LogCustomTableWidget::LogCustomTableWidget(QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::CustomTableWidget)
    , m_sourceModel(new QStandardItemModel(this))
    , m_proxy(new PaginationProxyModel(this))
    , m_pageSize(10)
{
    ui->setupUi(this);

    // 테이블 헤더 설정
    ui->eventTable->verticalHeader()->setVisible(false);
    ui->eventTable->horizontalHeader()
        ->setSectionResizeMode(QHeaderView::Stretch);

    // 모델 ↔ 프록시 연결
    m_proxy->setSourceModel(m_sourceModel);
    m_proxy->setPageSize(m_pageSize);
    ui->eventTable->setModel(m_proxy);

    // 1) 기존 로그 전체 로드
    LogSystemManager::instance().populateModel(m_sourceModel);

    // 2) 초기 버튼 상태
    updateButtons();

    // 3) logsLoaded 시그널 연결 → 전체 리셋
    connect(&LogSystemManager::instance(),
            &LogSystemManager::logsLoaded,
            this,
            &LogCustomTableWidget::onLogsLoaded);

    // 4) logAppended 시그널 연결 → 한 줄씩 추가
    connect(&LogSystemManager::instance(),
            &LogSystemManager::logAppended,
            this,
            &LogCustomTableWidget::onLogAppended);
}

LogCustomTableWidget::~LogCustomTableWidget(){
    delete ui;
}

void LogCustomTableWidget::setPageSize(int sz) {
    m_pageSize = sz;
    m_proxy->setPageSize(sz);
    updateButtons();
}

void LogCustomTableWidget::setData(const QList<QStringList>& rows) {
    // 직접 전체 데이터를 채우고 싶을 때
    m_sourceModel->clear();
    m_sourceModel->setColumnCount(3);
    m_sourceModel->setHorizontalHeaderLabels(
        QStringList{"Date", "Eventname", "Image"});

    for (const auto& fields : rows) {
        QList<QStandardItem*> items;
        items << new QStandardItem(fields.value(0))
              << new QStandardItem(fields.value(1));

        QStandardItem* imgItem = new QStandardItem();
        QPixmap pix(fields.value(2));
        imgItem->setData(pix, Qt::DecorationRole);
        items << imgItem;

        for (auto* it : items)
            it->setTextAlignment(Qt::AlignCenter);

        m_sourceModel->appendRow(items);
    }

    updateButtons();
}

void LogCustomTableWidget::on_prevButton_clicked() {
    int pg = m_proxy->currentPage();
    if (pg > 0) {
        m_proxy->setCurrentPage(pg - 1);
        updateButtons();
    }
}

void LogCustomTableWidget::on_nextButton_clicked() {
    int pg = m_proxy->currentPage();
    if (pg + 1 < m_proxy->pageCount()) {
        m_proxy->setCurrentPage(pg + 1);
        updateButtons();
    }
}

void LogCustomTableWidget::updateButtons() {
    int pg = m_proxy->currentPage();
    int pc = m_proxy->pageCount();
    ui->prevButton->setEnabled(pg > 0);
    ui->nextButton->setEnabled(pg + 1 < pc);
}

// ─── 슬롯: 전체 로그를 다시 불러와 모델 리셋 ─────────────────
void LogCustomTableWidget::onLogsLoaded() {
    LogSystemManager::instance().populateModel(m_sourceModel);
    updateButtons();
}

// ─── 슬롯: 새 로그 한 줄 append ─────────────────────────
void LogCustomTableWidget::onLogAppended(const LogEntry& e) {
    QList<QStandardItem*> row;
    row << new QStandardItem(e.timestamp.toString("yyyy-MM-dd hh:mm:ss"))
        << new QStandardItem(e.eventName)
        << new QStandardItem(e.eventDetail);
    for (auto* it : row){
        it->setTextAlignment(Qt::AlignCenter);
        it->setForeground(QColor("#ffffff"));
    }

    // 맨 위(0번 행)에 삽입
    m_sourceModel->insertRow(0, row);
    updateButtons();
}
