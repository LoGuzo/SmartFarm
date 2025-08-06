#include "maildialog.h"
#include "ui_maildialog.h"
#include "clientUtil.h"
#include "network.h"
#include <QPushButton>
#include <QtConcurrent>

MailDialog::MailDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::mailDialog)
{
    ui->setupUi(this);
    setWindowTitle("메일 설정");
    setModal(true);
    connect(ui->buttonBox->button(QDialogButtonBox::Ok), &QPushButton::clicked, this, &MailDialog::okButtonClicked);
    this->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
}

MailDialog::~MailDialog()
{
    delete ui;
}

void MailDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);

    chkNowUserMail();
}

QString MailDialog::getEmail() const
{
    return ui->emailLineEdit->text();
}

QString MailDialog::getPassword() const
{
    return ui->passwordLineEdit->text();
}

QString MailDialog::getCredential() const
{
    return getEmail() + "|" + getPassword();
}

void MailDialog::okButtonClicked(){
    if(getEmail() == "" || getPassword() == "")
        return;

    if(getEmail() == ui->mymailLabel->text())
        return;

    QByteArray ba = getCredential().toUtf8();   // 유지되는 QByteArray
    const char* cstr = ba.constData();          // 유효한 메모리 영역

    qDebug() << QString::fromUtf8(cstr);

    sendFile(cstr, "NEWEMAIL");
}

void MailDialog::chkNowUserMail(){
    (void)QtConcurrent::run([this]() {
        sendFile("1", "NOWEMAIL");

        char buffer[1024];
        int n = SSL_read(sock_fd, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';

            QString received = QString::fromUtf8(buffer);

            qDebug() << received;
            // QLabel에 설정
            ui->mymailLabel->setText(received);
        }
    });
}
