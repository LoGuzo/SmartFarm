#include <QApplication>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QThread>
#include <mainwindow.h>
#include "main.h"
#include "network.h"
#include "handshake.h"
#include "sensorreceive.h"
#include "mapactivereceive.h"
#include "LogSystemManager.h"
#include <QApplication>
#include <QFontDatabase>
#include <QFont>

#define IP "192.168.0.46"
#define PORT "60000"  // 문자열 형태로 getaddrinfo에 넘김
#define SENSORPORT "60002"  // 문자열 형태로 getaddrinfo에 넘김
#define USERPORT "60003"  // 문자열 형태로 getaddrinfo에 넘김
#define MAPPORT "60004"  // 문자열 형태로 getaddrinfo에 넘김

int main(int argc, char *argv[]) {
    gst_init(nullptr, nullptr);

    init_openssl();
    SSL_CTX *ctx = create_context();

    int sockfd = socketNetwork(IP, PORT);
    if(network(sockfd, ctx) < 1){
        perror("SSL");
        exit(1);
    }
    handshakeClient("USER", NULL);
    returnSocket();

    int userfd = socketNetwork(IP, USERPORT);
    if(network(userfd, ctx) < 1){
        perror("SSL");
        exit(1);
    }

    int sensorfd = socketNetwork(IP, SENSORPORT);
    SSL* sensor = sensorNetwork(sensorfd, ctx);
    QThread* recvThread = startReceiveSensorThread(sensor);


    int mapfd = socketNetwork(IP, MAPPORT);
    SSL* mapSSL = sensorNetwork(mapfd, ctx);
    QThread* mapRecvThread = startReceiveMapThread(mapSSL);

    LogSystemManager::instance().loadLogData();

    QApplication app(argc, argv);

    // ✅ 폰트 등록 및 전역 적용
    int id = QFontDatabase::addApplicationFont(":/prefix/prefix/Greyscale Basic Regular.ttf");

    if (id == -1) {
        qDebug() << "❌ 폰트 등록 실패: 리소스 경로를 확인하세요.";
    } else {
        QStringList families = QFontDatabase::applicationFontFamilies(id);
        if (families.isEmpty()) {
            qDebug() << "❌ 폰트 등록은 되었지만 패밀리 이름을 가져올 수 없습니다.";
        } else {
            QString Greyscale_Basic = families.at(0);
            qDebug() << "✅ 폰트 등록 성공! 패밀리 이름:" << Greyscale_Basic;

            QFont customFont(Greyscale_Basic);
        }
    }

    MainWindow w;
    w.setWindowTitle("🌱 스마트팜 통합 모니터링 시스템");
    w.resize(1000, 700);
    w.show();

    QObject::connect(&w, &QObject::destroyed, [=]() {
        sensorStop = 1;
        mapActiveStop = 1;
        // 2) 쓰레드가 끝나기를 대기
        recvThread->quit();
        recvThread->wait();

        mapRecvThread->quit();
        mapRecvThread->wait();

        SSL_shutdown(sensor);
    #ifdef _WIN32
        closesocket(sockfd);
        closesocket(userfd);
        closesocket(sensorfd);
        closesocket(mapfd);
        closesocket(SSL_get_fd(sensor));
        closesocket(SSL_get_fd(mapSSL));
    #else
        close(sockfd);
        close(userfd);
        close(sensorfd);
        close(mapfd);
        close(SSL_get_fd(sensor));
        close(SSL_get_fd(mapSSL));
    #endif
        returnSocket();
        SSL_free(sensor);
        SSL_free(mapSSL);
        SSL_CTX_free(ctx);

        LogSystemManager::instance().saveLogData();
    });
    return app.exec();
}

// OpenSSL 초기화
void init_openssl() {
    SSL_load_error_strings();
    OpenSSL_add_ssl_algorithms();
}

// SSL 컨텍스트 생성 (클라이언트용)
SSL_CTX *create_context() {
    const SSL_METHOD *method;
    SSL_CTX *ctx;

    method = SSLv23_client_method();
    ctx = SSL_CTX_new(method);
    if (!ctx) {
        perror("SSL 컨텍스트 생성 실패");
        ERR_print_errors_fp(stderr);
        exit(EXIT_FAILURE);
    }

    return ctx;
}
