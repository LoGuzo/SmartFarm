#include <QApplication>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <mainwindow.h>
#include "main.h"
#include "network.h"
#include "handshake.h"
#include "clientUtil.h"

#define IP "192.168.0.46"
#define PORT "60000"  // 문자열 형태로 getaddrinfo에 넘김
#define SENSORPORT "60002"  // 문자열 형태로 getaddrinfo에 넘김
#define USERPORT "60003"  // 문자열 형태로 getaddrinfo에 넘김

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

    QApplication app(argc, argv);
    MainWindow w;
    w.setWindowTitle("🌱 스마트팜 통합 모니터링 시스템");
    w.resize(1000, 700);
    w.show();

    QObject::connect(&w, &QObject::destroyed, [=]() {
        SSL_shutdown(sensor);
    #ifdef _WIN32
        closesocket(sockfd);
        closesocket(userfd);
        closesocket(sensorfd);
        closesocket(SSL_get_fd(sensor));
    #else
        close(sockfd);
        close(userfd);
        close(sensorfd);
        close(SSL_get_fd(sensor));
    #endif
        returnSocket();
        SSL_free(sensor);
        SSL_CTX_free(ctx);
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
